#!/usr/bin/env python3
# poncelet — single-header amalgamation.
# SPDX-License-Identifier: MIT
#
# Concatenates every public + internal header and every src/*.cpp (with the
# generated data_tables.inc / fixed_lut_tables.inc folded in) into one file,
# dist/poncelet_single.hpp, for game codebases that vendor exactly one file.
#
#   python3 tools/amalgamate.py            # write dist/poncelet_single.hpp
#   python3 tools/amalgamate.py --check    # exit non-zero if the committed file is stale
#   python3 tools/amalgamate.py --minify   # also write dist/poncelet_single.min.hpp
#                                           # (comments + blank lines stripped, ~half
#                                           # the size) for the size-conscious vendor.
#                                           # Not committed / freshness-checked — a
#                                           # build-time convenience, regenerate as needed.
#
# stb-style layout: declarations always compile; the implementation only when
# PONCELET_SINGLE_IMPLEMENTATION is defined, which you do in exactly ONE .cpp:
#
#     #define PONCELET_SINGLE_IMPLEMENTATION
#     #include "poncelet_single.hpp"
#
# every other translation unit just #includes it.
#
# The one hazard of folding many .cpp into one TU is that each has a
# `namespace { ... }` of file-local helpers and the names repeat (kPi, an Rng,
# small math). Those anonymous namespaces merge; the duplicate names then clash.
# isolate_anon() renames each file's own anonymous-namespace declarations to
# `<name>_<stem>` so the merged namespace has no duplicates. The real
# `namespace pon { ... }` blocks are left exactly as written. The C ABI
# (<poncelet/poncelet.h>) is folded in too.

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INCLUDE_DIR = ROOT / "include"
SRC_DIR = ROOT / "src"
OUT = ROOT / "dist" / "poncelet_single.hpp"
MIN_OUT = ROOT / "dist" / "poncelet_single.min.hpp"

_QUOTE_INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"\s*(?://.*)?$')
_SYS_INCLUDE = re.compile(r'^\s*#\s*include\s+<([^>]+)>\s*(?://.*)?$')
_PRAGMA_ONCE = re.compile(r'^\s*#\s*pragma\s+once\s*$')
_ANON_NS = re.compile(r'\bnamespace\s*\{')
_DECL_TYPE = re.compile(r'\b(?:struct|class|union|enum)\s+(\w+)')
# A file-local helper worth renaming: a function, or a `k`-prefixed constant.
# Deliberately conservative — never a bare `x` / `i` that would rewrite `pos.x`.
_DECL_FUNC = re.compile(
    r'^\s*(?:template\s*<[^;{]*>\s*)?'
    r'[A-Za-z_][\w:]*(?:\s*<[^;{]+>)?(?:\s*[*&]+\s*|\s+)([A-Za-z_]\w*)\s*\(')
_DECL_KCONST = re.compile(
    r'^\s*(?:inline\s+)?(?:constexpr|const|static|inline)\s+'
    r'[\w:]+(?:\s*<[^;{]+>)?\s+(k[A-Z_]\w*)\s*[=\[]')
_NOT_NAMES = {"return", "else", "if", "for", "while", "switch", "sizeof", "static_cast",
              "reinterpret_cast", "const_cast", "do", "case",
              # std-library names a helper might shadow — never rewrite these
              "round", "floor", "ceil", "abs", "fabs", "min", "max", "swap", "move",
              "sort", "clamp", "begin", "end", "size", "data", "get", "sqrt", "pow",
              "unit", "where", "cat"}

PUBLIC_HEADERS = [
    "poncelet/version.hpp", "poncelet/types.hpp", "poncelet/units.hpp",
    "poncelet/projectile.hpp", "poncelet/drag.hpp", "poncelet/environment.hpp",
    "poncelet/material.hpp", "poncelet/catalog.hpp", "poncelet/world.hpp",
    "poncelet/terminal.hpp", "poncelet/warhead.hpp", "poncelet/explosion.hpp",
    "poncelet/sim.hpp", "poncelet/fragmentation.hpp", "poncelet/shapedcharge.hpp",
    "poncelet/guidance.hpp", "poncelet/atmosphere.hpp",
]

# src/integrate.cpp, src/drag_tables.cpp, src/ball_profiles.cpp, src/sim.cpp
# and src/guidance.cpp are built by CMake with /fp:precise /fp:except- (MSVC)
# or -ffp-contract=off -fno-fast-math (GCC/Clang) — see CMakeLists.txt's
# determinism-prep comment for why each needs it (interp_curve()'s
# `a + (b-a)*f`, sim.cpp's Vec3::cross()/Quat::operator*, and guidance.cpp's
# PN/APN dot()/cross() chain — all textbook `a*b-c*d` FMA bait; a real
# cross-platform CI run caught the sim.cpp case diverging on ARM64 macOS). A
# single-TU build loses that per-file flag; re-assert what a pragma can. A
# whole-TU fast-math build still can't promise cross-platform BitExact from
# the single header — use the multi-file build for that (see dist/README.md).
_FP_PRE = ("\n#if defined(_MSC_VER)\n#  pragma float_control(precise, on, push)\n#endif\n"
           "#if defined(__clang__)\n#  pragma clang fp contract(off)\n#endif\n"
           "#pragma STDC FP_CONTRACT OFF\n")
_FP_POST = "\n#if defined(_MSC_VER)\n#  pragma float_control(pop)\n#endif\n"


def resolve(name: str, from_dir: Path):
    for base in (from_dir, INCLUDE_DIR, SRC_DIR):
        cand = (base / name).resolve()
        if cand.is_file():
            return cand
    return None


def match_brace(text: str, open_idx: int) -> int:
    """Index of the `}` matching the `{` at text[open_idx]; comment/string aware."""
    depth, i, n = 0, open_idx, len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "//":
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if two == "/*":
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
            continue
        c = text[i]
        if c in ('"', "'"):
            i += 1
            while i < n and text[i] != c:
                i += 2 if text[i] == "\\" else 1
            i += 1
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return n


def minify(text: str) -> str:
    """Strip // and /* */ comments (string/char-literal aware, same scan as
    match_brace) and blank lines. Conservative on purpose: no identifier
    renaming, no line-joining, no whitespace collapsing within a line — just
    the comments and the lines left empty once they're gone."""
    out = []
    i, n = 0, len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "//":
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if two == "/*":
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
            continue
        c = text[i]
        if c in ('"', "'"):
            start = i
            i += 1
            while i < n and text[i] != c:
                i += 2 if text[i] == "\\" else 1
            i += 1
            out.append(text[start:i])
            continue
        out.append(c)
        i += 1
    lines = [ln.strip() for ln in "".join(out).splitlines()]
    lines = [ln for ln in lines if ln]
    return "\n".join(lines) + "\n"


def isolate_anon(text: str, stem: str) -> str:
    """Each .cpp has a `namespace { ... }` of file-local helpers, and the names
    repeat across files (kPi, an Rng, small math). Concatenated into one TU those
    anonymous namespaces merge and the names clash. Rename every name declared at
    the top level of this file's anonymous namespace(s) to `<name>_<stem>` so the
    merged anonymous namespace has no duplicates — no wrappers, the real
    `namespace pon { ... }` blocks are untouched."""
    names: set[str] = set()
    for m in _ANON_NS.finditer(text):
        open_idx = text.index("{", m.start())
        close_idx = match_brace(text, open_idx)
        depth = 0
        for line in text[open_idx + 1:close_idx].splitlines():
            if depth == 0:
                for dm in _DECL_TYPE.finditer(line):
                    names.add(dm.group(1))
                for rx in (_DECL_FUNC, _DECL_KCONST):
                    nm = rx.match(line)
                    if nm and nm.group(1) not in _NOT_NAMES:
                        names.add(nm.group(1))
            depth += line.count("{") - line.count("}")
    for name in sorted(names, key=len, reverse=True):
        text = re.sub(rf'\b{re.escape(name)}\b', f'{name}_{stem}', text)
    return text


def expand(path: Path, seen: set, sysinc: dict, divert_sys: bool) -> str:
    if path in seen:
        return ""
    seen.add(path)
    parts = [f"\n// ===== {path.relative_to(ROOT).as_posix()} =====\n"]
    for line in path.read_text(encoding="utf-8").splitlines():
        if _PRAGMA_ONCE.match(line):
            continue
        m = _QUOTE_INCLUDE.match(line)
        if m:
            target = resolve(m.group(1), path.parent)
            if target is not None:
                parts.append(expand(target, seen, sysinc, divert_sys))
                continue
        s = _SYS_INCLUDE.match(line)
        if s and divert_sys:
            sysinc.setdefault(s.group(1), None)
            continue
        parts.append(line + "\n")
    return "".join(parts)


def build() -> str:
    seen: set = set()
    sysinc: dict = {}
    decl = [expand((INCLUDE_DIR / h).resolve(), seen, sysinc, False) for h in PUBLIC_HEADERS]
    decl.append(expand((INCLUDE_DIR / "poncelet" / "poncelet.h").resolve(), seen, sysinc, False))
    for hpp in sorted(SRC_DIR.glob("*.hpp")):
        decl.append(expand(hpp.resolve(), seen, sysinc, False))

    impl = []
    for cpp in sorted(SRC_DIR.glob("*.cpp")):
        body = isolate_anon(expand(cpp.resolve(), seen, sysinc, True), cpp.stem)
        if cpp.name in ("integrate.cpp", "drag_tables.cpp", "ball_profiles.cpp", "sim.cpp",
                        "guidance.cpp"):
            body = _FP_PRE + body + _FP_POST
        impl.append(body)

    out = [
        "// poncelet — single-header amalgamation. GENERATED by tools/amalgamate.py.\n",
        "// SPDX-License-Identifier: MIT\n//\n",
        "// In exactly one .cpp:  #define PONCELET_SINGLE_IMPLEMENTATION before the\n",
        "// #include. Every other translation unit just #includes this file.\n",
        "\n#ifndef PONCELET_SINGLE_HPP_INCLUDED\n#define PONCELET_SINGLE_HPP_INCLUDED\n",
    ]
    out += decl
    out.append("\n#endif // PONCELET_SINGLE_HPP_INCLUDED\n")
    out.append("\n#ifdef PONCELET_SINGLE_IMPLEMENTATION\n"
               "#ifndef PONCELET_SINGLE_IMPL_INCLUDED\n#define PONCELET_SINGLE_IMPL_INCLUDED\n")
    out.append("\n// standard-library includes hoisted from the .cpp bodies\n")
    out += [f"#include <{name}>\n" for name in sysinc]
    out += impl
    out.append("\n#endif // PONCELET_SINGLE_IMPL_INCLUDED\n#endif // PONCELET_SINGLE_IMPLEMENTATION\n")
    return "".join(out)


def main() -> int:
    text = build()
    if "--check" in sys.argv[1:]:
        cur = OUT.read_text(encoding="utf-8") if OUT.is_file() else ""
        if cur != text:
            sys.stderr.write(
                "dist/poncelet_single.hpp is stale — run: python3 tools/amalgamate.py\n")
            return 1
        print("dist/poncelet_single.hpp is fresh")
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    print("wrote", OUT.relative_to(ROOT), f"({len(text)} bytes)")
    if "--minify" in sys.argv[1:]:
        minified = minify(text)
        MIN_OUT.write_text(minified, encoding="utf-8")
        print("wrote", MIN_OUT.relative_to(ROOT),
              f"({len(minified)} bytes, {100 * len(minified) // len(text)}% of unminified)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
