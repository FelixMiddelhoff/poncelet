#!/usr/bin/env python3
# poncelet — Doxygen input filter.
# SPDX-License-Identifier: MIT
#
# The public headers are commented with plain `//`, which Doxygen does not treat
# as documentation. This filter rewrites, on the fly (the source never changes):
#   * a full-line `//` comment      -> `///`   (attaches to the next entity)
#   * a trailing `code;   // note`   -> `code;   ///< note`
# so `doxygen docs/Doxyfile` turns the existing prose into a browsable reference
# without more writing. Doxygen invokes it per file; it prints to stdout.

import re
import sys

# A `//` that opens a comment: not already `///` / `//!`, not part of `://`.
_FULL = re.compile(r'^(\s*)//(?!/|!)( ?)(.*)$')
_TRAIL = re.compile(r'^(?P<code>.*\S)\s+//(?!/|!)( ?)(?P<text>.*)$')
_SKIP = re.compile(r'SPDX-License-Identifier|^\s*//\s*={3,}')


def convert(text: str) -> str:
    out = []
    seen_code = False
    for line in text.splitlines():
        if not seen_code:
            # leave the file's license/banner block as plain comments
            if line.strip() and not line.lstrip().startswith("//"):
                seen_code = True
            else:
                out.append(line)
                continue
        if _SKIP.search(line):
            out.append(line)
            continue
        m = _FULL.match(line)
        if m:
            out.append(f"{m.group(1)}///{(' ' + m.group(3)).rstrip()}")
            continue
        t = _TRAIL.match(line)
        if t and "://" not in line and '"' not in t.group("code"):
            out.append(f'{t.group("code")}  ///< {t.group("text")}'.rstrip())
            continue
        out.append(line)
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    src = sys.argv[1] if len(sys.argv) > 1 else "-"
    data = sys.stdin.read() if src == "-" else open(src, encoding="utf-8").read()
    sys.stdout.write(convert(data))
