# poncelet — WASM browser demo

`index.html` + `wasm_shim.cpp`: `pon::preview_arc` (no `Sim`, no `World` — the
same Sim-less predicted-flight-path call the native cookbook recipe 6 /
aim-UI uses) compiled to WebAssembly and driven by two sliders (muzzle speed,
launch angle), drawing the trajectory on a `<canvas>`.

## Build

Needs an [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)
activated in the current shell:

```
git clone https://github.com/emscripten-core/emsdk.git
emsdk/emsdk.bat install latest      # emsdk/emsdk.sh on macOS/Linux
emsdk/emsdk.bat activate latest
. emsdk/emsdk_env.ps1                # PowerShell; `source emsdk_env.sh` elsewhere
```

then:

```powershell
pwsh docs/examples/web/build.ps1
```

This cross-compiles poncelet itself to WASM (a separate `build-wasm/` CMake
tree — the library needs no changes, it's already dependency-free C++17) and
links `wasm_shim.cpp` against it, producing `poncelet_demo.js` /
`poncelet_demo.wasm` next to `index.html`. Neither is committed — a compiled
binary, same reasoning as `build/*.lib` never being committed (the *source*,
`dist/poncelet_single.hpp`, is the one build product this repo does commit).

## Run

Most browsers refuse to `fetch()` a `.wasm` file from a `file://` URL — serve
this directory instead:

```
python3 -m http.server 8000 --directory docs/examples/web
```

then open `http://localhost:8000`.

## Why a hand-written shim instead of embind

The C ABI (`<poncelet/poncelet.h>`) is already flat / POD, so Emscripten's
plain `ccall`/`cwrap` (numbers, strings) is enough — no embind, no generated
bindings glue. `wasm_shim.cpp` adds only what a browser page can't do
itself: `pon_projectile_desc`/`pon_vec3` struct construction stays on the C++
side, and — the one thing the C ABI's `pon_preview_arc` can't do — an
optional per-call muzzle-speed override, which is what makes the demo's speed
slider actually reach the simulation (found by testing the running page: the
first version silently ignored it, since `pon_preview_arc` always uses the
type's fixed muzzle speed. `pon::preview_arc`'s `LaunchParams::speed` doesn't
have that limit, so the shim calls that directly).

## Verified

Built and run in a real browser (this repo's own tooling) — both sliders
correctly change the rendered trajectory (100 m/s / 3° lands at ~121 m;
360 m/s / 3° lands at ~671 m; 100 m/s / 40.5° is still climbing at the demo's
8 s cutoff). `BitExact` determinism should cross-compile cleanly too (pure
integer arithmetic, no platform libm) but hasn't specifically been checked
under WASM.
