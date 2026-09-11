# poncelet — builds the WASM browser demo (index.html + wasm_shim.c) into
# poncelet_demo.js / poncelet_demo.wasm next to this script.
# SPDX-License-Identifier: MIT
#
# Needs an activated Emscripten SDK (https://emscripten.org/docs/getting_started/downloads.html):
#   git clone https://github.com/emscripten-core/emsdk.git
#   emsdk\emsdk.bat install latest
#   emsdk\emsdk.bat activate latest
#   . emsdk\emsdk_env.ps1          # PowerShell; emsdk_env.sh on macOS/Linux
#
# Then, from anywhere:
#   pwsh docs/examples/web/build.ps1
#
# Output (poncelet_demo.js/.wasm) is a build product, not committed — same
# reasoning as dist/poncelet_single.hpp's *source* being committed but
# build/*.lib never is. Open index.html in a browser after building (most
# browsers need it served over http:// rather than file:// for a .wasm fetch
# — `python3 -m http.server` from this directory works).
$ErrorActionPreference = "Stop"
$Here = $PSScriptRoot
$RepoRoot = Resolve-Path (Join-Path $Here "..\..\..")

if (-not (Get-Command emcc -ErrorAction SilentlyContinue)) {
    throw "emcc not found on PATH — activate the Emscripten SDK first (see this script's header comment)."
}

# poncelet itself, cross-compiled to WASM (no tests/bench/examples/docs — just
# the library). A separate build dir from the native one.
$WasmBuildDir = Join-Path $RepoRoot "build-wasm"
& emcmake cmake -S $RepoRoot -B $WasmBuildDir -G Ninja -DCMAKE_BUILD_TYPE=Release `
    -DPONCELET_BUILD_TESTS=OFF -DPONCELET_BUILD_BENCH=OFF -DPONCELET_BUILD_EXAMPLES=OFF -DPONCELET_BUILD_DOCS=OFF
if ($LASTEXITCODE -ne 0) { throw "emcmake configure failed" }
& cmake --build $WasmBuildDir --target poncelet
if ($LASTEXITCODE -ne 0) { throw "poncelet WASM build failed" }

$PonceletLib = Join-Path $WasmBuildDir "libponcelet.a"

& em++ (Join-Path $Here "wasm_shim.cpp") $PonceletLib `
    -I (Join-Path $RepoRoot "include") `
    -O2 `
    -s MODULARIZE=1 -s EXPORT_NAME=createPonceletModule `
    -s ALLOW_MEMORY_GROWTH=1 `
    -s "EXPORTED_FUNCTIONS=['_wasm_init','_wasm_register_bullet','_wasm_preview_arc','_wasm_last_error','_malloc','_free']" `
    -s "EXPORTED_RUNTIME_METHODS=['ccall','getValue']" `
    -o (Join-Path $Here "poncelet_demo.js")
if ($LASTEXITCODE -ne 0) { throw "em++ build failed" }

Write-Host "Done: $Here\poncelet_demo.js / poncelet_demo.wasm"
