# poncelet — vendors a built poncelet into the Unreal plugin's ThirdParty/ dir.
# SPDX-License-Identifier: MIT
#
# Builds poncelet (Debug + Release static lib — both, because MSVC's std::
# container ABI depends on the build config and the plugin needs to match
# whichever UE config it's compiled into) and copies the headers + both .libs
# into bindings/unreal/Poncelet/ThirdParty/poncelet/, which Poncelet.Build.cs
# reads from. Re-run after pulling a poncelet update.
#
# Usage (from anywhere):  pwsh bindings/unreal/setup_thirdparty.ps1
param(
    [string]$CMakePath = "cmake"
)

$ErrorActionPreference = "Stop"
$RepoRoot   = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$BuildDir   = Join-Path $RepoRoot "build"
$ThirdParty = Join-Path $PSScriptRoot "Poncelet\ThirdParty\poncelet"

Write-Host "poncelet repo: $RepoRoot"
Write-Host "vendoring into: $ThirdParty"

& $CMakePath -S $RepoRoot -B $BuildDir -DPONCELET_BUILD_TESTS=OFF -DPONCELET_BUILD_BENCH=OFF -DPONCELET_BUILD_EXAMPLES=OFF
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

foreach ($Config in @("Debug", "Release")) {
    & $CMakePath --build $BuildDir --config $Config --target poncelet
    if ($LASTEXITCODE -ne 0) { throw "cmake --build --config $Config failed" }
}

# Remove any previously-vendored headers first: Copy-Item -Recurse onto an
# ALREADY-EXISTING destination directory nests the source folder inside it
# (.../include/poncelet/poncelet/...) instead of overwriting its contents —
# a real PowerShell gotcha this script used to get wrong silently (no error,
# it just left the old headers in place and created the stray nested copy).
$ThirdPartyPoncelet = Join-Path $ThirdParty "include\poncelet"
if (Test-Path $ThirdPartyPoncelet) { Remove-Item -Recurse -Force $ThirdPartyPoncelet }
New-Item -ItemType Directory -Force -Path (Join-Path $ThirdParty "include") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $ThirdParty "lib\Debug") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $ThirdParty "lib\Release") | Out-Null

Copy-Item -Path (Join-Path $RepoRoot "include\poncelet") -Destination (Join-Path $ThirdParty "include\poncelet") -Recurse -Force
Copy-Item -Path (Join-Path $BuildDir "Debug\poncelet.lib")   -Destination (Join-Path $ThirdParty "lib\Debug\poncelet.lib") -Force
Copy-Item -Path (Join-Path $BuildDir "Release\poncelet.lib") -Destination (Join-Path $ThirdParty "lib\Release\poncelet.lib") -Force

Write-Host "Done. ThirdParty/poncelet/ is ready for the Poncelet plugin to build against."
