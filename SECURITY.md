# Security policy

## Reporting a vulnerability

Please **do not** open a public issue for a security vulnerability. Use
GitHub's private reporting instead:

1. Go to the [Security tab](https://github.com/FelixMiddelhoff/poncelet/security).
2. Click **Report a vulnerability**.
3. Describe the issue and, if possible, how to reproduce it.

This opens a private advisory visible only to the maintainer and you, so
the issue can be fixed before it's public.

## Supported versions

poncelet is pre-1.x-stable and currently tracks a single moving line: the
latest commit on `main` / the most recently tagged `v1.0.0` release.
Security fixes land there; there is no older release branch receiving
backports at this time.

## Scope

poncelet is a physics/simulation library with no network I/O, no file
parsing beyond CSV data tables you control (`data/*.csv`), and no
dependencies in the core library. Realistic concerns are things like
out-of-bounds access, integer overflow, or undefined behavior reachable
from public API inputs (a `ProjectileType`/`LaunchParams`/`Environment`
a caller controls) — not, for example, supply-chain issues in a
dependency, since the core has none. The engine bindings under
`bindings/` (Godot/Unity/Unreal/raylib) pull in external SDKs for their own
build only and are sample code, not part of the library surface itself.
