# poncelet — quickstart

Never built poncelet before? Copy this whole `quickstart/` folder out
anywhere (it deliberately doesn't assume the rest of the poncelet repo is
around) and:

```
cmake -S . -B build
cmake --build build
./build/quickstart          # build/Debug/quickstart.exe on a multi-config generator (Visual Studio)
```

`CMakeLists.txt` pulls poncelet in via `FetchContent` — the same one-call
integration [guide.md § Building against poncelet](../../guide.md#building-against-poncelet)
covers for a real project. `main.cpp` is the smallest complete program: one
catalog round, one `preview_arc` call, print where it landed.

poncelet has no public remote yet, so today this needs a local checkout —
set the environment variable before configuring:

```
export PONCELET_QUICKSTART_SOURCE_DIR=/path/to/poncelet   # or $env:... on PowerShell
cmake -S . -B build
```

Once poncelet has a real remote + tagged release, `CMakeLists.txt`'s
`GIT_REPOSITORY`/`GIT_TAG` placeholder gets filled in and this env var won't
be needed.

From here: [docs/cookbook.md](../../cookbook.md) for more complete recipes
(shotgun, grenade arc, terminal ballistics, netcode), or
[docs/guide.md](../../guide.md) for the full API.
