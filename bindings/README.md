# poncelet — engine bindings

Thin sample bindings that drop poncelet into a game engine. Each is a *starting
point* — enough to fire shots, step them, read their state and draw a predicted
arc — not an exhaustive wrapper. They live here rather than in the main build
because each needs an external SDK; they are **not** built by poncelet's CI.

| Binding | Path | Talks to | Needs |
|---|---|---|---|
| Godot 4 GDExtension | [`godot/`](godot/) | the C++ API (`<poncelet/poncelet.hpp>`) | `godot-cpp` (matching your Godot 4.x), CMake |
| Unity native plugin | [`unity/`](unity/) | the C ABI (`<poncelet/poncelet.h>`) | a `-DPONCELET_SHARED=ON` build of poncelet |
| Unreal Engine plugin | [`unreal/`](unreal/) | the C++ API (`<poncelet/poncelet.hpp>`) | a static poncelet build vendored via `unreal/setup_thirdparty.ps1`; UE 5.x |
| raylib sample | [`raylib/`](raylib/) | the C++ API (`<poncelet/poncelet.hpp>`) | CMake (`FetchContent`-pulls raylib itself) |

Godot and Unity treat poncelet's world frame as the engine's directly: **SI
metres, Y up, right-handed** — both engines' default metre-scale world frame,
so positions and directions pass straight through. Scale your own units in/out
if your game does not use metres. **Unreal is the odd one out** — centimetres,
Z-up — so the Unreal plugin does a real coordinate conversion rather than a
straight pass-through; see `unreal/README.md` for the one real gotcha that
comes with it (a chirality-dependent sign flip on spin/curve effects, not
catchable by an automated test). raylib isn't an engine and has no world-frame
opinion of its own — the sample just projects poncelet's metres straight onto
2D pixels itself (see `raylib/README.md`).

None of these bindings pull in geometry — poncelet never owns collision (the
`World` callback in the C++ API, not yet in the C ABI). Hit detection against
your engine's physics is the integrator's job; feed poncelet the muzzle
transform and read back positions.
