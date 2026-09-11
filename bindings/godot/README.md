# poncelet — Godot 4 GDExtension sample

A `PonceletSim` class (extends `RefCounted`) exposing poncelet to GDScript /
C#. It wraps the **C++** API directly (GDExtension is C++), so you get the
catalog, `preview_arc` and `describe` for free.

> Sample, not a finished addon: it covers exterior flight + the aim arc. Terminal
> ballistics, warheads, guidance and the `World` geometry callback are in the C++
> API — extend `poncelet_sim.*` the same way to surface them.

## Build

```bash
cd bindings/godot
git clone https://github.com/godotengine/godot-cpp             # a recent 4.x checkout
cmake -S . -B build -DGODOTCPP_API_VERSION=4.7                  # match your Godot
cmake --build build --config Release --target poncelet_godot
cp build/Release/libponcelet_godot.*  demo/                    # .dll / .so / .dylib
```

`godot-cpp` (its `master`) bundles the GDExtension API JSON for 4.3–4.7, so
`-DGODOTCPP_API_VERSION=<your 4.x>` is all it needs — no separate
`--dump-extension-api`. `CMakeLists.txt` pulls the poncelet core from two
directories up (`add_subdirectory(../..)`) — no install needed. The first build
compiles godot-cpp itself and takes a while.

## Run the demo

Open `bindings/godot/demo/` in Godot 4. Add a `Node3D` named `Main`, attach
`main.gd`, add a `Camera3D` looking down +X from a few metres back, press play.
Every ~1.2 s it fires a `762x51_175gr_smk` down +X, draws the predicted arc
(yellow) and the live tracer segments (red), and prints `describe()` each shot.

## API

```gdscript
var sim := PonceletSim.new()
sim.set_atmosphere(0.0)                       # ISA, metres altitude

var t := sim.register_catalog("762x51_175gr_smk")
# or: sim.register_bullet("my_round", 0.0113, 0.00782, 0.243, 800.0)

var id := sim.fire(t, Vector3(0, 1.7, 0), Vector3(1, 0, 0))
sim.step(delta)                               # once per _physics_process
var p := sim.get_position(id)
var arc := sim.preview_arc(t, muzzle, aim, 0.05, 6.0, 0.0)   # PackedVector3Array
print(sim.describe(id))
```

Frame convention: poncelet is SI metres, Y up — Godot's default — so `Vector3`s
pass straight through. poncelet owns no geometry; do hit tests with Godot
physics against `get_position()`.
