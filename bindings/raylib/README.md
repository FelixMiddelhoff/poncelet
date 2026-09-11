# poncelet — raylib sample

A single-file raylib program: a keyboard-steerable aim-preview arc, drawn
with raylib's 2D primitives. It calls `pon::preview_arc` directly — no
`Sim`/`World` needed for a pure trajectory prediction — so this is the
smallest possible "poncelet in a real game loop" sample: no engine plugin
system, no editor, just `main()`.

> Sample, not a finished demo: one shot, steered live, no catalog UI. Firing
> live shots (`Sim::spawn`/`step`) and reading them back works the same way
> — see `docs/examples/basic_shot.cpp` — this just shows the arc-preview half.

## Build

```bash
cd bindings/raylib
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/poncelet_raylib          # Release/poncelet_raylib.exe on a multi-config generator
```

`CMakeLists.txt` fetches raylib itself (`FetchContent`, pinned to `5.5` — no
manual clone step, unlike `bindings/godot`'s `godot-cpp`) and pulls the
poncelet core from two directories up (`add_subdirectory(../..)`), same as
every other binding here. The first configure builds raylib from source and
takes a while; every build after that is quick.

## Run it

`Left`/`Right` steers the launch angle, `Up`/`Down` the muzzle speed; the
arc redraws live. The demo round is a lobbed, draggy 40 mm shell (same
parameters as `docs/examples/aim_preview.cpp`) rather than a flat rifle
round — a fast, flat trajectory travels far enough that the interesting part
(the arc) never fits on screen before you'd need to scroll.

## API

```cpp
pon::ProjectileType type;
type.klass = pon::ProjectileClass::Shell;
type.dragModel = pon::DragModel::ConstantCd;
type.dragCoefficient = 0.30;
type.mass_kg = 0.230;
type.refDiameter_m = pon::inches(1.57);
type.muzzleSpeed_mps = 76.0;

pon::LaunchParams lp;
lp.position = {0, 1.6, 0};
lp.direction = {std::cos(angle), std::sin(angle), 0};
lp.speed = speedMps;

std::vector<pon::Vec3> arc;
pon::preview_arc(type, lp, pon::Environment{}, 0.01, 8.0, arc, /*groundY=*/0.0);
// arc[i].x = downrange metres, arc[i].y = height metres
```

Frame convention: poncelet is SI metres, Y up, right-handed — raylib has no
opinion on world units/axes (it just draws whatever pixels you give it), so
this sample picks a simple downrange/height 2D projection itself
(`WorldToScreen` in `main.cpp`) rather than relying on an engine default the
way the Godot/Unity/Unreal bindings do. poncelet owns no geometry or
rendering; draw whatever `preview_arc` or a live shot's position hands back.
