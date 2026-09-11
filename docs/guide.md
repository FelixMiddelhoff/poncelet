# poncelet — usage guide

This is the practical guide to using `poncelet` from your own code. It covers
the concepts you need, how to build against the library, an annotated
walkthrough, and a reference entry with a short example for every public call.

- New to the library? Read *Concepts* then *Walkthrough*.
- Looking up one function? Jump to *API reference* here, or generate the Doxygen
  HTML reference (`cmake --build build --target poncelet_docs` → `docs/api/html`,
  also published to GitHub Pages).
- Ready-made recipes? See [cookbook.md](cookbook.md).
- Binding from another language? See *C API*.

> **Status — v1.2+ (Phase 18 + Phase 19 items 1–6 complete).** Every model
> described here is implemented and validated against published data within
> documented tolerances (see [Validation](#validation)). The public API is
> stable. v1.1 added 6-DOF rigid-body flight; v1.2 added explosive warheads;
> the batch past v1.2 (fragmentation, shaped charges / EFP, guided munitions,
> destruction coupling) is landed but version-untagged on purpose — one release
> tag when the whole Phase 19 set is settled. Two capabilities are
> tracked follow-ups, called out where they are relevant: explicit 4/8-wide
> **SIMD intrinsics** (the portable SoA batch path is in) and extending the
> **`BitExact`** fixed-point core to the guidance law (the integrator itself,
> including the `AdaptiveRKF45` step-size controller, is fixed-point). The
> G1/G7
> drag tables are the full-resolution BRL/McCoy standard curves (JBM
> `mcg1.txt` / `mcg7.txt`); a single-BC standard-projectile model still can't
> track an individual bullet's post-transonic drag rise, so supply a
> Doppler-derived `CustomCurve` for match-grade sub-Mach-1 work.

---

## Contents

1. [Concepts](#concepts)
   - [Fidelity tiers](#fidelity-tiers)
   - [Spinning balls — the `BallProfile` system](#spinning-balls--the-ballprofile-system)
   - [Determinism](#determinism) · [Validation](#validation)
2. [Building against poncelet](#building-against-poncelet)
3. [Walkthrough](#walkthrough)
4. [API reference](#api-reference)
   - [Vectors and math](#vectors-and-math)
   - [Sim](#sim)
   - [ProjectileType](#projectiletype)
   - [The named catalog](#the-named-catalog)
   - [LaunchParams and spawning](#launchparams-and-spawning)
   - [Precision effects](#precision-effects) · [6-DOF flight](#6-dof-flight-precisionflagsixdof)
   - [ProjectileState](#projectilestate)
   - [Environment](#environment)
   - [The World callback](#the-world-callback)
   - [Materials](#materials)
   - [Events](#events)
   - [Explosive warheads](#explosive-warheads)
   - [Version](#version)
5. [C API](#c-api)
6. [Determinism](#determinism) · [Rollback netcode](#rollback-netcode)
7. [Gotchas and FAQ](#gotchas-and-faq)
8. [Cookbook](cookbook.md) — copy-pasteable recipes (sniper dope, shotgun, grenade arc, APFSDS, netcode)

---

## Concepts

### SI units, everywhere

Every number crossing the API boundary is SI: metres, seconds, kilograms,
kelvin, pascals, radians. There is no imperial anywhere and no unit tags to
get wrong. Field names carry the unit as a suffix (`mass_kg`, `muzzleSpeed_mps`,
`refDiameter_m`). Convert at *your* boundary, not ours.

Published ballistics data is almost always imperial, so `<poncelet/units.hpp>`
(pulled in by the umbrella header) has `constexpr` conversion **functions** —
not new types, so the API is unchanged:

```cpp
t.mass_kg       = pon::grains(168);      // gr   -> kg
t.refDiameter_m = pon::inches(0.308);    // in   -> m
lp.speed        = pon::fps(2650);        // ft/s -> m/s
lp.position.y   = pon::feet(6);
Real spreadRad  = pon::moa(1.5);         // minutes of angle -> radians
```

Also `pounds` / `ounces` / `yards` / `miles` / `mph` / `psi` / `inhg` /
`fahrenheit` / `celsius` / `degrees` / `mil_nato`, and `to_fps` / `to_yards` /
`to_grains` / `to_ftlb` / `to_moa` / … as inverses for HUD and logging.

### `Sim` owns the simulation

A `pon::Sim` holds the environment, the registry of projectile *types*, and
the set of live projectile *instances*. You create one (often one per world),
register the projectile types your game uses once at load time, then `spawn`
and `step` during play.

```cpp
pon::Sim sim;                       // default environment (Earth sea-level air)
```

### Type vs. state

- A **`ProjectileType`** is the *description* of a kind of projectile — its
  mass, diameter, drag model, default muzzle speed, terminal parameters. You
  register it once and get back a cheap `TypeId`. Registration is where the
  per-type acceleration lookup tables get precomputed, so do it at load time,
  not per shot.
- A **`ProjectileState`** is one *instance* in flight — position, velocity,
  spin, how far it has travelled, which medium it is in. `spawn` creates one
  and returns a `StateId`.

### You own the geometry — the `World` callback

`poncelet` never sees your meshes, your BVH, or your scene graph. When it needs
to know "does the segment from A to B hit anything", it calls **your**
implementation of the `pon::World` interface. You bring broad-phase and
ray/segment intersection; the library brings the ballistics. A `pon::EmptyWorld`
that hits nothing is provided for free-flight tests.

### Fidelity tiers

Each shot picks how much CPU its hit detection is worth
(`LaunchParams::tier`):

| Tier | What it does | Cost |
|---|---|---|
| `Hitscan` | one straight-line raycast, no drag, no drop | one `World::raycast` |
| `AnalyticDrag` | closed-form linear-drag trajectory sampled to a polyline, then swept | ~µs |
| `Integrated` | fixed-step RK4 / semi-implicit Euler with a deterministic adaptive sub-step count and a swept query every sub-step | tens of µs per projectile-second |

`AnalyticDrag` linearises the drag retardation at the current speed each frame
— exact for linear drag, a few percent off over a full flight, and re-anchored
every frame so it does not drift. The re-linearisation reads whatever drag
model the type uses (`ConstantCd`, a `G1`/`G7` / custom `Cd(Mach)` curve, or a
`BallProfile` `Cd(Reynolds)` curve), so all of them work on this tier. It falls
back to `Integrated` whenever a transverse force is in play — a wind field, an
active Magnus term (a spinning ball), or a non-spherical `BallProfile` shape —
since the closed form can only bend the trajectory along gravity and drag.

A common pattern: `Hitscan` to decide whether a shot is interesting, then
re-simulate the interesting ones with `Integrated` for a kill-cam.

### Spinning balls — the `BallProfile` system

Sports balls are not generic spheres: a golf ball, a seamed baseball and a
panelled football each hit their **drag crisis** (the sudden Cd drop as the
boundary layer goes turbulent) at a different Reynolds number, and each has its
own **Magnus** response to spin — a backspun golf ball climbs, a panelled
football even has a brief *reverse*-Magnus regime at very low spin.

Set `dragModel = BallProfile` and name a profile:

```cpp
pon::ProjectileType ball;
ball.klass        = pon::ProjectileClass::SportsBall;
ball.dragModel    = pon::DragModel::BallProfile;
ball.ballProfile  = "fifa_football";   // fills Ø / mass / typical speed too
ball.spinAxisMode = pon::SpinAxisMode::Fixed;

pon::LaunchParams kick;
kick.position = {0, 0.3, 0};
kick.direction = {1, 0.12, 0};
kick.speed = 30.0;
kick.spin = 70.0;                       // rad/s
kick.spinAxis = {0, 1, 0};              // vertical axis ⇒ it bends sideways
kick.tier = pon::FidelityTier::Integrated;
```

Profiles: `fifa_football`, `american_football`, `rugby_ball`, `baseball`,
`cricket_ball`, `tennis_ball`, `golf_ball`, `basketball`, `volleyball`,
`field_hockey_ball`, `ice_hockey_puck`, `rock`. An unknown id falls back to a
plain rough sphere. Spheres and the puck default to a `Fixed` world-frame spin
axis; a `ProlateSpheroid` (`american_football` / `rugby_ball`) keeps
`AlongVelocity` for a clean low-drag spiral, or takes `Fixed` + a square axis
for a high-drag end-over-end tumble. The `Cd(Reynolds)` curve scales with the
firing site's air viscosity — call `setAtmosphere()` for a real one.

The profile table is `data/ball_profiles.csv` (curve shapes from Mehta & Pallis,
Kensrud, Goff & Carré — cited in the file), baked to a header at build time;
`docs/examples/curveball.cpp` is a runnable demo.

### Determinism

The default (`PlatformStable`) gives you identical trajectories for the same
platform, compiler and inputs — enough for local replay and demo recording.
`BitExact` runs the integrator on a Q32.32 fixed-point core so the trajectory
is bit-identical across platforms too. See [Determinism](#determinism).

### Validation

The test suite includes a `validation_` group (run it alone with
`ctest -R poncelet_validation`, or `poncelet_tests validation_`) that reproduces
published data within documented tolerances:

| Check | Reference | Tolerance |
|---|---|---|
| .308 168 gr Match retained velocity / ToF @ 1000 m | Sierra / JBM dope | ~340 m/s transonic band (BRL/McCoy G7 table + independent RK4 agree) |
| 5.56 M193 retained velocity @ 300 m | published | ±13 % |
| .308 175 gr SMK dope @ 900 m | Berger / JBM | within ~2 % retained velocity |
| 9×19 124 gr FMJ retained velocity @ 100 m | published | band |
| AnalyticDrag tier vs Integrated over 800 m | self-consistency | 5 % speed, 0.30 m drop |
| Oak penetration ordering (arrow ≪ rifle) | Poncelet dataset (R²≈0.98) | qualitative envelope + bounds |
| Bulk penetration depth — .308-class into mild steel / 35 MPa concrete / dry earth | Forrestal–Frew–Hanchak, Recht–Ipson, dry-sand rate studies | ~7–40 mm / 30–250 mm / 0.1–0.9 m bands |
| Rifle round stopped by water | high-speed footage | 0.2–3 m to sub-100 m/s |
| Arrow vs concrete | — | spalls at the face, no channel |
| Same-platform determinism | — | bit-identical |
| Fuzz: 3000 random projectile × material × angle × tier | — | finite, terminates |
| 6-DOF: spin-stabilised round stays point-first, unspun round tumbles | classic gyroscopic result | peak yaw < 0.35 rad vs > 1.05 rad |
| 6-DOF: right-hand twist drifts right, magnitude | Litz spin drift | sign + within ~2× at 1000 m |
| 6-DOF fuzz: 800 random class × spin × yaw shots | — | finite, unit quaternion, terminates |

Throughput is gated in CI too (`poncelet_perf_gate`) against loose ceilings that
catch a gross regression without flaking on a slow runner.

---

## Building against poncelet

`poncelet` is a plain CMake project with no dependencies. Pick whichever fits
your setup.

### As a subdirectory (submodule / vendored)

```cmake
add_subdirectory(external/poncelet)
target_link_libraries(my_game PRIVATE poncelet::poncelet)
```

Turn off the parts you do not need before `add_subdirectory`:

```cmake
set(PONCELET_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_BENCH    OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
```

### With FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(poncelet
  GIT_REPOSITORY <repo-url>
  GIT_TAG        <tag>)
FetchContent_MakeAvailable(poncelet)
target_link_libraries(my_game PRIVATE poncelet::poncelet)
```

### Standalone build / install + `find_package`

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPONCELET_SHARED=ON
cmake --build build
cmake --install build --prefix _install
```

`PONCELET_SHARED=ON` additionally builds a shared library (`poncelet_shared`)
alongside the static one. The static target is `poncelet` / `poncelet::poncelet`.

An installed poncelet is a normal CMake package:

```cmake
find_package(poncelet 1.2 CONFIG REQUIRED)   # point CMAKE_PREFIX_PATH at the install
target_link_libraries(my_game PRIVATE poncelet::poncelet)
```

All three routes (`add_subdirectory`, `FetchContent`, `find_package`) give the
same `poncelet::poncelet` target, and it carries `cxx_std_17` plus the include
path — you do not need to set the C++ standard yourself.

The install also writes a `poncelet.pc` pkg-config file
(`<prefix>/lib/pkgconfig/poncelet.pc`) for Make / Meson / hand-rolled builds
that don't consume the CMake package:

```
g++ -std=c++17 $(pkg-config --cflags --libs poncelet) my_game.cpp
```

### Package managers (vcpkg / Conan)

Both are **overlay** ports today — usable immediately, not yet submitted to
the central vcpkg/conan-center registries (that needs a real GitHub remote +
their own review process):

```
vcpkg install poncelet --overlay-ports=vcpkg-ports          # vcpkg-ports/README.md
conan create .                                              # from the repo root
```

Both are verified end-to-end, not just written — `vcpkg install` builds
both configs and passes vcpkg's post-build validation; `conan create .`
builds + packages. In each case a `find_package(poncelet)` consumer
(`test_package/` for Conan) was actually configured, compiled, linked, and
run against the installed package.

One real gotcha the Conan pass surfaced (vcpkg's own toolchain didn't hit
this): `target_compile_features(poncelet PUBLIC cxx_std_17)`
does not reliably raise a *consumer's* effective C++ standard under
MSVC + Conan's CMake toolchain when the consumer's profile defaults to an
older `compiler.cppstd` (Conan's own default is 14) — the recipe's
`validate()` calls `check_min_cppstd(self, "17")` so that shows up as a clear
"poncelet requires C++17" at `conan install` time instead of a confusing
error deep in `<poncelet/...>` headers. If you hit the same thing outside
Conan (a build system that doesn't fully propagate `INTERFACE_COMPILE_FEATURES`),
set your consumer's C++ standard to 17 explicitly rather than relying on
propagation from the poncelet target.

### WebAssembly

poncelet is dependency-free C++17, so it cross-compiles to WASM with no
changes — point CMake at Emscripten's toolchain file the usual way
(`emcmake cmake -S . -B build-wasm ...`, or `-DCMAKE_TOOLCHAIN_FILE=.../Emscripten.cmake`
directly). `docs/examples/web/` is a working browser demo (`pon::preview_arc`
driving a `<canvas>` trajectory from two sliders) with its own build script
and README — see `docs/examples/web/README.md`.

### Just the headers + sources

The library is small. You can also drop `include/` on your include path and add
the files in `src/` to your build. If you do, compile `src/integrate.cpp`,
`src/drag_tables.cpp` and `src/ball_profiles.cpp` with contraction off and no
fast-math (`/fp:precise` on MSVC, `-ffp-contract=off -fno-fast-math` on
GCC/Clang) — they are the FP-sensitive translation units (the last two
compile the drag LUT's linear interpolation, `a + (b-a)*f` — exactly the
shape a contraction-permissive compiler folds into an FMA instruction on an
ISA that has one natively, e.g. ARM64; a real cross-platform CI run caught
`macos-latest`'s Apple Silicon runners diverging from x86-64 Windows/Linux
here). `src/generated/data_tables.inc` is committed, so a Python-less
build works; re-run `python3 tools/bake_data.py` after editing a `data/*.csv`
(the CMake build does this automatically when a Python 3 interpreter is found).

### Single-header

`dist/poncelet_single.hpp` is the whole library — every header and `.cpp` (and
the generated tables) folded into one file, committed. Vendor that one file, then
in **exactly one** `.cpp`:

```cpp
#define PONCELET_SINGLE_IMPLEMENTATION
#include "poncelet_single.hpp"
```

every other translation unit just `#include "poncelet_single.hpp"`. Nothing else
to build. It is regenerated by `python3 tools/amalgamate.py` (the ctest
`amalgamation_fresh` gate keeps it in sync; `poncelet_single_header` compiles
it). The single-TU implementation targets `PlatformStable` determinism — for
guaranteed cross-platform `BitExact` from the single header, build that TU with
FP contraction and fast-math off, or use the multi-file build.

`python3 tools/amalgamate.py --minify` additionally writes
`dist/poncelet_single.min.hpp` — comments and blank lines stripped, ~40%
smaller, for the size-conscious vendor. Not committed; regenerate on demand.

### Include

```cpp
#include <poncelet/poncelet.hpp>   // everything (C++)
#include <poncelet/poncelet.h>     // the C ABI
```

Individual headers (`<poncelet/sim.hpp>`, `<poncelet/projectile.hpp>`, …) are
also usable directly.

---

## Walkthrough

A complete program: fire a pistol round across a room, print where it is each
frame, and react to the impact. This is `docs/examples/basic_shot.cpp`.

```cpp
#include <poncelet/poncelet.hpp>
#include <cstdio>

int main() {
    // 1. One Sim per world. The default environment is Earth sea-level air.
    pon::Sim sim;

    // 2. Describe the projectile once, at load time.
    pon::ProjectileType pistol;
    pistol.id                   = "9x19_124gr_fmj";
    pistol.klass                = pon::ProjectileClass::Bullet;
    pistol.dragModel            = pon::DragModel::G1;   // standard flat-base curve
    pistol.ballisticCoefficient = 0.15;                 // published G1 BC (lb/in^2)
    pistol.mass_kg              = 0.00804;
    pistol.refDiameter_m        = 0.00902;
    pistol.muzzleSpeed_mps      = 360.0;
    const pon::TypeId kPistol = sim.registerType(pistol);

    // 3. Fire a shot. Leaving speed unset uses the type's muzzle speed.
    pon::LaunchParams shot;
    shot.position  = {0.0, 1.6, 0.0};   // muzzle at 1.6 m
    shot.direction = {1.0, 0.0, 0.0};   // down +X, need not be normalized
    shot.tier      = pon::FidelityTier::Integrated;
    const pon::StateId bullet = sim.spawn(kPistol, shot);

    // 4. Your geometry. EmptyWorld hits nothing; see docs/examples/custom_world.cpp
    //    for a real one.
    pon::EmptyWorld world;
    pon::VectorEventSink events;

    // 5. Step it each frame.
    for (int frame = 0; frame < 120; ++frame) {
        sim.step(1.0 / 60.0, world, events);

        const pon::ProjectileState& s = sim.state(bullet);
        std::printf("t=%.3f  x=%.2f  y=%.2f  v=%.1f m/s\n",
                    s.timeAlive_s, s.position.x, s.position.y,
                    pon::length(s.velocity));
        if (!s.alive) break;
    }

    // 6. React to whatever happened.
    for (const pon::Event& e : events.events) {
        if (e.type == pon::EventType::Stopped)
            std::printf("stopped at %.2f, %.2f\n", e.point.x, e.point.y);
    }
}
```

Key points:

- `registerType` once, `spawn` per shot.
- `step` advances *every* live projectile in the `Sim` by `dt`. Call it once
  per frame, not once per projectile.
- `sim.state(id)` is a read-only view; it stays valid until the next `step`.
- Events accumulate in your sink. Clear it when you have processed a frame's
  worth (`events.events.clear()`).

---

## API reference

Every entry lists the signature, what it does, and a minimal example.

### Vectors and math

`pon::Vec3` is a POD `{double x, y, z}` with the usual operators. No SIMD type,
no external math library — convert to/from your engine's vector at the boundary.

```cpp
pon::Vec3 a{1, 2, 3}, b{0, 1, 0};
pon::Vec3 c   = a + b * 2.0;         // operators: + - * / and compound
double    d   = pon::dot(a, b);      // 2.0
pon::Vec3 n   = pon::cross(a, b);
double    len = pon::length(a);      // 3.7416...
pon::Vec3 u   = pon::normalized(a);  // zero vector in -> zero vector out
double    l2  = pon::length_sq(a);   // 14.0, no sqrt
```

`pon::Ray` is `{Vec3 origin, Vec3 dir}` with `dir` expected normalized.

### Sim

#### `Sim::Sim(Environment env = {}, SimConfig cfg = {})`

Constructs a simulation. Both arguments are optional. `cfg.determinism ==
BitExact` routes the integrator through the Q32.32 fixed-point core;
`status()` is `Status::Ok`.

```cpp
pon::Sim sim;                                   // defaults

pon::Environment env;
env.gravity = {0, -3.71, 0};                    // Mars
env.airDensity_kgm3 = 0.020;
pon::SimConfig cfg;
cfg.fixedStep_s = 1.0 / 2000.0;                 // finer nominal sub-step
cfg.integrator  = pon::Integrator::SemiImplicit; // default is Integrator::RK4
pon::Sim marsSim(env, cfg);
```

#### `const Environment& Sim::environment() const` / `Environment& Sim::environment()`

Access the environment. The non-const overload lets you change wind, gravity or
air density between steps.

```cpp
sim.environment().wind = [](pon::Vec3, pon::Seconds) {
    return pon::Vec3{-4.0, 0, 0};               // steady 4 m/s headwind
};
```

#### `const SimConfig& Sim::config() const`

Read back the config the Sim was built with.

| Field | Default | Meaning |
|---|---|---|
| `determinism` | `PlatformStable` | see [Determinism](#determinism) |
| `fixedStep_s` | `1/1000` | nominal `Integrated` sub-step |
| `integrator` | `RK4` | `RK4` or `SemiImplicit` for the `Integrated` tier |
| `posTolerance_m` | `0.02` | adaptive sub-step position-error target |
| `maxSubstepDist_m` | `1.0` | anti-tunnelling: cap on travel per sub-step |
| `maxSubsteps` | `64` | hard cap on subdivisions of one nominal step |
| `analyticSampleDist_m` | `2.0` | `AnalyticDrag` polyline sample spacing |
| `rngSeed` | fixed | seed for the per-spawn RNG: `LaunchParams::precisionMrad` muzzle dispersion, and (deferred) the `Free`-spin knuckle wobble — `Free` is treated as `Fixed` for now |
| `batchIntegrator` | `false` | opt-in SoA batch path for the `Integrated` tier — see [Batch integrator and trajectory cache](#batch-integrator-and-trajectory-cache) |
| `batchMinGroup` | `8` | a batch-eligible group smaller than this is advanced per-shot |
| `trajectoryCacheFrames` | `0` | per-frame flight-path samples kept per live shot (`0` = cache off) |
| `traceSink` | null | optional per-shot diagnostic callback — see [Diagnostic trace](#diagnostic-trace) |

```cpp
const double h = sim.config().fixedStep_s;
```

#### `Status Sim::status() const`

`Status::Ok` for every supported configuration, including `BitExact`.
Check it once after construction.

```cpp
if (sim.status() != pon::Status::Ok)
    std::puts("requested determinism mode not available yet");
```

#### `TypeId Sim::registerType(ProjectileType type)`

Registers a projectile description and returns its `TypeId`. Missing fields are
filled from class defaults (see [ProjectileType](#projectiletype)). Returns
`pon::kInvalidType` if the description cannot be made usable (e.g. a zero mass
that no default covers). Do this at load time — it precomputes per-type tables.

```cpp
pon::ProjectileType arrow;
arrow.id    = "longbow_bodkin_60g";
arrow.klass = pon::ProjectileClass::Arrow;      // everything else defaulted
const pon::TypeId kArrow = sim.registerType(arrow);
if (kArrow == pon::kInvalidType) { /* bad description */ }
```

#### `const ProjectileType& Sim::type(TypeId id) const`

Returns the *resolved* type (after defaults were applied). Useful to read back
what a class-defaulted field became. An out-of-range id returns a default-
constructed reference.

```cpp
const pon::ProjectileType& t = sim.type(kArrow);
std::printf("arrow mass resolved to %.3f kg\n", t.mass_kg);
```

#### `StateId Sim::spawn(TypeId type, const LaunchParams& launch)`

Creates one live projectile. Returns a `StateId`, or `pon::kInvalidState` if
the type id is bad or the launch direction is zero-length. Reuses freed slots,
so ids are recycled after `despawn` / natural death.

```cpp
pon::LaunchParams lp;
lp.position  = muzzleWorldPos;
lp.direction = aimDir;                          // not required to be unit
lp.speed     = 410.0;                           // override the catalog default
const pon::StateId h = sim.spawn(kArrow, lp);
```

#### `StateId Sim::fire(const ProjectileType& type, Vec3 muzzle, Vec3 aimDir, FidelityTier tier = Integrated)`

One-call opener for the common case. Registers `type` the first time its `id`
string is seen on this `Sim` (the `id -> TypeId` map is cached — a later `fire()`
with the same id reuses the registration), then spawns one shot from `muzzle`
toward `aimDir` at the type / catalog default speed and spin. Returns
`pon::kInvalidState` on a bad description or a zero `aimDir`; `lastError()`
explains.

```cpp
const pon::StateId h = sim.fire(arrow, muzzleWorldPos, aimDir);
```

`type.id` must be non-empty (as `registerType()` requires). The cache is keyed
by `id` alone — give the type a fresh id when its fields change, and drop to
`registerType()` + `spawn()` for per-shot control (speed, spin, precision flags,
`precisionMrad` dispersion, starting medium).

#### `void Sim::step(Seconds dt, const World& world, EventSink& sink)`

Advances every live projectile by `dt` seconds, each through its
`LaunchParams::tier`. The `Integrated` tier sub-steps at `config().fixedStep_s`
(subdivided further where the adaptive controller needs it), does a swept
`world.raycast` per sub-step, detects medium changes via `world.mediumAt`, and
emits events into `sink`. Call once per frame.

```cpp
pon::VectorEventSink sink;
sim.step(frameDt, myWorld, sink);
for (const pon::Event& e : sink.events) handle(e);
sink.events.clear();
```

#### `const ProjectileState& Sim::state(StateId id) const`

Read-only view of one projectile. Valid until the next `step`. An out-of-range
id returns a default-constructed reference (`alive == true`, everything zero) —
prefer to hold only ids you know are live.

```cpp
const pon::ProjectileState& s = sim.state(h);
if (s.alive) drawTracer(s.position, s.velocity);
```

#### `std::size_t Sim::liveCount() const`

Number of projectiles currently alive.

```cpp
std::printf("%zu rounds in the air\n", sim.liveCount());
```

#### `void Sim::despawn(StateId id)`

Removes a projectile immediately, no event emitted. Safe to call with a stale
id.

```cpp
sim.despawn(h);   // player left the area, stop tracking this shot
```

#### `void Sim::liveIds(std::vector<StateId>& out) const` / `template <class F> void Sim::forEachLive(F&& f) const`

Ask the Sim which shots are alive without tracking every `spawn()` return
value yourself. `liveIds()` clears `out` and fills it with the id of every
live projectile, in slot order; `forEachLive(f)` calls `f(id, state)` for the
same set without the intermediate vector or a `state()` lookup per id.

```cpp
std::vector<StateId> live;
sim.liveIds(live);
for (StateId id : live) hud.track(id, sim.state(id));

sim.forEachLive([&](StateId id, const ProjectileState& s) {
    hud.track(id, s);
});
```

#### `std::vector<std::byte> Sim::snapshot() const` / `bool Sim::restore(const std::byte* data, std::size_t size)`

Save and roll back to a frame — the missing half of `stateHash()` (which
*detects* a desync but cannot undo one). `snapshot()` serializes everything the
next `step()` reads: every live projectile's full dynamic state, the RNG
cursor, and the trajectory cache ring; `restore()` puts a Sim back into that
exact state. See [Rollback netcode](#rollback-netcode) below and
[cookbook.md § 7](cookbook.md#7-netcode).

```cpp
const std::vector<std::byte> snap = sim.snapshot();
// ... step the sim forward, speculatively, off a predicted input ...
if (mispredicted) sim.restore(snap.data(), snap.size());   // roll back and re-simulate
```

`restore()` requires a Sim with the same types registered in the same order
and the same `SimConfig::trajectoryCacheFrames` as the Sim the snapshot came
from; it returns `false` (leaving the Sim untouched) on a truncated buffer or
a version / type-registry / trajectory-cache mismatch. Does **not** serialize
the registered types or drag LUTs — those are static content, not per-frame
state.

#### Batch integrator and trajectory cache

Two opt-in levers on `SimConfig` for scenes with many shots in the air. Neither
changes the physics model; both are off by default.

**`batchIntegrator`** — when set, `step()` gathers the live `Integrated`-tier
shots that share a projectile type and are on the plain fast path (still air, no
wind field, no Magnus / orientation-shaped drag, no `PrecisionFlag`s) and
advances each such group in lockstep as structure-of-arrays: the force model is
built once for the group instead of once per shot, and the adaptive sub-step
count is the group maximum. A lane that impacts, expires or crosses a medium
boundary drops out and finishes on the ordinary per-shot path. Results can
differ from the per-shot path only where a slow lane is carried at a finer step
than it would pick alone — strictly *more* accurate, still inside
`posTolerance_m`, and still `PlatformStable`-deterministic. Groups below
`batchMinGroup` are left on the per-shot path. Explicit 4/8-wide SIMD intrinsics
are a tracked follow-up (the design doc marks them optional); this is the
portable SoA restructuring that sets them up.

**`trajectoryCacheFrames`** — keep the last *N* per-frame samples
(`position`, `velocity`, `time_s`) of every live shot's flight path. A shot is
then a polyline you can re-query without re-integrating it:

```cpp
pon::SimConfig cfg;
cfg.trajectoryCacheFrames = 512;      // ~8.5 s at 60 Hz
pon::Sim sim({}, cfg);
// ... spawn `shot`, step the sim ...

pon::TrajectorySample buf[512];
const std::size_t n = sim.trajectory(shot, buf, 512);   // oldest → newest

pon::Vec3 p, v;
if (sim.sampleTrajectory(shot, 0.35, p, v))              // pose at flight-time 0.35 s
    drawBulletCamFrame(p, v);                            // interpolated, clamped at the ends
```

`trajectory()` copies the ring out oldest-first and returns the count;
`trajectorySize()` is just that count; `sampleTrajectory()` linearly
interpolates `position` and `velocity` between the two bracketing samples and
clamps before the first / after the last (returns `false` with fewer than two
samples). Use it for a slow-motion bullet-cam, or for a client that joined the
session late and needs to reconstruct where a shot has been. See
`docs/examples/bullet_cam.cpp`.

#### Diagnostic trace

`SimConfig::traceSink` is an optional `std::function<void(const pon::TraceEvent&)>`.
Null by default and zero-cost when null; when set, `step()` reports what each
live shot is doing, so "why did that shot behave like that" is a log line, not a
debugger session. The callback is **read-only** — a trace never changes the
simulation, so a traced run folds to the same `stateHash()` as an untraced one.

```cpp
pon::SimConfig cfg;
cfg.traceSink = [](const pon::TraceEvent& e) {
    if (e.kind == pon::TraceKind::TransonicEnter)
        std::printf("shot %u went transonic at t=%.2fs, %.0f m/s\n",
                    e.shot, e.time_s, pon::length(e.velocity));
};
```

| `TraceKind` | When | `i0` / `r0` |
|---|---|---|
| `Frame` | once per live shot per `step()` | `i0` = `FidelityTier`, `r0` = speed m/s |
| `Substep` | adaptive sub-step budget used that frame | `i0` = peak sub-step count |
| `TransonicEnter` / `TransonicExit` | Mach crosses into / out of 0.8–1.2 | — (needs `PrecisionFlag::TransonicFlag`) |
| `MediumChanged` | shot crossed into a new medium | `i0` = new `MediumId` |
| `GuidanceLost` | seeker track dropped — the shot now coasts | — |

Every `TraceEvent` also carries `shot` (the `StateId`), `time_s`, `position` and
`velocity`. `Substep` is reported for the geometry-heuristic sub-step path;
`PrecisionFlag::AdaptiveRKF45` shots get `Frame` but not `Substep`.

`Sim::setTraceSink(sink)` installs / clears the same callback on a live `Sim`
(pass `{}` to clear), so a game can arm tracing only when it needs it — e.g.
after a netcode desync — and disarm it again without rebuilding the `Sim`. The C
ABI mirrors it: `pon_sim_set_trace_sink(sim, fn, user)` with `pon_trace_event`
(`fn = NULL` clears).

#### `const SimStats& Sim::stats() const`

`traceSink` is the per-shot view; `stats()` is the aggregate a dev profiles
against — filled fresh by every `step()`, no opt-in required:

```cpp
sim.step(dt, world, sink);
const pon::SimStats& st = sim.stats();
std::printf("live: %u/%u/%u (hit/analytic/integrated)  substeps=%llu  "
           "queries=%llu  batches=%u  events=%u\n",
           st.liveHitscan, st.liveAnalytic, st.liveIntegrated,
           (unsigned long long)st.subSteps, (unsigned long long)st.sweptQueries,
           st.batchGroups, st.eventsEmitted);
```

Near-zero cost: the per-tier / sub-step / batch-group counts are increments
already implicit in `step()`'s loops, and `sweptQueries` / `eventsEmitted`
come from step() wrapping the caller's `World` / `EventSink` in a one-line
forwarding proxy for the frame — no behavior change, no per-call-site
instrumentation. C ABI: `pon_sim_get_stats(sim, pon_sim_stats*)`.

### ProjectileType

The description you register. Fields left at `0` / `std::nullopt` are filled
from the coarse class defaults for `klass` (see the table in
`docs/ballistics-phase-plan.md` §4).

| Field | Meaning |
|---|---|
| `id` | your string key, informational |
| `klass` | `ProjectileClass` — seeds defaults and picks the code path *only* |
| `dragModel` | `ConstantCd` / `G1` / `G7` / `CustomCurve` / `BallProfile` (all work) |
| `mass_kg` | mass; heavier flies flatter and penetrates more |
| `refDiameter_m` | reference diameter for the drag area |
| `ballisticCoefficient` | `std::optional` — published G1/G7 BC in lb/in²; scales the standard curve to this round |
| `dragCoefficient` | `std::optional` — for `ConstantCd` (and the `CustomCurve` fallback) |
| `customDragCurve` | `std::vector<DragCurvePoint>` — `{mach, cd}` samples for `CustomCurve` (Doppler-radar data); need not be uniformly spaced |
| `ballProfile` | string id into the ball-profile table (`"golf_ball"`, `"fifa_football"`, `"baseball"`, …), for `BallProfile`; also fills `refDiameter_m` / `mass_kg` / `muzzleSpeed_mps` when left at 0 |
| `muzzleSpeed_mps` | `std::optional` — default launch speed |
| `spinRate_radps` | `std::optional` — spin magnitude; drives the Magnus lift for `BallProfile` balls |
| `twistRate_m` | signed barrel twist (m/turn) for `PrecisionFlag::SpinDrift`; only the sign is read (`>=0` right-hand ⇒ drifts right) |
| `millerStability` | Miller stability `Sg` for spin drift + aero jump; `<=0` ⇒ 1.8 assumed |
| `spinAxisMode` | `AlongVelocity` (rifling / a clean spiral — no Magnus) / `Fixed` (curveball, backspin, tumble — world-frame axis) / `Free` (knuckle wobble; treated as `Fixed` for now) |
| `spinAxis` | default spin axis for `Fixed` / `Free` (world frame); `{0,0,0}` ⇒ a horizontal axis square to the launch direction |
| `terminal` | `TerminalParams` — `noseShapeFactor` N* (0 ⇒ class default), `hardness`, `deformable`, `fragile`; drives the terminal-ballistics pipeline |
| `aero` | `AeroAngular` — length, inertia tensor and the angular-aero coefficients for `PrecisionFlag::SixDOF`; every `0` field is seeded from mass / diameter / class |
| `maxLifetime_s`, `maxRange_m` | hard caps; the projectile is reaped with an `Expired` event |

```cpp
pon::ProjectileType t;
t.id                   = "556x45_62gr_m855";
t.klass                = pon::ProjectileClass::Bullet;
t.dragModel            = pon::DragModel::G7;
t.ballisticCoefficient = 0.151;   // published G7 BC (lb/in^2)
t.mass_kg              = 0.00402;
t.refDiameter_m        = 0.00570;
t.muzzleSpeed_mps      = 940.0;
t.maxLifetime_s  = 8.0;
```

### The named catalog

`#include <poncelet/catalog.hpp>`. Instead of filling a `ProjectileType` by
hand, take a fully-populated one from the shipped table:

```cpp
pon::ProjectileType t = pon::catalog::get("9x19_124gr_fmj");
const pon::TypeId id  = sim.registerType(t);          // class defaults still fill any gap
```

The string id *is* the round — `klass` only seeds gaps and picks the code path.
`find(id)` returns `std::optional<ProjectileType>` (empty on an unknown id);
`get(id)` never fails (an unknown id comes back as a bare `Custom` type with just
`id` set); `has(id)` / `ids()` enumerate.

The tables live in `data/*.csv` (projectile catalog, class defaults, ball
profiles, materials), baked to a C++ fragment at build time. Every value carries
its source in a comment at the top of its file. Ships with ~45 rounds/arrows/
balls; a few examples: `9x19_124gr_fmj`, `556x45_62gr_m855`, `762x51_175gr_smk`,
`50bmg_660gr_fmj`, `longbow_bodkin_60g`, `crossbow_bolt_28g`, `atlatl_dart_450g`,
`fifa_matchball`, `golf_ball`, `baseball_mlb`.

A game layers its own rows on top (later id wins, baked or overlay):

```cpp
std::string err;
if (pon::catalog::load_csv("mygame/rounds.csv", &err) < 0)
    std::fprintf(stderr, "%s\n", err.c_str());
pon::catalog::add(myHandBuiltType);   // or one at a time
pon::catalog::clear_overlay();        // back to the baked table
```

`pon::materials::find("oak")` / `load_csv` / `add` work the same way for the
material table (§4) — a convenience for callers that want the shipped seeds
rather than authoring every `Material` their `World` returns.

`pon::catalog::to_csv()` / `save_csv(path)` dump the current catalog (baked
rows, with any overlay row replacing its baked counterpart) back to CSV text —
the same column order `data/projectiles.csv` uses. There is otherwise no way
to get the shipped rounds back out; a dev who wants to tweak one has to
transcribe it by hand. `pon::materials::to_csv()` / `save_csv()` do the same
for the material table. "export → edit → `load_csv`" is the intended
authoring loop:

```cpp
std::string csv = pon::catalog::to_csv();   // or save_csv("rounds.csv")
// hand-edit rounds.csv...
pon::catalog::load_csv("rounds.csv");       // later ids override the baked table
```

### LaunchParams and spawning

| Field | Default | Meaning |
|---|---|---|
| `position` | — | muzzle position, world space |
| `direction` | — | aim direction; normalized for you |
| `speed` | `std::nullopt` | launch speed; `nullopt` uses the type's `muzzleSpeed_mps` |
| `spin` | `std::nullopt` | spin rate; `nullopt` uses the type's `spinRate_radps` |
| `spinAxis` | `std::nullopt` | spin axis for this shot (world frame); `nullopt` uses the type default / a mode-derived axis. Ignored for `AlongVelocity` |
| `medium` | `std::nullopt` | medium the shot starts in (index into `Environment::media`); `nullopt` → air. Set it when spawning already submerged |
| `tier` | `AnalyticDrag` | hit-detection fidelity (see [Fidelity tiers](#fidelity-tiers)) |
| `precision` | `PrecisionFlag::None` | opt-in second-order effects (see [Precision effects](#precision-effects)) |
| `precisionMrad` | `0` | opt-in muzzle dispersion cone half-angle, milliradians (`0` = exact aim). See [Dispersion](#dispersion-is-yours) |
| `initialYaw` | `std::nullopt` | 6-DOF only — an initial total-yaw angle (rad) between the nose and the launch velocity (a muzzle tip-off) |

```cpp
pon::LaunchParams lp;
lp.position  = {0, 1.7, 0};
lp.direction = {0.9, 0.05, 0.1};
lp.tier      = pon::FidelityTier::Hitscan;      // cheap "did it connect" check
// lp.speed / lp.spin left as nullopt -> catalog defaults
```

### Precision effects

Everything above gets a projectile "roughly right". The second-order effects
that matter past a few hundred metres are opt-in per shot via a `PrecisionFlag`
bitmask (default `None` = the fast path, byte-unchanged). Combine with `|`:

```cpp
lp.precision = pon::PrecisionFlag::SpinDrift | pon::PrecisionFlag::Coriolis
             | pon::PrecisionFlag::LocalSpeedSound;
```

| Flag | Effect | Inputs it reads |
|---|---|---|
| `SpinDrift` | gyroscopic drift, Litz closed form `D(t) ≈ 1.25·(Sg+1.2)·t¹·⁸³` inches, applied as a per-step lateral acceleration — no 6-DOF solve (~0.2 m right at 1000 m for a .308) | `ProjectileType::twistRate_m` (sign), `millerStability` |
| `Coriolis` | Earth-rotation deflection `a = −2 Ω × v` (horizontal + Eötvös vertical), constant in the world frame | `Environment::latitude`, `northAzimuth` (or `setCoriolis()`) |
| `AeroJump` | one-time vertical muzzle kick from a muzzle crosswind, scaled by `Sg` | the wind field at the muzzle, `millerStability` |
| `LocalSpeedSound` | the Mach abscissa tracks the ISA temperature lapse with altitude (matters for high-angle / mountain shots) | `Environment::setAtmosphere()` |
| `AdaptiveRKF45` | the `Integrated` tier uses an embedded Cash-Karp RK4(5) step controller targeting `SimConfig::posTolerance_m` instead of the geometry-distance heuristic (~2× cost) | `SimConfig::posTolerance_m` |
| `TransonicFlag` | emits a `TransonicWindow` event the first time the round enters Mach 0.8–1.2 and latches `kFlagPastTransonic` in `ProjectileState::flags` on the way out (informational — no trajectory change) | — |
| `SixDOF` | full rigid-body angular flight — see below | `ProjectileType::aero`, `LaunchParams::initialYaw`, `twistRate_m` (sign) |

`SpinDrift`, `Coriolis`, `LocalSpeedSound`, `AdaptiveRKF45` and `SixDOF` push an
`AnalyticDrag` shot onto the integrator (the closed form cannot carry them).

### 6-DOF flight (`PrecisionFlag::SixDOF`)

Turns the scalar-spin approximation into a real rigid body: the projectile's
nose direction and angular velocity are integrated alongside the trajectory.
**Tumbling, the yaw of repose (and the spin drift it produces), epicyclic
coning and its damping, and transonic instability all emerge** from the
angular aerodynamics — nothing is a stored curve. A spin-stabilised bullet
holds its nose on the path; the same bullet with no spin goes end over end; an
under-stabilised round flips through the transonic window.

The angular aerodynamics come from `ProjectileType::aero` (`AeroAngular`). Every
field left at `0` is seeded from mass / diameter / class — a reference length
and inertia tensor plus a coefficient set (overturning `C_Mα`, pitch damping
`C_Mq`, Magnus moment `C_Mpα`, roll damping `C_lp`, lift `C_Lα`, yaw drag). The
seeds give plausible behaviour (a 1-in-10″ .308 is gyroscopically stable, its
spin drift lands within ~2× of the Litz value); a game tunes them per round
against range data. Finned classes (`Arrow` / `Bolt` / `Spear`) get a
statically-*stable* seed (negative `C_Mα`).

```cpp
shot.precision  = pon::PrecisionFlag::SixDOF;
shot.spin       = 19500.0;      // rad/s (≈ 1-in-10″ at 790 m/s); sign from twistRate_m
shot.initialYaw = 0.035;        // optional ~2° muzzle tip-off — coning then damps
// ...
const pon::ProjectileState& s = sim.state(h);
s.orientation;                  // Quat, body→world; nose = orientation.rotate({1,0,0})
s.angVel_radps;                 // body frame; x is the spin rate p
s.angleOfAttack_rad;            // current total yaw between nose and airflow
// s.flags & pon::kFlagTumbling  — latched once the yaw passes ~60°

// Rendering a tracer? Skip the quaternion math:
pon::nose_direction(s);         // world-frame nose, = orientation.rotate({1,0,0})
pon::up_direction(s);           // world-frame "up" reference, = orientation.rotate({0,1,0})
pon::spin_phase(s);             // accumulated roll angle (rad), = s.spinPhase_rad
```

All three are defined for every fidelity tier, not just `SixDOF` — `orientation`
is identity elsewhere, so `nose_direction`/`up_direction` come back as the
unrotated `{1,0,0}`/`{0,1,0}` and `spin_phase` as `0`; a renderer doesn't need
to branch on the tier before calling them.

Cost is roughly the `Integrated` tier plus the angular RK4 (one `acos` and a
handful of cross products per sub-step); the axial spin is handled without a
tiny step. `docs/examples/sixdof_flight.cpp` is a full run.

`pon::mv_from_powder_temp(baseMv, baseT_K, sensitivity_mps_per_K, T_K)` is an
input helper for powder-temperature muzzle-velocity sensitivity — nothing
applies it automatically; you decide whether to jitter per shot.

### Dispersion is yours

Round-to-round spread is a weapon / shooter property, not physics — poncelet
never injects it into a trajectory (per-trajectory RNG would break lag-comp
rewind). Model it in your weapon code.

For the common case there is an opt-in convenience: `LaunchParams::precisionMrad`
is a cone half-angle in milliradians; when it is `> 0`, `spawn()` rotates the
aim `direction` by a random offset inside that cone, uniform in solid angle,
drawn from the Sim's seeded RNG (`SimConfig::rngSeed`) and advanced once per
spawn. Same seed + same spawn order → the same spread, so it survives a rollback
and replay.

```cpp
lp.precisionMrad = 0.3;   // ~1 MOA weapon; per-shot cone
sim.spawn(kRifle, lp);
```

It is **`PlatformStable`, not `BitExact`** — the cone sample uses
`std::sin/cos/sqrt`. Leave it `0` (the default) for `BitExact` server-side hit
validation and apply your own fixed-point spread upstream if you need one. It is
independent of `PrecisionFlag` and of the `Free`-spin knuckle wobble.

The `Coriolis` world-frame convention: the shot's downrange direction is world
`+x` horizontal, up is `+y`, and `northAzimuth` is that direction's compass
bearing (0 = north, clockwise).

### Trajectory preview (`preview_arc`)

For an aim indicator or a grenade-toss arc you want the *predicted* path before
committing a shot — without a `Sim` or a `World`:

```cpp
std::vector<pon::Vec3> arc;
pon::preview_arc(loadedRound, aim, env,
                 /*dt*/ 0.05, /*maxTime*/ 8.0, arc, /*groundY*/ 0.0);
// arc[0] == aim.position; one point every dt until maxTime, the round
// expires, or y drops to groundY. Draw it as a polyline.
```

It runs the `AnalyticDrag` closed form (drag + gravity + wind), so it tracks a
spawned `Integrated` shot within a few percent over a flat trajectory. Cost is
a throwaway drag-LUT compile plus the sampling — microseconds, fine to call
per frame for UI, not for thousands of previews at once. A `type` that fails
[`validate`](#gotchas-and-faq) leaves `arc` empty. See
`docs/examples/aim_preview.cpp`.

An overload fills `std::vector<pon::TrajectorySample>` instead — the same
points, each carrying `velocity` and `time_s` alongside `position`, so an aim
UI can read impact speed or time-to-target at the reticle instead of just
drawing a line:

```cpp
std::vector<pon::TrajectorySample> arc;
pon::preview_arc(loadedRound, aim, env, 0.05, 8.0, arc, 0.0);
printf("impact in %.2fs at %.0f m/s\n", arc.back().time_s, length(arc.back().velocity));
```

### ProjectileState

Read-only during play (via `Sim::state`). Fields:

| Field | Meaning |
|---|---|
| `position`, `velocity` | current, world space (m, m/s) |
| `spin_radps`, `spinAxis` | current spin |
| `mediumId` | `kMediumAir` / `kMediumWater` / a registered id |
| `distanceTravelled_m` | path length so far |
| `timeAlive_s` | seconds since spawn |
| `flags` | status bits — `kFlagInTransonic` / `kFlagPastTransonic` (under `PrecisionFlag::TransonicFlag`), `kFlagExpanded` (a `deformable` round has mushroomed), `kFlagTumbling` (6-DOF: yaw passed ~60°) |
| `orientation`, `angVel_radps`, `angleOfAttack_rad` | rigid-body state — identity / zero unless `PrecisionFlag::SixDOF` is set (see [6-DOF flight](#6-dof-flight-precisionflagsixdof)) |
| `typeId` | the type it was spawned from |
| `impactYaw` | keyholing scalar, raised after a ricochet / perforation; scales presented area + nose factor on the next hit |
| `expandedDiameter_m` | mushroomed diameter after a hard hit (`0` ⇒ use `refDiameter_m`); terminal-only in v1 |
| `alive` | `false` once it has stopped, expired, or been despawned |

```cpp
const pon::ProjectileState& s = sim.state(h);
const double drop = launchHeight - s.position.y;
const double ke   = 0.5 * sim.type(s.typeId).mass_kg * pon::length_sq(s.velocity);
```

<a id="describe"></a>
`pon::describe(state)` returns a one-line summary for a log or an on-screen
debug readout — `"v=709 m/s  t=0.20 s  dist=151 m  pos=(151,2,0)  spin=0 rad/s
[integrated]"`. Pass the type (and optionally the environment) for the energy
and Mach: `pon::describe(s, sim.type(s.typeId), sim.environment())` adds
`"  E=2848 J  M=2.08"`.

`pon::mach(state, env)` / `pon::kinetic_energy_J(state, type)` are those same
two numbers as free functions, for a HUD that wants the values themselves
instead of parsing `describe()`'s string:

```cpp
const double m = pon::mach(s, sim.environment());
const double e = pon::kinetic_energy_J(s, sim.type(s.typeId)); // 0 if mass_kg <= 0
```

`pon::describe(const ProjectileType&)` is the type-level counterpart — content
authoring / a catalog browser, not a live shot: `"9x19_124gr_fmj: 8.0 g,
9.0 mm, Cd 0.30, ~360 m/s, ~521 J muzzle"`. Pass a type that has been through
`Sim::registerType()` (e.g. `sim.type(id)`, or `pon::catalog::get(id)`) to see
resolved class defaults instead of raw 0 / unset caller input:

```cpp
for (const std::string& id : pon::catalog::ids())
    std::puts(pon::describe(pon::catalog::get(id)).c_str());
```

### Environment

| Field | Default | Meaning |
|---|---|---|
| `gravity` | `{0, -9.80665, 0}` | gravity vector; change it for other bodies or arcade feel |
| `airDensity_kgm3` | `1.225` | air density the hot loop reads; set directly for a flat override, or via `setAtmosphere()` |
| `speedOfSound_mps` | `340.294` | local speed of sound; maps trajectory speed onto the `G1`/`G7` `Cd(Mach)` curve |
| `wind` | empty | `std::function<Vec3(Vec3 pos, Seconds t)>`; empty means still air |
| `latitude`, `northAzimuth` | `0` | Coriolis inputs (`setCoriolis()` sets both), read only with `PrecisionFlag::Coriolis` |
| `siteTemperature_K` | `288.15` | firing-site air temperature from the last `setAtmosphere()`; read only with `PrecisionFlag::LocalSpeedSound` |
| `media` | air + water | the `MediumRegistry` |

`env.setAtmosphere({altitude_m, temperature_K?, pressure_Pa?, relativeHumidity})`
runs the 1976 ISA model (valid to ~32 km) and stores the resulting air density
*and* local speed of sound. Pass `std::nullopt` for temperature / pressure to
use the ISA value at that altitude; humidity is `0..1`. `pon::isa_atmosphere()`
returns the full `AtmoState` (density, speed of sound, temperature, pressure)
without touching an `Environment`.

```cpp
pon::Environment env;
env.gravity = {0, -1.62, 0};                    // the Moon
env.setAtmosphere({/*altitude_m*/ 1600.0});     // mile-high range, ISA
env.wind = [](pon::Vec3 p, pon::Seconds t) {
    return pon::Vec3{2.0 * std::sin(0.2 * t), 0, 0};  // gusting crosswind
};
```

The `t` passed to the callback is each projectile's own time-alive (seconds
since it was spawned), so a time-varying gust is reproducible per shot.
`v_rel = v_proj − wind` feeds both the drag and Magnus terms; with the
`AnalyticDrag` tier a wind field falls back to the integrator (a wind adds a
transverse force the closed form cannot carry). `env.windAt(pos, t)` is a helper
that returns `{0,0,0}` when no wind is set.

Each medium carries a `density_kgm3`, a `dragScale` (a plain multiplier on the
drag deceleration — *not* a second density term), a `buoyancy` fraction of
`|gravity|` (0 = none, 1 = neutrally buoyant, >1 = floats), and a
`supercavitation` flag (see *Special media & behaviours* under Terminal
ballistics). A projectile's
current medium comes from `World::mediumAt`; spawn already-submerged with
`LaunchParams::medium`.

#### Medium registry

```cpp
// Built in: pon::kMediumAir (0), pon::kMediumWater (1).
pon::MediumDesc mud;
mud.name = "mud"; mud.density_kgm3 = 1500; mud.dragScale = 1200.0;
const pon::MediumId kMud = env.media.add(mud);
const pon::MediumDesc& d = env.media.get(kMud);
```

### The World callback

Implement this so the library can query your geometry. Only `raycast`,
`mediumAt` and `material` are required.

```cpp
struct MyWorld final : pon::World {
    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        // Segment a->b. Return false for a miss. On a hit, fill:
        //   out.point   world hit point
        //   out.normal  unit, facing back toward a
        //   out.t       fraction along a->b in [0,1]
        //   out.surface an id you can resolve in material()
        //   out.surfaceVelocity  target velocity at impact (for moving targets)
        return myScene.segmentCast(a, b, out);
    }
    pon::MediumId mediumAt(pon::Vec3 p) const override {
        return p.y < waterLevel ? pon::kMediumWater : pon::kMediumAir;
    }
    const pon::Material& material(pon::SurfaceId s) const override {
        return materials[s];
    }
};
```

`pon::HitResult` fields: `point`, `normal`, `t`, `surface`, `surfaceVelocity`.

For a test scene or the cookbook, `<poncelet/worlds.hpp>` ships a handful of
trivial `World`s so you don't have to write one just to see a ricochet:
`PlaneWorld` (one infinite plane), `SphereWorld` / `AabbWorld` (one primitive),
`SlabStackWorld` (N parallel planes along a shared axis, each with its own
`Material` — layered armor / a stack of boards), and `CompositeWorld` (holds
pointers to other `World`s, returns the nearest hit — compose primitives into
a scene). See `docs/examples/stock_worlds.cpp` and
[cookbook.md § 4](cookbook.md#4-apfsds-vs-an-rha-plate).

#### Swept-vs-swept and time-of-impact

Every tier does a swept **segment** query per sub-step (not a point test), so a
fast projectile cannot tunnel a thin wall as long as your `raycast` does a real
segment intersection. On a hit the library reports an exact fractional
time-of-impact — `Event::time_s` and the projectile's final `position` /
`velocity` are interpolated to the contact instant, not snapped to a step
boundary.

If the surface is moving, fill `out.surfaceVelocity` (world frame). The stepper
then corrects the impact point and time for the surface's travel over the
sub-step — a plate swinging toward the shooter is met sooner and nearer than its
frozen pose implies — and `Event::energy_J` / `Event::residualSpeed_mps` are
computed from the **closing** velocity (`v_projectile − surfaceVelocity`). Keep
your `raycast` testing the surface's current-frame pose; the sub-frame sweep is
the library's job. See `docs/examples/moving_target.cpp`.

`pon::EmptyWorld` is a ready-made implementation that hits nothing and reports
air everywhere — use it for exterior-ballistics tests and before your geometry
bridge is wired.

```cpp
pon::EmptyWorld world;   // free flight, no impacts
```

### Materials

Your `World::material(SurfaceId)` returns a `pon::Material` for each surface a
`raycast` reports. On a confirmed impact the library runs the terminal-ballistics
pipeline (§3.5) against it — ricochet check → obliquity → ballistic-limit → embed
(Poncelet channel depth) or perforate (residual velocity) — and emits the
outcome event(s).

| Field | Meaning |
|---|---|
| `name` | informational |
| `behaviour` | `Brittle` (concrete/glass) / `Ductile` (metal) / `Fibrous` (wood/flesh) / `Membrane` (balloon/foliage — always perforates) / `Granular` (sand) / `Fluid` (water — switches the projectile's medium, no discrete stop) |
| `density_kgm3` | mass density (the Poncelet inertial term) |
| `strength_Pa` | resistive strength (the Poncelet quasi-static term) |
| `toughness` | energy per unit crack area (J/m²) |
| `thickness_m` | slab thickness; **`<= 0` ⇒ bulk** (earth, a thick block — never perforates) |
| `elasticity` | restitution for the bounce case |
| `entersMedium` | medium the projectile enters through a `Fluid` surface (e.g. water); `<0` keeps the current medium |

The projectile side comes from `ProjectileType::terminal` (`noseShapeFactor` N*
— sharp broadhead ~0.4, round ~0.9, flat ~1.3; `hardness`; `deformable` — lead
mushrooms and loses sectional density between layers; `fragile` — a thrown pot
shatters). Blank `noseShapeFactor` is filled from the class default.

```cpp
pon::Material oak;
oak.name = "oak"; oak.behaviour = pon::MaterialBehaviour::Fibrous;
oak.density_kgm3 = 750; oak.strength_Pa = 90e6; oak.thickness_m = 0.05;

pon::Material w = pon::material_water();   // built-in helpers
pon::Material a = pon::material_air();

// or take a shipped seed (data/materials.csv, §4):
if (auto oakSeed = pon::materials::find("oak")) oak = *oakSeed;
```

`pon::materials::find` / `has` / `names` read the shipped table;
`load_csv` / `add` / `clear_overlay` layer a game's own materials on top (a later
name wins). This is purely a convenience — the library never consults the table
itself, it only uses the `Material` your `World` hands back.

A shot that **ricochets** or **perforates** keeps flying — it is swept on
through the rest of the frame, so spaced targets ("through the door, into the
wall behind") resolve by chaining, each layer emitting its own event and handing
a slower (and possibly deformed / yawed) projectile to the next. See
`docs/examples/terminal_ballistics.cpp`.

### Special media & behaviours

**Water entry.** Crossing into a much denser medium (air → water, ≥ 4× density)
costs a one-time **surface impulse** at the interface — a single-digit-% step
loss carried on the `MediumChanged` event's `residualSpeed_mps` — after which
the new medium's drag takes over. Water is ~815× the density of air, so the
`Integrated` tier bleeds a rifle round below wounding speed inside ~1–2 m
(pistol rounds a little further), then `buoyancy` sinks it. `AnalyticDrag`
shots hand the rest of the frame to the integrator at the boundary.

**Supercavitation.** Set `MediumDesc::supercavitation` on a medium and a round
moving faster than the cavity-formation speed (~60 m/s) rides a gas bubble:
effective drag collapses to a few percent and underwater reach jumps from ~1 m
to tens of metres. Below that speed the cavity closes and full wetted drag
returns. Speed-gated, so it stays deterministic.

**Arrow vs concrete.** A sharp, low-sectional-density projectile
(`noseShapeFactor < 0.5`, SD `= m/A` below ~1500 kg/m², under 300 m/s) against a
`Brittle` surface can't sustain the contact load — it spalls the face and
**stops** there, no channel, whatever the penetration ODE would say. A rifle
round (high SD) still perforates the same panel.

See `docs/examples/special_media.cpp`.

### Events

`Sim::step` writes `pon::Event` records into the `EventSink` you pass.

`pon::EventType`: `SurfaceCrossed`, `Ricochet`, `Embedded`, `Perforated`,
`Stopped`, `MediumChanged`, `Shattered`, `TransonicWindow`, `Expired`
(`Popped` is reserved). A perforation emits `SurfaceCrossed` at the near face
then `Perforated` at the far face.

`pon::Event` fields: `type`, `point`, `normal`, `time_s`, `energy_J` (deposited
into the surface), `residualSpeed_mps` (speed after a ricochet / perforation),
`channelAxis` + `channelDepth_m` + `channelWiden_m` (penetration channel),
`projectile`.

Provide your own sink for the hot path (no allocation), or use the bundled
`VectorEventSink` for tools and tests:

```cpp
struct RingSink final : pon::EventSink {
    void emit(const pon::Event& e) override { buffer[head++ % N] = e; }
};

pon::VectorEventSink sink;                  // std::vector<Event> events;
sim.step(dt, world, sink);
for (const pon::Event& e : sink.events) {
    switch (e.type) {
        case pon::EventType::Stopped:       spawnImpactDecal(e.point, e.normal); break;
        case pon::EventType::MediumChanged: splash(e.point); break;
        default: break;
    }
}
sink.events.clear();
```

An `EventType::Detonated` is emitted when a fuzed [warhead](#explosive-warheads)
goes off — `point` / `time_s` are the detonation, `energy_J` the chemical yield,
`payload_kg` the effective TNT-equivalent charge.

### Explosive warheads

Give a `ProjectileType` a `WarheadDesc` and the shot becomes a live round.
`Sim::step` picks the detonation point and time from the fuze and emits an
`EventType::Detonated` there — nothing runs for an inert round
(`chargeMass_kg <= 0`, the default), so existing behaviour is unchanged.

```cpp
pon::ProjectileType shell = /* ... */;
shell.warhead.chargeMass_kg  = 6.6;          // explosive fill
shell.warhead.tntEquivalence = 1.1;          // Comp-B ≈ 1.1, RDX ≈ 1.3
shell.warhead.fuze           = pon::FuzeMode::Contact;
shell.warhead.surfaceBurst   = true;         // ~1.8× yield for a ground burst
```

Fuze modes: `Contact` (first surface hit), `Delayed` (`fuzeDelay_s` after the
first hit — the round is spent at the face, the emitted event's `time_s` carries
the future instant), `TimedAirburst` (`fuzeDelay_s` after launch), `Proximity`
(geometry within `proximityRadius_m` of the flight path). A live warhead ends its
flight at the first contact — the kinetic terminal outcome is not resolved.

The blast field is a standalone `pon::Burst` you build from the same
`WarheadDesc` (`#include <poncelet/explosion.hpp>`):

```cpp
for (const pon::Event& e : sink.events) {
    if (e.type != pon::EventType::Detonated) continue;
    pon::Burst burst(e.point, shell.warhead, sim.environment(), e.time_s);

    pon::BlastTarget crate{ /*centroid*/ e.point + pon::Vec3{5,0,0},
                            /*area_m2*/ 0.8, /*mass_kg*/ 60.0 };
    double los = pon::blast_line_of_sight(world, e.point, crate.centroid);
    pon::BlastLoad load = burst.loadOnBody(crate, los);
    body.velocity += toEngine(load.deltaVelocity_mps);      // push the crate

    double p = burst.overpressureAt(playerPos, now);         // for a shockwave FX
}
```

`Burst::sampleAt(point)` returns the full `BlastSample` — incident and reflected
peak overpressure, dynamic (blast-wind) pressure, arrival time, positive-phase
duration, specific impulse, and the Friedlander decay constant.
`overpressureAt(point, t)` evaluates the Friedlander waveform (0 outside the
positive phase; the suction phase is not modelled).

Air bursts use the Kinney & Graham scaled-distance fits for a spherical TNT
charge (`Z = R / W^{1/3}`); `WarheadDesc::underwater` switches to Cole similitude
(~50 MPa·(W^⅓/R)^1.13 near-field shock, exponential decay). `thermobaric > 0`
stretches the positive phase and boosts delivered impulse (enhanced-blast /
afterburn). These are engineering fits with a ~10–20 % spread — seeds a game
tunes, not laboratory numbers; the blast model never touches the trajectory core.

See `docs/examples/explosive_warhead.cpp`.

### Fragmentation

Give a `WarheadDesc` a `FragmentationDesc` (`fragmentation.casingMass_kg > 0`) and
the detonation can be turned into a fragment spray — a Mott mass spectrum
launched at Gurney velocity, each fragment its own projectile with its own drag
and terminal ballistics (`#include <poncelet/fragmentation.hpp>`):

```cpp
pon::FragmentationDesc& f = shell.warhead.fragmentation;
f.casingMass_kg          = 3.0;                    // steel that breaks up
f.gurneyVelocity_mps     = 2440.0;                 // √(2E′): TNT 2440, Comp-B 2680
f.casingInnerDiameter_m  = 0.070;                  // Mott µ from the geometry
f.casingWallThickness_m  = 0.008;                  //   (or set mottMu_kg directly)
f.spray                  = pon::FragmentSpray::CylinderBeam; // artillery beam
f.sprayAxis              = shellVelocityAtBurst;   // casing long axis
f.maxFragments           = 300;                    // hard cap — lightest cut first

// off a Detonated event:
std::vector<pon::FragmentSpec> spray;
pon::generate_fragments(f, det.point, shellVelAtBurst,
                        shell.warhead.chargeMass_kg, spray);
pon::spawn_fragments(sim, spray, pon::FidelityTier::Integrated, "frag");
```

`generate_fragments` is a pure, deterministic (seeded, allocation-free) function
— it never touches the flight core. Fragment count ≈ `casingMass / (2µ)`; the
Mott parameter µ comes from `mottMu_kg` if set, else
`µ^½ = B · t^{5/6} · d_i^{1/3} · (1 + t/d_i)`. Fragment speed is the Gurney
cylinder form `√(2E′) · (M/C + ½)^{-½}` (M = casing, C = charge) with a
per-fragment scatter, plus the warhead's own velocity at burst. Directions
follow `spray`: `Isotropic` (symmetric grenade), `Cone` (`coneHalfAngle_rad`
about `sprayAxis` — a directional mine / nose-fuzed shell), or `CylinderBeam`
(the equatorial side-spray of a cylindrical casing, `beamHalfWidth_rad` wide,
tilted forward by `beamForwardTilt_rad`).

`FragmentSpec` carries `mass_kg`, sphere-equivalent `diameter_m`, `dragCoefficient`
and `representsCount` (real fragments this spec stands for, when the casing yields
more than `maxFragments`). `spawn_fragments` buckets the spray into
`massClasses` steel-fragment `ProjectileType`s (blunt, hard, `ConstantCd`) and
spawns each fragment onto its class — for a game that fires the same round often,
`plan_fragment_classes` + `fragment_class_type` let you register the class set
once and spawn onto it yourself. The mass spectrum and coefficients are
engineering seeds (~tens of % spread), tuned per casing/filler against test data.

See `docs/examples/fragmentation.cpp`.

### Shaped charges / EFP

Give a `WarheadDesc` a `ShapedChargeDesc` (`shapedCharge.linerMass_kg > 0`) and
the detonation forms a HEAT jet or an explosively formed penetrator
(`#include <poncelet/shapedcharge.hpp>`):

```cpp
pon::ShapedChargeDesc& sc = warhead.shapedCharge;
sc.linerMass_kg       = 0.35;                      // copper cone
sc.chargeDiameter_m   = 0.100;                     // CD — the governing length
sc.kind               = pon::ShapedChargeType::ConicalJet;  // or ::EFP
sc.coneApexAngle_rad  = 60.0 * M_PI / 180.0;
sc.gurneyVelocity_mps = 2930.0;                    // Octol filler

// off a Detonated event (its channelAxis is the jet aim, its point the origin):
const auto jet = pon::shaped_charge_formation(sc, warhead.chargeMass_kg);
const auto pen = pon::shaped_charge_penetration(sc, plateMaterial, standoff_m,
                                                plateThickness_m,
                                                warhead.chargeMass_kg);
if (pen.perforated || pen.spall) {
    std::vector<pon::FragmentSpec> debris;
    pon::shaped_charge_behind_armour(sc, plateMaterial, exitPoint, aimDir,
                                     standoff_m, plateThickness_m,
                                     warhead.chargeMass_kg, debris);
    pon::spawn_fragments(sim, debris, pon::FidelityTier::Integrated, "bah");
}
```

`shaped_charge_formation` is the Birkhoff/PER steady-state collapse: an
open-faced-sandwich Gurney velocity `V0` from the charge-to-metal ratio, a jet
tip near `V0·1.3/sin β`, the slowest jet element near `V0`, and a jet-vs-slug
mass split from the cone angle. The jet stretches at `(v_tip − v_tail)` until the
liner metal particulates (`particulationTime_s`, copper ≈ 160 µs) — that sets
`coherentLength_m` and `breakupStandoff_m`.

`shaped_charge_penetration` is the hydrodynamic limit
`P = L_eff·√(ρ_jet/ρ_target)` with a modified-Bernoulli velocity cut for target
strength (near-zero for a hot jet into steel, large for a slow EFP). `L_eff` is
the stretched length reaching the face, with an efficiency taper that peaks the
depth at a few CD of standoff (the classic HEAT standoff curve) and falls off
fast past jet breakup. With `targetThickness_m > 0` the result carries the
`perforated` / `spall` verdict, the residual jet length + velocity, and the hole
diameter; `shaped_charge_behind_armour` turns that into a residual-jet cone plus
a plate-spall cone as `FragmentSpec`s. An `EFP` liner skips the stretch model: a
single coherent slug (~1.5–2.5 km/s), shallow (≈0.5–1 CD) but effective to many
hundreds of CD of standoff.

Like the blast and fragmentation models this is all engineering fits, never the
`fp-contract-off` core — coefficients are seeds, tune per warhead against test
data.

See `docs/examples/shaped_charge.cpp`.

### Guided munitions

Two layers, both riding a per-step **external-acceleration hook**
(`FlightModel::externalAccel`, added to the force model every sub-step):

```cpp
// Generic hook — thrust, a scripted correction, your own guidance law:
sim.setExternalAccel(missile, Vec3{0, 0, 40.0});   // m/s², world frame, persists
sim.setExternalAccel(missile, {});                 // clear

// Built-in law — give the type a GuidanceDesc, feed the seeker track each frame:
pon::ProjectileType sam;
sam.guidance.law               = pon::GuidanceLaw::ProportionalNav; // or Pursuit / AugmentedPN
sam.guidance.navConstant       = 4.0;    // N (3–5)
sam.guidance.maxLateralAccel_g = 40.0;   // airframe g-limit
sam.guidance.thrustAccel_mps2  = 250.0;  // optional axial sustainer
sam.guidance.burnTime_s        = 2.0;
sam.guidance.seekerHalfFov_rad = 0.7;    // target past this off-boresight ⇒ lock lost
sam.guidance.activationDelay_s = 0.3;    // coast the first t (boost / separation)

const pon::StateId m = sim.spawn(sim.registerType(sam), lp);
for (;;) {
    sim.guide(m, targetPos, targetVel);   // update the track each frame
    sim.step(dt, world, sink);
}
sim.clearGuidanceTarget(m);               // drop lock ⇒ coast ballistic
```

`Pursuit` steers the velocity vector straight at the target (a tail chase, trails
a crosser). `ProportionalNav` commands `a = N · Vc · λ̇` perpendicular to the
line of sight — the standard homing law, it leads a crossing target onto a
collision course. `AugmentedPN` adds `N/2 · a_target,⟂` from a finite-difference
of the track velocity, for a manoeuvring target. The command is clamped to
`maxLateralAccel_g`, an optional `inducedDragFactor` bleeds speed with the
manoeuvre, and a lost lock (`seekerHalfFov_rad` exceeded) latches — the shot
coasts. A guided launch is silently promoted to the `Integrated` tier.

The guidance command is evaluated once per `Sim::step` from the frame-start state
(a per-frame update is plenty for the g-loads involved) and is deterministic, but
it is **not** in the `fp-contract-off` fixed-point core — it feeds the trajectory
as a `double` external-acceleration term, so a guided shot under `BitExact` is
deterministic per platform but not cross-platform bit-identical. Folding
`compute_guidance` into the Q32.32 core is a tracked follow-up.

See `docs/examples/guided_missile.cpp`.

### Version

```cpp
#include <poncelet/version.hpp>
pon::Version v = pon::library_version();          // {major, minor, patch}
const char*  s = pon::library_version_string();   // "0.1.0"
// Compile-time: PONCELET_VERSION_MAJOR / _MINOR / _PATCH / _STRING
```

---

## C API

`<poncelet/poncelet.h>` is a flat, stable C ABI over the same core, for binding
from C, Rust, C#, Python, etc. It exposes the create → register → spawn → step
→ query loop; the `World` callback surface reaches C in item 6, so for now C
callers get free-flight simulation.

```c
#include <poncelet/poncelet.h>

pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
pon_sim_set_gravity(sim, (pon_vec3){0, -9.80665, 0});
pon_sim_set_atmosphere(sim, /*altitude_m*/ 0.0, 0.0, 0.0, 0.0); /* ISA sea level */

pon_projectile_desc d = {0};
d.id = "9x19";
d.klass = 0;                       /* pon::ProjectileClass::Bullet */
d.drag_model = PON_DRAG_G1;
d.mass_kg = 0.00804;
d.ref_diameter_m = 0.00902;
d.ballistic_coefficient = 0.15;    /* published G1 BC (lb/in^2) */
d.muzzle_speed_mps = 360.0;
uint32_t type = pon_register_type(sim, &d);

uint32_t shot = pon_spawn(sim, type,
                          (pon_vec3){0, 1.6, 0},   /* position */
                          (pon_vec3){1, 0, 0},     /* direction */
                          0.0);                    /* 0 -> use muzzle speed */

/* Wind: a C function pointer + a user-data pointer (must outlive the call). */
static pon_vec3 my_wind(pon_vec3 pos, double t, void* user) {
    (void)pos; (void)user;
    return (pon_vec3){0, 0, 3.0 + 2.0 * (t > 1.0)}; /* gusting crosswind */
}
pon_sim_set_wind(sim, my_wind, NULL);          /* NULL fn clears it */

/* Media: air (0) and water (1) are built in; register your own. */
pon_medium_desc mud = {0};
mud.name = "mud"; mud.density_kgm3 = 1600.0; mud.drag_scale = 3.0;
mud.buoyancy = 0.2;
int32_t medium = pon_sim_add_medium(sim, &mud);
uint32_t submerged = pon_spawn_in_medium(sim, type,
                                         (pon_vec3){0, 0, 0}, (pon_vec3){1, 0, 0},
                                         400.0, medium);

/* Precision effects: OR the PON_PRECISION_* bits on a precise spawn. */
pon_sim_set_coriolis(sim, /*latitude*/ 0.75, /*azimuth from north*/ 0.0);
uint32_t precise = pon_spawn_precise(
    sim, type, (pon_vec3){0, 2, 0}, (pon_vec3){1, 0, 0}, 792.0,
    /*Integrated*/ 2,
    PON_PRECISION_SPIN_DRIFT | PON_PRECISION_CORIOLIS | PON_PRECISION_TRANSONIC_FLAG);

for (int i = 0; i < 120; ++i) pon_step(sim, 1.0 / 60.0);

/* 6-DOF: point pon_projectile_desc::aero at a pon_aero_angular (or leave NULL
 * for seeds), spawn with PON_PRECISION_SIX_DOF, then read the rigid-body state
 * back from pon_state — orientation_quat (w,x,y,z), ang_vel_radps (body frame,
 * x = spin) and angle_of_attack_rad. PON_FLAG_TUMBLING latches past ~60° yaw. */

pon_state st;
if (pon_get_state(sim, shot, &st) == PON_OK)
    printf("x=%.2f y=%.2f flags=%u alive=%d\n",
           st.position.x, st.position.y, st.flags, st.alive);

pon_sim_destroy(sim);
```

| Function | Purpose |
|---|---|
| `pon_version_string()` | library version string |
| `pon_sim_create(det)` / `pon_sim_destroy(sim)` | lifetime |
| `pon_sim_create_ex(det, cfg)` | as `pon_sim_create`, plus `fixed_step_s` / `integrator` / `pos_tolerance_m` / `batch_integrator` / `trajectory_cache_frames`; `cfg = NULL` is `pon_sim_create` |
| `pon_sim_set_gravity(sim, g)` / `pon_sim_set_air_density(sim, rho)` | environment |
| `pon_sim_set_atmosphere(sim, alt, tempK, presPa, rh)` | ISA density + speed of sound (0 → ISA value) |
| `pon_sim_set_wind(sim, fn, user)` | install a wind callback; `fn = NULL` clears it |
| `pon_sim_set_trace_sink(sim, fn, user)` | install the diagnostic trace callback (`pon_trace_event`); `fn = NULL` clears it |
| `pon_sim_add_medium(sim, desc)` | register a medium; returns its id (≥ 2), or `-1` |
| `pon_sim_set_coriolis(sim, lat, azimuth)` | Coriolis inputs (radians) |
| `pon_mv_from_powder_temp(mv, T0, sens, T)` | powder-temperature muzzle-velocity helper |
| `pon_register_type(sim, desc)` | returns a type id, or `0xFFFFFFFF` |
| `pon_catalog_has(id)` | 1 if `id` is a known catalog round (baked or overlay) |
| `pon_register_catalog_type(sim, id)` | register a named catalog round; type id or `0xFFFFFFFF` if unknown |
| `pon_spawn(sim, type, pos, dir, speed)` | returns a state id, or `0xFFFFFFFF`; `speed <= 0` uses the type default |
| `pon_spawn_in_medium(sim, type, pos, dir, speed, medium)` | as `pon_spawn`, but starts in `medium` |
| `pon_spawn_precise(sim, type, pos, dir, speed, tier, precision)` | as `pon_spawn`, plus a fidelity tier + `PON_PRECISION_*` bitmask |
| `pon_step(sim, dt)` | advance all projectiles (free-flight for now) |
| `pon_get_state(sim, id, out)` | fills `pon_state`; returns `pon_status` |
| `pon_preview_arc(sim, type, muzzle, aim, dt, maxTime, groundY, out, max)` | predicted flight path without spawning; returns the arc's point count (`out = NULL` to measure) |
| `pon_preview_arc_ex(sim, type, muzzle, aim, dt, maxTime, groundY, out, max)` | as `pon_preview_arc`, but `out` is `pon_trajectory_sample` (position + velocity + time) |
| `pon_describe(sim, id, buf, cap)` | one-line debug string, NUL-terminated and truncated to `cap`; returns the untruncated length |
| `pon_describe_type(sim, type, buf, cap)` | same, for a registered TYPE (content authoring) rather than a live shot |
| `pon_shot_mach(sim, id)` / `pon_shot_energy_j(sim, id)` | the same numbers `pon_describe` folds into its `E=`/`M=` tail, as plain doubles |
| `pon_shot_nose(sim, id)` / `pon_shot_up(sim, id)` / `pon_shot_spin_phase(sim, id)` | 6-DOF world-frame nose / "up" direction + accumulated roll (rad); identity/0 outside `SixDOF` |
| `pon_last_error(sim)` | why the last `pon_register_type` / `pon_spawn*` on this sim failed, or `""` |
| `pon_live_count(sim)` | live projectile count |
| `pon_live_ids(sim, out, max)` | id of every live projectile; returns the live count (`out = NULL` to measure) |
| `pon_sim_get_stats(sim, out)` | fills `pon_sim_stats` with the last `pon_step()`'s per-frame counters |
| `pon_despawn(sim, id)` | remove one |
| `pon_sim_snapshot(sim, buf, cap)` | serialize into `buf`; returns bytes needed (call with `cap = 0` to measure) |
| `pon_sim_restore(sim, buf, size)` | restore a snapshot; `1` on success, `0` on a truncated / mismatched buffer |
| `pon_trajectory(sim, id, out, max)` | cached flight-path samples, oldest first; returns the count (`out = NULL` / `max = 0` to measure) |
| `pon_trajectory_size(sim, id)` | number of samples currently cached for `id` |
| `pon_sample_trajectory(sim, id, t, out_pos, out_vel)` | interpolated pose at flight-time `t`; `1` on success, `0` with fewer than two samples cached |

Warheads: set `pon_projectile_desc::warhead` (a `pon_warhead_desc*`) and read the
detonation from a `Detonated` event. The blast field is standalone:
`pon_burst_create(origin, warhead, density, sound_speed, t0)` →
`pon_burst_sample` / `pon_burst_overpressure_at` / `pon_burst_load_on_body` →
`pon_burst_destroy`.

Fragmentation: `pon_generate_fragments(&desc, origin, source_velocity,
charge_mass_kg, out, max)` fills an array of `pon_fragment_spec` (Mott spectrum +
Gurney speed, deterministic); spawn each as its own projectile with the
create → register → spawn loop.

Guided munitions: set `pon_projectile_desc::guidance` (a `pon_guidance_desc*`),
then each frame `pon_guide(sim, state, target_pos, target_vel)`;
`pon_clear_guidance` drops the track and `pon_set_external_accel(sim, state,
accel)` is the raw per-step force hook.

Shaped charges: `pon_shaped_charge_formation(&desc, charge_mass_kg, &out)`,
`pon_shaped_charge_optimal_standoff(&desc, charge_mass_kg)`, and
`pon_shaped_charge_penetrate(&desc, target_density, target_strength, behaviour,
standoff_m, target_thickness_m, charge_mass_kg, &out)` (behaviour `0..5` =
Brittle/Ductile/Fibrous/Membrane/Granular/Fluid). Behind-armour debris is C++
only (`pon::shaped_charge_behind_armour`).

Link against `poncelet` (static) or `poncelet_shared` (with
`-DPONCELET_SHARED=ON`); define `PONCELET_SHARED` when consuming the DLL on
Windows.

---

## Determinism

`pon::config::Determinism`, set via `SimConfig::determinism`:

| Mode | Guarantee | Status |
|---|---|---|
| `Loose` | none; fastest | available |
| `PlatformStable` | same platform + compiler + inputs → identical trajectory | **default**, available |
| `BitExact` | cross-platform bit-identical (rollback netcode, server-authoritative hits) | available — `Sim::status()` is `Status::Ok` |

`PlatformStable` is achieved with a fixed sub-step, fixed evaluation order and
a seeded RNG (`SimConfig::rngSeed`). `BitExact` additionally runs the integrator
on a **Q32.32 fixed-point core** (`Core<Fx32>` in `src/integrate.cpp`):
deterministic bitwise `sqrt`, uniform transcendental LUTs for `sin`/`acos`/`exp`/
`pow`, and fixed-point drag-LUT sampling — all integer arithmetic, so the folded
`stateHash()` digest is identical on every OS, compiler and optimisation level.

Two things stay `double` and so are per-platform-deterministic but not
cross-platform bit-identical under `BitExact`: the per-type drag LUT compiled at
`registerType()` (a one-time cost; ship the compiled table if you need it
identical), and the guidance law's external-acceleration term (`PrecisionFlag`s
aside, a guided munition's steering command is not yet on the fixed-point
path). The opt-in `AdaptiveRKF45` tier's step-size controller *is* fixed-point
now — its `ratio^0.2` runs through a dedicated LUT (`fx_pow_ratio02`) under
`BitExact` instead of `std::pow`, closing what used to be the one remaining
libm call in that path.

If you need reproducible runs and can't use `BitExact`: keep `dt` and
`fixedStep_s` constant, feed inputs in a fixed order, and pin `rngSeed`.

**`Sim::stateHash()`** returns a 64-bit FNV-1a digest of every live shot's full
dynamic state (position, velocity, spin, orientation, medium, fuze + guidance
bookkeeping) plus the RNG cursor, hashed field by field. Sample it every frame
and compare against the authority for a rollback-netcode desync check. The
`poncelet_bench --det-check` gate (ctest `poncelet_det_check`) runs one fixed
mixed-tier scene twice and asserts the two per-frame digest streams are
identical; CI runs it on ubuntu / windows / macos × Debug / Release, so a
platform libm or optimisation level breaking `PlatformStable` repeatability is
caught. `poncelet_bench --bitexact-golden` (ctest `poncelet_bitexact_golden`)
folds a fixed `BitExact` scene into one digest and checks it against a committed
golden constant — the same matrix then verifies that the Q32.32 digest is
identical *between* platforms, not merely repeatable on one. `--bitexact-print`
re-captures the constant if the scene changes.

**`Sim::stateHash(StateId id)`** is the same digest, folded over just `id`'s
own dynamic state — when the whole-sim `stateHash()` says a frame desynced,
this pinpoints *which* live shot did it instead of a bisection search over the
scene. Returns `0` for a dead / never-spawned / out-of-range id.

### Rollback netcode

`stateHash()` tells you a rollback is *needed* (client and server digests
diverged); `Sim::snapshot()` / `Sim::restore()` are what you roll back *with*.
The usual client-side-prediction loop:

1. Each tick, before stepping, call `snapshot()` and keep it (keyed by tick)
   alongside the input that produced that tick.
2. Step the Sim forward on the predicted input.
3. When an authoritative update arrives for an earlier tick, `restore()` the
   snapshot from that tick, then re-`step()` forward through the newer inputs
   (now corrected) to catch back up to the present.

`restore()` requires the same type registry (same ids, same order) and the
same `SimConfig::trajectoryCacheFrames` as the Sim the snapshot came from —
typically the client's own earlier self, so this holds automatically. See
[cookbook.md § 7](cookbook.md#7-netcode) for a worked example.

---

## Gotchas and FAQ

**`spawn` returned `kInvalidState` / `registerType` returned `kInvalidType`.**
Call `sim.lastError()` — it returns a sentence explaining which field is wrong
(zero `direction`, out-of-range `TypeId`, no usable mass/diameter, a
`CustomCurve` with no curve and no fallback `Cd`, a non-finite value, …). To
check a `ProjectileType` *before* registering it, call
`pon::validate(type)` — it returns `std::optional<std::string>` with the same
message, or `std::nullopt` when the type is good.

**My bullet does not drop / drag does nothing.** A `ConstantCd` type needs
`dragCoefficient` set; a `G1`/`G7` type needs `ballisticCoefficient` (or a
`klass` whose default supplies one); a `CustomCurve` type needs
`customDragCurve` (or a `dragCoefficient` fallback); a `BallProfile` type needs
a known `ballProfile` id (an unknown id falls back to a plain rough sphere).
Gravity always applies regardless.

**My ball flies dead straight — no curve.** The Magnus term needs a non-zero
`spin` *and* a spin axis that is not parallel to the velocity. `SpinAxisMode::
AlongVelocity` (the default for bullets, and for a `ProlateSpheroid` spiral) is
Magnus-free by design. Set `spinAxisMode = Fixed` and give a `spinAxis` square
to the shot (e.g. `{0,1,0}` for a curveball, the default horizontal axis for
back/topspin).

**Nothing ever hits anything.** You are probably using `EmptyWorld`. Implement
`pon::World` against your geometry.

**The `state()` reference went stale.** It is only guaranteed valid until the
next `step`. Re-fetch it each frame; do not cache the reference across steps.

**Ids came back that I had already despawned.** Slots are recycled. Do not hold
ids for dead projectiles; check `state(id).alive` or track lifetime via events.

**Which `dt` should I pass to `step`?** Your frame delta. The Sim sub-steps
internally at `SimConfig::fixedStep_s` (default 1 ms) regardless, so a large or
variable frame `dt` stays stable — but for deterministic replay keep it fixed.

**Threading.** A `Sim` is not internally synchronised. One thread per `Sim` is
fine; the engine adapter shards several `Sim`s across a job system, and inside a
`Sim` `SimConfig::batchIntegrator` advances same-type volleys as an SoA group
(see [Batch integrator and trajectory cache](#batch-integrator-and-trajectory-cache)).

**Can I change the environment mid-flight?** Yes — `sim.environment()` is
mutable between `step` calls (wind, gravity, air density).
