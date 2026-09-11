# poncelet

A realistic, very-low-latency **projectile & terminal-ballistics** library for
games and simulations. Engine-agnostic, MIT-licensed, SI units, C++17 with a
stable C ABI.

Named after **Jean-Victor Poncelet**, whose `a + b·v²` projectile-resistance
law (1829) the terminal-ballistics penetration model is built on.

Covers the whole arc of a thing thrown or fired through a world:

1. **Exterior ballistics** — gravity, aerodynamic drag (G1/G7 `Cd(Mach)`,
   constant-`Cd`, per-ball-type profiles, custom curves), ISA air density,
   wind, spin/Magnus. Drop *emerges* from integrating real drag + gravity —
   never a stored "drop rate".
2. **Precise hit detection** — per-sub-step swept queries against the caller's
   geometry, exact fractional time-of-impact, conservative advancement, three
   cost tiers (`Hitscan` / `AnalyticDrag` / `Integrated`).
3. **Terminal ballistics** — ricochet, ballistic-limit velocity, Poncelet
   penetration depth, residual velocity, penetration channels, water entry,
   membrane/brittle/fragile fast paths.

## Documentation

- **[docs/guide.md](docs/guide.md)** — concepts, build/integration, an annotated
  walkthrough, and a reference entry *with an example* for every public call.
- **[docs/cookbook.md](docs/cookbook.md)** — short copy-pasteable recipes:
  real-dope sniper zero + drop table, shotgun spread, grenade-launcher arc,
  APFSDS vs plate, arrow, aim-UI trajectory line, server-authoritative hit checks.
- **API reference** — Doxygen HTML from the header comments, published to GitHub
  Pages by `.github/workflows/docs.yml`. Build it locally with
  `cmake --build build --target poncelet_docs` (→ `docs/api/html/index.html`)
  or `doxygen docs/Doxyfile`.
- **[docs/examples/](docs/examples/)** — small complete programs, built by the
  standalone build so they cannot rot:
  [`basic_shot.cpp`](docs/examples/basic_shot.cpp),
  [`custom_world.cpp`](docs/examples/custom_world.cpp) (implementing `pon::World`),
  [`curveball.cpp`](docs/examples/curveball.cpp) (Magnus / `BallProfile`),
  [`wind_and_media.cpp`](docs/examples/wind_and_media.cpp) (wind field + media),
  [`precision_long_range.cpp`](docs/examples/precision_long_range.cpp) (spin drift / Coriolis / RKF45),
  [`terminal_ballistics.cpp`](docs/examples/terminal_ballistics.cpp) (ricochet / penetration / layered targets),
  [`catalog_shot.cpp`](docs/examples/catalog_shot.cpp) (named rounds + a game's CSV override),
  [`bullet_cam.cpp`](docs/examples/bullet_cam.cpp) (trajectory-cache replay),
  [`c_api_shot.c`](docs/examples/c_api_shot.c).
- **[bindings/](bindings/)** — engine-binding samples: a Godot 4 GDExtension
  (`PonceletSim` node), a Unity native plugin (C# P/Invoke over the C ABI),
  and an Unreal Engine plugin (`UPonceletSimComponent` over the C++ API).
  Each is a starting point, not a finished addon; none are built by CI.
- **[docs/examples/web/](docs/examples/web/)** — poncelet cross-compiled to
  WebAssembly, `pon::preview_arc` driving a browser `<canvas>` trajectory
  from two sliders. Own build script (needs an Emscripten SDK); not part of
  the standalone CMake build.
- **[data/](data/)** — the shipped default tables (projectile catalog, class
  defaults, ball profiles, materials) as editable CSV with their sources.

## Status — v1.0.0

Feature-complete and validated. The exterior integrator, G1/G7 + `BallProfile`
drag, environment / wind, precision effects, swept hit detection, terminal
ballistics, the named data tables, the portable SoA batch integrator +
trajectory cache, and the CI validation + throughput suite are all in
(`tests/` `validation_` group, `poncelet_perf_gate`). The public API is
stable.

**6-DOF flight** (`PrecisionFlag::SixDOF`) — full rigid-body angular flight,
so tumbling, the yaw of repose, spin drift, coning and transonic instability
are emergent rather than stored curves (`ProjectileType::aero`,
`docs/examples/sixdof_flight.cpp`). Its moment coefficients are seeds; tune
per round against range data.

**Explosive warheads** (`ProjectileType::warhead`, `<poncelet/explosion.hpp>`):
a fuze (contact / delayed / timed-airburst / proximity) drives an
`EventType::Detonated`, and a standalone `pon::Burst` gives the Friedlander
overpressure field, impulse + torque on rigid bodies, line-of-sight cover,
and underwater (Cole) / thermobaric variants (`docs/examples/explosive_warhead.cpp`).
The blast model is engineering fits (Kinney & Graham, Cole) with ~10–20 %
spread — seeds, not lab data — and never touches the trajectory core.

**Fragmentation** (`<poncelet/fragmentation.hpp>`): `WarheadDesc::fragmentation`
turns a detonation into a casing break-up — a Mott mass spectrum launched at
Gurney velocity, sprayed isotropically / in a cone / in a cylindrical fragment
beam, each fragment its own projectile with its own drag and penetration
(`pon::generate_fragments` / `pon::spawn_fragments`,
`docs/examples/fragmentation.cpp`). Pure deterministic generator, off the
flight path; the mass/velocity fits are seeds.

**Shaped charges / EFP** (`<poncelet/shapedcharge.hpp>`): `WarheadDesc::shapedCharge`
turns a detonation into a HEAT jet or an explosively formed penetrator.
`pon::shaped_charge_formation` gives the Birkhoff/PER collapse (tip/tail
velocity, jet vs slug mass, coherent length, breakup standoff);
`pon::shaped_charge_penetration` the standoff-dependent hydrodynamic depth
`P = L_eff·√(ρ_j/ρ_t)` with a strength cut, the perforation / back-face-spall
verdict against a plate, and the residual jet + spall as
`pon::shaped_charge_behind_armour` debris ready for `pon::spawn_fragments`
(`docs/examples/shaped_charge.cpp`). A ConicalJet peaks at a few charge
diameters of standoff and dies past jet particulation; an EFP is shallow but
holds to hundreds of CD. Engineering fits, off the flight path — coefficients
are seeds.

**Guided munitions** (`<poncelet/guidance.hpp>`): a generic per-shot
external-acceleration hook (`Sim::setExternalAccel`) for thrust / scripted
course correction / a custom law, plus a built-in guidance law on
`ProjectileType::guidance` — pursuit, proportional navigation
(`a = N·Vc·λ̇`), or augmented PN. Feed the seeker track each frame with
`Sim::guide(id, targetPos, targetVel)`; the Sim runs the law, clamps to the
airframe g-limit, adds an optional axial sustainer, and steers through the
same hook (`docs/examples/guided_missile.cpp`). The command is evaluated once
per `step()` and is deterministic, but — unlike the integrator core — it is
not yet folded into the Q32.32 fixed-point path, so a guided shot under
`BitExact` is deterministic per platform, not cross-platform bit-identical.

**Destruction coupling**: `Event::impulse_Ns` on an impact/detonation event
and a `pon::Burst` blast field feed a host destruction system — impulse over
a fracture threshold, chunk-scatter biased away from the blast
(`docs/examples/destruction_coupling.cpp`).

Pushing a `v*` tag runs `.github/workflows/release.yml`: the 3-OS matrix
builds + tests, then attaches the static/shared libs, `dist/poncelet_single.hpp`
and a zipped Doxygen reference to a GitHub Release.

Tracked follow-ups (called out in the guide where relevant): explicit 4/8-wide
SIMD intrinsics (the portable SoA batch path is in) and extending the
fixed-point core to the guidance law and the RKF45 step-size controller. The
G1/G7 drag tables are the full-resolution BRL/McCoy standard curves (JBM
`mcg1.txt` / `mcg7.txt`); for match-grade sub-Mach-1 work supply a
Doppler-derived `CustomCurve`, since a single-BC standard-projectile model
can't track an individual bullet's post-transonic drag rise.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Options: `-DPONCELET_SHARED=ON` (also build a shared lib),
`-DPONCELET_BUILD_TESTS=OFF`, `-DPONCELET_BUILD_BENCH=OFF`,
`-DPONCELET_BUILD_EXAMPLES=OFF`. See
[docs/guide.md § Building against poncelet](docs/guide.md#building-against-poncelet)
for `add_subdirectory` / `FetchContent` / `find_package` integration.

Or vendor one file: `dist/poncelet_single.hpp` is the whole library amalgamated
(`#define PONCELET_SINGLE_IMPLEMENTATION` in one TU). See
[docs/guide.md § Single-header](docs/guide.md#single-header).

## Use

```cpp
#include <poncelet/poncelet.hpp>

pon::Sim sim;

// A named round straight from the catalog (data/projectiles.csv) — the string
// id *is* the round. Or fill a ProjectileType by hand; `klass` seeds any gap.
auto id = sim.registerType(pon::catalog::get("9x19_124gr_fmj"));

pon::LaunchParams shot;
shot.position = {0, 1.6, 0};
shot.direction = {1, 0, 0};
auto h = sim.spawn(id, shot);            // speed defaults to the type's muzzle speed

pon::EmptyWorld world;                    // caller supplies real geometry queries
pon::VectorEventSink events;
sim.step(1.0 / 60.0, world, events);
```

A C ABI (`<poncelet/poncelet.h>`) mirrors this for non-C++ callers.

## Determinism

`pon::config::Determinism::PlatformStable` (default) — identical trajectories
for the same platform + compiler + inputs. `BitExact` runs the integrator on a
Q32.32 fixed-point core (deterministic sqrt + transcendental LUTs + fixed-point
drag sampling), so the trajectory folds to the same bits on every OS / compiler
/ optimisation level. The per-type drag LUT is still compiled in double at
`registerType()`; the guidance law and the RKF45 step-size controller are not
yet fixed-point (see the guide).

`Sim::stateHash()` is a 64-bit FNV-1a digest of every live shot's full state
(hashed field by field, no struct padding) — sample it per frame for a rollback
desync check, and `Sim::snapshot()` / `Sim::restore()` are what you roll back
*with*: serialize/restore every live shot's full state, the RNG cursor and the
trajectory cache (see the guide's "Rollback netcode" section and cookbook § 7).
`poncelet_bench --det-check` (ctest `poncelet_det_check`) runs a
fixed mixed-tier scene twice and asserts a bit-identical per-frame digest.
`poncelet_bench --bitexact-golden` (ctest `poncelet_bitexact_golden`) folds a
fixed `BitExact` scene into a digest and compares it to a committed golden; CI
runs both on every OS/config, so cross-platform bit-exactness is guarded.

## License

MIT — see `LICENSE`.
