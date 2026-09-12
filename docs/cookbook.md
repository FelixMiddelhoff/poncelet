# poncelet — cookbook

Short, copy-pasteable recipes. Each assumes:

```cpp
#include <poncelet/poncelet.hpp>
using namespace pon;
```

and leaves out error handling you would keep in real code (`registerType` /
`spawn` returning an invalid handle — call `sim.lastError()`). For the full
picture of any type or call, see [guide.md](guide.md).

Contents:

1. [Real-dope sniper rifle — zero + a drop table](#1-real-dope-sniper-rifle)
2. [Shotgun — N pellets with dispersion](#2-shotgun)
3. [Grenade launcher — arc indicator + contact fuze](#3-grenade-launcher)
4. [APFSDS vs an RHA plate](#4-apfsds-vs-an-rha-plate)
5. [Arrow — gravity only, no BC](#5-arrow)
6. [Aim-UI trajectory line](#6-aim-ui-trajectory-line)
7. [Netcode — server-authoritative hit validation](#7-netcode)
8. [6-DOF rigid-body flight](#8-6-dof-rigid-body-flight)
9. [Fragmentation spray](#9-fragmentation-spray)
10. [Shaped charge / EFP](#10-shaped-charge--efp)
11. [Guided munition (proportional navigation)](#11-guided-munition-proportional-navigation)
12. [Destruction coupling](#12-destruction-coupling)

---

## 1. Real-dope sniper rifle

A catalog round, zeroed at 100 m by solving for launch pitch, then a drop table
in MOA. poncelet has no built-in zeroing — it is a few lines of caller code
around `preview_arc` (no `Sim` needed).

```cpp
const ProjectileType round = catalog::get("762x51_175gr_smk");
Environment env;                               // sea-level ISA, no wind
const Vec3 muzzle{0, 1.6, 0};                  // scope-over-bore folded into the zero

// Landing height at range `x` for a given launch pitch (radians).
auto drop_at = [&](Real pitch, Real x) {
    LaunchParams lp;
    lp.position  = muzzle;
    lp.direction = {std::cos(pitch), std::sin(pitch), 0};
    std::vector<Vec3> arc;
    preview_arc(round, lp, env, /*dt*/ 0.002, /*maxTime*/ 4.0, arc);
    for (std::size_t i = 1; i < arc.size(); ++i)
        if (arc[i].x >= x) {                    // linear-interp the crossing
            const Real f = (x - arc[i-1].x) / (arc[i].x - arc[i-1].x);
            return arc[i-1].y + f * (arc[i].y - arc[i-1].y);
        }
    return arc.empty() ? muzzle.y : arc.back().y;
};

// Bisect launch pitch so point-of-impact == line-of-sight height at 100 m.
Real lo = 0.0, hi = moa(20.0);
for (int i = 0; i < 40; ++i) {
    const Real mid = 0.5 * (lo + hi);
    (drop_at(mid, 100.0) < muzzle.y ? lo : hi) = mid;
}
const Real zeroPitch = 0.5 * (lo + hi);

// Drop table: come-up in MOA past the zero.
for (Real range : {200.0, 300.0, 400.0, 600.0, 800.0, 1000.0}) {
    const Real dropBelowLoS = muzzle.y - drop_at(zeroPitch, range);
    std::printf("%5.0f m : %+.1f MOA\n", range,
                to_moa(std::atan2(dropBelowLoS, range)));
}
```

`preview_arc` runs the closed-form `AnalyticDrag` model — within a few percent of
the `Integrated` tier for a flat shot, and fine for a dope card. For sub-MOA
agreement with your in-game shots, spawn into a throwaway `Sim` at
`FidelityTier::Integrated` with the precision flags you ship (see recipe 6's
note) and read `state(h).position` at each range instead.

---

## 2. Shotgun

One `fire()` per pellet, spread with `LaunchParams::precisionMrad`. The spread is
drawn from the `Sim`'s seeded RNG and advanced per spawn, so the same shot
reproduces on a rollback.

```cpp
ProjectileType pellet;
pellet.id             = "00buck_pellet";
pellet.klass          = ProjectileClass::Bullet;
pellet.dragModel      = DragModel::ConstantCd;
pellet.dragCoefficient = 0.47;                  // ~sphere
pellet.mass_kg        = grains(53.8);           // one 00-buck ball
pellet.refDiameter_m  = inches(0.33);
pellet.muzzleSpeed_mps = fps(1325);

Sim sim;
const Vec3 muzzle{0, 1.5, 0}, aim{1, 0, 0};
const TypeId kPellet = sim.registerType(pellet);   // once — fire() can't carry precisionMrad

for (int i = 0; i < 9; ++i) {
    LaunchParams lp;
    lp.position      = muzzle;
    lp.direction     = aim;
    lp.tier          = FidelityTier::Integrated;
    lp.precisionMrad = 35.0;                        // ~2° cone → ~0.7 m pattern at 20 m
    sim.spawn(kPellet, lp);
}
```

The nine pellets take nine consecutive draws from the `Sim`'s RNG stream, so the
pattern is fixed for a given `SimConfig::rngSeed` and spawn order — do not reset
the seed per pellet or every pellet gets the same offset. `fire()` is the
one-liner for the *unspread* case (`sim.fire(pellet, muzzle, aim)`).

---

## 3. Grenade launcher

A lobbed 40 mm round: show the predicted arc while aiming, then fire a
contact-fuzed HE shell and build the blast off the `Detonated` event.

```cpp
ProjectileType he;
he.id              = "40mm_he";
he.klass           = ProjectileClass::Shell;
he.dragModel       = DragModel::ConstantCd;
he.dragCoefficient = 0.30;
he.mass_kg         = 0.230;
he.refDiameter_m   = inches(1.57);
he.muzzleSpeed_mps = 76.0;
he.warhead.chargeMass_kg = 0.048;              // Comp-B fill
he.warhead.tntEquivalence = 1.1;
he.warhead.fuze    = FuzeMode::Contact;
he.warhead.surfaceBurst = true;               // ground burst ≈ 1.8× free-air

Sim sim;
const Vec3 muzzle{0, 1.5, 0};

// --- while aiming: the indicator polyline ---
LaunchParams aim;
aim.position  = muzzle;
aim.direction = aimDir;
std::vector<Vec3> arc;
preview_arc(he, aim, sim.environment(), 0.05, 8.0, arc, /*groundY*/ 0.0);
draw_polyline(arc);                            // your renderer

// --- on trigger ---
const StateId h = sim.fire(he, muzzle, aimDir);
// ... each frame:
sim.step(dt, world, sink);
for (const Event& e : sink.events)
    if (e.type == EventType::Detonated) {
        Burst burst(e.point, he.warhead, sim.environment(), e.time_s);
        double p = burst.overpressureAt(playerPos, now);   // shockwave FX / damage
    }
```

For a fragment spray, add a `FragmentationDesc` and call
`generate_fragments` / `spawn_fragments` off that same `Detonated` event —
see [recipe 9](#9-fragmentation-spray).

---

## 4. APFSDS vs an RHA plate

Terminal ballistics is per-impact: your `World::material()` returns a `Material`
with a `MaterialBehaviour`, and `step()` emits `Perforated` / `Embedded` /
`Ricochet` / `Stopped`. A single flat target is a two-liner with
`<poncelet/worlds.hpp>` — no bespoke `World` to write:

```cpp
#include <poncelet/worlds.hpp>

PlaneWorld world;
world.position = {5, 0, 0};             // plane at x = 5 m
world.normal   = {-1, 0, 0};
world.mat.behaviour    = MaterialBehaviour::Ductile;   // metal
world.mat.density_kgm3 = 7850.0;
world.mat.strength_Pa  = 1.0e9;                         // ~RHA flow stress
world.mat.thickness_m  = 0.30;                          // 300 mm plate

ProjectileType dart;
dart.id             = "120mm_apfsds";
dart.klass          = ProjectileClass::Bullet;           // long rod
dart.dragModel      = DragModel::ConstantCd;
dart.dragCoefficient = 0.25;
dart.mass_kg        = 4.6;                               // penetrator only
dart.refDiameter_m  = 0.022;
dart.muzzleSpeed_mps = 1650.0;
dart.terminal.noseShapeFactor = 0.35;                    // sharp
dart.terminal.hardness = 3.0;                            // tungsten/DU rod

Sim sim;
VectorEventSink sink;
const StateId h = sim.fire(dart, {0, 0, 0}, {1, 0, 0});
for (int i = 0; i < 200 && sim.state(h).alive; ++i) {
    sim.step(1.0 / 500.0, world, sink);
    for (const Event& e : sink.events)
        if (e.type == EventType::Perforated) std::puts("through");
        else if (e.type == EventType::Stopped) std::puts("stopped in plate");
    sink.events.clear();
}
```

The penetration model uses Poncelet α/β seeds — plausible, not a laboratory
solution; tune `strength_Pa` / `noseShapeFactor` / `hardness` against your own
reference table.

Layered armor (spaced plate, a stack of boards) is `SlabStackWorld` instead —
one shared axis, a `Material` per slab; see `docs/examples/stock_worlds.cpp`
and [guide.md § The World callback](guide.md#the-world-callback).

---

## 5. Arrow

No ballistic coefficient — a fixed `Cd` on the shaft area, and gravity does the
rest. `ProjectileClass::Arrow` seeds the drag and (under `SixDOF`) a
statically-stable fin set.

```cpp
ProjectileType arrow;
arrow.id              = "longbow_bodkin";
arrow.klass           = ProjectileClass::Arrow;
arrow.dragModel       = DragModel::ConstantCd;
arrow.dragCoefficient = 2.0;                    // on the shaft frontal area
arrow.mass_kg         = grains(430);            // ~28 g war arrow
arrow.refDiameter_m   = inches(0.32);
arrow.muzzleSpeed_mps = 55.0;
arrow.terminal.focBias = 0.12;                  // front-of-centre

const StateId h = sim.fire(arrow, {0, 1.7, 0},
                           {std::cos(degrees(40)), std::sin(degrees(40)), 0});
```

Everything else defaulted. `describe(sim.state(h))` gives a one-line readout for
tuning the launch.

---

## 6. Aim-UI trajectory line

The minimal case of recipe 3 — no `Sim`, no `World`, just a polyline.

```cpp
LaunchParams aim;
aim.position  = cameraMuzzle;
aim.direction = crosshairDir;

std::vector<Vec3> arc;                          // reused across frames
preview_arc(loadedRound, aim, env,
            /*dt*/ 0.04, /*maxTime*/ 6.0, arc,
            /*groundY*/ terrainHeightBelowAim);

// arc[0] == aim.position; the last point is at groundY (or maxTime / max range).
render_line_strip(arc);
```

`preview_arc` compiles a throwaway drag LUT per call (~microseconds) — fine once
a frame for the local player, not for thousands of NPCs. It carries drag +
gravity + wind but not spin drift / Coriolis; for those, or for exact agreement
with the shot you will actually spawn, integrate a hidden `Sim` shot instead.

---

## 7. Netcode

Server-authoritative hit validation: the server replays the client's shot under
`BitExact` and compares `stateHash()` each frame. The Q32.32 fixed-point core
makes the digest identical across OS / compiler / optimisation level.

```cpp
SimConfig cfg;
cfg.determinism = config::Determinism::BitExact;
cfg.fixedStep_s = 1.0 / 128.0;                  // match the client tick
Sim sim(clientEnvironmentSnapshot, cfg);

const TypeId id = sim.registerType(clientRound);
LaunchParams lp;
lp.position  = shot.muzzle;                     // from the validated client packet
lp.direction = shot.aimDir;
lp.speed     = shot.muzzleSpeed;
lp.tier      = FidelityTier::Integrated;
// lp.precisionMrad stays 0 — dispersion is PlatformStable only; the client
// sends the already-dispersed aimDir instead.
const StateId h = sim.spawn(id, lp);

ServerWorld world;                              // the server's authoritative geometry
VectorEventSink sink;
for (int tick = 0; tick < shot.ticksToImpact; ++tick) {
    sim.step(cfg.fixedStep_s, world, sink);
    if (sim.stateHash() != shot.clientHashes[tick]) { reject(shot); return; }
}
for (const Event& e : sink.events)
    switch (e.type) {                                 // trust the server outcome
        case EventType::Embedded:
        case EventType::Perforated:
        case EventType::Stopped:     award_hit(e); break;
        default: break;
    }
```

Ship the compiled drag LUT (or pin the `registerType` inputs) so the one-time
`double` LUT compile is identical on both ends — see
[guide.md § Determinism](guide.md#determinism).

**Client-side rollback**, the other half: predict ahead on local input, and
when a correction arrives for an earlier tick, roll back and re-simulate
instead of re-simulating from scratch.

```cpp
std::map<int, std::vector<std::byte>> history; // tick -> snapshot taken BEFORE that tick's step

int tick = 0;
// each frame:
history[tick] = sim.snapshot();
sim.step(cfg.fixedStep_s, world, sink);         // predicted on the local input
++tick;

// a correction arrives for `correctedTick` (< tick):
void on_server_correction(int correctedTick, const Input& corrected) {
    sim.restore(history[correctedTick].data(), history[correctedTick].size());
    inputs[correctedTick] = corrected;          // replace the mispredicted input
    for (int t = correctedTick; t < tick; ++t) {
        history[t] = sim.snapshot();
        sim.step(cfg.fixedStep_s, world, sink); // re-simulate with the corrected input
    }
}
```

Prune `history` older than your max rollback window (a few hundred ms of
ticks). See [guide.md § Rollback netcode](guide.md#rollback-netcode).

---

## 8. 6-DOF rigid-body flight

`PrecisionFlag::SixDOF` integrates orientation + angular velocity alongside
the trajectory — tumbling, yaw of repose (and the spin drift it causes), and
coning all emerge from the aerodynamics instead of being canned curves. The
one question this answers in 20 seconds: *why does my round tumble?*

```cpp
ProjectileType t;
t.id                   = "762x51_175smk";
t.dragModel            = DragModel::G7;
t.ballisticCoefficient = 0.243;
t.mass_kg              = 0.01134;
t.refDiameter_m        = 0.00782;
t.twistRate_m          = 0.254;             // 1-in-10", right-hand — the stabilizing spin
// t.aero left at 0 — length/inertia/moment coefficients seed from mass +
// diameter + class; tune per round against real data for a shipping title.

LaunchParams shot;
shot.tier      = FidelityTier::Integrated;
shot.precision = PrecisionFlag::SixDOF;
shot.initialYaw = degrees(2.0);             // a little muzzle tip-off
shot.spin      = 2.0 * 3.14159265 * shot.speed / t.twistRate_m; // barrel-derived — 0 tumbles
```

Same round, two ways: with the barrel-twist spin above it stays point-first
(peak yaw ~3°, keeps ~86% of its spin) and the yaw of repose walks it a few
tenths of a metre right over a long shot; `shot.spin = 0` tumbles end over
end (peak yaw 180°, `ProjectileState::flags & kFlagTumbling` sets) within a
few hundred metres. See `docs/examples/sixdof_flight.cpp` for the full
three-way comparison this recipe is drawn from.

---

## 9. Fragmentation spray

`FragmentationDesc` + `generate_fragments` + `spawn_fragments` turn one
warhead detonation into hundreds of independently-simulated projectiles —
the Mott-spectrum casing break-up recipe 3 already points at without
showing.

```cpp
FragmentationDesc& frag = bomb.warhead.fragmentation;
frag.casingMass_kg         = 3.0;
frag.gurneyVelocity_mps    = 2440.0;        // TNT
frag.casingInnerDiameter_m = 0.070;
frag.casingWallThickness_m = 0.008;
frag.spray                 = FragmentSpray::Cone;
frag.sprayAxis             = {0, -1, 0};    // straight down — a nose-down airburst
frag.coneHalfAngle_rad     = 0.70;          // ~40°
frag.maxFragments          = 300;

std::vector<FragmentSpec> spray;
const std::size_t nf = generate_fragments(frag, burstPoint, bombVel,
                                          bomb.warhead.chargeMass_kg, spray);
std::vector<StateId> ids;
spawn_fragments(sim, spray, FidelityTier::Integrated, "frag", frag.massClasses, &ids);
// then step() as normal — each fragment is a full projectile: hits, perforates,
// ricochets, same as any other round.
```

An 81 mm mortar burst 6 m up throws ~300 fragments at 1.1–1.5 km/s; aimed
straight down over a target, essentially all of them land within 15 m. See
`docs/examples/fragmentation.cpp` — skip its custom flat-ground `World` here
and reuse `worlds.hpp`'s stock ones (recipe 4's `PlaneWorld`) or your own.

---

## 10. Shaped charge / EFP

`ShapedChargeDesc` + `shaped_charge_formation` / `shaped_charge_penetration`
model jet formation from a HEAT warhead and the standoff-dependent
penetration — there is a real optimum standoff, not "more is always better".

```cpp
ShapedChargeDesc heat;
heat.linerMass_kg      = 0.35;              // copper cone
heat.chargeDiameter_m  = 0.100;
heat.kind              = ShapedChargeType::ConicalJet;
heat.coneApexAngle_rad = degrees(60.0);
heat.gurneyVelocity_mps = 2930.0;           // Octol
const double fill_kg    = 0.6;              // explosive behind the liner

const JetFormation jet = shaped_charge_formation(heat, fill_kg);
// jet.tipVelocity_mps / jetMass_kg / breakupStandoff_m — a 100 mm HEAT liner
// forms a ~7300 m/s tip, coherent for tens of centimetres past breakup.

Material rha;                               // same Material as recipe 4's world.mat
rha.behaviour = MaterialBehaviour::Ductile;
rha.density_kgm3 = 7850.0; rha.strength_Pa = 1.0e9; rha.thickness_m = 0.060;

const double optS = shaped_charge_optimal_standoff(heat, fill_kg);
const auto pen = shaped_charge_penetration(heat, rha, optS, rha.thickness_m, fill_kg);
// pen.depth_m / pen.perforated / pen.spall / pen.standoffEfficiency
```

The 100 mm / 60 mm RHA case in `docs/examples/shaped_charge.cpp` finds its
best penetration (~440 mm, well past the plate) around 0.6 m standoff
(~5.7 CD) — closer *or* farther is worse, the counterintuitive result a
reader wouldn't guess from the struct fields alone. Swap `heat.kind` to
`ShapedChargeType::EFP` for a shallower-but-long-ranged variant (the same
example's second half: ~90 mm at 1 m standoff, still ~65 mm at 50 m).
Perforation or spall hands back debris via `shaped_charge_behind_armour` —
spawn it the same way recipe 9 spawns a fragmentation spray.

---

## 11. Guided munition (proportional navigation)

`GuidanceDesc` + `Sim::guide()` — the built-in pursuit / proportional-nav /
augmented-PN laws, and *why* PN specifically: it leads a crossing target,
where pursuit (steer straight at the target's *current* position) trails
and misses.

```cpp
ProjectileType sam;
sam.klass = ProjectileClass::Shell;
sam.guidance.law               = GuidanceLaw::ProportionalNav;
sam.guidance.navConstant       = 4.0;        // N; 3-5 typical
sam.guidance.maxLateralAccel_g = 40.0;       // airframe limit
sam.guidance.thrustAccel_mps2  = 250.0;      // sustainer
sam.guidance.burnTime_s        = 2.0;

const StateId m = sim.spawn(id, lp);
// each frame, feed the seeker track:
sim.guide(m, targetPos, targetVel);
sim.step(dt, world, sink);
```

Against a 260 m/s crossing jet (`docs/examples/guided_missile.cpp`), PN
converges to a ~3 m miss; pure pursuit misses by ~185 m; unguided (no
`guide()` call) misses by over 1 km — three numbers make the "why PN" case
better than prose. Guidance runs on the same `BitExact` fixed-point core as
the rest of the trajectory (cross-platform CI-verified), so a guided round
is exactly as reproducible for netcode/rollback (recipe 7) as an unguided
one.

---

## 12. Destruction coupling

`Event::impulse_Ns` and `Burst::loadOnBody` are the two seams a game's own
fracture/chunk system hooks into — a kinetic perforation and a blast,
respectively, both delivering a `Vec3` impulse your own physics applies.

```cpp
// Kinetic: a bond that fails past an impulse budget.
for (const Event& e : sink.events)
    if (e.type == EventType::Perforated || e.type == EventType::Embedded ||
        e.type == EventType::Stopped) {
        block.accumulated_Ns += length(e.impulse_Ns);
        if (block.accumulated_Ns > block.impulseBudget_Ns) detach(block);
    }

// Blast: Burst already has the physics — loadOnBody hands back an impulse +
// deltaVelocity for YOUR rigid-body/fracture system, given a target shape
// and the fraction of it that's actually exposed to the burst.
Burst burst(detonationPoint, warhead, sim.environment(), e.time_s);
BlastTarget t;
t.centroid = block.centre;
t.area_m2  = 0.16;                          // presented face area
t.mass_kg  = 12.0;                          // 0 => impulse only, no deltaVelocity
const Real los = blast_line_of_sight(world, detonationPoint, block.centre);
const BlastLoad load = burst.loadOnBody(t, los);
block.accumulated_Ns += length(load.impulse_Ns);
apply_to_your_physics(block, load.impulse_Ns, load.deltaVelocity_mps);
```

`docs/examples/destruction_coupling.cpp`'s wall: a single rifle round's
perforation impulse (~9 N·s) knocks loose exactly the one block it hits; a
grenade 1.5 m off — far more energetic, spread over the whole wall via
`blast_line_of_sight`'s per-block occlusion check — brings down roughly half
the wall without levelling all of it. This is exactly the seam a game engine
wires into its own chunk/fracture system, not a poncelet feature to
configure further.
