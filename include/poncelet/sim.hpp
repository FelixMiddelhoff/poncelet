// poncelet — the simulation front end: register types, spawn shots, step.
// SPDX-License-Identifier: MIT
//
// Item 1 ships a deliberately thin core: type registry + spawn + a
// fixed-step semi-implicit Euler advance with gravity and (if the type has a
// dragCoefficient) constant-Cd drag, plus per-step swept World queries that
// stop the projectile on the first hit. The real integrator (semi-implicit +
// RK4 + analytic fast path + adaptive sub-stepping) is WORKPLAN item 2;
// G1/G7, Magnus, media and terminal ballistics follow in items 3-8.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/projectile.hpp"
#include "poncelet/environment.hpp"
#include "poncelet/drag.hpp"
#include "poncelet/world.hpp"
#include "poncelet/terminal.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pon {

using StateId = std::uint32_t;
constexpr StateId kInvalidState = 0xFFFFFFFFu;

// --- Optional per-shot diagnostic trace (SimConfig::traceSink) --------------
//
// Off by default and zero-cost when the sink is null. When set, step() reports
// what each live shot is doing so "why did that shot behave like that" is a log
// line, not a debugger session. Read-only — a trace callback never changes the
// simulation, so a traced run folds to the same stateHash() as an untraced one.
enum class TraceKind {
    Frame,          // once per live shot per step(): i0 = FidelityTier, r0 = speed m/s
    Substep,        // adaptive sub-step budget used this frame: i0 = peak sub-step count
    TransonicEnter, // shot's Mach entered the 0.8..1.2 band (needs PrecisionFlag::TransonicFlag)
    TransonicExit,  // ...and left it
    MediumChanged,  // crossed into a new medium: i0 = new MediumId
    GuidanceLost,   // seeker track dropped — the shot now coasts ballistically
};

struct TraceEvent {
    TraceKind    kind;
    StateId      shot;
    Seconds      time_s;    // shot's timeAlive_s
    Vec3         position;
    Vec3         velocity;
    std::int32_t i0 = 0;    // kind-specific integer (see TraceKind)
    Real         r0 = 0;    // kind-specific real    (see TraceKind)
};

using TraceSink = std::function<void(const TraceEvent&)>;

// Which fixed-step scheme the Integrated tier runs. RK4 is the accurate
// default; SemiImplicit (symplectic Euler) is cheaper and conserves energy
// better over very long flights.
enum class Integrator { RK4, SemiImplicit };

// One point on a shot's flight path, recorded once per frame into the
// trajectory cache (SimConfig::trajectoryCacheFrames > 0). Bullet-cam replays
// and late-join clients reuse the stored polyline instead of re-integrating
// (docs/ballistics-phase-plan.md "v1 perf pass").
struct TrajectorySample {
    Vec3    position;
    Vec3    velocity;
    Seconds time_s;   // ProjectileState::timeAlive_s at this sample
};

struct SimConfig {
    config::Determinism determinism = config::Determinism::PlatformStable;
    Seconds             fixedStep_s = 1.0 / 1000.0; // nominal Integrated sub-step
    Integrator          integrator  = Integrator::RK4;
    // Adaptive sub-stepping (Integrated tier): the nominal step is subdivided
    // further when the low-order position error would exceed `posTolerance_m`
    // or a sub-step would travel more than `maxSubstepDist_m` (anti-tunnelling),
    // never past `maxSubsteps` subdivisions of one nominal step.
    Real                posTolerance_m   = 0.02;
    Real                maxSubstepDist_m = 1.0;
    std::uint32_t       maxSubsteps      = 64;
    // AnalyticDrag tier: target arc length between polyline samples. Coarser is
    // cheaper; the swept query between samples still catches anything on the
    // segment, only the chord-vs-arc sag is lost (negligible for flat shots).
    Real                analyticSampleDist_m = 2.0;
    std::uint64_t       rngSeed     = 0x9E3779B97F4A7C15ull;

    // --- v1 perf pass (WORKPLAN item 12) ---
    //
    // Portable SoA batch integrator. When set, step() groups live Integrated-
    // tier shots that share a type and are on the plain fast path (still air,
    // no wind, no Magnus / shaped drag, no precision flags) and advances the
    // group in lockstep: the force model is built once per group per block and
    // the sub-step count is the group maximum. Bit-for-bit it can differ from
    // the per-shot path only where a slow lane is carried at a finer step than
    // it would pick alone (strictly more accurate, still within posTolerance_m);
    // it is deterministic. Off by default — a pure perf lever.
    bool                batchIntegrator = false;
    // A group smaller than this many shots is advanced per-shot instead (the
    // grouping bookkeeping is not worth it for a handful).
    std::uint32_t       batchMinGroup   = 8;

    // Trajectory cache: keep the last N per-frame samples of every live shot's
    // flight path (0 = off). Read back with trajectory() / sampleTrajectory().
    std::uint32_t       trajectoryCacheFrames = 0;

    // Optional per-shot diagnostic callback (see TraceKind). Null (default) ⇒
    // no tracing, no cost. Called synchronously from step(); it must not touch
    // the Sim. Copied into the Sim at construction like the rest of SimConfig.
    TraceSink           traceSink;
};

// Aggregate per-frame counters the last step() filled — the profiling
// counterpart to SimConfig::traceSink's per-shot view. Near-zero cost
// (increments already implicit in the loop, or one virtual-call wrapper
// around the caller's World / EventSink for the frame); always on.
struct SimStats {
    std::uint32_t liveHitscan    = 0; // live shots on each tier at the start
    std::uint32_t liveAnalytic   = 0; // of this step() (before any expire)
    std::uint32_t liveIntegrated = 0;
    // Sub-step integrations run this frame — every Integrated / SixDOF lane,
    // including SimConfig::batchIntegrator lanes (one count per lane per
    // block, same as the per-shot path would have cost).
    std::uint64_t subSteps      = 0;
    // World::raycast() calls this frame (every swept query, hit or miss).
    std::uint64_t sweptQueries  = 0;
    // SimConfig::batchIntegrator groups formed this frame (0 when the lever
    // is off, or no group reached batchMinGroup).
    std::uint32_t batchGroups   = 0;
    // EventSink::emit() calls this frame.
    std::uint32_t eventsEmitted = 0;
};

class Sim {
public:
    explicit Sim(Environment env = {}, SimConfig cfg = {});

    const Environment& environment() const { return env_; }
    Environment&       environment()       { return env_; }
    const SimConfig&   config() const      { return cfg_; }
    // Counters from the last step() (0s before the first). See SimStats.
    const SimStats&    stats() const       { return stats_; }
    Status             status() const      { return status_; }

    // Install / replace / clear (pass {}) the diagnostic trace callback on a
    // live Sim — the runtime equivalent of SimConfig::traceSink, so a game can
    // arm tracing only when it needs it (e.g. after a netcode desync) and disarm
    // it again without rebuilding the Sim. Read-only: never changes stateHash().
    void setTraceSink(TraceSink sink) { cfg_.traceSink = std::move(sink); }

    // Register a projectile description; missing fields are filled from class
    // defaults. Returns kInvalidType on a bad description — call lastError() for
    // the reason, or pon::validate(type) before registering.
    TypeId registerType(ProjectileType type);
    const ProjectileType& type(TypeId id) const;

    // Spawn one shot. `speed`/`spin` nullopt -> type/catalog defaults. Returns
    // kInvalidState on a bad TypeId or a zero `direction` — lastError() explains.
    StateId spawn(TypeId type, const LaunchParams& launch);

    // One-call opener for the common case. Registers `type` the first time its
    // `id` string is seen on this Sim (the id -> TypeId map is cached; a later
    // fire() or type() with the same id reuses the registration), then spawns
    // one shot from `muzzle` toward `aimDir` at the type / catalog default speed
    // and spin. Returns kInvalidState on a bad description or a zero `aimDir` —
    // lastError() explains. `type.id` must be non-empty (registerType() requires
    // it). The cache is keyed by `id` alone: give the type a fresh id when its
    // fields change, and for per-shot control (speed, spin, precision flags,
    // dispersion, starting medium) build a LaunchParams and call registerType()
    // / spawn() directly.
    StateId fire(const ProjectileType& type, Vec3 muzzle, Vec3 aimDir,
                 FidelityTier tier = FidelityTier::Integrated);

    // A human-readable reason for the most recent registerType() / spawn() that
    // returned an invalid handle. "" when the last such call succeeded. The
    // pointer is to static storage — copy it if you need to keep it.
    const char* lastError() const { return lastError_; }

    // Advance every live projectile by `dt` against `world`, emitting events.
    void step(Seconds dt, const World& world, EventSink& sink);

    // Access / iterate live projectiles.
    const ProjectileState& state(StateId id) const;
    std::size_t            liveCount() const;
    void                   despawn(StateId id);

    // Fill `out` (cleared first) with the id of every live projectile, in slot
    // order — so a caller doesn't have to track every spawn()'s return value
    // itself just to know what's still in the air.
    void liveIds(std::vector<StateId>& out) const;

    // Call `f(id, state)` for every live projectile, in slot order. Same set as
    // liveIds() but without the intermediate id vector + a state() lookup per
    // id when you're just going to visit each one anyway.
    template <class F>
    void forEachLive(F&& f) const {
        for (std::size_t i = 0; i < states_.size(); ++i)
            if (used_[i] && states_[i].alive)
                f(static_cast<StateId>(i), states_[i]);
    }

    // --- Guided munitions (Phase 19 item 5) --------------------------------
    //
    // Update the seeker track for a guided shot (ProjectileType::guidance.law !=
    // None). Call once per frame before step(); the Sim runs the guidance law
    // from this and steers. `targetVel` may be zero for a static aim point.
    Status guide(StateId id, Vec3 targetPos, Vec3 targetVel);
    // Drop the track — the shot coasts (ballistic) from here.
    Status clearGuidanceTarget(StateId id);
    // Generic per-step external acceleration (m/s², world frame) added to the
    // force model every sub-step until changed — thrust, a scripted course
    // correction, or a guidance law the caller computes itself. Independent of
    // guide(); pass {0,0,0} to clear.
    Status setExternalAccel(StateId id, Vec3 accel_mps2);

    // --- Trajectory cache (SimConfig::trajectoryCacheFrames > 0) -------------
    //
    // Copy the cached flight-path samples for `id`, oldest first, into `out`
    // (up to `max`); returns the number written. Empty when the cache is off,
    // `id` never lived, or no frame has stepped yet.
    std::size_t trajectory(StateId id, TrajectorySample* out, std::size_t max) const;
    // Number of samples currently cached for `id`.
    std::size_t trajectorySize(StateId id) const;
    // Replay: position + velocity at flight time `t` (ProjectileState::timeAlive_s
    // units), linearly interpolated between the two bracketing cached samples and
    // clamped at the ends. Returns false when fewer than 2 samples are cached.
    bool sampleTrajectory(StateId id, Seconds t, Vec3& pos, Vec3& vel) const;

    // --- Determinism (config::Determinism) ---------------------------------
    //
    // A 64-bit FNV-1a digest of everything the next step() reads: every live
    // projectile's full dynamic state (position / velocity / spin / orientation
    // / medium / fuze + guidance bookkeeping), the slot-in-use map and the RNG
    // cursor. Hashed field by field (never a struct memcpy — padding is not
    // initialised), live slots in index order.
    //
    // Two Sims fed identical inputs return the same digest after each step on
    // the same build (PlatformStable); once the fixed-point core lands it will
    // also match across builds / platforms (BitExact). Cheap — meant to be
    // sampled per frame by a rollback-netcode desync check, and it backs the
    // `poncelet_bench --det-check` and `validation_` determinism gates.
    std::uint64_t stateHash() const;

    // Per-shot digest: the same field walk as stateHash(), folded over just
    // `id`'s dynamic state — pinpoints *which* live shot desynced instead of
    // only that the frame did. 0 for a dead / never-spawned / out-of-range id
    // (note this is not distinguishable from an astronomically unlikely real
    // digest collision, same caveat as any hash).
    std::uint64_t stateHash(StateId id) const;

    // --- Snapshot / restore (rollback netcode) ------------------------------
    //
    // snapshot() serializes everything the next step() reads: every live
    // projectile's full dynamic state (the same fields stateHash() walks, in
    // the same order), the slot-in-use map, the RNG cursor, and the trajectory
    // cache ring (SimConfig::trajectoryCacheFrames > 0, so a bullet-cam mid-
    // rollback does not glitch). It does NOT serialize the registered types or
    // drag LUTs — those are static content; restore into a Sim with the same
    // types registered in the same order. Deterministic (repeated snapshot()
    // calls on an unstepped Sim are byte-identical) and versioned: a magic +
    // version + type-registry digest header lets restore() reject a mismatched
    // Sim instead of silently corrupting it.
    //
    // snapshot() -> step(N) -> restore(that snapshot) -> step(N) again yields
    // the same stateHash() every frame as the original run.
    std::vector<std::byte> snapshot() const;

    // Restore a snapshot taken from a Sim with an identical type registry
    // (same count, same `id`s in the same order) and the same
    // SimConfig::trajectoryCacheFrames. Returns false — and leaves the Sim
    // untouched — on a truncated buffer or a magic / version / type-registry /
    // trajectory-cache mismatch; returns true and replaces the live/state data
    // otherwise.
    bool restore(const std::byte* data, std::size_t size);

private:
    // Per-tier frame advance for one live projectile (src/sim.cpp). `layer` is
    // the terminal-ballistics chaining depth (a shot that perforates a surface
    // is flown on through the rest of the frame); it bounds the recursion.
    void advanceHitscan(ProjectileState& s, const ProjectileType& t, Seconds dt,
                        const World& world, EventSink& sink, int layer = 0);
    void advanceAnalytic(ProjectileState& s, const ProjectileType& t, Seconds dt,
                         const World& world, EventSink& sink, int layer = 0);
    void advanceIntegrated(ProjectileState& s, const ProjectileType& t, Seconds dt,
                           const World& world, EventSink& sink, int layer = 0);
    // Evaluate the guidance law (if any) + the caller's raw external-accel poke
    // for this frame and return the total acceleration to fold into the force
    // model. Mutates s.guidance bookkeeping; a no-op (returns {0,0,0}) for an
    // unguided shot with no poke.
    Vec3 evalGuidance(ProjectileState& s, const ProjectileType& t, Seconds dt);
    void advanceIntegratedAdaptive(ProjectileState& s, const ProjectileType& t,
                                   Seconds dt, const World& world, EventSink& sink,
                                   int layer);
    // Full 6-DOF rigid-body angular flight (PrecisionFlag::SixDOF). Integrates
    // orientation + body angular velocity alongside position/velocity; yaw of
    // repose, spin drift, coning and tumbling are emergent. Same swept-query /
    // medium / expiry bookkeeping as advanceIntegrated.
    void advanceSixDOF(ProjectileState& s, const ProjectileType& t, Seconds dt,
                       const World& world, EventSink& sink, int layer);
    // Continue the current shot for `rest` seconds after a ricochet / perforation.
    void advanceRest(ProjectileState& s, const ProjectileType& t, Seconds rest,
                     const World& world, EventSink& sink, int layer);
    // Portable SoA batch advance of `n` Integrated-tier shots (states_ indices in
    // `idx`) that share `t` and are all on the plain fast path. Falls each lane
    // back to advanceIntegrated the moment it stops matching (impact, medium
    // change). SimConfig::batchIntegrator gates the grouping in step().
    void advanceIntegratedBatch(const std::uint32_t* idx, std::size_t n,
                                const ProjectileType& t, Seconds dt,
                                const World& world, EventSink& sink);
    // True if `s` can ride the batch path (plain fast path: still air, no wind,
    // no Magnus / orientation-shaped drag, no precision flags).
    bool batchEligible(const ProjectileState& s) const;

    // Append one per-frame trajectory sample for state index `i` (no-op when the
    // cache is off).
    void recordTrajectory(std::size_t i);
    // Emit one diagnostic trace for `s` (no-op when SimConfig::traceSink is null).
    // `s` must be an element of states_ (the shot id is its slot index).
    void emitTrace(TraceKind kind, const ProjectileState& s,
                   std::int32_t i0 = 0, Real r0 = 0) const;
    // Peak adaptive sub-step count seen for the shot currently being advanced;
    // reset per shot in step(), reported as a TraceKind::Substep. Only written
    // when traceSink is set.
    std::uint32_t traceSubsteps_ = 0;
    // Filled fresh by every step(); see stats().
    SimStats      stats_;
    // Warhead fuze resolution for one live shot (ProjectileType::warhead set).
    // Picks the detonation point/time from the fuze mode, emits a Detonated
    // event and marks the round spent. No-op for an inert round or one that has
    // already detonated. Called once per shot per step, after the advance.
    void resolveWarhead(ProjectileState& s, const ProjectileType& t,
                        const World& world, EventSink& sink);
    // Terminal-ballistics resolution of one confirmed impact. Emits events,
    // mutates `s`, and returns true if the shot survives (ricochet / perforate /
    // fluid entry) and should be flown on for the rest of the frame.
    bool handleImpact(ProjectileState& s, const ProjectileType& t,
                      const World& world, EventSink& sink, const HitResult& hit,
                      Vec3 impactVel, Seconds impactTime_s);

    Environment                 env_;
    SimConfig                   cfg_;
    Status                      status_ = Status::Ok;
    bool                        bitExact_ = false; // cfg_.determinism == BitExact
    const char*                 lastError_ = "";   // see lastError()
    std::vector<ProjectileType> types_;
    std::vector<DragLuts>       luts_;   // parallel to types_
    std::unordered_map<std::string, TypeId> typeIds_; // id -> index, for fire()
    std::vector<ProjectileState> states_;
    std::vector<std::uint8_t>   used_;   // parallel to states_
    std::uint64_t               rng_;

    // Trajectory cache — a fixed-capacity ring per state slot, parallel to
    // states_. `head`/`count` index `ring`; capacity is cfg_.trajectoryCacheFrames.
    struct TrajRing {
        std::vector<TrajectorySample> ring;
        std::size_t                   head  = 0; // next write
        std::size_t                   count = 0;
    };
    std::vector<TrajRing>       traj_;   // parallel to states_; empty when cache off

    // Scratch reused by the batch integrator so a busy frame does not allocate.
    std::vector<std::uint32_t>  batchScratch_;   // eligible indices, grouped by type
    std::vector<std::uint8_t>   batchHandled_;   // parallel to states_; advanced by a batch
};

// ---------------------------------------------------------------------------
// Trajectory preview — a predicted flight path WITHOUT a Sim / World, for an
// aim indicator or a grenade-toss arc. Fills `out` (cleared first) with world
// positions sampled every `dt` seconds, starting at `launch.position`, until
// `maxTime` elapses, the round expires, or it drops to `groundY`. Runs the
// AnalyticDrag closed form (drag + gravity + wind), so it tracks the real
// `Integrated` trajectory within a few percent over a flat shot but costs a
// throwaway drag-LUT compile (~microseconds) — fine per frame for UI, not for
// thousands of calls. `type` is validated; a bad type leaves `out` empty.
// ---------------------------------------------------------------------------
void preview_arc(const ProjectileType& type, const LaunchParams& launch,
                 const Environment& env, Seconds dt, Seconds maxTime,
                 std::vector<Vec3>& out, Real groundY = Real(-1e30));

// As above, but each sample carries velocity + flight time alongside position
// (the same TrajectorySample the trajectory cache records), so an aim UI can
// read impact speed / energy / time-to-target at the reticle instead of just
// drawing a line.
void preview_arc(const ProjectileType& type, const LaunchParams& launch,
                 const Environment& env, Seconds dt, Seconds maxTime,
                 std::vector<TrajectorySample>& out, Real groundY = Real(-1e30));

// Local Mach number of a live shot: |velocity| / speed of sound. Uses
// `env.speedOfSound_mps` if set, else the ISA sea-level value (340.294 m/s) —
// same fallback describe() uses.
Real mach(const ProjectileState& s, const Environment& env = {});

// Kinetic energy (J) of a live shot: 1/2 m v^2 using `type.mass_kg` (the type
// it was spawned with). 0 when `type.mass_kg <= 0` (an unset mass has no
// physical energy to report), matching describe()'s "omit E=" behavior.
Real kinetic_energy_J(const ProjectileState& s, const ProjectileType& type);

// One-line human summary of a live shot, for logging / an on-screen debug
// readout — turns "why is this shot doing that" from a debugger session into a
// printf. The bare overload has no mass or air data, so it omits energy and
// Mach:
//   "v=612 m/s  t=0.83 s  x=487 m  drop=1.4 m  spin=2670 rad/s  [transonic]"
// Pass the type (+ optionally the environment) to add "  E=2114 J  M=1.80":
std::string describe(const ProjectileState& s);
std::string describe(const ProjectileState& s, const ProjectileType& type,
                     const Environment& env = {});

// One-line human summary of a TYPE (not a live shot) — content authoring / a
// catalog browser: "9x19_124gr_fmj: 8.0 g, 9.0 mm, Cd 0.30, ~360 m/s, ~518 J
// muzzle". Reads the fields as given — pass a type that has been through
// Sim::registerType() (e.g. sim.type(id)) to see resolved class defaults
// instead of the raw 0 / unset caller input.
std::string describe(const ProjectileType& type);

} // namespace pon
