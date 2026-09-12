// poncelet — guided munitions: proportional navigation / course correction over
// a per-step external-acceleration hook (Phase 19 item 5, §8a).
// SPDX-License-Identifier: MIT
//
// Two layers:
//
//   * A generic per-shot external-acceleration term the caller pokes each frame
//     (`Sim::setExternalAccel`) — thrust, a scripted course correction, a
//     custom guidance law the game computes itself. It is added to the force
//     model every sub-step and persists until changed.
//
//   * A built-in guidance law (`ProjectileType::guidance`): give the type a
//     `GuidanceDesc` and call `Sim::guide(id, targetPos, targetVel)` each frame
//     with the seeker's track. The Sim runs pursuit / proportional-navigation /
//     augmented-PN, clamps to the airframe g-limit, and folds the command into
//     the same external-acceleration term.
//
// The guidance command is evaluated once per `Sim::step` from the frame-start
// state (missiles pull single- to low-tens of g, so a per-frame update is
// plenty). It is deterministic — same inputs, same command — and, like the
// integrator core, it is in the fp-contract-off TU: `compute_guidance` runs on
// the same swappable `Core<Acc>` pattern (`bitExact = true` selects the Q32.32
// fixed-point path, deterministic sqrt/acos included), so a guided round is
// on the same cross-platform BitExact footing as the rest of the trajectory.
#pragma once

#include "poncelet/types.hpp"

namespace pon {

enum class GuidanceLaw {
    None,             // unguided (default)
    Pursuit,          // steer the velocity vector straight at the target's
                      // current position (a heat-seeker tail chase)
    ProportionalNav,  // a_cmd = N · Vc · λ̇  ⟂ LOS  — true PN, the standard
                      // homing law; leads a crossing target
    AugmentedPN,      // PN + N/2 · a_target,⟂  — compensates a manoeuvring target
};

// Guidance configuration on a ProjectileType. law == None ⇒ the whole guidance
// path is skipped (an unguided round is byte-unchanged).
struct GuidanceDesc {
    GuidanceLaw law                = GuidanceLaw::None;
    Real     navConstant           = 4.0;    // N (PN / APN); 3–5 typical
    Real     maxLateralAccel_g     = 30.0;   // airframe limit on the commanded
                                             // lateral acceleration (× g0)
    Radians  seekerHalfFov_rad     = 0.70;   // target more than this off the
                                             // velocity vector ⇒ lock lost, coast
    Seconds  activationDelay_s     = 0.0;    // guidance inert for the first t of
                                             // flight (boost / separation / settle)
    MetersPerSec thrustAccel_mps2  = 0.0;    // > 0 ⇒ axial boost while burning
    Seconds  burnTime_s            = 0.0;    // thrust duration from launch
    Real     inducedDragFactor     = 0.0;    // > 0 ⇒ a speed loss ∝ this · |a_lat|
                                             // (manoeuvre drag); 0 ⇒ ignored
};

// Per-shot guidance state, carried on ProjectileState. The caller sets the track
// through Sim::guide(); the Sim advances timeGuided_s and latches lockLost.
struct GuidanceState {
    Vec3    targetPos;
    Vec3    targetVel;
    bool    hasTarget    = false;
    bool    lockLost     = false;   // seeker FOV was exceeded (latched — a lost
                                    // lock is not reacquired)
    Seconds timeAlive_s  = 0.0;     // shot time when the track was last updated
    Vec3    lastLos      = {0,0,0}; // unit line-of-sight at the previous update
    Seconds lastLosTime  = -1.0;
    Vec3    lastTargetVel = {0,0,0};// for the AugmentedPN target-accel estimate
    bool    haveLastTargetVel = false;
};

// Result of one guidance evaluation — the acceleration to add to the force model
// this frame, plus a little telemetry.
struct GuidanceCommand {
    Vec3 accel_mps2       = {0,0,0}; // lateral command + axial thrust, world frame
    Real losRate_radps    = 0.0;
    Real closingSpeed_mps = 0.0;
    Real lateralAccel_mps2 = 0.0;    // magnitude of the lateral part (post-clamp)
    bool active           = false;   // false ⇒ coasting (no track / pre-activation
                                     // / lock lost) — only thrust may still apply
};

// Evaluate the guidance law. Advances `gs` (timeGuided bookkeeping, lockLost
// latch, stored LOS for the next λ̇ estimate) by the step `dt`. `pos` / `vel`
// are the missile's frame-start state; `flightTime_s` its total time of flight.
// `bitExact` ⇒ run the Q32.32 fixed-point core (deterministic sqrt/acos, no
// FMA contraction) instead of double — same convention as every other
// integrate.hpp entry point; `Sim` passes its own `Determinism::BitExact` flag.
GuidanceCommand compute_guidance(const GuidanceDesc& d, GuidanceState& gs,
                                 Vec3 pos, Vec3 vel, Seconds flightTime_s,
                                 Seconds dt, bool bitExact = false);

} // namespace pon
