// poncelet — guided-munition steering laws (Phase 19 item 5).
// SPDX-License-Identifier: MIT
//
// Pursuit, true proportional navigation (a_cmd = N·Vc·Ω ⟂ LOS) and augmented PN
// (+ N/2 · a_target,⟂), an airframe g-limit clamp, a boost-phase activation
// delay, an axial thrust term and an optional manoeuvre-drag speed loss. The LOS
// rotation rate Ω = (r × v_rel)/R² is analytic from the seeker track, so no
// finite-difference of the angle is needed.
//
// Deterministic (no <random>, no allocation). NOT the fp-contract-off TU — it
// feeds the trajectory, so a BitExact build folds this into the swappable core
// alongside integrate.cpp.
#include "poncelet/guidance.hpp"

#include <algorithm>
#include <cmath>

namespace pon {
namespace {

constexpr Real kG0 = 9.80665;

// Component of `v` perpendicular to unit vector `u`.
Vec3 reject(Vec3 v, Vec3 u) { return v - u * dot(v, u); }

} // namespace

GuidanceCommand compute_guidance(const GuidanceDesc& d, GuidanceState& gs,
                                 Vec3 pos, Vec3 vel, Seconds flightTime_s,
                                 Seconds dt) {
    GuidanceCommand cmd;
    if (d.law == GuidanceLaw::None || !gs.hasTarget) {
        gs.haveLastTargetVel = false;
        return cmd;
    }

    const Real speed = length(vel);
    const Vec3 vHat  = speed > Real(1e-6) ? vel / speed : Vec3{1, 0, 0};

    // --- Axial thrust (independent of lock). ---
    Vec3 thrust{0, 0, 0};
    if (d.thrustAccel_mps2 > Real(0) && flightTime_s < d.burnTime_s)
        thrust = vHat * d.thrustAccel_mps2;

    // --- Line of sight. ---
    const Vec3 r  = gs.targetPos - pos;
    const Real R  = length(r);
    if (R < Real(1e-4)) {                       // essentially on top of the target
        cmd.accel_mps2 = thrust;
        return cmd;
    }
    const Vec3 rHat  = r / R;
    const Vec3 vRel  = gs.targetVel - vel;      // target minus missile
    const Real Vc    = -dot(vRel, rHat);        // closing speed (>0 closing)
    cmd.closingSpeed_mps = Vc;

    // --- Seeker field of view: target too far off boresight ⇒ lose lock. ---
    const Real look = std::acos(std::clamp(dot(rHat, vHat), Real(-1), Real(1)));
    if (look > d.seekerHalfFov_rad) gs.lockLost = true;

    // LOS rotation-rate vector Ω = (r × v_rel) / R².
    const Vec3 omega = cross(r, vRel) / (R * R);
    cmd.losRate_radps = length(omega);

    // Store the track for the next call / APN.
    const Vec3 aTgt = (gs.haveLastTargetVel && dt > Real(0))
                          ? (gs.targetVel - gs.lastTargetVel) / dt
                          : Vec3{0, 0, 0};
    gs.lastTargetVel     = gs.targetVel;
    gs.haveLastTargetVel = true;
    gs.lastLos           = rHat;
    gs.lastLosTime       = flightTime_s;
    gs.timeAlive_s       = flightTime_s;

    const bool guiding = !gs.lockLost && flightTime_s >= d.activationDelay_s;
    if (!guiding) {
        cmd.accel_mps2 = thrust;                // coast (only thrust)
        return cmd;
    }
    cmd.active = true;

    // --- Lateral command. ---
    Vec3 aLat{0, 0, 0};
    switch (d.law) {
        case GuidanceLaw::Pursuit: {
            // Turn the velocity vector toward the target: command ⟂ v, toward
            // the LOS, proportional to the angle error (saturating at ~12°).
            const Vec3 perp = reject(rHat, vHat);
            const Real e    = length(perp);
            if (e > Real(1e-6)) {
                const Real gain = std::min(Real(1), e / Real(0.20));
                aLat = (perp / e) * (gain * d.maxLateralAccel_g * kG0);
            }
            break;
        }
        case GuidanceLaw::ProportionalNav:
        case GuidanceLaw::AugmentedPN: {
            const Real N  = std::max(d.navConstant, Real(0));
            const Real vc = std::max(Vc, Real(0));     // no PN command when opening
            // a = N · Vc · (Ω × r̂)  — magnitude N·Vc·|Ω|, ⟂ LOS in the turn plane.
            aLat = cross(omega, rHat) * (N * vc);
            if (d.law == GuidanceLaw::AugmentedPN) {
                const Vec3 aTgtPerp = reject(aTgt, rHat);
                aLat += aTgtPerp * (Real(0.5) * N);
            }
            break;
        }
        case GuidanceLaw::None: break;
    }

    // Keep the command purely lateral (⟂ velocity) and clamp to the airframe.
    aLat = reject(aLat, vHat);
    const Real aMax = std::max(d.maxLateralAccel_g, Real(0)) * kG0;
    Real aMag = length(aLat);
    if (aMag > aMax && aMag > Real(0)) { aLat = aLat * (aMax / aMag); aMag = aMax; }
    cmd.lateralAccel_mps2 = aMag;

    // --- Manoeuvre (induced) drag: a speed loss opposing the velocity. ---
    Vec3 drag{0, 0, 0};
    if (d.inducedDragFactor > Real(0) && aMag > Real(0))
        drag = -vHat * (d.inducedDragFactor * aMag);

    cmd.accel_mps2 = aLat + thrust + drag;
    return cmd;
}

} // namespace pon
