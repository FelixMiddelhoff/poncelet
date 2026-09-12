// poncelet — guided-munition steering laws (Phase 19 item 5).
// SPDX-License-Identifier: MIT
//
// Pursuit, true proportional navigation (a_cmd = N·Vc·Ω ⟂ LOS) and augmented PN
// (+ N/2 · a_target,⟂), an airframe g-limit clamp, a boost-phase activation
// delay, an axial thrust term and an optional manoeuvre-drag speed loss. The LOS
// rotation rate Ω = (r × v_rel)/R² is analytic from the seeker track, so no
// finite-difference of the angle is needed.
//
// FP-CONTRACT-OFF TRANSLATION UNIT (see CMakeLists.txt). Deterministic (no
// <random>, no allocation), and — same pattern as integrate.cpp's `Core<Acc>`
// — templated on a swappable accumulator: `GuidanceCore<Real>` is the
// float/double path (byte-identical to the pre-BitExact-guidance code below),
// `GuidanceCore<Fx32>` is the Q32.32 fixed-point path selected by `bitExact`.
// Every vector op here is a genuine FMA-contraction or cross-platform-libm
// risk (dot/cross of independently-nonzero vectors feeding `acos`) — see
// poncelet-planning/bitexact-guidance-law-plan.md for the full inventory —
// so the whole function is templated rather than patched call-by-call.
#include "poncelet/guidance.hpp"

#include "avec3.hpp"
#include "accum_ops.hpp"

#include <algorithm>
#include <cmath>

namespace pon::detail {
namespace {
constexpr Real kG0 = 9.80665;
} // namespace

template <class Acc>
struct GuidanceCore {
    using AVec = AVec3<Acc>;

    static AVec A(Vec3 v) { return {Acc(v.x), Acc(v.y), Acc(v.z)}; }
    static Vec3 V(AVec a) { return {to_real(a.x), to_real(a.y), to_real(a.z)}; }
    static Acc  mn(Acc a, Acc b) { return a < b ? a : b; }
    static Acc  mx(Acc a, Acc b) { return a > b ? a : b; }

    // Component of `v` perpendicular to unit vector `u`.
    static AVec reject(AVec v, AVec u) { return v - u * dot(v, u); }

    static GuidanceCommand compute(const GuidanceDesc& d, GuidanceState& gs,
                                   Vec3 pos, Vec3 vel, Seconds flightTime_s,
                                   Seconds dt) {
        GuidanceCommand cmd;
        if (d.law == GuidanceLaw::None || !gs.hasTarget) {
            gs.haveLastTargetVel = false;
            return cmd;
        }

        const AVec velA  = A(vel);
        const Acc  speed = acc_length(velA);
        const AVec vHat  = speed > Acc(1e-6) ? velA * (Acc(1) / speed)
                                              : AVec{Acc(1), Acc(0), Acc(0)};

        // --- Axial thrust (independent of lock). ---
        AVec thrust{Acc(0), Acc(0), Acc(0)};
        if (d.thrustAccel_mps2 > Real(0) && flightTime_s < d.burnTime_s)
            thrust = vHat * Acc(d.thrustAccel_mps2);

        // --- Line of sight. ---
        const AVec r = A(gs.targetPos) - A(pos);
        const Acc  R = acc_length(r);
        if (R < Acc(1e-4)) {                        // essentially on top of the target
            cmd.accel_mps2 = V(thrust);
            return cmd;
        }
        const AVec rHat = r * (Acc(1) / R);
        const AVec vRel = A(gs.targetVel) - velA;   // target minus missile
        const Acc  Vc   = -dot(vRel, rHat);         // closing speed (>0 closing)
        cmd.closingSpeed_mps = to_real(Vc);

        // --- Seeker field of view: target too far off boresight ⇒ lose lock. ---
        Acc c = dot(rHat, vHat);
        c = c > Acc(1) ? Acc(1) : (c < Acc(-1) ? Acc(-1) : c);
        const Acc look = acc_acos(c);
        if (to_real(look) > d.seekerHalfFov_rad) gs.lockLost = true;

        // LOS rotation-rate vector Ω = (r × v_rel) / R².
        const AVec omega = cross(r, vRel) * (Acc(1) / (R * R));
        cmd.losRate_radps = to_real(acc_length(omega));

        // Store the track for the next call / APN. Plain double: componentwise
        // subtract/divide only, no multiply-add chain — not FMA-sensitive
        // regardless of Acc (see bitexact-guidance-law-plan.md's "not a risk"
        // list), so it stays outside the templated path.
        const Vec3 aTgt = (gs.haveLastTargetVel && dt > Real(0))
                              ? (gs.targetVel - gs.lastTargetVel) / dt
                              : Vec3{0, 0, 0};
        gs.lastTargetVel     = gs.targetVel;
        gs.haveLastTargetVel = true;
        gs.lastLos           = V(rHat);
        gs.lastLosTime       = flightTime_s;
        gs.timeAlive_s       = flightTime_s;

        const bool guiding = !gs.lockLost && flightTime_s >= d.activationDelay_s;
        if (!guiding) {
            cmd.accel_mps2 = V(thrust);             // coast (only thrust)
            return cmd;
        }
        cmd.active = true;

        // --- Lateral command. ---
        AVec aLat{Acc(0), Acc(0), Acc(0)};
        switch (d.law) {
            case GuidanceLaw::Pursuit: {
                // Turn the velocity vector toward the target: command ⟂ v, toward
                // the LOS, proportional to the angle error (saturating at ~12°).
                const AVec perp = reject(rHat, vHat);
                const Acc  e    = acc_length(perp);
                if (e > Acc(1e-6)) {
                    const Acc gain = mn(Acc(1), e / Acc(0.20));
                    aLat = (perp * (Acc(1) / e)) *
                           (gain * Acc(d.maxLateralAccel_g) * Acc(kG0));
                }
                break;
            }
            case GuidanceLaw::ProportionalNav:
            case GuidanceLaw::AugmentedPN: {
                const Acc N  = mx(Acc(d.navConstant), Acc(0));
                const Acc vc = mx(Vc, Acc(0));        // no PN command when opening
                // a = N · Vc · (Ω × r̂)  — magnitude N·Vc·|Ω|, ⟂ LOS in the turn plane.
                aLat = cross(omega, rHat) * (N * vc);
                if (d.law == GuidanceLaw::AugmentedPN) {
                    const AVec aTgtPerp = reject(A(aTgt), rHat);
                    aLat += aTgtPerp * (Acc(0.5) * N);
                }
                break;
            }
            case GuidanceLaw::None: break;
        }

        // Keep the command purely lateral (⟂ velocity) and clamp to the airframe.
        aLat = reject(aLat, vHat);
        const Acc aMax = mx(Acc(d.maxLateralAccel_g), Acc(0)) * Acc(kG0);
        Acc aMag = acc_length(aLat);
        if (aMag > aMax && aMag > Acc(0)) { aLat = aLat * (aMax / aMag); aMag = aMax; }
        cmd.lateralAccel_mps2 = to_real(aMag);

        // --- Manoeuvre (induced) drag: a speed loss opposing the velocity. ---
        AVec drag{Acc(0), Acc(0), Acc(0)};
        if (d.inducedDragFactor > Real(0) && aMag > Acc(0))
            drag = -(vHat * (Acc(d.inducedDragFactor) * aMag));

        cmd.accel_mps2 = V(aLat + thrust + drag);
        return cmd;
    }
};

} // namespace pon::detail

namespace pon {

GuidanceCommand compute_guidance(const GuidanceDesc& d, GuidanceState& gs,
                                 Vec3 pos, Vec3 vel, Seconds flightTime_s,
                                 Seconds dt, bool bitExact) {
    return bitExact
               ? detail::GuidanceCore<detail::Fx32>::compute(d, gs, pos, vel, flightTime_s, dt)
               : detail::GuidanceCore<Real>::compute(d, gs, pos, vel, flightTime_s, dt);
}

} // namespace pon
