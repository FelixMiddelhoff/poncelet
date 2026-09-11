// poncelet — internal integrator core.
// SPDX-License-Identifier: MIT
//
// Everything FP-sensitive lives in the matching .cpp, which is the ONLY
// translation unit compiled with fp-contract off / no fast-math (see
// CMakeLists.txt and docs/ballistics-phase-plan.md §3.6). Keeping the core
// behind this narrow interface is what kept the fixed-point (Q32.32) BitExact
// retrofit a core change, not an API change.
//
// The point-mass exterior-ballistics core:
//   * one `Core<Acc>` struct templated on the scalar accumulator with a fixed
//     step order — `Core<Real>` is the double path, `Core<Fx32>` the Q32.32
//     BitExact path (Part B); entry points dispatch on a `bool bitExact`;
//   * two fixed-step integrators — semi-implicit (symplectic) Euler and
//     classical RK4 — over one force model `flight_accel`;
//   * a deterministic adaptive sub-step count from a position-error and a
//     tunnelling bound;
//   * a closed-form analytic fast path (linear-drag exponential solution) used
//     by the AnalyticDrag fidelity tier.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/environment.hpp"
#include "poncelet/drag.hpp"
#include "avec3.hpp"

namespace pon::detail {

// ---------------------------------------------------------------------------
// Swappable accumulator. integrate.cpp expresses every arithmetic step on a
// template parameter `Acc` (see the `Core<Acc>` struct there); each public entry
// point below takes a `bool bitExact` and dispatches to `Core<Real>` (the
// float/double core — byte-identical to the pre-B6 path) or `Core<Fx32>` (the
// Q32.32 fixed-point core: deterministic sqrt + transcendental LUTs, bit-
// identical across platforms). `config::Determinism::BitExact` sets the flag.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Force model — total specific force (acceleration) on a point mass.
//   a(pos,vel,t) = gravity
//                − dragScaleExtra · dragRet(x) · rhoEff · |v_rel| · v_rel
//                + Cl(S) · liftAreaOver2m · rhoEff · |v_rel| · (ŝ × v_rel)
// with v_rel = vel − wind(pos,t); the drag-LUT abscissa x = |v_rel| ·
// dragAbscissaScale is Mach (scale = 1/a) or Reynolds (scale = d/ν); dragRet
// is the per-type retardation LUT (i·Cd(x)·A/2m, m²/kg). The Magnus term is
// active only when `liftVsSpin` is non-null and `spin_radps`·`spinAxis` are
// non-zero — S = |spin_radps|·spinRadius_m / |v_rel|. `rhoEff` is air density ×
// the medium drag-scale; `gravity` is already net of buoyancy. One unified LUT
// path covers ConstantCd (flat), G1/G7, custom curves and the BallProfile
// Cd(Re) spheres.
// ---------------------------------------------------------------------------
struct FlightModel {
    Vec3               gravity;               // m/s², already net of buoyancy
    const Lut1D*       dragRet   = nullptr;   // i·Cd(x)·A/2m vs abscissa; null ⇒ drag-free
    Real               rhoEff    = 0.0;       // air density × medium drag-scale
    Real               dragAbscissaScale = Real(1) / Real(340.294); // ×|v_rel| ⇒ Mach or Re
    Real               dragScaleExtra    = Real(1); // ProlateSpheroid axis-angle factor
    // Magnus / lift.
    const Lut1D*       liftVsSpin   = nullptr; // Cl(S); null ⇒ no Magnus
    Real               liftAreaOver2m = 0.0;   // 0.5·A/m
    Real               spin_radps     = 0.0;
    Real               spinRadius_m   = 0.0;
    Vec3               spinAxis       = {0, 0, 0}; // unit; zero ⇒ no Magnus
    const Environment* env       = nullptr;   // non-null only when a wind field is set
    Seconds            windTime  = 0.0;       // absolute sim time at t_rel = 0

    // --- Precision effects (§3.7). All default-zero ⇒ the fast path is byte
    // unchanged; sim.cpp fills these only for the flags set on the shot. ---
    Vec3    coriolisOmega   = {0, 0, 0}; // Earth rate, world frame; a += -2 Ω×v
    Real    spinDriftCoeff  = 0.0;       // Litz C (m): drift D(t) = C · t^1.83
    Vec3    spinDriftDir    = {0, 0, 0}; // world unit, already signed for twist
    Seconds flightTime0     = 0.0;       // total time of flight at tRel = 0
    Real    soundLapse_KpM  = 0.0;       // non-zero ⇒ Mach abscissa tracks T(alt)
    Real    siteTemp_K      = 288.15;    // reference T for the lapse scaling

    // Guided munitions (Phase 19 item 5): a constant external acceleration for
    // the frame — the guidance-law command + thrust + any raw caller poke.
    // Default zero ⇒ the fast path is unchanged. sim.cpp evaluates the guidance
    // law once per step and fills this before the sub-step loop.
    Vec3    externalAccel  = {0, 0, 0};
};

Vec3 flight_accel(const FlightModel& m, Vec3 pos, Vec3 vel, Seconds tRel,
                  bool bitExact = false);

// Linear-drag rate c (1/s) for the analytic fast path: the quadratic drag
// linearised at `speed`, i.e. dragRet(speed·scale) · rhoEff · speed. Evaluated in
// this TU so the fast path and the integrator share rounding.
Real linear_drag_rate(const FlightModel& m, Real speed, bool bitExact = false);

// ---------------------------------------------------------------------------
// 6-DOF rigid-body flight (PrecisionFlag::SixDOF, Phase 19 item 1).
//
// An axisymmetric projectile. Rather than integrate a full orientation
// quaternion (whose roll term spins at tens of krad/s and would force a tiny
// step), the angular state is the *aeroballistic* set: the nose unit vector
// `nose` (world), the transverse angular velocity `omegaT` (world, ⟂ nose — the
// precession/nutation rate) and the scalar spin `spin` (p). None of those
// oscillate at the roll rate, so a 0.5 ms step is stable. The visual roll phase
// is carried separately (`roll`, just ∫p) for display only.
//
// `RigidModel` reuses `FlightModel` for the translational force (gravity, base
// C_D0 drag, wind, the §3.7 precision accelerations) and adds the angular
// aerodynamics: a yaw-induced lift and drag-rise on the force side, and the
// overturning / pitch-damping / Magnus / roll-damping moments on the torque
// side. Yaw of repose, spin drift, epicyclic coning/nutation, spin decay and
// transonic instability are all emergent — nothing here is a stored "drift".
//
// Coefficients are dimensionless, per radian, on `refArea_m2` and `refLen_m`;
// the caller (sim.cpp) fills them from ProjectileType::aero (seeded or tuned).
// One `acos` + a few cross products per stage; kept in this fp-contract-off TU.
// ---------------------------------------------------------------------------
struct RigidModel {
    FlightModel flight;            // translational force (see above)
    Real        rhoEff      = 0.0; // air density × medium drag-scale (dyn. pressure)
    Real        refArea_m2  = 0.0; // π/4·d²
    Real        refLen_m    = 0.0; // projectile length (moment reference length)
    Real        mass_kg     = 0.0;
    Real        Ix          = 0.0; // axial moment of inertia
    Real        It          = 0.0; // transverse moment of inertia
    Real        CMa         = 0.0; // overturning-moment slope
    Real        CMq         = 0.0; // pitch-damping moment (< 0)
    Real        CMpa        = 0.0; // Magnus-moment slope
    Real        Clp         = 0.0; // roll-damping (< 0)
    Real        CLa         = 0.0; // lift-force slope
    Real        CDa2        = 0.0; // yaw-drag factor (ΔC_D = CDa2·α²)
    Real        invSpeedOfSound = 0.0; // for the transonic C_Mα multiplier; 0 ⇒ off
    Real        transonicMul    = 1.0; // C_Mα ×this in Mach 0.9..1.2
};

struct RigidState {
    Vec3 pos;
    Vec3 vel;
    Vec3 nose;          // world unit vector along the projectile axis
    Vec3 omegaT;        // world transverse angular velocity (⟂ nose), rad/s
    Real spin  = 0;     // axial spin p, rad/s
    Real roll  = 0;     // ∫p — visual roll phase only, rad
};

// One classical RK4 step of the aeroballistic angular + point-mass system.
// Re-normalises `nose` and re-projects `omegaT` ⟂ nose. Writes the total angle
// of attack at the start of the step to `alpha_out` (informational — the caller
// latches the tumble flag off it).
void step_rigid_rk4(const RigidModel& m, RigidState& s, Seconds tRel, Seconds h,
                    Real& alpha_out, bool bitExact = false);

// ---------------------------------------------------------------------------
// Integrators. Each advances (pos, vel) by exactly `h` seconds; `tRel` is the
// time of the sub-step start relative to the frame (only used to sample wind).
// ---------------------------------------------------------------------------
void step_semi_implicit(const FlightModel& m, Vec3& pos, Vec3& vel,
                        Seconds tRel, Seconds h, bool bitExact = false);
void step_rk4(const FlightModel& m, Vec3& pos, Vec3& vel,
              Seconds tRel, Seconds h, bool bitExact = false);

// One embedded Cash-Karp RK4(5) step (PrecisionFlag::AdaptiveRKF45). Does not
// mutate the inputs: writes the 5th-order result to (pos5, vel5) and the
// magnitude of the 5th-vs-4th difference to (posErr_m, velErr_mps) so the
// caller can run a step-size controller and keep the swept World query between
// accepted steps.
void step_rkck(const FlightModel& m, Vec3 pos, Vec3 vel, Seconds tRel, Seconds h,
               Vec3& pos5, Vec3& vel5, Real& posErr_m, Real& velErr_mps,
               bool bitExact = false);

// Deterministic sub-step count for advancing `dt` seconds from a state with
// the given velocity and (start-of-interval) acceleration. Bounds the
// semi-implicit position error by `posTol_m` (½·|a|·h² ≤ tol) and the
// per-sub-step travel by `maxDist_m` (anti-tunnelling), clamped to
// [1, maxSub]. Same inputs ⇒ same result.
int adaptive_substeps(Vec3 vel, Vec3 accel, Seconds dt,
                      Real posTol_m, Real maxDist_m, int maxSub,
                      bool bitExact = false);

// ---------------------------------------------------------------------------
// Analytic fast path. Exact solution of  a = gravity − c·v  (linear drag).
// Quadratic drag is approximated by linearising at a reference speed via
// linear_drag_rate(). No wind, constant gravity. Re-linearised by the caller
// each frame at the current speed, so it tracks the real curve — G1/G7
// included — well over a full flight while costing one exp per sample.
// ---------------------------------------------------------------------------
struct AnalyticTrajectory {
    Vec3 x0;
    Vec3 v0;
    Vec3 gravity;
    Real c = 0.0;   // linear drag rate (1/s); 0 ⇒ pure ballistic
    bool bitExact = false;

    void at(Seconds t, Vec3& pos, Vec3& vel) const;
};

} // namespace pon::detail
