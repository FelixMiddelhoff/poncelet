// poncelet — integrator core. FP-CONTRACT-OFF TRANSLATION UNIT.
// SPDX-License-Identifier: MIT
//
// Build rules for this file (enforced in CMakeLists.txt):
//   * no fast-math / -ffast-math anywhere
//   * -ffp-contract=off  (GCC/Clang)  |  /fp:precise  (MSVC)
//
// Every arithmetic step runs on a template parameter `Acc` via the `Core<Acc>`
// struct below. `Core<Real>` is the float/double core — byte-for-bit the pre-B6
// path (acc_* forward straight to <cmath>, AVec3<Real> matches the old Vec3
// component order). `Core<Fx32>` is the Q32.32 fixed-point BitExact core: the
// deterministic sqrt (B3), transcendental LUTs (B4) and drag-LUT sampling (B5),
// integer-only and identical across platforms. Each public entry point takes a
// `bool bitExact` and dispatches; config::Determinism::BitExact sets it.
#include "integrate.hpp"

#include "accum_ops.hpp"
#include "drag_fx.hpp"

#include <algorithm>
#include <cmath>

namespace pon::detail {

namespace {

// Drag / lift LUT sampling on the accumulator: the double path keeps
// Lut1D::sample(); the fixed-point path uses the bit-identical port (B5).
inline Real acc_lut_sample(const Lut1D& l, Real x) { return l.sample(x); }
inline Fx32 acc_lut_sample(const Lut1D& l, Fx32 x) { return lut_sample_fx(l, x); }

template <class Acc>
struct Core {
    using AVec = AVec3<Acc>;

    static AVec A(Vec3 v) { return {Acc(v.x), Acc(v.y), Acc(v.z)}; }
    static Vec3 V(AVec a) { return {to_real(a.x), to_real(a.y), to_real(a.z)}; }
    static Acc  mn(Acc a, Acc b) { return a < b ? a : b; }

    // --- total specific force on a point mass -----------------------------
    static AVec flight_accel_a(const FlightModel& m, AVec pos, AVec vel, Acc tRel) {
        AVec a = A(m.gravity);

        // Guidance / thrust / caller course-correction (Phase 19 item 5).
        if (m.externalAccel.x != 0.0 || m.externalAccel.y != 0.0 ||
            m.externalAccel.z != 0.0)
            a += A(m.externalAccel);

        // Coriolis:  a += -2 Ω × v. Gyroscopic spin drift — Litz closed form
        // D(t) = C·t^1.83 applied as D''(t) = C·1.83·0.83·t^-0.17 along the
        // signed drift direction (guarded below 1 ms). Both default-off.
        if (m.coriolisOmega.x != 0.0 || m.coriolisOmega.y != 0.0 ||
            m.coriolisOmega.z != 0.0)
            a -= cross(A(m.coriolisOmega), vel) * Acc(2);
        if (m.spinDriftCoeff > 0.0) {
            const Acc t = Acc(m.flightTime0) + tRel;
            if (t > Acc(1.0e-3)) {
                const Acc acc = Acc(m.spinDriftCoeff) * Acc(1.83) * Acc(0.83) *
                                acc_pow_neg017(t);
                a += A(m.spinDriftDir) * acc;
            }
        }

        if (m.rhoEff <= 0.0) return a;

        AVec vRel = vel;
        if (m.env)
            vRel -= A(m.env->windAt(V(pos), to_real(Acc(m.windTime) + tRel)));
        const Acc speed = acc_sqrt(dot(vRel, vRel));
        if (speed <= Acc(0)) return a;

        // Drag:  dragRet(x) = i·Cd(x)·A / 2m;  |a_drag| = dragRet·rho·v².
        // x = speed·dragAbscissaScale is Mach (1/a) or Reynolds (d/ν). Under
        // LocalSpeedSound the Mach abscissa is scaled by sqrt(siteT / T(alt)).
        if (m.dragRet) {
            Acc scale = Acc(m.dragAbscissaScale);
            if (m.soundLapse_KpM != 0.0) {
                const Acc Talt = Acc(m.siteTemp_K) + Acc(m.soundLapse_KpM) * pos.y;
                if (Talt > Acc(0))
                    scale *= acc_sqrt(Acc(m.siteTemp_K) / Talt);
            }
            const Acc x    = speed * scale;
            const Acc kret = acc_lut_sample(*m.dragRet, x) * Acc(m.dragScaleExtra);
            const Acc s    = kret * Acc(m.rhoEff) * speed; // 1/s, along -vRel
            a -= vRel * s;
        }

        // Magnus / lift:  a_lift = Cl(S)·(0.5 A/m)·rho·|v_rel|·(ŝ × v_rel).
        // S = |ω| r / |v_rel|. Skipped for rifle spin (spinAxis left zeroed).
        if (m.liftVsSpin && m.spin_radps != 0.0 &&
            (m.spinAxis.x != 0.0 || m.spinAxis.y != 0.0 || m.spinAxis.z != 0.0)) {
            const Acc S  = Acc(std::fabs(m.spin_radps) * m.spinRadius_m) / speed;
            const Acc cl = acc_lut_sample(*m.liftVsSpin, S);
            if (cl != Acc(0)) {
                const AVec dir = cross(A(m.spinAxis), vRel); // |·| = |v_rel| sinθ
                const Acc  k   = cl * Acc(m.liftAreaOver2m) * Acc(m.rhoEff) * speed;
                a += dir * k;
            }
        }
        return a;
    }

    static Real linear_drag_rate(const FlightModel& m, Real speedR) {
        if (!m.dragRet || m.rhoEff <= Real(0) || speedR <= Real(0)) return Real(0);
        const Acc speed = Acc(speedR);
        const Acc x     = speed * Acc(m.dragAbscissaScale);
        return to_real(acc_lut_sample(*m.dragRet, x) * Acc(m.dragScaleExtra) *
                       Acc(m.rhoEff) * speed);
    }

    // --- point-mass integrators -----------------------------------------
    static void step_semi_implicit(const FlightModel& m, Vec3& pos, Vec3& vel,
                                   Seconds tRel, Seconds h) {
        AVec      p = A(pos), v = A(vel);
        const Acc H = Acc(h);
        const AVec a = flight_accel_a(m, p, v, Acc(tRel));
        v += a * H;
        p += v * H;
        pos = V(p);
        vel = V(v);
    }

    static void step_rk4(const FlightModel& m, Vec3& pos, Vec3& vel,
                         Seconds tRel, Seconds h) {
        const AVec pos0 = A(pos), vel0 = A(vel);
        const Acc  H  = Acc(h);
        const Acc  t0 = Acc(tRel);
        const Acc  h2 = H * Acc(0.5);

        const AVec k1x = vel0;
        const AVec k1v = flight_accel_a(m, pos0, vel0, t0);

        const AVec p2 = pos0 + k1x * h2;
        const AVec v2 = vel0 + k1v * h2;
        const AVec k2x = v2;
        const AVec k2v = flight_accel_a(m, p2, v2, t0 + h2);

        const AVec p3 = pos0 + k2x * h2;
        const AVec v3 = vel0 + k2v * h2;
        const AVec k3x = v3;
        const AVec k3v = flight_accel_a(m, p3, v3, t0 + h2);

        const AVec p4 = pos0 + k3x * H;
        const AVec v4 = vel0 + k3v * H;
        const AVec k4x = v4;
        const AVec k4v = flight_accel_a(m, p4, v4, t0 + H);

        const Acc sixth = H / Acc(6);
        pos = V(pos0 + (k1x + (k2x + k3x) * Acc(2) + k4x) * sixth);
        vel = V(vel0 + (k1v + (k2v + k3v) * Acc(2) + k4v) * sixth);
    }

    static void step_rkck(const FlightModel& m, Vec3 pos, Vec3 vel, Seconds tRel,
                          Seconds h, Vec3& pos5, Vec3& vel5, Real& posErr_m,
                          Real& velErr_mps) {
        // Cash-Karp RK4(5). `const` not `constexpr` — Fx32 divide is a runtime
        // op; for Acc = Real the compiler still folds these to the same values.
        const Acc a2 = Acc(1) / Acc(5);
        const Acc a3 = Acc(3) / Acc(10);
        const Acc a4 = Acc(3) / Acc(5);
        const Acc a5 = Acc(1);
        const Acc a6 = Acc(7) / Acc(8);

        const Acc b21 = Acc(1) / Acc(5);
        const Acc b31 = Acc(3) / Acc(40),      b32 = Acc(9) / Acc(40);
        const Acc b41 = Acc(3) / Acc(10),      b42 = Acc(-9) / Acc(10),
                  b43 = Acc(6) / Acc(5);
        const Acc b51 = Acc(-11) / Acc(54),    b52 = Acc(5) / Acc(2),
                  b53 = Acc(-70) / Acc(27),    b54 = Acc(35) / Acc(27);
        const Acc b61 = Acc(1631) / Acc(55296), b62 = Acc(175) / Acc(512),
                  b63 = Acc(575) / Acc(13824),  b64 = Acc(44275) / Acc(110592),
                  b65 = Acc(253) / Acc(4096);

        const Acc c1 = Acc(37) / Acc(378),   c3 = Acc(250) / Acc(621),
                  c4 = Acc(125) / Acc(594),  c6 = Acc(512) / Acc(1771);
        const Acc d1 = Acc(2825) / Acc(27648), d3 = Acc(18575) / Acc(48384),
                  d4 = Acc(13525) / Acc(55296), d5 = Acc(277) / Acc(14336),
                  d6 = Acc(1) / Acc(4);

        const AVec pos0 = A(pos), vel0 = A(vel);
        const Acc  H  = Acc(h);
        const Acc  t0 = Acc(tRel);

        const AVec k1x = vel0;
        const AVec k1v = flight_accel_a(m, pos0, vel0, t0);

        const AVec k2x = vel0 + k1v * (H * b21);
        const AVec k2v = flight_accel_a(m, pos0 + k1x * (H * b21),
                                        vel0 + k1v * (H * b21), t0 + H * a2);

        const AVec k3x = vel0 + (k1v * b31 + k2v * b32) * H;
        const AVec k3v = flight_accel_a(m, pos0 + (k1x * b31 + k2x * b32) * H,
                                        vel0 + (k1v * b31 + k2v * b32) * H, t0 + H * a3);

        const AVec k4x = vel0 + (k1v * b41 + k2v * b42 + k3v * b43) * H;
        const AVec k4v = flight_accel_a(m, pos0 + (k1x * b41 + k2x * b42 + k3x * b43) * H,
                                        vel0 + (k1v * b41 + k2v * b42 + k3v * b43) * H,
                                        t0 + H * a4);

        const AVec k5x = vel0 + (k1v * b51 + k2v * b52 + k3v * b53 + k4v * b54) * H;
        const AVec k5v = flight_accel_a(
            m, pos0 + (k1x * b51 + k2x * b52 + k3x * b53 + k4x * b54) * H,
            vel0 + (k1v * b51 + k2v * b52 + k3v * b53 + k4v * b54) * H, t0 + H * a5);

        const AVec k6x = vel0 + (k1v * b61 + k2v * b62 + k3v * b63 + k4v * b64 + k5v * b65) * H;
        const AVec k6v = flight_accel_a(
            m, pos0 + (k1x * b61 + k2x * b62 + k3x * b63 + k4x * b64 + k5x * b65) * H,
            vel0 + (k1v * b61 + k2v * b62 + k3v * b63 + k4v * b64 + k5v * b65) * H,
            t0 + H * a6);

        const AVec p5 = pos0 + (k1x * c1 + k3x * c3 + k4x * c4 + k6x * c6) * H;
        const AVec v5 = vel0 + (k1v * c1 + k3v * c3 + k4v * c4 + k6v * c6) * H;
        pos5 = V(p5);
        vel5 = V(v5);

        const AVec pos4 = pos0 + (k1x * d1 + k3x * d3 + k4x * d4 + k5x * d5 + k6x * d6) * H;
        const AVec vel4 = vel0 + (k1v * d1 + k3v * d3 + k4v * d4 + k5v * d5 + k6v * d6) * H;

        posErr_m   = to_real(acc_sqrt(dot(p5 - pos4, p5 - pos4)));
        velErr_mps = to_real(acc_sqrt(dot(v5 - vel4, v5 - vel4)));
    }

    // --- 6-DOF rigid-body flight (PrecisionFlag::SixDOF) ---------------
    struct ARigidState {
        AVec pos, vel, nose, omegaT;
        Acc  spin{}, roll{};
    };
    struct RigidDeriv {
        AVec dpos, dvel, dnose, domegaT;
        Acc  dspin{}, droll{};
    };

    static RigidDeriv rigid_deriv(const RigidModel& m, const ARigidState& s,
                                  Acc tRel, Real* alphaOut) {
        RigidDeriv d;
        d.dpos    = s.vel;
        d.dnose   = cross(s.omegaT, s.nose);
        d.domegaT = AVec{Acc(0), Acc(0), Acc(0)};
        d.dspin   = Acc(0);
        d.droll   = s.spin;

        AVec a = flight_accel_a(m.flight, s.pos, s.vel, tRel);

        AVec vRel = s.vel;
        if (m.flight.env)
            vRel -= A(m.flight.env->windAt(V(s.pos),
                                           to_real(Acc(m.flight.windTime) + tRel)));
        const Acc speed = acc_sqrt(dot(vRel, vRel));

        Acc alpha = Acc(0);
        if (speed > Acc(1e-6) && m.rhoEff > Real(0)) {
            const AVec vhat = vRel * (Acc(1) / speed);
            Acc c = dot(s.nose, vhat);
            c = c > Acc(1) ? Acc(1) : (c < Acc(-1) ? Acc(-1) : c);
            alpha = acc_acos(c);
            const Acc sinA = acc_sin(alpha);

            const Acc qd_over_m =
                Acc(0.5) * Acc(m.rhoEff) * speed * speed * Acc(m.refArea_m2) / Acc(m.mass_kg);

            AVec yawDir = s.nose - vhat * c;
            const Acc yl = acc_sqrt(dot(yawDir, yawDir));
            if (yl > Acc(1e-9)) {
                yawDir = yawDir * (Acc(1) / yl);
                a += yawDir * (qd_over_m * Acc(m.CLa) * sinA);
            }
            a -= vhat * (qd_over_m * Acc(m.CDa2) * alpha * alpha);

            const Acc qdSd =
                Acc(0.5) * Acc(m.rhoEff) * speed * speed * Acc(m.refArea_m2) * Acc(m.refLen_m);
            Acc CMa = Acc(m.CMa);
            if (m.invSpeedOfSound > Real(0)) {
                const Acc mach = speed * Acc(m.invSpeedOfSound);
                if (mach >= Acc(0.9) && mach <= Acc(1.2)) CMa *= Acc(m.transonicMul);
            }

            AVec tau{Acc(0), Acc(0), Acc(0)};
            const AVec nxv = cross(s.nose, vhat);
            const Acc  nxvl = acc_sqrt(dot(nxv, nxv));
            if (nxvl > Acc(1e-9)) {
                const AVec sHat = nxv * (Acc(1) / nxvl);
                tau -= sHat * (qdSd * CMa * sinA);
                const AVec planeDir = cross(s.nose, sHat);
                const Acc  spinTerm = s.spin * Acc(m.refLen_m) / (Acc(2) * speed);
                tau += planeDir * (qdSd * Acc(m.CMpa) * spinTerm * sinA);
            }
            tau += s.omegaT * (qdSd * Acc(m.CMq) * Acc(m.refLen_m) / (Acc(2) * speed));
            d.dspin = qdSd * Acc(m.Clp) * (s.spin * Acc(m.refLen_m) / (Acc(2) * speed)) / Acc(m.Ix);

            const AVec tauPerp = tau - s.nose * dot(tau, s.nose);
            const AVec gyro    = cross(s.nose, s.omegaT) * (Acc(m.Ix) * s.spin);
            d.domegaT = (tauPerp - gyro) * (Acc(1) / Acc(m.It));
        }

        d.dvel = a;
        if (alphaOut) *alphaOut = to_real(alpha);
        return d;
    }

    static ARigidState rigid_advance(const ARigidState& a, const RigidDeriv& k, Acc f) {
        ARigidState r;
        r.pos    = a.pos    + k.dpos    * f;
        r.vel    = a.vel    + k.dvel    * f;
        r.nose   = a.nose   + k.dnose   * f;
        r.omegaT = a.omegaT + k.domegaT * f;
        r.spin   = a.spin   + k.dspin   * f;
        r.roll   = a.roll   + k.droll   * f;
        return r;
    }

    static void step_rigid_rk4(const RigidModel& m, RigidState& s, Seconds tRel,
                               Seconds h, Real& alpha_out) {
        const Acc H  = Acc(h);
        const Acc t0 = Acc(tRel);
        const Acc h2 = H * Acc(0.5);

        ARigidState s0;
        s0.pos = A(s.pos); s0.vel = A(s.vel); s0.nose = A(s.nose);
        s0.omegaT = A(s.omegaT); s0.spin = Acc(s.spin); s0.roll = Acc(s.roll);

        const RigidDeriv k1 = rigid_deriv(m, s0, t0, &alpha_out);
        const RigidDeriv k2 = rigid_deriv(m, rigid_advance(s0, k1, h2), t0 + h2, nullptr);
        const RigidDeriv k3 = rigid_deriv(m, rigid_advance(s0, k2, h2), t0 + h2, nullptr);
        const RigidDeriv k4 = rigid_deriv(m, rigid_advance(s0, k3, H),  t0 + H,  nullptr);

        const Acc sixth = H / Acc(6);
        AVec pos    = s0.pos    + (k1.dpos    + (k2.dpos    + k3.dpos)    * Acc(2) + k4.dpos)    * sixth;
        AVec vel    = s0.vel    + (k1.dvel    + (k2.dvel    + k3.dvel)    * Acc(2) + k4.dvel)    * sixth;
        AVec nose   = s0.nose   + (k1.dnose   + (k2.dnose   + k3.dnose)   * Acc(2) + k4.dnose)   * sixth;
        AVec omegaT = s0.omegaT + (k1.domegaT + (k2.domegaT + k3.domegaT) * Acc(2) + k4.domegaT) * sixth;
        Acc  spin   = s0.spin   + (k1.dspin   + (k2.dspin   + k3.dspin)   * Acc(2) + k4.dspin)   * sixth;
        Acc  roll   = s0.roll   + (k1.droll   + (k2.droll   + k3.droll)   * Acc(2) + k4.droll)   * sixth;

        const Acc nl = acc_sqrt(dot(nose, nose));
        if (nl > Acc(0)) nose = nose * (Acc(1) / nl);
        omegaT = omegaT - nose * dot(omegaT, nose);

        s.pos = V(pos); s.vel = V(vel); s.nose = V(nose); s.omegaT = V(omegaT);
        s.spin = to_real(spin); s.roll = to_real(roll);
    }

    static int adaptive_substeps(Vec3 vel, Vec3 accel, Seconds dt, Real posTol_m,
                                 Real maxDist_m, int maxSub) {
        if (dt <= Seconds(0) || maxSub < 1) return 1;

        const Acc speed = acc_sqrt(dot(A(vel), A(vel)));
        const Acc acc   = acc_sqrt(dot(A(accel), A(accel)));

        Acc h = Acc(dt);
        if (acc > Acc(0) && posTol_m > Real(0))
            h = mn(h, acc_sqrt(Acc(2) * Acc(posTol_m) / acc));
        if (speed > Acc(0) && maxDist_m > Real(0))
            h = mn(h, Acc(maxDist_m) / speed);

        if (h <= Acc(0)) return maxSub;
        int n = static_cast<int>(std::ceil(to_real(Acc(dt) / h)));
        if (n < 1) n = 1;
        if (n > maxSub) n = maxSub;
        return n;
    }

    static void analytic_at(const AnalyticTrajectory& tr, Seconds t, Vec3& pos, Vec3& vel) {
        const AVec x0a = A(tr.x0), v0a = A(tr.v0), ga = A(tr.gravity);
        const Acc  ta = Acc(t);
        const Acc  ca = Acc(tr.c);
        if (ca <= Acc(0)) {
            vel = V(v0a + ga * ta);
            pos = V(x0a + v0a * ta + ga * (Acc(0.5) * ta * ta));
            return;
        }
        const AVec vTerm = ga * (Acc(1) / ca);
        const Acc  e     = acc_exp(-ca * ta);
        const AVec dv0   = v0a - vTerm;
        vel = V(vTerm + dv0 * e);
        pos = V(x0a + vTerm * ta + dv0 * ((Acc(1) - e) / ca));
    }
};

} // namespace

// --- public entry points: dispatch on `bitExact` --------------------------

Vec3 flight_accel(const FlightModel& m, Vec3 pos, Vec3 vel, Seconds tRel, bool bx) {
    return bx ? Core<Fx32>::V(Core<Fx32>::flight_accel_a(
                    m, Core<Fx32>::A(pos), Core<Fx32>::A(vel), Fx32(tRel)))
              : Core<Real>::V(Core<Real>::flight_accel_a(
                    m, Core<Real>::A(pos), Core<Real>::A(vel), Real(tRel)));
}

Real linear_drag_rate(const FlightModel& m, Real speed, bool bx) {
    return bx ? Core<Fx32>::linear_drag_rate(m, speed)
              : Core<Real>::linear_drag_rate(m, speed);
}

void step_semi_implicit(const FlightModel& m, Vec3& pos, Vec3& vel, Seconds tRel,
                        Seconds h, bool bx) {
    if (bx) Core<Fx32>::step_semi_implicit(m, pos, vel, tRel, h);
    else    Core<Real>::step_semi_implicit(m, pos, vel, tRel, h);
}

void step_rk4(const FlightModel& m, Vec3& pos, Vec3& vel, Seconds tRel, Seconds h,
              bool bx) {
    if (bx) Core<Fx32>::step_rk4(m, pos, vel, tRel, h);
    else    Core<Real>::step_rk4(m, pos, vel, tRel, h);
}

void step_rkck(const FlightModel& m, Vec3 pos, Vec3 vel, Seconds tRel, Seconds h,
               Vec3& pos5, Vec3& vel5, Real& posErr_m, Real& velErr_mps, bool bx) {
    if (bx) Core<Fx32>::step_rkck(m, pos, vel, tRel, h, pos5, vel5, posErr_m, velErr_mps);
    else    Core<Real>::step_rkck(m, pos, vel, tRel, h, pos5, vel5, posErr_m, velErr_mps);
}

void step_rigid_rk4(const RigidModel& m, RigidState& s, Seconds tRel, Seconds h,
                    Real& alpha_out, bool bx) {
    if (bx) Core<Fx32>::step_rigid_rk4(m, s, tRel, h, alpha_out);
    else    Core<Real>::step_rigid_rk4(m, s, tRel, h, alpha_out);
}

int adaptive_substeps(Vec3 vel, Vec3 accel, Seconds dt, Real posTol_m,
                      Real maxDist_m, int maxSub, bool bx) {
    return bx ? Core<Fx32>::adaptive_substeps(vel, accel, dt, posTol_m, maxDist_m, maxSub)
              : Core<Real>::adaptive_substeps(vel, accel, dt, posTol_m, maxDist_m, maxSub);
}

void AnalyticTrajectory::at(Seconds t, Vec3& pos, Vec3& vel) const {
    if (bitExact) Core<Fx32>::analytic_at(*this, t, pos, vel);
    else          Core<Real>::analytic_at(*this, t, pos, vel);
}

} // namespace pon::detail
