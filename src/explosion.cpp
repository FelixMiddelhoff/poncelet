// poncelet — blast field implementation (Phase 19 item 2).
// SPDX-License-Identifier: MIT
//
// Air: Kinney & Graham (1985) scaled-distance fits for a spherical free-air TNT
// charge — peak overpressure, positive-phase duration, positive-phase impulse —
// with the ideal-gas normal-reflection and Rankine dynamic-pressure relations,
// and a Friedlander waveform whose decay constant is fitted to the KG impulse.
// Water: Cole similitude (1948) — near-field shock pressure ~52 MPa·(W^⅓/R)^1.13
// with an exponential decay time constant.
//
// This TU is deliberately NOT the fp-contract-off one — the blast model is a set
// of engineering fits with ~10-20% spread; bit-exactness would be false
// precision. It never touches the trajectory core.
#include "poncelet/explosion.hpp"

#include <algorithm>
#include <cmath>

namespace pon {
namespace {

constexpr Real kGamma      = 1.4;
constexpr Real kTntMJperKg = 4.184e6;
constexpr Real kWaterSound = 1481.0;
constexpr Real kWaterRho   = 1000.0;

// Kinney & Graham peak side-on overpressure ratio Ps/P0 for scaled distance Z
// (m/kg^1/3), spherical free-air burst.
Real kg_overpressure_ratio(Real Z) {
    const Real z1 = Z / 4.5;
    const Real a  = Real(1) + z1 * z1;
    const Real d1 = Real(1) + (Z / 0.048) * (Z / 0.048);
    const Real d2 = Real(1) + (Z / 0.32) * (Z / 0.32);
    const Real d3 = Real(1) + (Z / 1.35) * (Z / 1.35);
    return Real(808) * a / std::sqrt(d1 * d2 * d3);
}

// Kinney & Graham scaled positive-phase duration, ms per kg^1/3.
Real kg_scaled_duration_ms(Real Z) {
    const Real num = Real(980) * (Real(1) + std::pow(Z / 0.54, Real(10)));
    const Real d1  = Real(1) + std::pow(Z / 0.02, Real(3));
    const Real d2  = Real(1) + std::pow(Z / 0.74, Real(6));
    const Real d3  = std::sqrt(Real(1) + (Z / 6.9) * (Z / 6.9));
    return num / (d1 * d2 * d3);
}

// Kinney & Graham scaled positive-phase impulse, bar·ms per kg^1/3.
Real kg_scaled_impulse_barms(Real Z) {
    const Real num = Real(0.067) * std::sqrt(Real(1) + std::pow(Z / 0.23, Real(4)));
    const Real den = Z * Z * std::cbrt(Real(1) + std::pow(Z / 1.55, Real(3)));
    return num / den;
}

// Ideal-gas normal-reflection overpressure (Pr, above ambient) from side-on Ps.
// Ratio runs 2 (acoustic) -> 8 (strong shock, gamma = 1.4).
Real reflected_overpressure(Real Ps, Real P0) {
    return Real(2) * Ps +
           (kGamma + Real(1)) * Ps * Ps /
               ((kGamma - Real(1)) * Ps + Real(2) * kGamma * P0);
}

// Rankine peak dynamic (blast-wind stagnation) pressure from Ps.
Real dynamic_pressure(Real Ps, Real P0) {
    return Ps * Ps / (Real(2) * kGamma * P0 + (kGamma - Real(1)) * Ps);
}

// Friedlander waveform shape integral: for p(t) = Ps(1 - x)e^{-b x}, x = t/t+,
// the positive-phase impulse is Ps·t+·g(b) with g(b) = (b - 1 + e^{-b}) / b².
// g is monotone decreasing, 0.5 (b->0) down to 0 (b->inf).
Real friedlander_shape(Real b) {
    if (b < Real(1e-4)) return Real(0.5);
    return (b - Real(1) + std::exp(-b)) / (b * b);
}

// Solve g(b) = target for b in [1e-3, 60] by bisection (g is monotone).
Real solve_friedlander_b(Real target) {
    target = std::clamp(target, Real(1e-4), Real(0.4999));
    Real lo = Real(1e-3), hi = Real(60);
    for (int i = 0; i < 60; ++i) {
        const Real mid = Real(0.5) * (lo + hi);
        if (friedlander_shape(mid) > target) lo = mid; else hi = mid;
    }
    return Real(0.5) * (lo + hi);
}

// Pick the Friedlander decay `b` so the waveform's positive-phase impulse
// matches `s.specificImpulse_Pa_s`. When the fitted Ps·t+ cannot carry that much
// impulse (g would exceed 0.5 — happens in the far field where the KG duration
// and impulse fits diverge), stretch the positive-phase duration instead so the
// delivered impulse stays exact.
void fit_waveform(BlastSample& s) {
    if (s.peakOverpressure_Pa <= Real(0) || s.positiveDuration_s <= Real(0)) return;
    Real g = s.specificImpulse_Pa_s /
             (s.peakOverpressure_Pa * s.positiveDuration_s);
    if (g > Real(0.48)) {
        s.positiveDuration_s =
            s.specificImpulse_Pa_s / (s.peakOverpressure_Pa * Real(0.45));
        g = Real(0.45);
    }
    s.waveformDecay = solve_friedlander_b(g);
}

// Shock arrival time: integrate 1/U(r) from a small source radius to R, with the
// Rankine shock speed U = c0·sqrt(1 + (gamma+1)/(2 gamma)·Ps(r)/P0). Ps(r) from
// the KG fit. ~24 log-spaced steps — plenty for a monotone integrand.
Real shock_arrival_time(Real R, Real cubeRootW, Real P0, Real c0) {
    const Real r0 = std::max(Real(0.05) * cubeRootW, Real(1e-3));
    if (R <= r0) return R / c0;
    const int n = 24;
    const Real lr0 = std::log(r0), lr1 = std::log(R);
    Real t = r0 / c0; // pre-source-radius leg at ~sonic
    Real rPrev = r0;
    for (int i = 1; i <= n; ++i) {
        const Real r = std::exp(lr0 + (lr1 - lr0) * (Real(i) / n));
        const Real Zr  = r / cubeRootW;
        const Real Psr = kg_overpressure_ratio(Zr) * P0;
        const Real U   = c0 * std::sqrt(Real(1) + (kGamma + Real(1)) /
                                        (Real(2) * kGamma) * Psr / P0);
        t += (r - rPrev) / U;
        rPrev = r;
    }
    return t;
}

} // namespace

Burst::Burst(Vec3 origin, const WarheadDesc& w, const Environment& env,
             Seconds detonationTime_s)
    : origin_(origin), t0_(detonationTime_s), underwater_(w.underwater) {
    Real tnt = std::max(w.chargeMass_kg, Real(0)) * std::max(w.tntEquivalence, Real(0));
    if (w.thermobaric > Real(0)) {
        tnt            *= (Real(1) + w.thermobaric);
        durationStretch_ = Real(1) + Real(2) * w.thermobaric;
        impulseBoost_    = Real(1) + w.thermobaric;
    }
    if (w.surfaceBurst) tnt *= Real(1.8); // hemispherical ground reflection

    W_         = tnt;
    cubeRootW_ = tnt > Real(0) ? std::cbrt(tnt) : Real(0);

    if (underwater_) {
        ambientRho_ = kWaterRho;
        soundSpeed_ = kWaterSound;
        ambientP_   = Real(101325); // hydrostatic head is the caller's problem
    } else {
        ambientRho_ = env.airDensity_kgm3 > Real(0) ? env.airDensity_kgm3 : Real(1.225);
        soundSpeed_ = env.speedOfSound_mps > Real(0) ? env.speedOfSound_mps : Real(340.294);
        ambientP_   = ambientRho_ * soundSpeed_ * soundSpeed_ / kGamma;
    }
}

BlastSample Burst::sampleAt(Vec3 point) const {
    BlastSample s;
    const Real R = length(point - origin_);
    s.standoff_m = R;
    if (W_ <= Real(0) || R <= Real(0)) return s;

    if (underwater_) {
        // Cole similitude for TNT: Pm = 52.4 MPa·(W^⅓/R)^1.13,
        // decay time constant theta = 84 µs·W^⅓·(W^⅓/R)^-0.23.
        const Real wr    = cubeRootW_ / R;
        const Real Pm    = Real(52.4e6) * std::pow(wr, Real(1.13));
        const Real theta = Real(84e-6) * cubeRootW_ * std::pow(wr, Real(-0.23));
        s.scaledDistance           = R / cubeRootW_;
        s.arrivalTime_s            = R / soundSpeed_;
        s.peakOverpressure_Pa      = Pm;
        s.reflectedOverpressure_Pa = Real(2) * Pm;         // rigid reflection
        s.dynamicPressure_Pa       = Real(0);               // impulse is shock-pressure driven
        s.positiveDuration_s       = Real(5) * theta * durationStretch_;
        s.specificImpulse_Pa_s     = Pm * theta * impulseBoost_;
        fit_waveform(s);
        return s;
    }

    const Real Z = R / cubeRootW_;
    s.scaledDistance = Z;
    const Real Ps = kg_overpressure_ratio(Z) * ambientP_;
    s.peakOverpressure_Pa      = Ps;
    s.reflectedOverpressure_Pa = reflected_overpressure(Ps, ambientP_);
    s.dynamicPressure_Pa       = dynamic_pressure(Ps, ambientP_);
    s.positiveDuration_s = kg_scaled_duration_ms(Z) * Real(1e-3) * cubeRootW_ *
                           durationStretch_;
    // KG scaled impulse: bar·ms/kg^1/3 -> Pa·s (1 bar·ms = 100 Pa·s).
    s.specificImpulse_Pa_s = kg_scaled_impulse_barms(Z) * Real(100) * cubeRootW_ *
                             impulseBoost_;
    s.arrivalTime_s = shock_arrival_time(R, cubeRootW_, ambientP_, soundSpeed_);

    fit_waveform(s);
    return s;
}

Pascals Burst::overpressureAt(Vec3 point, Seconds t) const {
    const BlastSample s = sampleAt(point);
    if (s.peakOverpressure_Pa <= Real(0) || s.positiveDuration_s <= Real(0))
        return Real(0);
    const Real dt = t - t0_ - s.arrivalTime_s;
    if (dt < Real(0) || dt > s.positiveDuration_s) return Real(0);
    const Real x = dt / s.positiveDuration_s;
    return s.peakOverpressure_Pa * (Real(1) - x) * std::exp(-s.waveformDecay * x);
}

BlastLoad Burst::loadOnBody(const BlastTarget& tgt, Real losFraction,
                            Vec3 comOffset) const {
    BlastLoad out;
    out.lineOfSight = std::clamp(losFraction, Real(0), Real(1));
    const BlastSample s = sampleAt(tgt.centroid);
    if (s.peakOverpressure_Pa <= Real(0) || tgt.area_m2 <= Real(0) ||
        out.lineOfSight <= Real(0))
        return out;

    Vec3 dir = tgt.centroid - origin_;
    const Real R = length(dir);
    dir = R > Real(1e-6) ? dir * (Real(1) / R) : Vec3{0, 1, 0};

    // Diffraction (reflected) impulse on the facing area: scale the incident
    // specific impulse by the reflection ratio. Dominates in the near/mid field.
    const Real reflRatio = s.peakOverpressure_Pa > Real(0)
                               ? s.reflectedOverpressure_Pa / s.peakOverpressure_Pa
                               : Real(2);
    const Real iRefl = s.specificImpulse_Pa_s * reflRatio * tgt.reflectionFactor;
    // Drag (blast-wind) impulse: Cd·q_peak·(fraction of t+). The 0.2 lumps the
    // dynamic-pressure decay over the positive phase — a seed, tune per game.
    const Real iDrag = tgt.dragCoefficient * s.dynamicPressure_Pa *
                       s.positiveDuration_s * Real(0.2);

    const Real iTot = (iRefl + iDrag) * tgt.area_m2 * out.lineOfSight;
    out.impulse_Ns       = dir * iTot;
    out.torqueImpulse_Nms = cross(comOffset, out.impulse_Ns);
    out.deltaVelocity_mps = tgt.mass_kg > Real(0) ? out.impulse_Ns / tgt.mass_kg
                                                  : Vec3{};
    out.peakReflected_Pa  = s.reflectedOverpressure_Pa * tgt.reflectionFactor *
                            out.lineOfSight;
    return out;
}

Real blast_line_of_sight(const World& world, Vec3 origin, Vec3 target) {
    HitResult hr;
    if (!world.raycast(origin, target, hr)) return Real(1);
    return std::clamp(hr.t, Real(0), Real(1));
}

} // namespace pon
