// poncelet — terminal-ballistics resolution. §3.5 pipeline + §3.8 refinements.
// SPDX-License-Identifier: MIT
#include "terminal.hpp"

#include <algorithm>
#include <cmath>

namespace pon::detail {

namespace {

constexpr Real kPi = 3.14159265358979323846;
constexpr Real kDeg = kPi / Real(180);

Real clampr(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }

Vec3 unit(Vec3 v) {
    const Real l2 = dot(v, v);
    return l2 > Real(1e-20) ? v * (Real(1) / std::sqrt(l2)) : Vec3{1, 0, 0};
}

// Poncelet resistive law  F(v) = A·N*·σ·(α + β·ρ·v²/σ)  per behaviour class.
// α scales the quasi-static term (target strength → cavity-opening pressure),
// β the inertial (displaced-mass) term. Both are grounded in dynamic
// cavity-expansion theory rather than hand-fitted:
//
//   * Ductile (metal) — the dynamic spherical-cavity static pressure is
//     R ≈ (2/3)·Y·[1 + ln(2E/3Y)] ≈ 5–6·Y for steel (E/Y ~ 800), so α ≈ 6 on
//     the yield/flow strength; the inertial term is the incompressible
//     spherical-cavity value (3/2)·ρ·v², i.e. β = 1.5, with the nose bluntness
//     living in N* separately. Gives ~17 mm for a .30-cal AP-class round into
//     mild steel at ~820 m/s (ordnance figure ~20–25 mm; .30 M2 AP does
//     10.7–12.7 mm into the much harder RHA at 100 yd) while a 9 mm FMJ at
//     340 m/s is correctly stopped by 6 mm plate (v_bl ≈ 510 m/s). Refs:
//     Forrestal & Warren, *Int. J. Impact Eng.* (2008); Hill, cavity expansion.
//   * Brittle (concrete/glass) — Forrestal–Frew: R = S·f_c with the empirical
//     S ≈ 82.6·f_c(MPa)^-0.544 ≈ 12 for 35 MPa concrete; inertial coefficient
//     ~1. Gives ~65 mm for a 7.62 ball-class round at ~830 m/s into 35 MPa
//     concrete (Forrestal–Frew–Hanchak envelope). Ref: Frew et al., *Int. J.
//     Impact Eng.* 23 (1998).
//   * Fibrous (wood/flesh) — the classic Poncelet {α,β} = {1,1}; the design
//     doc's oak dataset fits it at R² ≈ 0.98.
//   * Granular (sand/dry earth) — inertial coefficient ~1 from the dry-sand
//     rate studies (Allen/Mayfield/Morrison; "rate-dependent penetration of dry
//     sand", 2020); the confined static bearing factor is ~9× the (small)
//     cohesive strength. Gives ~0.3 m for a rifle bullet into dry earth.
//   * Membrane / Fluid — negligible / handled out of band (Fluid returns early).
struct PonCoeff { Real alpha, beta; };
PonCoeff pon_coeff(MaterialBehaviour b) {
    switch (b) {
        case MaterialBehaviour::Ductile:  return {6.0, 1.5};
        case MaterialBehaviour::Brittle:  return {12.0, 1.0};
        case MaterialBehaviour::Fibrous:  return {1.0, 1.0};
        case MaterialBehaviour::Granular: return {9.0, 1.0};
        case MaterialBehaviour::Membrane: return {0.05, 0.2};
        case MaterialBehaviour::Fluid:    return {0.02, 1.0};
    }
    return {1.0, 1.0};
}

// Grazing angle (velocity vs surface plane) below which the round skips off.
Real ricochet_crit_deg(MaterialBehaviour b) {
    switch (b) {
        case MaterialBehaviour::Ductile:  return 22.0; // steel plate skips easily
        case MaterialBehaviour::Brittle:  return 13.0;
        case MaterialBehaviour::Fibrous:  return 6.0;  // wood/flesh grabs
        case MaterialBehaviour::Granular: return 10.0; // sand/turf: a plunging shot sticks
        case MaterialBehaviour::Fluid:    return 30.0;
        case MaterialBehaviour::Membrane: return 0.0;
    }
    return 0.0;
}

} // namespace

TerminalResult resolve_terminal(const TerminalProjectile& p, const Material& mat,
                                Vec3 vDir, Vec3 n, Vec3 hitPoint) {
    TerminalResult r;

    vDir = unit(vDir);
    n    = unit(n);
    // Make n oppose the travel direction (points back toward the shooter).
    if (dot(vDir, n) > Real(0)) n = -n;

    const Real m   = std::max(p.mass_kg, Real(1e-6));
    const Real d   = std::max(p.diameter_m, Real(1e-4));
    const Real v0  = std::max(p.speed_mps, Real(0));
    const Real w0  = v0 * v0;
    const Real yaw = clampr(p.impactYaw, Real(0), Real(1));

    // Incidence: cosInc = 1 head-on, → 0 grazing.  secTheta = 1/cosInc path
    // length multiplier for an oblique hit (§3.8).
    const Real cosInc  = clampr(-dot(vDir, n), Real(0.05), Real(1));
    const Real secθ    = Real(1) / cosInc;

    const Real area    = kPi * Real(0.25) * d * d;
    const Real areaEff = area * (Real(1) + Real(2) * yaw);        // keyholing
    const Real nStar   = std::max(p.noseShapeFactor, Real(0.05)) *
                         (Real(1) + Real(0.6) * yaw);
    const Real sigma   = std::max(mat.strength_Pa, Real(1));
    const Real rhoT    = std::max(mat.density_kgm3, Real(1));
    const bool bulk    = mat.thickness_m <= Real(0);
    const Real thick   = bulk ? Real(1e9) : mat.thickness_m;
    const Real pathLen = thick * secθ;                            // oblique slab

    r.spinScale = 1.0;

    // --- Fluid: no discrete stop. sim switches the medium (item: special media
    // refines the surface impulse / supercavitation). ---
    if (mat.behaviour == MaterialBehaviour::Fluid) {
        r.outcome           = TerminalOutcome::EnteredFluid;
        r.residualSpeed_mps = v0;
        r.exitDir           = vDir;
        return r;
    }

    // --- Fragile projectile into anything solid: it breaks up. ---
    if (p.fragile && v0 > Real(15) &&
        mat.behaviour != MaterialBehaviour::Membrane) {
        r.outcome           = TerminalOutcome::Shattered;
        r.energyDeposited_J = Real(0.5) * m * w0;
        return r;
    }

    // --- Membrane (balloon, foliage, paper): always perforates, tiny loss. ---
    if (mat.behaviour == MaterialBehaviour::Membrane) {
        const Real vres = v0 * Real(0.97);
        r.outcome           = TerminalOutcome::Perforated;
        r.residualSpeed_mps = vres;
        r.exitDir           = vDir;
        r.exitPoint         = hitPoint + vDir * std::max(mat.thickness_m, Real(0));
        r.channelDepth_m    = std::max(mat.thickness_m, Real(0));
        r.energyDeposited_J = Real(0.5) * m * (w0 - vres * vres);
        return r;
    }

    // --- Ricochet check (§3.5.1). Shallow grazing angle + not too fast + not a
    // soft/expanding round → skip off. ---
    {
        Real critDeg = ricochet_crit_deg(mat.behaviour);
        // Slower rounds skip at a steeper angle; faster rounds bite in.
        critDeg *= std::sqrt(Real(300) / std::max(v0, Real(300)));
        if (p.deformable) critDeg *= Real(0.4);
        // A sharp nose (broadhead, spitzer) digs in; a blunt one skips.
        critDeg *= clampr(Real(0.4) + Real(0.7) * p.noseShapeFactor, Real(0.3), Real(1.3));
        const Real sinCrit = std::sin(critDeg * kDeg);
        if (!p.fragile && cosInc < sinCrit && critDeg > Real(0)) {
            const Vec3 refl = vDir - n * (Real(2) * dot(vDir, n));
            const Vec3 tang = unit(vDir - n * dot(vDir, n));       // along-surface
            const Vec3 outD = unit(refl + tang * Real(0.35));
            // Shallower hit ⇒ less energy lost.
            const Real frac = cosInc / std::max(sinCrit, Real(1e-4)); // 0..1
            const Real keep = clampr(Real(0.55) + Real(0.35) * frac,
                                     Real(0.2), Real(0.9));
            const Real vres = v0 * keep;
            r.outcome           = TerminalOutcome::Ricochet;
            r.residualSpeed_mps = vres;
            r.exitDir           = outD;
            r.spinScale         = 0.6;
            r.energyDeposited_J = Real(0.5) * m * (w0 - vres * vres);
            r.newImpactYaw      = std::min(Real(1), yaw + Real(0.25));
            return r;
        }
    }

    // --- "Arrow vs concrete" (§3.5, special-behaviour shortcuts). A sharp,
    // low-sectional-density round
    // against a brittle solid cannot sustain the contact load to open a clean
    // channel — the face spalls and it stops there, whatever the penetration
    // ODE would say. SD = m / A (kg/m²): an arrow/bolt is a few hundred, a
    // bullet several thousand. Deformable/fast rounds are excluded (they punch
    // or splash). ---
    if (mat.behaviour == MaterialBehaviour::Brittle && !p.deformable) {
        const Real sd = m / area;
        if (p.noseShapeFactor < Real(0.5) && sd < Real(1500) && v0 < Real(300)) {
            r.outcome           = TerminalOutcome::Stopped;
            r.energyDeposited_J = Real(0.5) * m * w0;
            return r;
        }
    }

    // --- Poncelet penetration ODE (§3.5.3), solved on w = v² :
    //   (m/2) dw/dx = −A·N*·σ·α − A·N*·β·ρ·w
    //   dw/dx = −C0 − C1·w ,  w(x) = (w0 + C0/C1)·e^(−C1 x) − C0/C1
    //   x_stop = (1/C1)·ln(1 + C1·w0/C0)         (matches the doc's closed form)
    const PonCoeff pc = pon_coeff(mat.behaviour);
    const Real C0 = Real(2) * areaEff * nStar * sigma * pc.alpha / m; // (m/s²)·? → 1/s² on w
    const Real C1 = Real(2) * areaEff * nStar * pc.beta * rhoT / m;   // 1/m
    const Real ratio = C0 / std::max(C1, Real(1e-9));                 // = w at which static==0
    const Real xStop = (C1 > Real(1e-9))
                         ? std::log1p(C1 * w0 / std::max(C0, Real(1e-12))) / C1
                         : w0 / std::max(C0, Real(1e-12));

    // Ballistic-limit speed: xStop == pathLen  ⇒  w_bl = ratio·(e^(C1·pathLen) − 1).
    if (!bulk) {
        const Real e = std::exp(clampr(C1 * pathLen, Real(0), Real(60)));
        r.ballisticLimit_mps = std::sqrt(std::max(ratio * (e - Real(1)), Real(0)));
    }

    const Real Lyaw = Real(12) * d;                 // soft-media tumble onset

    if (bulk || xStop <= pathLen) {
        // Embeds (or stops at the face for a brittle chip / near-zero depth).
        const Real depth = std::max(xStop, Real(0));
        if (mat.behaviour == MaterialBehaviour::Brittle && depth < Real(3) * d) {
            r.outcome           = TerminalOutcome::Stopped;
            r.energyDeposited_J = Real(0.5) * m * w0;
            return r;
        }
        if (depth < Real(1e-4)) {
            r.outcome           = TerminalOutcome::Stopped;
            r.energyDeposited_J = Real(0.5) * m * w0;
            return r;
        }
        r.outcome           = TerminalOutcome::Embedded;
        r.channelDepth_m    = depth;
        r.exitDir           = vDir;
        r.energyDeposited_J = Real(0.5) * m * w0;
        if ((mat.behaviour == MaterialBehaviour::Fibrous ||
             mat.behaviour == MaterialBehaviour::Granular) && depth > Lyaw)
            r.channelWiden_m = Real(0.5) * d + Real(0.15) * (depth - Lyaw);
        return r;
    }

    // Perforates: residual speed from the same ODE at x = pathLen.
    const Real eDecay = std::exp(-clampr(C1 * pathLen, Real(0), Real(60)));
    const Real wRes   = (w0 + ratio) * eDecay - ratio;
    const Real vRes   = std::sqrt(std::max(wRes, Real(0)));

    // Slight deflection toward the plate normal on exit (obliquity torque).
    const Real defl = Real(0.15) * (Real(1) - cosInc);
    const Vec3 exitD = unit(vDir + (-n - vDir) * defl);

    r.outcome           = TerminalOutcome::Perforated;
    r.residualSpeed_mps = vRes;
    r.exitDir           = exitD;
    r.exitPoint         = hitPoint + vDir * pathLen;
    r.channelDepth_m    = pathLen;
    r.energyDeposited_J = Real(0.5) * m * std::max(w0 - wRes, Real(0));
    r.spinScale         = 0.85;
    r.newImpactYaw      = std::min(Real(1), yaw + Real(0.3)); // perforation induces yaw

    // Deformable round mushrooms on the first hard (non-soft) hit.
    if (p.deformable && !p.alreadyExpanded && v0 > Real(150) &&
        (mat.behaviour == MaterialBehaviour::Ductile ||
         mat.behaviour == MaterialBehaviour::Brittle ||
         mat.behaviour == MaterialBehaviour::Fibrous))
        r.newDiameter_m = d * Real(1.6);

    if (mat.behaviour == MaterialBehaviour::Fibrous && pathLen > Lyaw)
        r.channelWiden_m = Real(0.5) * d + Real(0.15) * (pathLen - Lyaw);

    return r;
}

} // namespace pon::detail
