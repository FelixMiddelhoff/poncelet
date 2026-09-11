// poncelet — shaped-charge / EFP implementation (Phase 19 item 4).
// SPDX-License-Identifier: MIT
//
// Jet formation: a PER (Pugh-Eichelberger-Rostoker) steady-state collapse gives
// the tip / tail / slug split from a Gurney collapse velocity and the cone
// half-angle. Stretching: the jet elongates at (v_tip - v_tail) until the liner
// metal particulates (particulationTime_s), after which a dispersion penalty
// takes over. Penetration: the hydrodynamic limit P = L_eff·√(ρ_jet/ρ_target)
// with a target-strength velocity cut (modified Bernoulli). EFP: a single
// coherent slug, no stretch, effective to very long standoff.
//
// NOT the fp-contract-off TU — this is all engineering fits (~tens of % spread),
// never feeds the trajectory core. Seeded splitmix64 for the behind-armour spray
// so it is bit-reproducible on one platform, matching explosion.cpp /
// fragmentation.cpp.
#include "poncelet/shapedcharge.hpp"

#include <algorithm>
#include <cmath>

namespace pon {
namespace {

constexpr Real kPi = 3.14159265358979323846;

struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed) {}
    std::uint64_t next() {
        s += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    Real unit() { return Real(next() >> 11) * (Real(1) / Real(9007199254740992.0)); }
};

void basis_from_axis(Vec3 axis, Vec3& e0, Vec3& e1) {
    const Vec3 up = std::fabs(axis.z) < Real(0.9) ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    e0 = normalized(cross(up, axis));
    e1 = cross(axis, e0);
}

// Charge diameter: the field if set, else a coarse fallback from the liner mass
// treated as a thin cone shell ~one CD tall (V ≈ 0.12·CD³ of metal).
Real charge_diameter(const ShapedChargeDesc& d) {
    if (d.chargeDiameter_m > Real(0)) return d.chargeDiameter_m;
    const Real rho = std::max(d.linerDensity_kgm3, Real(1));
    const Real m   = std::max(d.linerMass_kg, Real(1e-9));
    return std::cbrt(m / (rho * Real(0.12)));
}

// Gurney open-faced-sandwich collapse velocity for a charge-to-metal ratio r.
Real gurney_collapse(Real gurneyV, Real r) {
    r = std::max(r, Real(1e-4));
    return gurneyV * std::sqrt(r / (Real(1) + Real(0.5) * r));
}

// Target resistance stress for the modified-Bernoulli velocity cut. Ductile
// metal resists at a few times its flow stress; brittle / fibrous / granular
// media far less; fluids not at all.
Real target_resistance_Pa(const Material& t) {
    Real k = 3.0;
    switch (t.behaviour) {
        case MaterialBehaviour::Ductile:  k = 3.5; break;
        case MaterialBehaviour::Brittle:  k = 1.4; break;
        case MaterialBehaviour::Fibrous:  k = 1.0; break;
        case MaterialBehaviour::Granular: k = 0.6; break;
        case MaterialBehaviour::Membrane: k = 0.2; break;
        case MaterialBehaviour::Fluid:    return Real(0);
    }
    return k * std::max(t.strength_Pa, Real(0));
}

} // namespace

JetFormation shaped_charge_formation(const ShapedChargeDesc& d,
                                     Kilograms chargeMass_kg) {
    JetFormation f;
    if (d.linerMass_kg <= Real(0)) return f;

    const Real CD   = charge_diameter(d);
    const Real rhoJ = std::max(d.linerDensity_kgm3, Real(1));
    const Real r    = std::max(chargeMass_kg, Real(1e-6)) /
                      std::max(d.linerMass_kg, Real(1e-9));
    const Real V0   = gurney_collapse(std::max(d.gurneyVelocity_mps, Real(1)), r);

    f.isEFP = d.kind == ShapedChargeType::EFP;

    if (f.isEFP) {
        // One coherent slug: the dish everts and stretches modestly. Slug speed
        // ~1.4-2.0·V0; ~85% of the liner ends up in it.
        f.jetMass_kg      = Real(0.85) * d.linerMass_kg;
        f.slugMass_kg     = Real(0);
        f.tipVelocity_mps = d.jetTipVelocity_mps > Real(0) ? d.jetTipVelocity_mps
                                                           : std::clamp(Real(1.7) * V0,
                                                                        Real(1200), Real(2800));
        f.tailVelocity_mps = f.tipVelocity_mps;      // no gradient
        f.avgVelocity_mps  = f.tipVelocity_mps;
        // Slug geometry from its mass at ~0.22·CD diameter (L/D ~ a few).
        f.jetDiameter_m    = Real(0.22) * CD;
        const Real area    = kPi * Real(0.25) * f.jetDiameter_m * f.jetDiameter_m;
        f.initialLength_m  = f.jetMass_kg / (rhoJ * std::max(area, Real(1e-12)));
        f.coherentLength_m = f.initialLength_m;
        f.breakupStandoff_m = Real(1e9);            // effectively never
        return f;
    }

    // --- ConicalJet (HEAT) ---
    const Real beta = std::clamp(Real(0.5) * d.coneApexAngle_rad, Real(0.15), Real(1.30));
    // PER steady-state: the jet tip runs well ahead of the collapse velocity —
    // ~V0·(1/sin β)·1.3 for a copper cone (≈7.3 km/s for a 60° cone at V0 ≈ 2.8
    // km/s, ≈9.5 for a 42° cone), the slowest jet element near V0 itself.
    const Real tipDerived = V0 / std::max(std::sin(beta), Real(1e-3)) * Real(1.3);
    f.tipVelocity_mps = d.jetTipVelocity_mps > Real(0)
                            ? d.jetTipVelocity_mps
                            : std::clamp(tipDerived, Real(3000), Real(11000));
    f.tailVelocity_mps = d.jetTailVelocity_mps > Real(0)
                             ? d.jetTailVelocity_mps
                             : std::clamp(V0, Real(1500), Real(3500));
    if (f.tailVelocity_mps >= f.tipVelocity_mps)
        f.tailVelocity_mps = f.tipVelocity_mps * Real(0.4);
    f.avgVelocity_mps = Real(0.5) * (f.tipVelocity_mps + f.tailVelocity_mps);

    // Jet vs slug mass split: shallower cones throw more of the liner into the
    // jet. Seed ≈ 0.4·(1 - cos β) + 0.06, clamped to a realistic 8-40%.
    Real fj = d.jetMassFraction > Real(0)
                  ? d.jetMassFraction
                  : std::clamp(Real(0.4) * (Real(1) - std::cos(beta)) + Real(0.06),
                               Real(0.08), Real(0.40));
    f.jetMass_kg  = fj * d.linerMass_kg;
    f.slugMass_kg = (Real(1) - fj) * d.linerMass_kg;

    // L0 ≈ the liner slant length ≈ (CD/2)/sin β.
    f.initialLength_m = (Real(0.5) * CD) / std::max(std::sin(beta), Real(1e-3));
    f.jetDiameter_m   = std::clamp(Real(0.12) * CD, Real(1e-4), Real(0.5) * CD);

    const Real tb = std::max(d.particulationTime_s, Real(1e-6));
    f.coherentLength_m  = f.initialLength_m + (f.tipVelocity_mps - f.tailVelocity_mps) * tb;
    f.breakupStandoff_m = f.avgVelocity_mps * tb;
    return f;
}

Meters shaped_charge_optimal_standoff(const ShapedChargeDesc& d,
                                      Kilograms chargeMass_kg) {
    const JetFormation f = shaped_charge_formation(d, chargeMass_kg);
    const Real CD = charge_diameter(d);
    if (f.isEFP) return Real(100) * CD;           // representative "good" EFP standoff
    // Just short of particulation, and inside the classic 2-6 CD band.
    return std::clamp(Real(0.7) * f.breakupStandoff_m, Real(2) * CD, Real(6) * CD);
}

namespace {

// Stretched jet length reaching a target `standoff_m` away, plus the dispersion
// penalty once the jet has particulated.
Real effective_jet_length(const JetFormation& f, const ShapedChargeDesc& d,
                          Real standoff_m, Real CD) {
    if (f.isEFP) return f.initialLength_m;
    standoff_m = std::max(standoff_m, Real(0));
    const Real tb = std::max(d.particulationTime_s, Real(1e-6));
    const Real t  = standoff_m / std::max(f.avgVelocity_mps, Real(1));
    Real L = f.initialLength_m +
             (f.tipVelocity_mps - f.tailVelocity_mps) * std::min(t, tb);
    // Penetration efficiency taper. Even before the jet fully particulates its
    // velocity gradient thins and slightly disperses it, so the depth stops
    // tracking standoff linearly and peaks at a few CD (the classic HEAT
    // stand-off curve); once past breakup the particle stream drifts apart fast.
    const Real onset = Real(3) * CD;
    if (standoff_m > onset)
        L *= std::exp(-(standoff_m - onset) / std::max(Real(7) * CD, Real(1e-3)));
    if (standoff_m > f.breakupStandoff_m) {
        const Real over = standoff_m - f.breakupStandoff_m;
        L *= std::exp(-over / std::max(Real(12) * CD, Real(1e-3)));
    }
    return std::max(L, Real(0));
}

// Hydrodynamic depth for an effective jet length L into `target`, with the
// modified-Bernoulli strength cut at the driving velocity `vDrive`.
Real hydro_depth(Real L, Real rhoJet, const Material& target, Real vDrive) {
    const Real rhoT = std::max(target.density_kgm3, Real(1));
    Real P = L * std::sqrt(rhoJet / rhoT);
    const Real Rt = target_resistance_Pa(target);
    if (Rt > Real(0) && vDrive > Real(0)) {
        const Real uMin2 = Real(2) * Rt / rhoJet;
        const Real ratio = uMin2 / (vDrive * vDrive);
        P *= std::sqrt(std::max(Real(0), Real(1) - ratio));
    }
    return std::max(P, Real(0));
}

} // namespace

ShapedChargePenetration shaped_charge_penetration(const ShapedChargeDesc& d,
                                                  const Material& target,
                                                  Meters standoff_m,
                                                  Meters targetThickness_m,
                                                  Kilograms chargeMass_kg) {
    ShapedChargePenetration out;
    if (d.linerMass_kg <= Real(0)) return out;

    const JetFormation f = shaped_charge_formation(d, chargeMass_kg);
    const Real CD   = charge_diameter(d);
    const Real rhoJ = std::max(d.linerDensity_kgm3, Real(1));

    const Real L   = effective_jet_length(f, d, std::max(standoff_m, Real(0)), CD);
    out.effectiveJetLength_m = L;
    out.depth_m = hydro_depth(L, rhoJ, target, f.tipVelocity_mps);

    if (f.isEFP) {
        // EFP loses a little to drift/drag over long standoff (the flight sim
        // owns the real drag; this is the terminal-effectiveness taper).
        const Real taper = std::clamp(Real(1) - standoff_m / (Real(2000) * CD),
                                      Real(0.3), Real(1));
        out.depth_m *= taper;
    }

    // Standoff efficiency vs the optimal-standoff depth.
    {
        const Real So = shaped_charge_optimal_standoff(d, chargeMass_kg);
        const Real Lo = effective_jet_length(f, d, So, CD);
        Real Po = hydro_depth(Lo, rhoJ, target, f.tipVelocity_mps);
        if (f.isEFP) Po *= Real(1);  // taper ≈ 1 at 100 CD
        out.standoffEfficiency = Po > Real(0) ? std::clamp(out.depth_m / Po, Real(0), Real(1))
                                              : Real(0);
    }

    out.holeDiameter_m = f.isEFP ? f.jetDiameter_m * Real(1.3)
                                 : std::max(f.jetDiameter_m * Real(2.2), Real(0.03) * CD);

    if (targetThickness_m > Real(0)) {
        if (out.depth_m >= targetThickness_m) {
            out.perforated = true;
            const Real beyond = (out.depth_m - targetThickness_m) / std::max(out.depth_m, Real(1e-9));
            out.residualLength_m     = L * beyond;
            out.residualVelocity_mps = f.tipVelocity_mps * std::pow(std::clamp(beyond, Real(0), Real(1)),
                                                                    Real(0.35));
            out.spall = true; // a perforation always scabs the exit lip
        } else if (out.depth_m >= Real(0.7) * targetThickness_m) {
            out.spall = true; // back-face scabbing without a through hole
        }
    }
    return out;
}

std::size_t shaped_charge_behind_armour(const ShapedChargeDesc& d,
                                        const Material& target, Vec3 exitPoint,
                                        Vec3 aimDir, Meters standoff_m,
                                        Meters targetThickness_m,
                                        Kilograms chargeMass_kg,
                                        std::vector<FragmentSpec>& out) {
    out.clear();
    const ShapedChargePenetration pen =
        shaped_charge_penetration(d, target, standoff_m, targetThickness_m, chargeMass_kg);
    if (!pen.perforated && !pen.spall) return 0;

    Vec3 axis = normalized(aimDir);
    if (length_sq(axis) <= Real(0)) axis = Vec3{1, 0, 0};
    Vec3 e0, e1;
    basis_from_axis(axis, e0, e1);
    Rng rng(d.seed ^ 0xA5A5F00Dull);

    const Real rhoJ = std::max(d.linerDensity_kgm3, Real(1));
    const Real rhoT = std::max(target.density_kgm3, Real(1000));

    // --- Residual jet particles (perforation only): a tight forward cone. ---
    if (pen.perforated && pen.residualLength_m > Real(1e-4) &&
        pen.residualVelocity_mps > Real(50)) {
        const int n = 8;
        const Real dia  = std::max(d.chargeDiameter_m > Real(0) ? Real(0.12) * d.chargeDiameter_m
                                                               : Real(0.003),
                                   Real(5e-4));
        const Real segLen = pen.residualLength_m / n;
        const Real segArea = kPi * Real(0.25) * dia * dia;
        const Real segMass = std::max(rhoJ * segArea * segLen, Real(1e-6));
        for (int i = 0; i < n; ++i) {
            const Real u1 = rng.unit(), u2 = rng.unit();
            const Real half = Real(0.05);        // ~3° residual-jet cone
            const Real ct = Real(1) - u1 * (Real(1) - std::cos(half));
            const Real st = std::sqrt(std::max(Real(0), Real(1) - ct * ct));
            const Real ph = Real(2) * kPi * u2;
            const Vec3 dir = axis * ct + e0 * (st * std::cos(ph)) + e1 * (st * std::sin(ph));
            // Lead particle (i = 0) at the residual tip speed, trailing ones slower.
            const Real spd = pen.residualVelocity_mps * (Real(1) - Real(0.5) * (Real(i) / Real(n)));
            FragmentSpec fs;
            fs.position        = exitPoint;
            fs.velocity        = normalized(dir) * std::max(spd, Real(50));
            fs.mass_kg         = segMass;
            fs.diameter_m      = dia;
            fs.dragCoefficient = 1.0;
            fs.representsCount  = 1.0;
            out.push_back(fs);
        }
    }

    // --- Spall cone: plate metal scabbed off the back face. ---
    {
        const int n = pen.perforated ? 16 : 10;
        const Real coneR   = std::max(pen.holeDiameter_m * Real(3), Real(0.02));
        const Real discT   = std::max(Real(0.1) * targetThickness_m, Real(1e-3));
        const Real totMass = rhoT * kPi * coneR * coneR * discT;
        const Real each    = std::max(totMass / n, Real(1e-6));
        // Spall velocity: a fraction of the jet driving velocity, higher on a
        // clean perforation.
        const Real vBase = (pen.perforated ? Real(0.20) : Real(0.10)) *
                           (pen.residualVelocity_mps > Real(0) ? pen.residualVelocity_mps
                                                              : Real(1500));
        const Real half = std::max(d.spallConeHalfAngle_rad, Real(0.05));
        for (int i = 0; i < n; ++i) {
            const Real u1 = rng.unit(), u2 = rng.unit(), u3 = rng.unit();
            const Real ct = Real(1) - u1 * (Real(1) - std::cos(half));
            const Real st = std::sqrt(std::max(Real(0), Real(1) - ct * ct));
            const Real ph = Real(2) * kPi * u2;
            const Vec3 dir = axis * ct + e0 * (st * std::cos(ph)) + e1 * (st * std::sin(ph));
            const Real dia = std::cbrt(Real(6) * each / (kPi * rhoT));
            FragmentSpec fs;
            fs.position        = exitPoint;
            fs.velocity        = normalized(dir) * std::max(vBase * (Real(0.5) + u3), Real(30));
            fs.mass_kg         = each;
            fs.diameter_m      = dia;
            fs.dragCoefficient = 1.1;
            fs.representsCount  = 1.0;
            out.push_back(fs);
        }
    }
    return out.size();
}

} // namespace pon
