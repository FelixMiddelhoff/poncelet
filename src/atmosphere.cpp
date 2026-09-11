// poncelet — ISA atmosphere solve.
// SPDX-License-Identifier: MIT
//
// Not an FP-contract-off translation unit: the atmosphere is resolved once
// per environment change (not in the sub-stepped hot loop), and its output
// feeds the drag LUT abscissa, not the integrator accumulator. PlatformStable
// determinism only needs same-platform repeatability, which a plain TU gives.
#include "poncelet/atmosphere.hpp"

#include <cmath>

namespace pon {

namespace {

// 1976 ISA constants.
constexpr Real kG0       = 9.80665;       // m/s^2
constexpr Real kRair     = 287.052874;    // J/(kg K), dry air
constexpr Real kGamma    = 1.4;           // ratio of specific heats, air
constexpr Real kT0       = 288.15;        // K   at 0 m
constexpr Real kP0       = 101325.0;      // Pa  at 0 m
constexpr Real kRvapor   = 461.495;       // J/(kg K), water vapour

struct Layer {
    Real baseAlt_m;
    Real baseT_K;
    Real lapse_KpM;   // dT/dh
};

// Geopotential layer bases (ISA). Enough for any small-arms / artillery use.
constexpr Layer kLayers[] = {
    {     0.0, 288.15, -0.0065},
    { 11000.0, 216.65,  0.0    },
    { 20000.0, 216.65,  0.001  },
    { 32000.0, 228.65,  0.0    },
};

// ISA temperature and pressure at geometric altitude h (treated as
// geopotential — the difference is sub-metre in this range).
void isa_tp(Real h, Real& T, Real& P) {
    T = kT0;
    P = kP0;
    Real baseP = kP0;
    for (std::size_t i = 0; i < sizeof(kLayers) / sizeof(kLayers[0]); ++i) {
        const Layer& L = kLayers[i];
        const Real top =
            (i + 1 < sizeof(kLayers) / sizeof(kLayers[0])) ? kLayers[i + 1].baseAlt_m
                                                           : 1.0e12;
        const Real dh   = (h < top ? h : top) - L.baseAlt_m;
        const Real Tbot = L.baseT_K;
        Real Ptop;
        if (std::fabs(L.lapse_KpM) > 1e-12) {
            const Real Ttop = Tbot + L.lapse_KpM * dh;
            Ptop = baseP * std::pow(Ttop / Tbot, -kG0 / (kRair * L.lapse_KpM));
            if (h < top) { T = Ttop; P = Ptop; return; }
            baseP = Ptop;
        } else {
            const Real Ttop = Tbot;
            Ptop = baseP * std::exp(-kG0 * dh / (kRair * Tbot));
            if (h < top) { T = Ttop; P = Ptop; return; }
            baseP = Ptop;
        }
    }
}

// Saturation vapour pressure over water, Pa (Tetens, T in K).
Real p_sat(Real T_K) {
    const Real Tc = T_K - 273.15;
    return 610.94 * std::exp(17.625 * Tc / (Tc + 243.04));
}

} // namespace

AtmoState isa_atmosphere(const IsaConditions& c) {
    Real T, P;
    isa_tp(c.altitude_m > 0.0 ? c.altitude_m : 0.0, T, P);
    if (c.temperature_K) T = *c.temperature_K;
    if (c.pressure_Pa)   P = *c.pressure_Pa;

    // Humid-air density: split the total pressure into dry and vapour partial
    // pressures, each with its own gas constant. rho = pd/(Rd T) + pv/(Rv T).
    const Real rh = c.relativeHumidity < 0.0 ? 0.0
                    : (c.relativeHumidity > 1.0 ? 1.0 : c.relativeHumidity);
    const Real pv = rh * p_sat(T);
    const Real pd = P - pv;
    const Real rho = pd / (kRair * T) + pv / (kRvapor * T);

    // Speed of sound in moist air: use the effective gas constant of the
    // mixture (mass-weighted), gamma shifts only marginally with humidity so
    // hold it at the dry-air value.
    const Real Reff = (pd * kRair + pv * kRvapor) / (pd + pv > 0.0 ? pd + pv : 1.0);
    const Real a = std::sqrt(kGamma * Reff * T);

    // Dynamic viscosity from Sutherland's law (air): μ = μ0 (T/T0)^1.5 (T0+S)/(T+S).
    constexpr Real kMu0 = 1.716e-5, kTmu0 = 273.15, kSuth = 110.4;
    const Real mu = kMu0 * std::pow(T / kTmu0, Real(1.5)) * (kTmu0 + kSuth) / (T + kSuth);

    AtmoState out;
    out.density_kgm3     = rho;
    out.speedOfSound_mps = a;
    out.temperature_K    = T;
    out.pressure_Pa      = P;
    out.kinematicViscosity_m2s = mu / (rho > 0.0 ? rho : 1.225);
    return out;
}

} // namespace pon
