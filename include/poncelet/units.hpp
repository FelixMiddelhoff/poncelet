// poncelet — imperial <-> SI conversion helpers.
// SPDX-License-Identifier: MIT
//
// The whole poncelet API is SI (metres, seconds, kilograms, kelvin, pascals,
// radians — owner decision 4). Published ballistics data is almost always
// imperial, so these `constexpr` free functions let a caller transcribe a load
// table or a piece of dope without hand-converting (and mis-converting) it.
//
// They are plain `Real`-returning functions, NOT new types — nothing in the API
// changes, and there is no unit checking. Use them at the call boundary:
//
//   ProjectileType t = catalog::get("308_168gr_hpbt");
//   LaunchParams lp;
//   lp.speed = pon::fps(2650);                 // 2650 ft/s -> m/s
//   t.mass_kg = pon::grains(168);              // 168 gr    -> kg
//   env.airDensity_kgm3 = ...;                 // (still SI in / out)
//
// The `to_*` inverses are for HUD / logging (see also pon::describe()).
#pragma once

#include "poncelet/types.hpp"

namespace pon {

// --- exact ratios (NIST) ---------------------------------------------------
inline constexpr Real kMetrePerFoot   = 0.3048;              // exact
inline constexpr Real kMetrePerInch   = 0.0254;              // exact
inline constexpr Real kMetrePerYard   = 0.9144;              // exact
inline constexpr Real kKgPerGrain     = 6.479891e-5;         // 1 gr = 64.79891 mg, exact
inline constexpr Real kKgPerPound     = 0.45359237;          // exact
inline constexpr Real kJoulePerFootPound = 1.3558179483314004; // ft·lbf, exact
inline constexpr Real kPascalPerInchHg   = 3386.389;         // conventional inHg at 0 °C
inline constexpr Real kPascalPerPsi      = 6894.757293168361;
inline constexpr Real kRadianPerDegree   = 0.017453292519943295;
inline constexpr Real kRadianPerMOA      = kRadianPerDegree / 60.0; // 1 MOA = 1/60 deg
inline constexpr Real kRadianPerMil_NATO = 0.0009817477042468103;  // 6400 mils / 2π

// --- imperial -> SI (the common direction) -------------------------------
inline constexpr Meters       feet(Real ft)      { return ft * kMetrePerFoot; }
inline constexpr Meters       inches(Real in)    { return in * kMetrePerInch; }
inline constexpr Meters       yards(Real yd)     { return yd * kMetrePerYard; }
inline constexpr Meters       miles(Real mi)     { return mi * 1609.344; }
inline constexpr MetersPerSec fps(Real ftps)     { return ftps * kMetrePerFoot; }
inline constexpr MetersPerSec mph(Real mileph)   { return mileph * 0.44704; }
inline constexpr Kilograms    grains(Real gr)    { return gr * kKgPerGrain; }
inline constexpr Kilograms    pounds(Real lb)    { return lb * kKgPerPound; }
inline constexpr Kilograms    ounces(Real oz)    { return oz * (kKgPerPound / 16.0); }
inline constexpr Real         ftlb(Real e)       { return e * kJoulePerFootPound; }  // -> J
inline constexpr Pascals      inhg(Real p)       { return p * kPascalPerInchHg; }
inline constexpr Pascals      psi(Real p)        { return p * kPascalPerPsi; }
inline constexpr Kelvin       fahrenheit(Real f) { return (f - 32.0) * (5.0 / 9.0) + 273.15; }
inline constexpr Kelvin       celsius(Real c)    { return c + 273.15; }
inline constexpr Radians      degrees(Real d)    { return d * kRadianPerDegree; }
inline constexpr Radians      moa(Real m)        { return m * kRadianPerMOA; }
inline constexpr Radians      mil_nato(Real m)   { return m * kRadianPerMil_NATO; }
// KgPerM3 for a published imperial air density (lb/ft^3) — rare but shows up.
inline constexpr KgPerM3      lb_per_ft3(Real d) { return d * 16.018463373960138; }

// --- SI -> imperial (display / logging) ---------------------------------
inline constexpr Real to_feet(Meters m)          { return m / kMetrePerFoot; }
inline constexpr Real to_yards(Meters m)         { return m / kMetrePerYard; }
inline constexpr Real to_inches(Meters m)        { return m / kMetrePerInch; }
inline constexpr Real to_fps(MetersPerSec v)     { return v / kMetrePerFoot; }
inline constexpr Real to_mph(MetersPerSec v)     { return v / 0.44704; }
inline constexpr Real to_grains(Kilograms kg)    { return kg / kKgPerGrain; }
inline constexpr Real to_pounds(Kilograms kg)    { return kg / kKgPerPound; }
inline constexpr Real to_ftlb(Real joules)       { return joules / kJoulePerFootPound; }
inline constexpr Real to_moa(Radians r)          { return r / kRadianPerMOA; }
inline constexpr Real to_degrees(Radians r)      { return r / kRadianPerDegree; }
inline constexpr Real to_fahrenheit(Kelvin k)    { return (k - 273.15) * (9.0 / 5.0) + 32.0; }
inline constexpr Real to_celsius(Kelvin k)       { return k - 273.15; }

} // namespace pon
