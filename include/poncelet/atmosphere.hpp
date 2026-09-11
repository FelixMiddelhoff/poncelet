// poncelet — ISA atmosphere model: altitude / temperature / pressure /
// humidity -> air density AND local speed of sound.
// SPDX-License-Identifier: MIT
//
// Item 3 delivers the standard-atmosphere solve; item 5 wires the wind / medium
// API on top of it and item 6 the PrecisionFlag::LocalSpeedSound altitude
// tracking. G1/G7 drag is Cd(Mach), so the
// speed of sound this produces is what maps trajectory speed onto the drag
// curve — a fixed 340 m/s reads the curve at the wrong abscissa and long-range
// drop comes out wrong (design doc §3.3, §3.7).
#pragma once

#include "poncelet/types.hpp"

#include <optional>

namespace pon {

// Firing-site conditions. `altitude_m` drives the ISA layer solve; the three
// optionals override the station values the ISA model would otherwise derive
// (a Kestrel reading, a range weather station). Humidity lowers density
// slightly (water vapour is lighter than dry air) and raises the speed of
// sound a touch.
struct IsaConditions {
    Meters                 altitude_m        = 0.0;
    std::optional<Kelvin>  temperature_K;     // station air temperature
    std::optional<Pascals> pressure_Pa;       // station absolute pressure
    Real                   relativeHumidity  = 0.0; // 0..1
};

struct AtmoState {
    KgPerM3      density_kgm3     = 1.225;
    MetersPerSec speedOfSound_mps = 340.294;
    Kelvin       temperature_K    = 288.15;
    Pascals      pressure_Pa      = 101325.0;
    // Kinematic viscosity ν = μ/ρ, m²/s (μ from Sutherland's law at T). Feeds
    // the Reynolds number for the BallProfile drag curves.
    Real         kinematicViscosity_m2s = 1.4607e-5;
};

// Solve the 1976 International Standard Atmosphere (troposphere + lower
// stratosphere, valid to ~32 km) for `c`, then apply the humidity correction.
// A supplied temperature/pressure override replaces the ISA-derived value at
// the given altitude before the density and speed-of-sound calculation.
AtmoState isa_atmosphere(const IsaConditions& c);

} // namespace pon
