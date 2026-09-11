// poncelet — flight environment: gravity, air, wind, media.
// SPDX-License-Identifier: MIT
//
// Item 3: the ISA model (altitude / pressure / temperature / humidity ->
// density AND local speed of sound) feeds the two scalar fields below via
// setAtmosphere().
// Item 5: the wind callback and the medium registry below are wired all the way
// into the integrator's force model (relative airspeed for drag; per-medium
// density / drag-scale / buoyancy). Water-entry impulse and supercavitation
// dynamics are refined in item 7.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/atmosphere.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace pon {

// MediumId / kMediumAir / kMediumWater are in types.hpp.

// One entry in the medium registry (§3.3). `air` and `water` are seeded; games
// register their own.
struct MediumDesc {
    std::string name         = "air";
    KgPerM3     density_kgm3  = 1.225;
    Real        dragScale     = 1.0;   // multiplies the drag deceleration
    // Upward acceleration as a fraction of |gravity| (medium-level buoyancy
    // model, §3.3). 0 = none; 1 = neutrally buoyant; >1 = floats up. The water
    // default assumes a dense metal projectile — it still sinks, slowly.
    Real        buoyancy      = 0.0;
    bool        supercavitation = false;
};

class MediumRegistry {
public:
    MediumRegistry() {
        media_.push_back({"air",   1.225,  1.0, 0.0, false});
        // Water density alone is ~815x air, which is what shortens a rifle
        // round's underwater travel to ~1 m; dragScale stays 1.0 (a per-medium
        // fudge factor, not a second density term). buoyancy 0.1: a dense metal
        // projectile still sinks. Water-entry surface impulse is item 7.
        media_.push_back({"water", 1000.0, 1.0, 0.1, false});
    }
    MediumId add(const MediumDesc& d) {
        media_.push_back(d);
        return static_cast<MediumId>(media_.size() - 1);
    }
    const MediumDesc& get(MediumId id) const {
        const std::size_t i = (id >= 0 && static_cast<std::size_t>(id) < media_.size())
                                  ? static_cast<std::size_t>(id) : 0;
        return media_[i];
    }
    std::size_t count() const { return media_.size(); }

private:
    std::vector<MediumDesc> media_;
};

// wind(pos, t) -> wind velocity in m/s, world frame. Relative airspeed
// v_rel = v_proj - v_wind feeds drag.
using WindField = std::function<Vec3(Vec3 pos, Seconds t)>;

struct Environment {
    Vec3 gravity = {0.0, -9.80665, 0.0};

    // Air state the hot loop actually reads. Defaults are ISA sea level, 15 C,
    // dry. Call setAtmosphere() to derive both from firing-site conditions, or
    // set them directly for a flat override / arcade feel.
    KgPerM3      airDensity_kgm3   = 1.225;
    MetersPerSec speedOfSound_mps  = 340.294;
    // Kinematic viscosity ν = μ/ρ of the air, m²/s. Only read by the
    // BallProfile spheres, whose drag LUT is Cd(Reynolds) and Re = |v| d / ν.
    // ISA sea level, 15 °C, dry. setAtmosphere() derives it (Sutherland μ / ρ).
    Real         airKinematicViscosity_m2s = 1.4607e-5;

    // Resolve `conditions` through the ISA model and store the resulting air
    // density + local speed of sound. The speed of sound is what maps
    // trajectory speed onto the G1/G7 Cd(Mach) curve.
    void setAtmosphere(const IsaConditions& conditions) {
        const AtmoState a = isa_atmosphere(conditions);
        airDensity_kgm3  = a.density_kgm3;
        speedOfSound_mps = a.speedOfSound_mps;
        airKinematicViscosity_m2s = a.kinematicViscosity_m2s;
        siteTemperature_K = a.temperature_K;
    }

    // Firing-site air temperature (K) from the last setAtmosphere() call. Only
    // read under PrecisionFlag::LocalSpeedSound, where the Mach abscissa is
    // scaled by sqrt(T(altitude)/siteTemperature) down the trajectory so a
    // high-angle shot's drag curve tracks the ISA lapse rate.
    Kelvin siteTemperature_K = 288.15;

    WindField wind; // empty -> still air

    // Coriolis / Eotvos inputs (§3.7), only read when PrecisionFlag::Coriolis
    // is set on the shot.
    Radians latitude      = 0.0;
    Radians northAzimuth  = 0.0; // firing azimuth measured from true north

    MediumRegistry media;

    Vec3 windAt(Vec3 pos, Seconds t) const { return wind ? wind(pos, t) : Vec3{}; }

    // Convenience: set the Coriolis inputs (PrecisionFlag::Coriolis). `latitude`
    // is signed (northern positive); `firingAzimuthFromNorth` is the compass
    // bearing of the shot's downrange direction (0 = north, clockwise).
    void setCoriolis(Radians latitude_, Radians firingAzimuthFromNorth) {
        latitude     = latitude_;
        northAzimuth = firingAzimuthFromNorth;
    }
};

// Muzzle-velocity shift from powder temperature (§3.7): a linear sensitivity
// (m/s per K) about a reference temperature. The caller decides whether to add
// per-shot jitter on top. Not applied automatically anywhere — it is an input
// helper.
inline MetersPerSec mv_from_powder_temp(MetersPerSec baseMv_mps, Kelvin baseT_K,
                                        Real sensitivity_mps_per_K, Kelvin T_K) {
    return baseMv_mps + sensitivity_mps_per_K * (T_K - baseT_K);
}

} // namespace pon
