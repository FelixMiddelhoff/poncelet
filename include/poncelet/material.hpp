// poncelet — target-material properties for terminal ballistics.
// SPDX-License-Identifier: MIT
//
// The struct + the always-available air / water helpers. The full seeded table
// (oak, concrete, mild steel, ...) is data/materials.csv, reachable through
// <poncelet/catalog.hpp> (pon::materials::find / load_csv).
#pragma once

#include "poncelet/types.hpp"

#include <cstdint>

namespace pon {

using SurfaceId = std::uint32_t;

struct Material {
    const char*       name        = "unknown";
    MaterialBehaviour behaviour   = MaterialBehaviour::Ductile;
    KgPerM3           density_kgm3 = 1000.0;
    // Resistive "hardness" the Poncelet model needs: compressive strength for
    // brittle, shear/flow stress for ductile, tear strength for membranes.
    Pascals           strength_Pa  = 1.0e7;
    Real              toughness    = 1.0e3;  // energy per unit crack area (J/m^2)
    Meters            thickness_m  = 0.1;    // slab thickness; large for bulk media
    Real              elasticity   = 0.2;    // restitution for the bounce case
    // Which medium a projectile enters if it perforates / enters this surface
    // (e.g. water surface -> kMediumWater). <0 keeps the current medium.
    MediumId          entersMedium = -1;
};

// Built-ins available before the data table loads.
inline Material material_air() {
    return {"air", MaterialBehaviour::Fluid, 1.225, 0.0, 0.0, 1.0e9, 0.0, kMediumAir};
}
inline Material material_water() {
    return {"water", MaterialBehaviour::Fluid, 1000.0, 0.0, 0.0, 1.0e9, 1.0, kMediumWater};
}

} // namespace pon
