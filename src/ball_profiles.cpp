// poncelet — BallProfile lookup over the baked table.
// SPDX-License-Identifier: MIT
//
// The table itself is data/ball_profiles.csv, baked to src/generated/
// data_tables.inc and translated by src/catalog.cpp
// (detail::baked_ball_profiles()). Each sports ball / thrown rock is a *named
// profile*, not a generic sphere: its own Cd curve (drag crisis where that
// ball's surface actually puts it), its own signed Cl(spin parameter) Magnus
// curve, and a shape tag. `SportsBall` / `Rock` types pick one by string id via
// ProjectileType::ballProfile; a missing / unknown id falls back to a plain
// rough sphere.
#include "ball_profiles.hpp"

#include "catalog_internal.hpp"

namespace pon::detail {

const BallProfile* find_ball_profile(const std::string& id) {
    if (id.empty()) return nullptr;
    for (const BallProfile& p : baked_ball_profiles())
        if (id == p.id) return &p;
    return nullptr;
}

const BallProfile& default_ball_profile() {
    static const BallProfile fallback = [] {
        BallProfile p;
        p.id = "_rough_sphere";
        p.shape = BallShape::Sphere;
        p.diameter_m = 0.070;
        p.mass_kg = 0.150;
        p.typicalSpeed_mps = 25.0;
        p.abscissa = DragAbscissa::Reynolds;
        p.cdCurve = {{0.0, 0.50}, {1.5e6, 0.50}};
        p.spinRadius_m = 0.035;
        return p;
    }();
    return fallback;
}

} // namespace pon::detail
