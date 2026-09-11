// poncelet — internal accessors onto the baked tables, for defaults.cpp and
// ball_profiles.cpp. Public code uses <poncelet/catalog.hpp>.
// SPDX-License-Identifier: MIT
#pragma once

#include "poncelet/types.hpp"
#include "ball_profiles.hpp"

#include <vector>

namespace pon::detail {

// Resolved class-default row (docs/ballistics-phase-plan.md §4).
struct ClassDefaults {
    Real      refDiameter_m        = 0.010;
    Real      mass_kg              = 0.010;
    DragModel dragModel            = DragModel::ConstantCd;
    Real      dragCoefficient      = 0.30;  // ConstantCd
    Real      ballisticCoefficient = 0.0;   // G1/G7
    Real      muzzleSpeed_mps      = 100.0;
    Real      noseShapeFactor      = 0.90;
};

ClassDefaults class_defaults_for(ProjectileClass k);

// The baked BallProfile table (data/ball_profiles.csv). Built once.
const std::vector<BallProfile>& baked_ball_profiles();

} // namespace pon::detail
