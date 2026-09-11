// poncelet — the BallProfile table (§3.2, §4). Internal.
// SPDX-License-Identifier: MIT
//
// Each sports ball / thrown rock is a *named profile*, not a generic sphere: it
// carries its own Cd(Reynolds) curve (with the drag crisis where that ball's
// surface actually puts it), its own signed Cl(spin parameter) Magnus curve,
// and a shape tag. `SportsBall` / `Rock` projectiles pick one by string id via
// ProjectileType::ballProfile; a missing / unknown id falls back to a plain
// rough sphere.
//
// Not FP-contract-off: consumed only by compile_drag_lut() at registerType().
#pragma once

#include "poncelet/drag.hpp"
#include "poncelet/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace pon::detail {

struct BallProfile {
    const char* id            = "";
    BallShape   shape         = BallShape::Sphere;
    Real        diameter_m    = 0;   // sphere Ø, or spheroid long axis, or puck Ø
    Real        mass_kg       = 0;
    Real        typicalSpeed_mps = 0;

    DragAbscissa abscissa     = DragAbscissa::Reynolds; // spheres: Re; others: Mach
    std::vector<DragCurvePoint> cdCurve;  // {abscissa value, Cd}
    std::vector<DragCurvePoint> clCurve;  // {S, Cl}; empty ⇒ no Magnus

    // ProlateSpheroid only.
    Real cdSpiral = 0;
    Real cdTumble = 0;
    // Radius used in the spin parameter S = |ω| r / |v|. Sphere: Ø/2. Set
    // explicitly for the non-spherical shapes.
    Real spinRadius_m = 0;
};

// Look up a profile by id. Returns nullptr if unknown.
const BallProfile* find_ball_profile(const std::string& id);

// The fallback when a SportsBall / Rock names no profile or an unknown one.
const BallProfile& default_ball_profile();

} // namespace pon::detail
