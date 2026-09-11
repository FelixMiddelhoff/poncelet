// poncelet — class-default gap filling for ProjectileType.
// SPDX-License-Identifier: MIT
#pragma once

#include "poncelet/projectile.hpp"

namespace pon::detail {

// Fill any zero/nullopt field of `t` from the coarse class defaults
// (docs/ballistics-phase-plan.md §4, via data/class_defaults.csv). The named
// catalog (<poncelet/catalog.hpp>) layers on top of this. Returns false if the
// result is still unusable.
bool apply_class_defaults(ProjectileType& t);

} // namespace pon::detail
