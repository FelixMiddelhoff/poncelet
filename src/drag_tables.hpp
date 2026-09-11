// poncelet — standard drag functions (G1/G7) and the per-type drag-LUT
// compiler. Internal.
// SPDX-License-Identifier: MIT
//
// Not FP-contract-off: the LUT is built once at registerType(), never in the
// sub-stepped hot loop. Only + - * / here — no transcendentals.
#pragma once

#include "poncelet/drag.hpp"
#include "poncelet/projectile.hpp"

namespace pon::detail {

// Fill `out` (the per-type aerodynamic tables) from `t`'s drag model. Assumes
// t.refDiameter_m and t.mass_kg are final (class defaults + BallProfile dims
// already resolved).
//   * ConstantCd  — flat Cd(Mach) LUT at t.dragCoefficient.
//   * G1 / G7     — the standard curve, scaled by the SI form factor derived
//                   from t.ballisticCoefficient (a published lb/in^2 BC).
//   * CustomCurve — t.customDragCurve resampled; falls back to ConstantCd if
//                   empty.
//   * BallProfile — the named profile's Cd(Reynolds) curve (spheres) or a flat
//                   Cd(Mach) spiral level (ProlateSpheroid / Disc), plus its
//                   signed Cl(spin-parameter) Magnus curve and shape data.
// Sets out.dragRet / out.abscissa / out.refArea_m2 / out.liftCoeffVsSpin /
// out.liftRadius_m / out.liftAreaOver2m / out.shape / out.cdSpiral / out.cdTumble.
void compile_drag_lut(const ProjectileType& t, DragLuts& out);

// Standard-projectile drag coefficient at a Mach number, linearly interpolated
// from the tabulated curve. Exposed for tests / tooling.
Real g1_cd(Real mach);
Real g7_cd(Real mach);

} // namespace pon::detail
