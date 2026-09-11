// poncelet — terminal-ballistics resolution (internal). §3.5 + §3.8.
// SPDX-License-Identifier: MIT
//
// Not FP-contract-off and not on the sub-stepped hot path: this runs once per
// confirmed impact (target < ~2 µs). The whole pipeline is closed-form — a
// ricochet test, a De Marre-class ballistic-limit check, and the Poncelet
// penetration ODE solved analytically for depth (embed) or residual velocity
// (perforate). No per-impact integration.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/material.hpp"

namespace pon::detail {

// What the arriving projectile presents to the surface. Built by sim.cpp from
// the ProjectileType terminal params + the live ProjectileState.
struct TerminalProjectile {
    Real mass_kg        = 0.0;
    Real diameter_m     = 0.0;   // current (expanded) presented diameter
    Real speed_mps      = 0.0;   // closing speed |v_proj - v_surface| at impact
    Real noseShapeFactor = 1.0;  // N*: sharp broadhead < round < flat
    Real hardness       = 1.0;
    Real impactYaw      = 0.0;   // 0 clean .. 1 fully keyholed
    bool deformable     = false;
    bool fragile        = false;
    bool alreadyExpanded = false;
};

enum class TerminalOutcome {
    Ricochet,   // deflected off the surface, still flying
    Embedded,   // stopped inside the material at `channelDepth_m`
    Perforated, // punched through, continues at `residualSpeed_mps`
    Stopped,    // stopped at the surface (no meaningful channel)
    Shattered,  // fragile projectile broke up
    EnteredFluid, // Fluid behaviour — sim switches medium, no discrete stop
};

struct TerminalResult {
    TerminalOutcome outcome = TerminalOutcome::Stopped;
    Real  residualSpeed_mps = 0.0;   // Ricochet / Perforated
    Vec3  exitDir           = {0,0,0}; // unit; Ricochet / Perforated
    Vec3  exitPoint         = {0,0,0}; // Perforated: far-face exit (world frame)
    Real  channelDepth_m    = 0.0;    // Embedded depth / Perforated near-face path
    Real  channelWiden_m    = 0.0;    // soft-media yaw widening (L_yaw ~ 12 d)
    Real  energyDeposited_J = 0.0;
    Real  ballisticLimit_mps = 0.0;   // informational
    Real  newDiameter_m     = 0.0;    // >0 ⇒ deformable round mushroomed
    Real  newImpactYaw      = 0.0;
    Real  spinScale         = 1.0;
};

// Resolve one impact. `vDir` is the projectile's unit travel direction, `n` the
// unit surface normal from the swept hit (points back toward the projectile, so
// dot(vDir,n) < 0). `hitPoint` is the world contact point.
TerminalResult resolve_terminal(const TerminalProjectile& p, const Material& mat,
                                Vec3 vDir, Vec3 n, Vec3 hitPoint);

} // namespace pon::detail
