// poncelet — shaped charges & explosively formed penetrators: simplified
// Birkhoff/PER jet formation, standoff-dependent hydrodynamic penetration,
// back-face spall (Phase 19 item 4, §8a).
// SPDX-License-Identifier: MIT
//
// A `WarheadDesc::shapedCharge` block turns a detonation into a jet (HEAT) or a
// slug (EFP). Off a Detonated event (its `channelAxis` is the jet aim, its
// `point` the detonation origin) call:
//
//   const auto form = pon::shaped_charge_formation(sc, chargeMass_kg);
//   const auto pen  = pon::shaped_charge_penetration(sc, rha, standoff_m,
//                                                    plateThickness_m,
//                                                    chargeMass_kg);
//   if (pen.perforated || pen.spall) {
//       std::vector<pon::FragmentSpec> debris;
//       pon::shaped_charge_behind_armour(sc, rha, exitPoint, aimDir,
//                                        standoff_m, plateThickness_m,
//                                        chargeMass_kg, debris);
//       pon::spawn_fragments(sim, debris, pon::FidelityTier::Integrated, "bad");
//   }
//
// All of this is a set of engineering fits (PER steady-state jet + Allison-Vitali
// stretching + hydrodynamic P = L·√(ρj/ρt) with a strength cut). Like
// explosion.cpp / fragmentation.cpp it is NOT the fp-contract-off TU and never
// touches the trajectory core. Coefficients are seeds — tune per warhead.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/warhead.hpp"
#include "poncelet/material.hpp"
#include "poncelet/fragmentation.hpp"   // FragmentSpec, reused for behind-armour debris

#include <cstddef>
#include <vector>

namespace pon {

// Result of collapsing the liner (before it meets any target).
struct JetFormation {
    Kilograms    jetMass_kg        = 0.0;  // metal in the jet (ConicalJet) or slug (EFP)
    Kilograms    slugMass_kg       = 0.0;  // slow trailing slug (ConicalJet); 0 for EFP
    MetersPerSec tipVelocity_mps   = 0.0;  // fastest jet element / the EFP slug
    MetersPerSec tailVelocity_mps  = 0.0;  // slowest jet element (ConicalJet)
    MetersPerSec avgVelocity_mps   = 0.0;
    Meters       initialLength_m   = 0.0;  // jet length at the charge base (≈ L0)
    Meters       coherentLength_m  = 0.0;  // max stretched length before particulation
    Meters       breakupStandoff_m = 0.0;  // standoff at which the jet particulates
    Meters       jetDiameter_m     = 0.0;  // effective jet / slug diameter
    bool         isEFP             = false;
};

// Penetration of the jet/slug into a target of `target` material, arriving after
// `standoff_m` of flight from the charge. `targetThickness_m > 0` enables the
// perforation / spall verdict; <= 0 treats the target as semi-infinite.
struct ShapedChargePenetration {
    Meters       depth_m               = 0.0;  // into a semi-infinite target
    Meters       effectiveJetLength_m  = 0.0;  // stretched length reaching the face
    Real         standoffEfficiency    = 0.0;  // depth(standoff) / depth(optimal), 0..1
    Meters       holeDiameter_m        = 0.0;
    bool         perforated            = false;
    bool         spall                 = false; // back-face scabbing without full perforation
    Meters       residualLength_m      = 0.0;  // jet/slug left after perforation
    MetersPerSec residualVelocity_mps  = 0.0;  // of what exits the back face
};

// Collapse the liner. `chargeMass_kg` is the parent warhead's explosive fill
// (WarheadDesc::chargeMass_kg) — it sets the charge-to-metal ratio for the
// Gurney collapse velocity.
JetFormation shaped_charge_formation(const ShapedChargeDesc& d,
                                     Kilograms chargeMass_kg);

// Standoff (m) that maximises penetration: a few CD for a ConicalJet (just short
// of jet particulation), a large representative value for an EFP (it stays
// coherent — efficiency is near 1 over a wide band).
Meters shaped_charge_optimal_standoff(const ShapedChargeDesc& d,
                                      Kilograms chargeMass_kg);

// Penetrate `target` after `standoff_m` of flight.
ShapedChargePenetration shaped_charge_penetration(const ShapedChargeDesc& d,
                                                  const Material& target,
                                                  Meters standoff_m,
                                                  Meters targetThickness_m,
                                                  Kilograms chargeMass_kg);

// Behind-armour debris — the residual jet cone (on perforation) plus the plate
// spall cone (on perforation or near-perforation) — as FragmentSpecs ready for
// pon::spawn_fragments. `exitPoint` is where the jet leaves the back face,
// `aimDir` the (unit) jet axis. `out` is cleared first; returns the count
// written. Empty when the target was neither perforated nor spalled.
std::size_t shaped_charge_behind_armour(const ShapedChargeDesc& d,
                                        const Material& target, Vec3 exitPoint,
                                        Vec3 aimDir, Meters standoff_m,
                                        Meters targetThickness_m,
                                        Kilograms chargeMass_kg,
                                        std::vector<FragmentSpec>& out);

} // namespace pon
