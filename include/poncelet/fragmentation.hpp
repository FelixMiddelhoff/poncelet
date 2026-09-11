// poncelet — casing fragmentation: Mott mass spectrum + Gurney launch speed,
// each fragment a poncelet projectile with its own drag & penetration
// (Phase 19 item 3, §8a).
// SPDX-License-Identifier: MIT
//
// A `WarheadDesc::fragmentation` block turns a detonation into a fragment spray.
// Off a Detonated event (or a hand-placed point) call:
//
//   std::vector<pon::FragmentSpec> frags;
//   pon::generate_fragments(shell.warhead.fragmentation, det.point, shellVel,
//                           shell.warhead.chargeMass_kg, frags);
//   pon::spawn_fragments(sim, frags, pon::FidelityTier::Integrated, "frag");
//
// generate_fragments is a pure, deterministic function of its inputs (seeded
// RNG, no <random>, no allocation beyond `out`). It never touches the flight
// core — the fragment spectrum is a set of engineering fits.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/warhead.hpp"
#include "poncelet/sim.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace pon {

// One fragment, ready to become a projectile. `velocity` already includes the
// warhead's own velocity at detonation (fragments inherit it).
struct FragmentSpec {
    Vec3      position;                 // = the detonation point
    Vec3      velocity;                 // world frame, m/s
    Kilograms mass_kg          = 0.0;
    Meters    diameter_m       = 0.0;   // sphere-equivalent for the drag area
    Real      dragCoefficient  = 1.10;
    Real      representsCount   = 1.0;  // real fragments this spec stands for
                                        // (>1 when the casing yields more than
                                        //  FragmentationDesc::maxFragments)
};

// Effective Gurney fragment velocity for a cylindrical casing (m/s):
//   V0 = gurneyVelocity · (M/C + 1/2)^(-1/2),  M = casing mass, C = charge mass.
MetersPerSec gurney_fragment_velocity(const FragmentationDesc& d,
                                      Kilograms chargeMass_kg);

// Mott mean-fragment-mass parameter µ (kg): from `mottMu_kg` if set, else the
// casing-geometry formula, else a coarse fallback from casing mass alone.
Kilograms mott_mu_kg(const FragmentationDesc& d);

// Fill `out` (cleared first) with the fragment spray. Returns the count written
// (<= maxFragments). `sourceVelocity` is the warhead velocity at detonation.
std::size_t generate_fragments(const FragmentationDesc& d, Vec3 origin,
                               Vec3 sourceVelocity, Kilograms chargeMass_kg,
                               std::vector<FragmentSpec>& out);

// Convenience: register up to `desc.massClasses` steel-fragment ProjectileTypes
// spanning the mass range of `specs` (geometric buckets) as
// "<idPrefix>_cNN", then spawn every fragment onto its nearest bucket at
// fidelity `tier`. Appends the new StateIds to `outIds` when given. Returns the
// number of shots spawned.
//
// NB: each call adds `massClasses` entries to the Sim's type table. For a game
// firing many identical fragmenting rounds, register a fragment-class set once
// (plan_fragment_classes) and spawn onto it yourself instead.
std::size_t spawn_fragments(Sim& sim, const std::vector<FragmentSpec>& specs,
                            FidelityTier tier, const std::string& idPrefix,
                            std::uint32_t massClasses = 12,
                            std::vector<StateId>* outIds = nullptr);

// A registered fragment mass class (for callers that manage their own type set).
struct FragmentClass {
    Kilograms mass_kg         = 0.0;   // bucket representative mass
    Meters    diameter_m      = 0.0;
    Real      dragCoefficient = 1.10;
};

// Partition `specs` into up to `maxClasses` geometric-mass buckets. Returns the
// non-empty class list (ascending mass); `classOf` (when given, sized to
// specs.size()) receives each fragment's class index.
std::vector<FragmentClass> plan_fragment_classes(
    const std::vector<FragmentSpec>& specs, std::uint32_t maxClasses,
    std::vector<std::uint32_t>* classOf = nullptr);

// Build the ProjectileType for one fragment class (steel chunk: blunt nose,
// hard, non-deformable, ConstantCd). `id` must be unique in the Sim.
ProjectileType fragment_class_type(const FragmentClass& c, const std::string& id);

} // namespace pon
