// poncelet example — shaped charge & EFP: Birkhoff/PER jet formation,
// standoff-dependent penetration, back-face spall (Phase 19 item 4, §8a).
// SPDX-License-Identifier: MIT
//
// A 100 mm HEAT warhead (copper cone) detonates against a 60 mm RHA plate. We
// form the jet, sweep the penetration against standoff to find the optimum,
// resolve the hit at a realistic standoff, and spawn the behind-armour debris
// (residual jet + plate spall) as projectiles that fly on into the crew space.
// Then the same charge as an EFP, to show it stays effective at long range.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_shaped_charge
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

int main() {
    // --- The RHA plate the crew sits behind. -------------------------------
    pon::Material rha;
    rha.name = "RHA"; rha.behaviour = pon::MaterialBehaviour::Ductile;
    rha.density_kgm3 = 7850.0; rha.strength_Pa = 1.0e9; rha.thickness_m = 0.060;
    const double plate_m = rha.thickness_m;

    // --- 100 mm HEAT warhead. --------------------------------------------
    pon::ShapedChargeDesc heat;
    heat.linerMass_kg       = 0.35;         // copper cone
    heat.chargeDiameter_m   = 0.100;        // CD
    heat.kind               = pon::ShapedChargeType::ConicalJet;
    heat.coneApexAngle_rad  = 60.0 * 3.14159265 / 180.0;
    heat.gurneyVelocity_mps = 2930.0;       // Octol
    const double fill_kg    = 0.6;          // explosive behind the liner

    const pon::JetFormation jet = pon::shaped_charge_formation(heat, fill_kg);
    std::printf("jet: tip %.0f m/s  tail %.0f m/s  jet mass %.0f g  slug %.0f g\n",
                jet.tipVelocity_mps, jet.tailVelocity_mps,
                jet.jetMass_kg * 1e3, jet.slugMass_kg * 1e3);
    std::printf("     L0 %.0f mm  coherent %.0f mm  breakup standoff %.2f m\n",
                jet.initialLength_m * 1e3, jet.coherentLength_m * 1e3,
                jet.breakupStandoff_m);

    // --- Standoff sweep. --------------------------------------------------
    double bestP = 0, bestS = 0;
    for (double s = 0.05; s <= 1.5; s += 0.05) {
        const auto p = pon::shaped_charge_penetration(heat, rha, s, -1.0, fill_kg);
        if (p.depth_m > bestP) { bestP = p.depth_m; bestS = s; }
    }
    const double optS = pon::shaped_charge_optimal_standoff(heat, fill_kg);
    std::printf("best penetration %.0f mm at %.2f m standoff "
                "(optimal-standoff estimate %.2f m = %.1f CD)\n",
                bestP * 1e3, bestS, optS, optS / heat.chargeDiameter_m);

    // --- Resolve the hit at the optimal standoff. -----------------------
    const auto pen = pon::shaped_charge_penetration(heat, rha, optS, plate_m, fill_kg);
    std::printf("vs %.0f mm RHA @ %.2f m: depth %.0f mm, %s (efficiency %.2f)\n",
                plate_m * 1e3, optS, pen.depth_m * 1e3,
                pen.perforated ? "PERFORATED" : (pen.spall ? "back-face spall" : "stopped"),
                pen.standoffEfficiency);

    int spawned = 0;
    if (pen.perforated || pen.spall) {
        std::vector<pon::FragmentSpec> debris;
        pon::shaped_charge_behind_armour(heat, rha, pon::Vec3{0, 0, 0},
                                         pon::Vec3{1, 0, 0}, optS, plate_m, fill_kg,
                                         debris);
        pon::Sim sim;
        std::vector<pon::StateId> ids;
        spawned = static_cast<int>(pon::spawn_fragments(
            sim, debris, pon::FidelityTier::Integrated, "bah", 6, &ids));

        struct Empty final : pon::World {
            bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
            pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
            const pon::Material& material(pon::SurfaceId) const override {
                static pon::Material a; return a;
            }
        } world;
        pon::VectorEventSink sink;
        double reach = 0;
        for (int i = 0; i < 400; ++i) {
            sim.step(1.0 / 1000.0, world, sink);
            for (pon::StateId h : ids) reach = std::max(reach, sim.state(h).position.x);
        }
        std::printf("behind-armour: %d debris projectiles, reached %.1f m into the compartment\n",
                    spawned, reach);
    }

    // --- Same charge as an EFP: shallow penetration, but effective far off. ---
    pon::ShapedChargeDesc efp = heat;
    efp.kind = pon::ShapedChargeType::EFP;
    const auto near_efp = pon::shaped_charge_penetration(efp, rha, 1.0, plate_m, fill_kg);
    const auto far_efp  = pon::shaped_charge_penetration(efp, rha, 50.0, plate_m, fill_kg);
    std::printf("EFP: %.0f mm @ 1 m (eff %.2f)  vs  %.0f mm @ 50 m (eff %.2f)\n",
                near_efp.depth_m * 1e3, near_efp.standoffEfficiency,
                far_efp.depth_m * 1e3, far_efp.standoffEfficiency);

    // ctest gate: the jet forms at hypervelocity, the HEAT round drills several
    // CD and perforates the plate, best standoff is a few CD, debris flies on,
    // and the EFP is shallower but holds efficiency to long standoff.
    const bool ok =
        jet.tipVelocity_mps > 6000.0 && jet.tipVelocity_mps < 11000.0 &&
        bestP > 4.0 * heat.chargeDiameter_m && bestP < 9.0 * heat.chargeDiameter_m &&
        bestS > 1.5 * heat.chargeDiameter_m && bestS < 7.0 * heat.chargeDiameter_m &&
        pen.perforated && spawned > 0 &&
        near_efp.depth_m < bestP && far_efp.standoffEfficiency > 0.5;
    return ok ? 0 : 1;
}
