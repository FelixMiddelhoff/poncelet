// poncelet example — a predicted trajectory arc for an aim indicator, with no
// Sim and no World (pon::preview_arc).
// SPDX-License-Identifier: MIT
//
// Run: ./build/<cfg>/poncelet_example_aim_preview
#include <poncelet/poncelet.hpp>

#include <cstdio>
#include <vector>

int main() {
    // The round the player has loaded. A 40 mm grenade-ish arc — heavy, slow,
    // draggy — so the drop is dramatic and the preview clearly earns its keep.
    pon::ProjectileType grenade;
    grenade.id             = "40mm_he";
    grenade.klass          = pon::ProjectileClass::Shell;
    grenade.dragModel      = pon::DragModel::ConstantCd;
    grenade.dragCoefficient = 0.30;
    grenade.mass_kg        = 0.230;
    grenade.refDiameter_m  = pon::inches(1.57);       // 40 mm
    grenade.muzzleSpeed_mps = 76.0;

    pon::Environment env;                              // sea-level air, no wind

    // Where the player is aiming this frame: muzzle at 1.6 m, 25° up.
    pon::LaunchParams aim;
    aim.position  = {0, 1.6, 0};
    aim.direction = {std::cos(pon::degrees(25)), std::sin(pon::degrees(25)), 0};

    std::vector<pon::Vec3> arc;
    pon::preview_arc(grenade, aim, env,
                     /*dt*/ 0.05, /*maxTime*/ 8.0, arc, /*groundY*/ 0.0);

    std::printf("preview: %zu points\n", arc.size());
    for (std::size_t i = 0; i < arc.size(); i += 4)   // every ~0.2 s
        std::printf("  t=%4.2f s   x=%6.1f m   y=%5.1f m\n",
                    i * 0.05, arc[i].x, arc[i].y);
    if (!arc.empty())
        std::printf("lands at x=%.1f m after ~%.1f s\n",
                    arc.back().x, (arc.size() - 1) * 0.05);
    return 0;
}
