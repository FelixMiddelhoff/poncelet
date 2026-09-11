// poncelet example — fire a named catalog round, then a game's own override.
// SPDX-License-Identifier: MIT
//
// The string id *is* the round: pon::catalog::get("id") returns a fully
// populated ProjectileType. A game loads its own CSV on top (later ids win).
#include <poncelet/poncelet.hpp>

#include <cstdio>

int main() {
    pon::Sim sim;

    // 1. A shipped round, straight from the catalog. No field juggling.
    if (!pon::catalog::has("762x51_175gr_smk")) {
        std::puts("catalog missing the baked entry");
        return 1;
    }
    pon::ProjectileType smk = pon::catalog::get("762x51_175gr_smk");
    const pon::TypeId kSmk = sim.registerType(smk);

    // 2. A game overrides one entry and adds a fictional one via CSV. A row
    //    whose id already exists replaces the baked entry.
    const char* game_csv =
        "id,klass,drag_model,mass_kg,ref_diameter_m,ballistic_coefficient,muzzle_speed_mps,nose_shape_factor\n"
        "762x51_175gr_smk,Bullet,G7,0.01134,0.00782,0.250,810,0.55\n"
        "plasma_bolt,Custom,ConstantCd,0.05,0.02,,1200,0.9\n";
    std::string err;
    const int applied = pon::catalog::load_csv_string(game_csv, &err);
    if (applied < 0) { std::printf("csv error: %s\n", err.c_str()); return 1; }
    std::printf("applied %d overlay row(s)\n", applied);

    const pon::TypeId kSmk2 = sim.registerType(pon::catalog::get("762x51_175gr_smk"));
    const pon::TypeId kBolt = sim.registerType(pon::catalog::get("plasma_bolt"));

    // 3. Fire each and report the retained velocity at ~1 s.
    pon::EmptyWorld world;
    pon::VectorEventSink events;
    auto fire = [&](pon::TypeId id, const char* label) {
        pon::LaunchParams shot;
        shot.position  = {0, 1.8, 0};
        shot.direction = {1, 0, 0};
        shot.tier      = pon::FidelityTier::Integrated;
        const pon::StateId s = sim.spawn(id, shot);
        for (int f = 0; f < 60; ++f) sim.step(1.0 / 60.0, world, events);
        const pon::ProjectileState& st = sim.state(s);
        std::printf("%-22s  x=%.0f m  v=%.0f m/s\n",
                    label, st.position.x, pon::length(st.velocity));
    };
    fire(kSmk,  "175 SMK (baked)");
    fire(kSmk2, "175 SMK (game BC 0.25)");
    fire(kBolt, "plasma_bolt (fictional)");

    // 4. A shipped material seed, for callers that want one.
    if (auto oak = pon::materials::find("oak"))
        std::printf("oak: %.0f kg/m^3, %.0f MPa\n",
                    oak->density_kgm3, oak->strength_Pa / 1e6);
    return 0;
}
