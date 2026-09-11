// poncelet example — wind field + medium registry (Phase 18 item 5).
// SPDX-License-Identifier: MIT
//
// Part 1: fire one shot in still air and the same shot into a steady 12 m/s
//         crosswind, and print how far the wind walks it off the aim line.
// Part 2: fire a rifle round that is already underwater and watch ~815x the air
//         density bleed 800 m/s off it inside a couple of metres.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_wind_and_media
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

pon::TypeId register_slug(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id             = "10g_slug";
    t.klass          = pon::ProjectileClass::Bullet;
    t.dragModel      = pon::DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    t.mass_kg        = 0.010;
    t.refDiameter_m  = 0.0095;
    return sim.registerType(t);
}

// Lateral drift at the ground for a flat 300 m/s shot along +x.
double crosswind_drift(pon::Vec3 wind) {
    pon::Sim sim;
    const pon::TypeId id = register_slug(sim);
    sim.environment().wind = [wind](pon::Vec3, pon::Seconds) { return wind; };

    pon::LaunchParams shot;
    shot.position  = {0.0, 1.8, 0.0};
    shot.direction = {1.0, 0.0, 0.0};
    shot.speed     = 300.0;
    shot.tier      = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(id, shot);

    pon::EmptyWorld world;
    pon::VectorEventSink events;
    for (int i = 0; i < 20000 && sim.state(h).position.y > 0.0; ++i)
        sim.step(1.0 / 1000.0, world, events);
    return sim.state(h).position.z;
}

// A world that is water everywhere and has no geometry — just enough to keep a
// submerged projectile in the water medium. A real game returns kMediumWater
// only below its water surface (see docs/examples/custom_world.cpp).
struct OpenWater final : pon::World {
    bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumWater; }
    const pon::Material& material(pon::SurfaceId) const override {
        static const pon::Material m = pon::material_water();
        return m;
    }
};

} // namespace

int main() {
    const double still = crosswind_drift({0.0, 0.0, 0.0});
    const double windy = crosswind_drift({0.0, 0.0, 12.0});
    std::printf("crosswind: still air %+.3f m   12 m/s wind %+.3f m of drift\n",
                still, windy);

    // --- underwater ---------------------------------------------------------
    pon::Sim sim;
    const pon::TypeId id = register_slug(sim);
    pon::LaunchParams dive;
    dive.position  = {0.0, 0.0, 0.0};
    dive.direction = {1.0, 0.0, 0.0};
    dive.speed     = 800.0;
    dive.tier      = pon::FidelityTier::Integrated;
    dive.medium    = pon::kMediumWater;          // spawn already submerged
    const pon::StateId h = sim.spawn(id, dive);

    OpenWater water;
    pon::VectorEventSink events;
    for (int i = 0; i < 8000 && sim.state(h).alive &&
                    sim.state(h).position.x < 2.0; ++i)
        sim.step(1.0 / 4000.0, water, events);

    const pon::ProjectileState& s = sim.state(h);
    std::printf("underwater: %.0f m/s left after %.2f m (from 800 m/s)\n",
                pon::length(s.velocity), s.position.x);

    // ctest sanity gates: the wind must push the shot with it, and water must
    // take most of the round's speed inside 2 m.
    const bool wind_ok  = windy > still + 0.3;
    const bool water_ok = pon::length(s.velocity) < 200.0;
    return (wind_ok && water_ok) ? 0 : 1;
}
