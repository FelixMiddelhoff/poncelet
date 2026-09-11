// poncelet example — fire one round, print the trajectory, react to the impact.
// SPDX-License-Identifier: MIT
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_basic_shot
#include <poncelet/poncelet.hpp>

#include <cstdio>

int main() {
    // 1. One Sim per world. Default environment = Earth sea-level air. Call
    //    sim.environment().setAtmosphere({altitude_m, ...}) for a real ISA
    //    density + speed of sound at the firing site.
    pon::Sim sim;

    // 2. Describe the projectile once, at load time. A G1 bullet: the standard
    //    drag curve, scaled to this round's published ballistic coefficient.
    pon::ProjectileType pistol;
    pistol.id                   = "9x19_124gr_fmj";
    pistol.klass                = pon::ProjectileClass::Bullet;
    pistol.dragModel            = pon::DragModel::G1;
    pistol.ballisticCoefficient = 0.15;   // published G1 BC (lb/in^2)
    pistol.mass_kg              = 0.00804;
    pistol.refDiameter_m        = 0.00902;
    pistol.muzzleSpeed_mps      = 360.0;

    const pon::TypeId kPistol = sim.registerType(pistol);
    if (kPistol == pon::kInvalidType) {
        std::puts("bad projectile description");
        return 1;
    }

    // 3. Fire. Leaving `speed` unset uses the type's muzzle speed.
    pon::LaunchParams shot;
    shot.position  = {0.0, 1.6, 0.0};  // muzzle 1.6 m up
    shot.direction = {1.0, 0.0, 0.0};  // down +X (need not be normalized)
    shot.tier      = pon::FidelityTier::Integrated;

    const pon::StateId bullet = sim.spawn(kPistol, shot);

    // 4. Geometry + event sink. EmptyWorld hits nothing; see custom_world.cpp.
    pon::EmptyWorld world;
    pon::VectorEventSink events;

    // 5. Step once per frame.
    for (int frame = 0; frame < 240; ++frame) {
        sim.step(1.0 / 60.0, world, events);

        const pon::ProjectileState& s = sim.state(bullet);
        if (frame % 15 == 0)
            std::printf("t=%.2fs  x=%.1fm  y=%.2fm  v=%.0f m/s\n",
                        s.timeAlive_s, s.position.x, s.position.y,
                        pon::length(s.velocity));
        if (!s.alive) break;
    }

    // 6. React.
    for (const pon::Event& e : events.events) {
        if (e.type == pon::EventType::Expired)
            std::printf("round expired after %.2f s / %.0f m\n",
                        e.time_s, sim.state(bullet).distanceTravelled_m);
        if (e.type == pon::EventType::Stopped)
            std::printf("impact at %.1f, %.2f  (%.0f J)\n",
                        e.point.x, e.point.y, e.energy_J);
    }
    return 0;
}
