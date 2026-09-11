// poncelet example — long-range precision effects (Phase 18 item 6, §3.7).
// SPDX-License-Identifier: MIT
//
// Fires a .308 Win / 175 gr match round to 1000 m twice: once with the fast
// path, once with the precision flags (gyroscopic spin drift, Coriolis, local
// speed of sound, error-controlled RKF45 step, transonic-window flag) turned
// on. Prints the extra drift the second-order effects add.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_precision_long_range
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

pon::TypeId register_308(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id                = "762x51_175smk";
    t.klass             = pon::ProjectileClass::Bullet;
    t.dragModel         = pon::DragModel::G7;
    t.ballisticCoefficient = 0.243;          // published G7 BC
    t.mass_kg           = 0.01134;
    t.refDiameter_m     = 0.00782;
    t.twistRate_m       = 0.254;             // 1-in-10", right-hand
    t.millerStability   = 1.8;
    return sim.registerType(t);
}

struct Result { double drop_m, driftRight_m, tof_s, speed_mps; bool wentTransonic; };

Result fire(pon::PrecisionFlag precision) {
    pon::Sim sim;
    const pon::TypeId id = register_308(sim);
    sim.environment().setAtmosphere({ /*altitude_m*/ 1200.0 });   // a mountain range
    sim.environment().setCoriolis(0.75, 0.0);                      // ~43° N, firing north

    pon::LaunchParams shot;
    shot.position  = {0.0, 2.0, 0.0};
    shot.direction = {1.0, 0.0, 0.0};
    shot.speed     = 792.0;
    shot.tier      = pon::FidelityTier::Integrated;
    shot.precision = precision;
    const pon::StateId h = sim.spawn(id, shot);

    pon::EmptyWorld world;
    pon::VectorEventSink events;
    for (int i = 0; i < 400000 && sim.state(h).alive &&
                    sim.state(h).position.x < 1000.0; ++i)
        sim.step(1.0 / 2000.0, world, events);

    bool transonic = false;
    for (const pon::Event& e : events.events)
        if (e.type == pon::EventType::TransonicWindow) transonic = true;

    const pon::ProjectileState& s = sim.state(h);
    return { 2.0 - s.position.y, -s.position.z, s.timeAlive_s,
             pon::length(s.velocity),
             transonic || (s.flags & pon::kFlagInTransonic) ||
                          (s.flags & pon::kFlagPastTransonic) };
}

} // namespace

int main() {
    const pon::PrecisionFlag all =
        pon::PrecisionFlag::SpinDrift | pon::PrecisionFlag::Coriolis |
        pon::PrecisionFlag::LocalSpeedSound | pon::PrecisionFlag::AdaptiveRKF45 |
        pon::PrecisionFlag::TransonicFlag;

    const Result fast = fire(pon::PrecisionFlag::None);
    const Result prec = fire(all);

    std::printf("fast path : drop %.2f m  drift %+.3f m  %.0f m/s at 1000 m\n",
                fast.drop_m, fast.driftRight_m, fast.speed_mps);
    std::printf("precision : drop %.2f m  drift %+.3f m  %.0f m/s   transonic=%d\n",
                prec.drop_m, prec.driftRight_m, prec.speed_mps, prec.wentTransonic);
    std::printf("second-order drift added: %+.3f m right\n",
                prec.driftRight_m - fast.driftRight_m);

    // ctest gate: the precision run must add a right-ward drift of a few cm to
    // tens of cm (spin drift dominates at this range) and flag the transonic
    // crossing.
    const double extra = prec.driftRight_m - fast.driftRight_m;
    return (extra > 0.03 && extra < 1.0 && prec.wentTransonic) ? 0 : 1;
}
