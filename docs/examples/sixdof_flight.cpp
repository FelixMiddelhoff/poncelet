// poncelet example — 6-DOF rigid-body flight (Phase 19 item 1).
// SPDX-License-Identifier: MIT
//
// PrecisionFlag::SixDOF integrates the projectile's orientation and angular
// velocity alongside the trajectory, so tumbling, the yaw of repose (and the
// spin drift it produces), epicyclic coning and its damping all emerge from the
// aerodynamics instead of being stored curves.
//
// This fires a .308 Win / 175 gr match round three ways, all with a 2° muzzle
// tip-off, and prints the peak angle of attack and the wind-independent drift:
//   1. full barrel-twist spin  — gyroscopically stable, nose tracks the path
//   2. no spin                 — statically unstable, tumbles end over end
//   3. spin, longer range      — the yaw of repose walks the shot right
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_sixdof_flight
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

pon::TypeId register_308(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id                   = "762x51_175smk";
    t.klass                = pon::ProjectileClass::Bullet;
    t.dragModel            = pon::DragModel::G7;
    t.ballisticCoefficient = 0.243;
    t.mass_kg              = 0.01134;
    t.refDiameter_m        = 0.00782;
    t.twistRate_m          = 0.254;   // 1-in-10", right-hand
    // t.aero left at 0 — length, inertia and the moment coefficients are seeded
    // from mass / diameter / class. A game tunes t.aero per round against data.
    return sim.registerType(t);
}

struct Result { double maxYaw_deg, endYaw_deg, driftRight_m, spinKept; bool tumbled; };

Result fire(double spin_radps, double range_m, double initialYaw_rad) {
    pon::Sim sim;
    const pon::TypeId id = register_308(sim);

    pon::LaunchParams shot;
    shot.position   = {0.0, 3000.0, 0.0};   // high start so it never hits ground
    shot.direction  = {1.0, 0.0, 0.0};
    shot.speed      = 790.0;
    shot.tier       = pon::FidelityTier::Integrated;
    shot.precision  = pon::PrecisionFlag::SixDOF;
    shot.spin       = spin_radps;
    if (initialYaw_rad != 0.0) shot.initialYaw = initialYaw_rad;
    const pon::StateId h = sim.spawn(id, shot);
    const double spin0 = sim.state(h).angVel_radps.x;

    pon::EmptyWorld world;
    pon::VectorEventSink events;
    double maxYaw = 0.0;
    for (int i = 0; i < 200000 && sim.state(h).alive &&
                    sim.state(h).position.x < range_m; ++i) {
        sim.step(1.0 / 2000.0, world, events);
        maxYaw = std::max(maxYaw, sim.state(h).angleOfAttack_rad);
    }
    const pon::ProjectileState& s = sim.state(h);
    const double deg = 180.0 / 3.14159265358979;
    return { maxYaw * deg, s.angleOfAttack_rad * deg, -s.position.z,
             spin0 != 0.0 ? s.angVel_radps.x / spin0 : 0.0,
             (s.flags & pon::kFlagTumbling) != 0 };
}

} // namespace

int main() {
    const double spin = 2.0 * 3.14159265358979 * 790.0 / 0.254; // ~19.5 krad/s

    const Result spun   = fire(spin, 700.0,  0.035); // 2° tip-off — coning damps
    const Result unspun = fire(0.0,  400.0,  0.035);
    const Result far    = fire(spin, 1000.0, 0.0);   // clean — pure yaw of repose

    std::printf("spin  @700 m : peak yaw %5.1f deg, settles to %.2f deg, "
                "spin kept %.0f%%, tumbled=%d\n",
                spun.maxYaw_deg, spun.endYaw_deg, spun.spinKept * 100.0, spun.tumbled);
    std::printf("nospin@400 m : peak yaw %5.1f deg, tumbled=%d\n",
                unspun.maxYaw_deg, unspun.tumbled);
    std::printf("spin  @1000 m: yaw of repose drift %+.2f m right\n",
                far.driftRight_m);

    // ctest gate: the spun round stays point-first and keeps most of its spin;
    // the unspun round tumbles; the yaw of repose drifts the long shot right.
    const bool ok = !spun.tumbled && spun.maxYaw_deg < 30.0 &&
                    spun.spinKept > 0.6 &&
                    unspun.tumbled && unspun.maxYaw_deg > 60.0 &&
                    far.driftRight_m > 0.05 && far.driftRight_m < 3.0;
    return ok ? 0 : 1;
}
