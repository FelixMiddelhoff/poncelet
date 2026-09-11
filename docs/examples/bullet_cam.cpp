// poncelet example — trajectory cache replay for a bullet-cam / late join
// (Phase 18 item 12).
// SPDX-License-Identifier: MIT
//
// A single Integrated-tier shot is stepped once at the sim's frame rate with
// SimConfig::trajectoryCacheFrames set. The flight path is then a stored
// polyline: a slow-motion "bullet cam", or a client that joined the session
// late, reconstructs any in-between pose with sampleTrajectory() instead of
// re-integrating the shot.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_bullet_cam
#include <poncelet/poncelet.hpp>

#include <cstdio>

int main() {
    pon::SimConfig cfg;
    cfg.trajectoryCacheFrames = 512; // ~8.5 s of 60 Hz flight
    pon::Sim sim({}, cfg);

    pon::ProjectileType t;
    t.id = "762x51_175gr_smk";
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::G7;
    t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134;
    t.refDiameter_m = 0.00782;
    const pon::TypeId id = sim.registerType(t);

    pon::LaunchParams lp;
    lp.position  = {0, 1.8, 0};
    lp.direction = {1, 0.02, 0};
    lp.speed     = 790.0;
    lp.tier      = pon::FidelityTier::Integrated;
    const pon::StateId shot = sim.spawn(id, lp);

    pon::EmptyWorld world;
    pon::VectorEventSink sink;
    for (int f = 0; f < 180; ++f) sim.step(1.0 / 60.0, world, sink); // 3 s of flight

    std::printf("cached %zu frames of flight path\n", sim.trajectorySize(shot));

    // Bullet-cam: replay the first 0.5 s at 10x slow motion (200 Hz virtual
    // frames) straight out of the cache — no extra integration.
    std::puts("  t(s)      x(m)     y(m)     speed(m/s)");
    for (int k = 0; k <= 10; ++k) {
        const double tt = k * 0.05;
        pon::Vec3 p, v;
        if (sim.sampleTrajectory(shot, tt, p, v))
            std::printf("  %5.2f  %8.2f %8.3f  %8.2f\n",
                        tt, p.x, p.y, pon::length(v));
    }
    return 0;
}
