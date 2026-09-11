// poncelet example — a spinning ball (the BallProfile / Magnus system).
// SPDX-License-Identifier: MIT
//
// Fires the same fastball twice — once with backspin, once with a vertical
// spin axis (a curveball) — and prints how far each one has moved off the
// straight line by the time it reaches the plate.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_curveball
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

// Distance the ball has drifted sideways / vertically off the aim line by the
// time it passes `x_plate` metres downrange.
struct Break { double side_m, drop_m, speed_mps; };

Break pitch(const char* label, pon::Vec3 spinAxis, double spin_radps) {
    pon::Sim sim;

    pon::ProjectileType ball;
    ball.id           = "baseball";
    ball.klass        = pon::ProjectileClass::SportsBall;
    ball.dragModel    = pon::DragModel::BallProfile;
    ball.ballProfile  = "baseball";          // fills Ø / mass / typical speed
    ball.spinAxisMode = pon::SpinAxisMode::Fixed;
    const pon::TypeId kBall = sim.registerType(ball);

    pon::LaunchParams shot;
    shot.position  = {0.0, 1.8, 0.0};
    shot.direction = {1.0, 0.0, 0.0};
    shot.speed     = 40.0;                   // ~90 mph
    shot.spin      = spin_radps;
    shot.spinAxis  = spinAxis;
    shot.tier      = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(kBall, shot);

    pon::EmptyWorld world;
    pon::VectorEventSink events;
    while (sim.state(h).position.x < 18.44 && sim.state(h).alive)
        sim.step(1.0 / 1000.0, world, events);

    const pon::ProjectileState& s = sim.state(h);
    const Break b{s.position.z, 1.8 - s.position.y, pon::length(s.velocity)};
    std::printf("%-10s  side %+.3f m   drop %.3f m   %.0f m/s at the plate\n",
                label, b.side_m, b.drop_m, b.speed_mps);
    return b;
}

} // namespace

int main() {
    // Backspin: a horizontal axis square to the pitch. s_hat x v points up, so
    // the Magnus force fights gravity — a "rising" fastball that drops less.
    pitch("fastball", {0.0, 0.0, 1.0}, 190.0);

    // Curveball: a vertical spin axis. s_hat x v points to -z, so the ball
    // sweeps sideways.
    const Break curve = pitch("curveball", {0.0, 1.0, 0.0}, 190.0);

    return std::fabs(curve.side_m) > 0.1 ? 0 : 1; // sanity gate for ctest
}
