// poncelet example — a guided missile intercepting a crossing target with
// proportional navigation (Phase 19 item 5, §8a).
// SPDX-License-Identifier: MIT
//
// A surface-to-air missile is launched at a jet crossing left-to-right 1500 m
// downrange and 600 m up. Each frame we feed the seeker track to the Sim with
// pon::Sim::guide(); the built-in PN law steers via the per-step
// external-acceleration hook. We compare the PN miss distance against a
// pure-pursuit shot (which trails a crosser) and an unguided ballistic lob.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_guided_missile
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

struct Sky final : pon::World {
    bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override {
        static pon::Material a; return a;
    }
};

// Fly one missile to closest approach against the moving target. Returns the
// miss distance (m); writes the time of flight to `tof`.
double intercept(pon::Sim& sim, pon::TypeId type, pon::GuidanceLaw law,
                 pon::Vec3 tgt0, pon::Vec3 tgtVel, double& tof) {
    pon::LaunchParams lp;
    lp.position  = {0, 0, 0};
    lp.direction = {0.55, 0.84, 0};     // a lead-ish launch elevation
    lp.speed     = 350.0;
    lp.tier      = pon::FidelityTier::Integrated;
    const pon::StateId m = sim.spawn(type, lp);

    Sky sky;
    pon::VectorEventSink sink;
    const double dt = 1.0 / 200.0;
    double best = 1e30;
    pon::Vec3 tgt = tgt0;
    for (int i = 0; i < 4000 && sim.state(m).alive; ++i) {
        tgt = tgt + tgtVel * dt;
        if (law != pon::GuidanceLaw::None) sim.guide(m, tgt, tgtVel);
        sim.step(dt, sky, sink);
        const pon::Vec3 d = sim.state(m).position - tgt;
        const double r = pon::length(d);
        if (r < best) { best = r; tof = sim.state(m).timeAlive_s; }
        if (r < 3.0 || (r > best + 50.0 && best < 1e29)) break; // hit, or past CPA
    }
    return best;
}

pon::ProjectileType sam(const char* id, pon::GuidanceLaw law) {
    pon::ProjectileType t;
    t.id            = id;
    t.klass         = pon::ProjectileClass::Shell;
    t.dragModel     = pon::DragModel::ConstantCd;
    t.dragCoefficient = 0.20;
    t.mass_kg       = 22.0;
    t.refDiameter_m = 0.13;
    t.maxLifetime_s = 30.0;
    t.guidance.law             = law;
    t.guidance.navConstant     = 4.0;
    t.guidance.maxLateralAccel_g = 40.0;
    t.guidance.thrustAccel_mps2 = 250.0;   // a 2 s sustainer
    t.guidance.burnTime_s       = 2.0;
    return t;
}

} // namespace

int main() {
    const pon::Vec3 tgt0{1500, 600, -400};
    const pon::Vec3 tgtVel{0, 0, 260};      // 260 m/s crossing, left to right

    double tofPN = 0, tofPursuit = 0, tofBallistic = 0;

    pon::Sim simPN;
    const double missPN =
        intercept(simPN, simPN.registerType(sam("sam_pn", pon::GuidanceLaw::ProportionalNav)),
                  pon::GuidanceLaw::ProportionalNav, tgt0, tgtVel, tofPN);

    pon::Sim simPur;
    const double missPursuit =
        intercept(simPur, simPur.registerType(sam("sam_pursuit", pon::GuidanceLaw::Pursuit)),
                  pon::GuidanceLaw::Pursuit, tgt0, tgtVel, tofPursuit);

    pon::Sim simB;
    pon::ProjectileType dumb = sam("sam_dumb", pon::GuidanceLaw::None);
    const double missBallistic =
        intercept(simB, simB.registerType(dumb), pon::GuidanceLaw::None, tgt0, tgtVel,
                  tofBallistic);

    std::printf("crossing target at (%.0f, %.0f, %.0f), %.0f m/s\n",
                tgt0.x, tgt0.y, tgt0.z, pon::length(tgtVel));
    std::printf("  proportional nav : miss %6.1f m  (t+%.2fs)\n", missPN, tofPN);
    std::printf("  pure pursuit     : miss %6.1f m  (t+%.2fs)\n", missPursuit, tofPursuit);
    std::printf("  unguided         : miss %6.1f m  (t+%.2fs)\n", missBallistic, tofBallistic);

    // ctest gate: PN converges to a near-hit, and clearly beats pure pursuit,
    // which clearly beats the unguided lob.
    const bool ok = missPN < 10.0 &&
                    missPN < missPursuit &&
                    missPursuit < missBallistic &&
                    missBallistic > 100.0;
    return ok ? 0 : 1;
}
