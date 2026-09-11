// poncelet example — swept-vs-swept hit detection against a moving target
// (Phase 18 item 8).
// SPDX-License-Identifier: MIT
//
// A steel plate hangs at x = 60 m and swings toward the shooter at 25 m/s. The
// World reports the plate's frozen pose plus its velocity in HitResult; the
// stepper corrects the time-of-impact and hit point for the plate's motion over
// the sub-step, and reports the *closing* energy (½m·v_rel²), not the
// ground-frame energy. We fire the same shot at a static plate for comparison.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_moving_target
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

// One plane perpendicular to +x at `plateX`, animated by the caller between
// frames; `plateVel` is handed back so the stepper can sweep against a moving
// surface rather than a stale one.
class PlateWorld final : public pon::World {
public:
    PlateWorld(double x, pon::Vec3 v) : plateX_(x), plateVel_(v) {
        steel_.name = "steel_plate";
        steel_.behaviour = pon::MaterialBehaviour::Ductile;
        steel_.density_kgm3 = 7850;
        steel_.strength_Pa = 2.5e8;
        steel_.thickness_m = 0.012;
    }

    void advance(double dt) { plateX_ += plateVel_.x * dt; }

    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        const double da = a.x - plateX_, db = b.x - plateX_;
        if ((da > 0.0) == (db > 0.0)) return false; // no sign change ⇒ no crossing
        const double t = da / (da - db);
        out.t = t;
        out.point = a + (b - a) * t;
        out.normal = {da > 0.0 ? 1.0 : -1.0, 0, 0};
        out.surface = 0;
        out.surfaceVelocity = plateVel_; // the bit item 8 acts on
        return true;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override { return steel_; }

private:
    double       plateX_;
    pon::Vec3    plateVel_;
    pon::Material steel_;
};

pon::Event fire(pon::Vec3 plateVel) {
    pon::Sim sim;
    pon::ProjectileType t;
    t.id = "9x19_124gr_fmj";
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::G1;
    t.ballisticCoefficient = 0.15;
    t.mass_kg = 0.008;
    t.refDiameter_m = 0.009;
    const pon::TypeId id = sim.registerType(t);

    pon::LaunchParams lp;
    lp.position = {0, 1.6, 0};
    lp.direction = {1, 0, 0};
    lp.speed = 360.0;
    lp.tier = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(id, lp);

    PlateWorld world(60.0, plateVel);
    pon::VectorEventSink sink;
    const double dt = 1.0 / 120.0;
    for (int i = 0; i < 4000 && sim.state(h).alive; ++i) {
        sim.step(dt, world, sink);
        world.advance(dt);
    }
    for (const pon::Event& e : sink.events)
        if (e.type == pon::EventType::Stopped) return e;
    pon::Event none; none.type = pon::EventType::Expired; return none;
}

} // namespace

int main() {
    const pon::Event stat = fire({0, 0, 0});
    const pon::Event move = fire({-25, 0, 0}); // swinging toward the shooter

    std::printf("static plate : hit x=%.3f  t=%.4f s  energy=%.1f J\n",
                stat.point.x, stat.time_s, stat.energy_J);
    std::printf("closing plate: hit x=%.3f  t=%.4f s  energy=%.1f J  (closing %.0f m/s)\n",
                move.point.x, move.time_s, move.energy_J, move.residualSpeed_mps);
    std::printf("→ met %.2f m nearer, %.1f ms sooner, %.0f%% more energy\n",
                stat.point.x - move.point.x,
                (stat.time_s - move.time_s) * 1e3,
                (move.energy_J / stat.energy_J - 1.0) * 100.0);
    return 0;
}
