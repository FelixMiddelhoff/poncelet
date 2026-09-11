// poncelet example — explosive warheads: fuze, Friedlander blast field, impulse
// on rigid bodies (Phase 19 item 2, §8a).
// SPDX-License-Identifier: MIT
//
// A 155 mm HE shell (≈ 7 kg TNT-equivalent fill) is lobbed at a wall. The Sim
// picks the detonation point from the fuze and emits EventType::Detonated; from
// that point we build a pon::Burst and read the overpressure and the impulse it
// delivers to a nearby crate, in the open and behind cover.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_explosive_warhead
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>

namespace {

// A wall at x = 40 the shell detonates against; a second slab at x = 44 that
// puts the far crate in cover.
struct TwoWalls final : pon::World {
    pon::Material concrete = [] {
        pon::Material m;
        m.name = "concrete"; m.behaviour = pon::MaterialBehaviour::Brittle;
        m.density_kgm3 = 2300; m.strength_Pa = 3.0e7; m.thickness_m = 0.3;
        return m;
    }();
    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        for (double wx : {40.0, 44.0}) {
            if ((a.x - wx) * (b.x - wx) < 0.0) {
                const double f = (wx - a.x) / (b.x - a.x);
                out.point   = a + (b - a) * f;
                out.normal  = {a.x < wx ? -1.0 : 1.0, 0, 0};
                out.t       = f;
                out.surface = 1;
                return true;
            }
        }
        return false;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override { return concrete; }
};

} // namespace

int main() {
    pon::Sim sim;

    pon::ProjectileType shell;
    shell.id             = "155mm_he";
    shell.klass          = pon::ProjectileClass::Shell;
    shell.dragModel      = pon::DragModel::G1;
    shell.ballisticCoefficient = 0.6;
    shell.mass_kg        = 43.0;
    shell.refDiameter_m  = 0.155;
    shell.warhead.chargeMass_kg  = 6.6;    // ~Comp B fill
    shell.warhead.tntEquivalence = 1.1;
    shell.warhead.fuze           = pon::FuzeMode::Contact;
    shell.warhead.surfaceBurst   = true;   // it goes off against the wall
    const pon::TypeId id = sim.registerType(shell);

    pon::LaunchParams lob;
    lob.position  = {0, 0, 0};
    lob.direction = {std::cos(0.5), std::sin(0.5), 0}; // ~29° up
    lob.speed     = 250.0;
    lob.tier      = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(id, lob);

    TwoWalls world;
    pon::VectorEventSink events;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 500.0, world, events);

    const pon::Event* det = nullptr;
    for (const auto& e : events.events)
        if (e.type == pon::EventType::Detonated) det = &e;
    if (!det) { std::puts("no detonation"); return 1; }

    std::printf("detonated at (%.1f, %.1f, %.1f) t=%.2fs  yield %.1f kg TNT-eq  %.1f MJ\n",
                det->point.x, det->point.y, det->point.z, det->time_s,
                det->payload_kg, det->energy_J * 1e-6);

    pon::Burst burst(det->point, shell.warhead, sim.environment(), det->time_s);

    // A 50 kg crate 6 m from the burst, on the near side of the wall.
    pon::BlastTarget crate;
    crate.centroid = det->point + pon::Vec3{-6, 0, 0};
    crate.area_m2  = 0.6;
    crate.mass_kg  = 50.0;

    const pon::BlastSample bs = burst.sampleAt(crate.centroid);
    const pon::BlastLoad open = burst.loadOnBody(crate, 1.0);

    // The same crate behind the x = 44 slab — line of sight is blocked.
    pon::BlastTarget shadowed = crate;
    shadowed.centroid = det->point + pon::Vec3{8, 0, 0};
    const double los = pon::blast_line_of_sight(world, det->point, shadowed.centroid);
    const pon::BlastLoad covered = burst.loadOnBody(shadowed, los);

    std::printf("6 m  : %.1f kPa peak, %.0f Pa·s impulse/area, "
                "crate delta-v %.1f m/s\n",
                bs.peakOverpressure_Pa * 1e-3, bs.specificImpulse_Pa_s,
                pon::length(open.deltaVelocity_mps));
    std::printf("cover: line-of-sight %.2f -> crate delta-v %.2f m/s\n",
                los, pon::length(covered.deltaVelocity_mps));

    // ctest gate: a surface HE burst detonated on the wall, the near crate is
    // thrown several m/s, cover cuts that right down, and overpressure falls
    // off with distance.
    const bool near_wall = std::fabs(det->point.x - 40.0) < 0.5;
    const pon::BlastSample far = burst.sampleAt(det->point + pon::Vec3{-15, 0, 0});
    const bool ok = near_wall &&
                    det->payload_kg > 10.0 && det->payload_kg < 15.0 &&
                    pon::length(open.deltaVelocity_mps) > 1.0 &&
                    pon::length(covered.deltaVelocity_mps) <
                        pon::length(open.deltaVelocity_mps) * 0.5 &&
                    far.peakOverpressure_Pa < bs.peakOverpressure_Pa;
    return ok ? 0 : 1;
}
