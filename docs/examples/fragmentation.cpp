// poncelet example — casing fragmentation: Mott spectrum + Gurney speed, each
// fragment its own projectile (Phase 19 item 3, §8a).
// SPDX-License-Identifier: MIT
//
// An 81 mm mortar bomb (≈ 3 kg steel body, ≈ 0.9 kg TNT-equivalent fill) bursts
// 6 m above a target. We build the fragment spray with pon::generate_fragments
// off that point, spawn every fragment into the sim as its own projectile, and
// count how many rain down onto a 30 m steel plate below.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_fragmentation
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

// Flat ground at y = 0; every hit below counts, and a 30 m square is the "plate".
struct Ground final : pon::World {
    pon::Material steel = [] {
        pon::Material m;
        m.name = "plate"; m.behaviour = pon::MaterialBehaviour::Ductile;
        m.density_kgm3 = 7850; m.strength_Pa = 3.5e8; m.thickness_m = 0.006;
        return m;
    }();
    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        if (a.y > 0.0 && b.y <= 0.0) {
            const double f = a.y / (a.y - b.y);
            out.point = a + (b - a) * f;
            out.normal = {0, 1, 0};
            out.t = f;
            out.surface = 1;
            return true;
        }
        return false;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override { return steel; }
};

} // namespace

int main() {
    pon::Sim sim;

    pon::ProjectileType bomb;
    bomb.id                     = "81mm_mortar";
    bomb.klass                  = pon::ProjectileClass::Shell;
    bomb.dragModel              = pon::DragModel::G1;
    bomb.ballisticCoefficient   = 0.4;
    bomb.mass_kg                = 4.0;
    bomb.refDiameter_m          = 0.081;
    bomb.warhead.chargeMass_kg  = 0.9;
    bomb.warhead.tntEquivalence = 1.0;

    // Casing break-up. A nose-down airburst over a target throws its fragments
    // into a downward cone — the "rain of steel" a mortar is aimed for.
    pon::FragmentationDesc& frag = bomb.warhead.fragmentation;
    frag.casingMass_kg          = 3.0;
    frag.gurneyVelocity_mps     = 2440.0;      // TNT
    frag.casingInnerDiameter_m  = 0.070;
    frag.casingWallThickness_m  = 0.008;
    frag.spray                  = pon::FragmentSpray::Cone;
    frag.sprayAxis              = {0, -1, 0};   // straight down
    frag.coneHalfAngle_rad      = 0.70;         // ~40°
    frag.maxFragments           = 300;

    sim.registerType(bomb);
    Ground world;

    // The burst point (6 m up) — the Sim would pick this from a fuze; here we
    // place it, as generate_fragments works off any point. The bomb still has
    // ~40 m/s of descent left, which every fragment inherits.
    const pon::Vec3 burstPoint{0, 6, 0};
    const pon::Vec3 bombVel{15, -40, 0};

    std::vector<pon::FragmentSpec> spray;
    const std::size_t nf = pon::generate_fragments(
        frag, burstPoint, bombVel, bomb.warhead.chargeMass_kg, spray);

    double vMin = 1e9, vMax = 0, mMax = 0;
    for (const auto& f : spray) {
        const double v = pon::length(f.velocity);
        vMin = std::min(vMin, v); vMax = std::max(vMax, v);
        mMax = std::max(mMax, f.mass_kg);
    }

    std::vector<pon::StateId> frIds;
    const std::size_t spawned = pon::spawn_fragments(
        sim, spray, pon::FidelityTier::Integrated, "mortar_frag",
        frag.massClasses, &frIds);

    pon::VectorEventSink fragEvents;
    for (int i = 0; i < 4000; ++i) {
        std::size_t live = 0;
        for (pon::StateId f : frIds) if (sim.state(f).alive) ++live;
        if (!live) break;
        sim.step(1.0 / 500.0, world, fragEvents);
    }

    int hitPlate = 0, perforated = 0;
    double rMax = 0;
    for (const auto& e : fragEvents.events) {
        if (e.type != pon::EventType::Embedded && e.type != pon::EventType::Perforated &&
            e.type != pon::EventType::Stopped && e.type != pon::EventType::Ricochet)
            continue;
        const double r = std::sqrt(e.point.x * e.point.x + e.point.z * e.point.z);
        rMax = std::max(rMax, r);
        if (r <= 15.0) ++hitPlate;
        if (e.type == pon::EventType::Perforated) ++perforated;
    }

    std::printf("burst at (%.0f, %.0f, %.0f)\n",
                burstPoint.x, burstPoint.y, burstPoint.z);
    std::printf("%zu fragments  (heaviest %.1f g)  speed %.0f..%.0f m/s\n",
                nf, mMax * 1e3, vMin, vMax);
    std::printf("spawned %zu projectiles  ->  %d hit within 15 m "
                "(%d perforated the 6 mm plate), spread to %.0f m\n",
                spawned, hitPlate, perforated, rMax);

    // ctest gate: a real spray, launched at fragment speeds, spawned one
    // projectile each, most landing on the target plate under the burst.
    const bool ok = nf > 50 && spawned == nf &&
                    vMax > 800.0 && vMax < 2600.0 && vMin > 300.0 &&
                    mMax > 0.001 &&
                    hitPlate > static_cast<int>(nf) / 3;
    return ok ? 0 : 1;
}
