// poncelet example — special media & behaviours (Phase 18).
// SPDX-License-Identifier: MIT
//
// Part 1: a rifle round shot down into a pond. It crosses the surface (a small
//         one-time impulse), then ~815x the air density drags it below wounding
//         speed inside the first metre or two.
// Part 2: the same round in a supercavitation-flagged medium — it rides a gas
//         cavity and carries lethal speed for tens of metres.
// Part 3: a crossbow bolt into a thin concrete panel: "arrow vs concrete" — it
//         stops at the face, no channel.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_special_media
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

pon::TypeId register_slug(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id              = "10g_slug";
    t.klass           = pon::ProjectileClass::Bullet;
    t.dragModel       = pon::DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    t.mass_kg         = 0.010;
    t.refDiameter_m   = 0.0095;
    return sim.registerType(t);
}

pon::TypeId register_bolt(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id    = "crossbow_bolt";
    t.klass = pon::ProjectileClass::Bolt; // sharp nose + low SD from the defaults
    return sim.registerType(t);
}

// Water everywhere below y = 0; air above. A real game returns kMediumWater from
// its own water volume test.
struct Pond final : pon::World {
    bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
    pon::MediumId mediumAt(pon::Vec3 p) const override {
        return p.y <= 0.0 ? pon::kMediumWater : pon::kMediumAir;
    }
    const pon::Material& material(pon::SurfaceId) const override {
        static const pon::Material w = pon::material_water();
        return w;
    }
};

// One concrete panel ⟂ +x at x = 15 m.
struct Panel final : pon::World {
    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        if ((a.x - 15.0 > 0.0) == (b.x - 15.0 > 0.0)) return false;
        const double t = (a.x - 15.0) / (a.x - b.x);
        out.t = t;
        out.point = a + (b - a) * t;
        out.normal = {-1, 0, 0};
        out.surface = 0;
        return true;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override {
        static pon::Material c = [] {
            pon::Material m;
            m.name         = "concrete_panel";
            m.behaviour    = pon::MaterialBehaviour::Brittle;
            m.density_kgm3 = 2400.0;
            m.strength_Pa  = 3.0e7;
            m.thickness_m  = 0.04;
            return m;
        }();
        return c;
    }
};

double dive_reach(bool supercavitating) {
    pon::Sim sim;
    const pon::TypeId id = register_slug(sim);
    if (supercavitating) {
        pon::MediumDesc m;
        m.name = "cav_water";
        m.density_kgm3 = 1000.0;
        m.buoyancy = 0.1;
        m.supercavitation = true;
        const pon::MediumId mid = sim.environment().media.add(m);
        pon::LaunchParams lp;
        lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
        lp.speed = 700.0; lp.tier = pon::FidelityTier::Integrated; lp.medium = mid;
        const pon::StateId h = sim.spawn(id, lp);
        pon::VectorEventSink sink;
        struct W final : pon::World {
            pon::MediumId m;
            bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
            pon::MediumId mediumAt(pon::Vec3) const override { return m; }
            const pon::Material& material(pon::SurfaceId) const override {
                static const pon::Material w = pon::material_water(); return w;
            }
        } world; world.m = mid;
        for (int i = 0; i < 40000 && sim.state(h).alive &&
                        pon::length(sim.state(h).velocity) > 150.0; ++i)
            sim.step(1.0 / 8000.0, world, sink);
        return sim.state(h).position.x;
    }
    pon::LaunchParams lp;
    lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 700.0; lp.tier = pon::FidelityTier::Integrated;
    lp.medium = pon::kMediumWater;
    const pon::StateId h = sim.spawn(id, lp);
    struct W final : pon::World {
        bool raycast(pon::Vec3, pon::Vec3, pon::HitResult&) const override { return false; }
        pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumWater; }
        const pon::Material& material(pon::SurfaceId) const override {
            static const pon::Material w = pon::material_water(); return w;
        }
    } world;
    pon::VectorEventSink sink;
    for (int i = 0; i < 40000 && sim.state(h).alive &&
                    pon::length(sim.state(h).velocity) > 150.0; ++i)
        sim.step(1.0 / 8000.0, world, sink);
    return sim.state(h).position.x;
}

} // namespace

int main() {
    // --- Part 1: shot into a pond -----------------------------------------
    {
        pon::Sim sim;
        const pon::TypeId id = register_slug(sim);
        pon::LaunchParams lp;
        lp.position  = {0, 3.0, 0};
        lp.direction = {0.3, -1.0, 0.0};
        lp.speed     = 600.0;
        lp.tier      = pon::FidelityTier::Integrated;
        const pon::StateId h = sim.spawn(id, lp);

        Pond pond;
        pon::VectorEventSink sink;
        for (int i = 0; i < 40000 && sim.state(h).alive; ++i)
            sim.step(1.0 / 8000.0, pond, sink);

        for (const pon::Event& e : sink.events)
            if (e.type == pon::EventType::MediumChanged)
                std::printf("pond: crossed the surface at %.1f m/s (was 600)\n",
                            e.residualSpeed_mps);
        std::printf("pond: came to rest %.2f m past the entry point\n",
                    sim.state(h).position.x);
    }

    // --- Part 2: supercavitation ----------------------------------------------
    const double plain  = dive_reach(false);
    const double cavity = dive_reach(true);
    std::printf("underwater reach to 150 m/s:  plain water %.2f m   "
                "supercavitating %.1f m\n", plain, cavity);

    // --- Part 3: arrow vs concrete ------------------------------------------
    {
        pon::Sim sim;
        const pon::TypeId id = register_bolt(sim);
        pon::LaunchParams lp;
        lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
        lp.speed = 120.0; lp.tier = pon::FidelityTier::Integrated;
        const pon::StateId h = sim.spawn(id, lp);
        Panel panel;
        pon::VectorEventSink sink;
        for (int i = 0; i < 8000 && sim.state(h).alive; ++i)
            sim.step(1.0 / 2000.0, panel, sink);
        for (const pon::Event& e : sink.events)
            if (e.type == pon::EventType::Stopped)
                std::printf("concrete: bolt stopped at the face (x=%.2f), "
                            "%.0f J spalled\n", e.point.x, e.energy_J);
    }

    const bool ok = plain < 3.0 && cavity > 4.0 * plain;
    return ok ? 0 : 1;
}
