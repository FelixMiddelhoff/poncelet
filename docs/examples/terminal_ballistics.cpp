// poncelet example — terminal ballistics: ricochet / embed / perforate
// (Phase 18 item 8).
// SPDX-License-Identifier: MIT
//
// One rifle round fired at three targets:
//   * a stack of two pine boards  → perforates both, slowing each time
//   * a thick oak block           → embeds (Poncelet channel depth)
//   * a steel plate at 12°        → ricochets off
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_terminal_ballistics
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

pon::Material mat(pon::MaterialBehaviour b, double rho, double strength_Pa,
                  double thick_m) {
    pon::Material m;
    m.behaviour = b;
    m.density_kgm3 = rho;
    m.strength_Pa = strength_Pa;
    m.thickness_m = thick_m; // <= 0 ⇒ bulk
    return m;
}

// A stack of planes ⟂ +x, each with its own material.
class SlabWorld final : public pon::World {
public:
    struct Slab { double x; pon::Material m; };
    std::vector<Slab> slabs;

    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        double best = 1.0; int idx = -1;
        for (std::size_t i = 0; i < slabs.size(); ++i) {
            const double da = a.x - slabs[i].x, db = b.x - slabs[i].x;
            if ((da > 0.0) == (db > 0.0)) continue;
            const double t = da / (da - db);
            if (t >= 0.0 && t < best) { best = t; idx = int(i); }
        }
        if (idx < 0) return false;
        out.t = best;
        out.point = a + (b - a) * best;
        out.normal = {a.x < slabs[idx].x ? -1.0 : 1.0, 0, 0};
        out.surface = pon::SurfaceId(idx);
        return true;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId s) const override {
        return slabs[s].m;
    }
};

pon::TypeId rifle(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id = "762x51_175gr";
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::G7;
    t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134;
    t.refDiameter_m = 0.00782;
    return sim.registerType(t);
}

const char* name(pon::EventType t) {
    switch (t) {
        case pon::EventType::SurfaceCrossed: return "crossed ";
        case pon::EventType::Perforated:     return "perforated";
        case pon::EventType::Embedded:       return "embedded";
        case pon::EventType::Ricochet:       return "ricochet";
        case pon::EventType::Stopped:        return "stopped ";
        default:                             return "event   ";
    }
}

void fire(const char* label, const pon::World& world, pon::Vec3 dir) {
    pon::Sim sim;
    const pon::TypeId r = rifle(sim);
    pon::LaunchParams lp;
    lp.position = {0, 0, 0};
    lp.direction = dir;
    lp.speed = 800.0;
    lp.tier = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(r, lp);

    pon::VectorEventSink sink;
    for (int i = 0; i < 6000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 1000.0, world, sink);

    std::printf("%s\n", label);
    for (const pon::Event& e : sink.events)
        std::printf("  %-10s  E=%7.0f J   v_res=%6.1f m/s   depth=%.4f m\n",
                    name(e.type), e.energy_J, e.residualSpeed_mps, e.channelDepth_m);
}

} // namespace

int main() {
    {
        SlabWorld boards;
        boards.slabs = {
            {10.0, mat(pon::MaterialBehaviour::Fibrous, 500, 4.0e7, 0.02)},
            {10.5, mat(pon::MaterialBehaviour::Fibrous, 500, 4.0e7, 0.02)},
        };
        fire("two pine boards:", boards, {1, 0, 0});
    }
    {
        SlabWorld oak;
        oak.slabs = {{10.0, mat(pon::MaterialBehaviour::Fibrous, 750, 9.0e7, -1.0)}};
        fire("oak block:", oak, {1, 0, 0});
    }
    {
        SlabWorld plate;
        plate.slabs = {{10.0, mat(pon::MaterialBehaviour::Ductile, 7850, 2.5e8, 0.02)}};
        // ~8° grazing angle onto the plate face (normal is +x).
        const double a = 8.0 * 3.14159265 / 180.0;
        fire("steel plate at 8 deg grazing:", plate, {std::sin(a), 0, std::cos(a)});
    }
    return 0;
}
