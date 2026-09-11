// poncelet example — <poncelet/worlds.hpp>: PlaneWorld / AabbWorld /
// SphereWorld / SlabStackWorld / CompositeWorld, for prototyping without
// writing a bespoke World.
// SPDX-License-Identifier: MIT
#include <poncelet/poncelet.hpp>
#include <poncelet/worlds.hpp>

#include <cmath>
#include <cstdio>

namespace {

pon::Material steel_plate() {
    pon::Material m;
    m.behaviour = pon::MaterialBehaviour::Ductile;
    m.density_kgm3 = 7850.0;
    m.strength_Pa = 4.0e8;
    m.thickness_m = 0.01; // 10 mm — thin enough for a rifle round to perforate
    return m;
}

pon::Material pine_board() {
    pon::Material m;
    m.behaviour = pon::MaterialBehaviour::Fibrous;
    m.density_kgm3 = 500.0;
    m.strength_Pa = 3.0e7;
    m.thickness_m = 0.02; // 20 mm board
    return m;
}

const char* event_name(pon::EventType t) {
    switch (t) {
        case pon::EventType::Perforated: return "perforated";
        case pon::EventType::Embedded:   return "embedded";
        case pon::EventType::Ricochet:   return "ricocheted";
        case pon::EventType::Stopped:    return "stopped";
        default:                         return "event";
    }
}

pon::TypeId rifle_round(pon::Sim& sim) {
    pon::ProjectileType t;
    t.id = "9x19_124gr_fmj";
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    t.mass_kg = 0.00804;
    t.refDiameter_m = 0.00902;
    t.muzzleSpeed_mps = 360.0;
    return sim.registerType(t);
}

void fire_and_report(const char* label, const pon::World& world) {
    pon::Sim sim;
    const pon::TypeId t = rifle_round(sim);
    const pon::StateId h = sim.spawn(t, pon::LaunchParams{{0, 1.5, 0}, {1, 0, 0}});
    pon::VectorEventSink sink;
    for (int i = 0; i < 400 && sim.state(h).alive; ++i)
        sim.step(1.0 / 500.0, world, sink);

    std::printf("%-22s", label);
    if (sink.events.empty()) {
        std::printf(" no hit (missed / still flying)\n");
        return;
    }
    for (const pon::Event& e : sink.events)
        std::printf(" %s@x=%.1f", event_name(e.type), e.point.x);
    std::printf("\n");
}

} // namespace

int main() {
    // One plane, angled slightly for a plausible glancing shot.
    pon::PlaneWorld plane;
    plane.position = {5, 0, 0};
    plane.normal = pon::normalized(pon::Vec3{-1, 0.2, 0});
    plane.mat = steel_plate();
    fire_and_report("PlaneWorld", plane);

    // One box the shot passes through the middle of.
    pon::AabbWorld box;
    box.boundsMin = {5, 1.0, -1.0};
    box.boundsMax = {5.3, 2.0, 1.0};
    box.mat = pine_board();
    fire_and_report("AabbWorld", box);

    // One sphere — a sandbag, roughly.
    pon::SphereWorld sphere;
    sphere.center = {5, 1.5, 0};
    sphere.radius_m = 0.4;
    sphere.mat = pine_board();
    fire_and_report("SphereWorld", sphere);

    // Three boards in a row — perforates the first two, likely stops or
    // embeds in the third depending on residual energy.
    pon::SlabStackWorld boards;
    boards.axis = {1, 0, 0};
    boards.slabs = {{3.0, pine_board()}, {3.5, pine_board()}, {4.0, pine_board()}};
    fire_and_report("SlabStackWorld", boards);

    // Compose a plate behind the board stack — CompositeWorld picks whichever
    // sub-world the shot reaches first.
    pon::CompositeWorld composite;
    pon::PlaneWorld backstop;
    backstop.position = {6, 0, 0};
    backstop.normal = {-1, 0, 0};
    backstop.mat = steel_plate();
    composite.worlds = {&boards, &backstop};
    fire_and_report("CompositeWorld", composite);

    return 0;
}
