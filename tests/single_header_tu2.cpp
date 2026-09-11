// poncelet — single-header amalgamation smoke test, translation unit 2.
// SPDX-License-Identifier: MIT
//
// Includes the amalgamation WITHOUT PONCELET_SINGLE_IMPLEMENTATION — proves the
// declarations are usable from a second TU and that linking it against the
// implementation TU produces no duplicate symbols.
#include "poncelet_single.hpp"

int single_header_tu2_probe() {
    pon::Sim sim;
    pon::ProjectileType t;
    t.id = "sh_tu2";
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    t.mass_kg = 0.008;
    t.refDiameter_m = 0.009;
    t.muzzleSpeed_mps = 360.0;
    const pon::StateId h = sim.fire(t, {0, 1, 0}, {1, 0, 0});
    pon::EmptyWorld world;
    pon::VectorEventSink sink;
    for (int i = 0; i < 20; ++i) sim.step(1.0 / 200.0, world, sink);
    return (h != pon::kInvalidState && sim.state(h).position.x > 0.0) ? 1 : 0;
}
