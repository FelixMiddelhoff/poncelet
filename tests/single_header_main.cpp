// poncelet — single-header amalgamation smoke test, implementation TU.
// SPDX-License-Identifier: MIT
//
// The one place PONCELET_SINGLE_IMPLEMENTATION is defined. Compiles every
// src/*.cpp body folded into dist/poncelet_single.hpp, links against the plain
// include from single_header_tu2.cpp, and runs a shot end to end. This is the
// CI gate that the amalgamation actually builds.
#define PONCELET_SINGLE_IMPLEMENTATION
#include "poncelet_single.hpp"

#include <cstdio>

int single_header_tu2_probe(); // from single_header_tu2.cpp

int main() {
    pon::Sim sim;
    pon::ProjectileType t = pon::catalog::get("762x51_175gr_smk");
    const pon::TypeId id = sim.registerType(t);
    if (id == pon::kInvalidType) {
        std::printf("registerType failed: %s\n", sim.lastError());
        return 1;
    }
    pon::LaunchParams lp;
    lp.position = {0, 1.8, 0};
    lp.direction = {1, 0, 0};
    lp.tier = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(id, lp);

    pon::EmptyWorld world;
    pon::VectorEventSink sink;
    for (int i = 0; i < 120 && sim.state(h).alive; ++i)
        sim.step(1.0 / 240.0, world, sink);

    const pon::ProjectileState& s = sim.state(h);
    std::printf("single-header: %s\n", pon::describe(s, t, sim.environment()).c_str());

    const bool ok = s.position.x > 100.0 && pon::length(s.velocity) < 900.0 &&
                    single_header_tu2_probe() == 1;
    return ok ? 0 : 1;
}
