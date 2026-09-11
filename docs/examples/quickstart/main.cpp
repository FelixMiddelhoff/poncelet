// poncelet — copy-paste quickstart: one catalog round, one preview_arc call,
// see a number change. No Sim, no World — the smallest complete "it works"
// program. SPDX-License-Identifier: MIT
#include <poncelet/poncelet.hpp>

#include <cstdio>
#include <vector>

int main() {
    const pon::ProjectileType round = pon::catalog::get("762x51_175gr_smk");

    pon::LaunchParams lp;
    lp.position = {0, 1.6, 0};   // 1.6 m off the ground
    lp.direction = {1, 0.01, 0}; // downrange, a hair of elevation

    pon::Environment env; // sea-level ISA, no wind
    std::vector<pon::Vec3> arc;
    pon::preview_arc(round, lp, env, /*dt*/ 0.01, /*maxTime*/ 5.0, arc, /*groundY*/ 0.0);

    if (arc.empty()) {
        std::fprintf(stderr, "preview_arc produced nothing — bad round data?\n");
        return 1;
    }
    const pon::Vec3& landed = arc.back();
    std::printf("poncelet quickstart: %s landed %.0f m downrange after %.2f s\n",
                round.id.c_str(), landed.x, arc.size() * 0.01);
    return 0;
}
