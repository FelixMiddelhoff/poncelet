// poncelet — Sim-less trajectory preview (aim indicators, toss arcs).
// SPDX-License-Identifier: MIT
//
// preview_arc() runs a private throwaway Sim on the AnalyticDrag tier and
// records the polyline. Using the real Sim (rather than re-deriving the force
// model here) means the preview can never drift from what a spawned shot on the
// same tier actually does.
#include "poncelet/sim.hpp"

#include "poncelet/world.hpp"

#include <vector>

namespace pon {

namespace {
// Shared driver for both preview_arc overloads — runs the throwaway Sim once
// and records the full TrajectorySample; the Vec3 overload just strips
// velocity/time back out. Keeps the two from drifting apart.
void preview_arc_impl(const ProjectileType& type, const LaunchParams& launch,
                      const Environment& env, Seconds dt, Seconds maxTime,
                      std::vector<TrajectorySample>& out, Real groundY) {
    out.clear();
    if (dt <= Seconds(0) || maxTime <= Seconds(0)) return;

    Sim sim(env);
    const TypeId id = sim.registerType(type);
    if (id == kInvalidType) return;

    LaunchParams lp = launch;
    lp.tier = FidelityTier::AnalyticDrag;
    const StateId h = sim.spawn(id, lp);
    if (h == kInvalidState) return;

    EmptyWorld world;
    VectorEventSink sink;

    auto sample = [&] {
        const ProjectileState& s = sim.state(h);
        return TrajectorySample{s.position, s.velocity, s.timeAlive_s};
    };

    out.push_back(sample());
    if (out.front().position.y <= groundY) return;

    const int steps = static_cast<int>(maxTime / dt) + 1;
    for (int i = 0; i < steps && sim.state(h).alive; ++i) {
        sim.step(dt, world, sink);
        const TrajectorySample smp = sample();
        out.push_back(smp);
        if (smp.position.y <= groundY) break;
    }
}
} // namespace

void preview_arc(const ProjectileType& type, const LaunchParams& launch,
                 const Environment& env, Seconds dt, Seconds maxTime,
                 std::vector<Vec3>& out, Real groundY) {
    std::vector<TrajectorySample> samples;
    preview_arc_impl(type, launch, env, dt, maxTime, samples, groundY);
    out.clear();
    out.reserve(samples.size());
    for (const auto& s : samples) out.push_back(s.position);
}

void preview_arc(const ProjectileType& type, const LaunchParams& launch,
                 const Environment& env, Seconds dt, Seconds maxTime,
                 std::vector<TrajectorySample>& out, Real groundY) {
    preview_arc_impl(type, launch, env, dt, maxTime, out, groundY);
}

} // namespace pon
