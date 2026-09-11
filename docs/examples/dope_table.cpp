// poncelet example — an arg-taking dope tool (cookbook recipe 1, runnable).
// SPDX-License-Identifier: MIT
//
// poncelet_example_dope_table --round 762x51_175gr_smk --zero 100 --to 1000
//                              --step 100 [--imperial]
//
// Zeroes a catalog round at `--zero` metres (bisecting launch pitch against
// preview_arc, same technique as guide.md's zeroing recipe), then prints a
// drop / drift / velocity / energy / time-of-flight table from the zero range
// out to `--to` every `--step` metres. `--imperial` adds an inches / fps /
// ft-lb column beside the SI one via <poncelet/units.hpp> — no new unit
// system, just the display-side conversion.
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

using namespace pon;

namespace {

struct Args {
    std::string round;
    Real zero_m = 100.0;
    Real to_m   = 1000.0;
    Real step_m = 100.0;
    bool imperial = false;
};

std::optional<Args> parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        if (arg == "--round") { if (const char* v = next()) a.round = v; }
        else if (arg == "--zero") { if (const char* v = next()) a.zero_m = std::atof(v); }
        else if (arg == "--to") { if (const char* v = next()) a.to_m = std::atof(v); }
        else if (arg == "--step") { if (const char* v = next()) a.step_m = std::atof(v); }
        else if (arg == "--imperial") a.imperial = true;
        else { std::fprintf(stderr, "unknown argument: %s\n", arg.c_str()); return std::nullopt; }
    }
    if (a.round.empty() || a.step_m <= 0.0) return std::nullopt;
    return a;
}

// Interpolated (position, velocity, time) at flight distance `x` along the
// arc, from the TrajectorySample straddling it — same lerp sampleTrajectory()
// does, over an already-computed preview_arc() rather than the live cache.
TrajectorySample sample_at_range(const std::vector<TrajectorySample>& arc, Real x) {
    if (arc.empty()) return {};
    if (x <= arc.front().position.x) return arc.front();
    for (std::size_t i = 1; i < arc.size(); ++i) {
        if (arc[i].position.x >= x) {
            const TrajectorySample& a = arc[i - 1];
            const TrajectorySample& b = arc[i];
            const Real span = b.position.x - a.position.x;
            const Real f = span > Real(0) ? (x - a.position.x) / span : Real(0);
            TrajectorySample s;
            s.position = a.position + (b.position - a.position) * f;
            s.velocity = a.velocity + (b.velocity - a.velocity) * f;
            s.time_s   = a.time_s + (b.time_s - a.time_s) * f;
            return s;
        }
    }
    return arc.back();
}

} // namespace

int main(int argc, char** argv) {
    const std::optional<Args> args = parse(argc, argv);
    if (!args) {
        std::fprintf(stderr,
                     "usage: poncelet_example_dope_table --round <catalog id> "
                     "[--zero m=100] [--to m=1000] [--step m=100] [--imperial]\n");
        return 1;
    }

    if (!catalog::has(args->round)) {
        std::fprintf(stderr, "unknown catalog round: %s\n", args->round.c_str());
        return 1;
    }
    const ProjectileType round = catalog::get(args->round);

    Environment env; // sea-level ISA, no wind
    const Vec3 muzzle{0, 1.6, 0};
    const Seconds dt = 0.002, maxTime = 8.0;

    // Landing height at range `x` for a given launch pitch (bisection target).
    auto drop_at = [&](Real pitch, Real x) {
        LaunchParams lp;
        lp.position = muzzle;
        lp.direction = {std::cos(pitch), std::sin(pitch), 0};
        std::vector<TrajectorySample> arc;
        preview_arc(round, lp, env, dt, maxTime, arc);
        return sample_at_range(arc, x).position.y;
    };

    Real lo = 0.0, hi = moa(60.0);
    for (int i = 0; i < 40; ++i) {
        const Real mid = 0.5 * (lo + hi);
        (drop_at(mid, args->zero_m) < muzzle.y ? lo : hi) = mid;
    }
    const Real zeroPitch = 0.5 * (lo + hi);

    LaunchParams lp;
    lp.position = muzzle;
    lp.direction = {std::cos(zeroPitch), std::sin(zeroPitch), 0};
    std::vector<TrajectorySample> arc;
    preview_arc(round, lp, env, dt, maxTime, arc);
    if (arc.empty()) {
        std::fprintf(stderr, "preview_arc produced no samples — bad round data?\n");
        return 1;
    }

    std::printf("%-14s %8s %8s %10s %8s %8s %8s", "round", "range_m", "drop_m",
               "drift_m", "v_mps", "E_J", "tof_s");
    if (args->imperial) std::printf(" %8s %8s %9s", "drop_in", "v_fps", "E_ftlb");
    std::printf("\n");

    for (Real range = args->zero_m; range <= args->to_m + 1e-6; range += args->step_m) {
        const TrajectorySample s = sample_at_range(arc, range);
        const Real drop = s.position.y - muzzle.y; // negative = below line of sight
        const Real drift = s.position.z;
        const Real v = length(s.velocity);
        const Real e = round.mass_kg > 0.0 ? 0.5 * round.mass_kg * v * v : 0.0;
        std::printf("%-14s %8.0f %8.3f %10.3f %8.1f %8.1f %8.3f",
                    args->round.c_str(), range, drop, drift, v, e, s.time_s);
        if (args->imperial)
            std::printf(" %8.2f %8.1f %9.1f", to_inches(drop), to_fps(v), to_ftlb(e));
        std::printf("\n");
    }
    return 0;
}
