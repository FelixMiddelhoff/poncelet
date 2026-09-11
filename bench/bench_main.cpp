// poncelet — perf bench skeleton. Real gates (per-projectile sub-µs, 10k
// concurrent < 1 ms/frame) are wired into CI in WORKPLAN item 13; this just
// establishes the harness.
// SPDX-License-Identifier: MIT
#include "poncelet/poncelet.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>

using namespace pon;
using clk = std::chrono::steady_clock;

static double run(FidelityTier tier, int n, int frames, bool batch = false) {
    SimConfig cfg;
    cfg.batchIntegrator = batch;
    cfg.batchMinGroup   = 8;
    Sim sim({}, cfg);
    ProjectileType t;
    t.id = "bench_bullet";
    t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    const TypeId id = sim.registerType(t);

    LaunchParams lp;
    lp.direction = {1, 0.02, 0};
    lp.speed = 850.0;
    lp.tier = tier;
    for (int i = 0; i < n; ++i) sim.spawn(id, lp);

    EmptyWorld world;
    VectorEventSink sink;

    const auto t0 = clk::now();
    for (int f = 0; f < frames; ++f) sim.step(1.0 / 60.0, world, sink);
    const auto t1 = clk::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / frames;
}

// Determinism gate: run one fixed multi-tier scenario twice from scratch and
// fold Sim::stateHash() after every step into a running digest. The two digests
// must be bit-identical (PlatformStable: same build, same platform). CI runs
// this on every OS/config, so it also catches an optimisation level or a
// platform libm making the flight non-reproducible.
static std::uint64_t det_run() {
    Environment env;
    SimConfig cfg;
    cfg.determinism = config::Determinism::PlatformStable;
    cfg.trajectoryCacheFrames = 8; // exercise the ring buffer too
    Sim sim(env, cfg);

    ProjectileType bullet;
    bullet.id = "det_308"; bullet.klass = ProjectileClass::Bullet;
    bullet.dragModel = DragModel::G7; bullet.ballisticCoefficient = 0.243;
    bullet.mass_kg = 0.0113; bullet.refDiameter_m = 0.00782;
    bullet.twistRate_m = 0.254;
    const TypeId bid = sim.registerType(bullet);

    ProjectileType rocket;
    rocket.id = "det_rkt"; rocket.klass = ProjectileClass::Shell;
    rocket.dragModel = DragModel::ConstantCd; rocket.dragCoefficient = 0.20;
    rocket.mass_kg = 12.0; rocket.refDiameter_m = 0.07;
    rocket.guidance.law = GuidanceLaw::ProportionalNav;
    rocket.guidance.thrustAccel_mps2 = 90.0;
    rocket.guidance.burnTime_s = 1.5;
    const TypeId rid = sim.registerType(rocket);

    auto fire = [&](TypeId id, Vec3 dir, double spd, FidelityTier tier,
                    PrecisionFlag pf) {
        LaunchParams lp;
        lp.position = {0, 2.0, 0};
        lp.direction = dir; lp.speed = spd; lp.tier = tier; lp.precision = pf;
        return sim.spawn(id, lp);
    };
    fire(bid, {1, 0.03, 0.00}, 820.0, FidelityTier::Integrated,   PrecisionFlag::None);
    fire(bid, {1, 0.05, 0.02}, 790.0, FidelityTier::AnalyticDrag, PrecisionFlag::None);
    fire(bid, {1, 0.02, 0.01}, 800.0, FidelityTier::Integrated,   PrecisionFlag::SixDOF);
    const StateId guided = fire(rid, {1, 0.20, 0.00}, 260.0, FidelityTier::Integrated,
                                PrecisionFlag::None);

    EmptyWorld world;
    VectorEventSink sink;
    std::uint64_t digest = 0xCBF29CE484222325ull;
    for (int f = 0; f < 400; ++f) {
        sim.guide(guided, Vec3{900, 20, 5}, Vec3{0, 0, 0});
        sim.step(1.0 / 200.0, world, sink);
        const std::uint64_t hs = sim.stateHash();
        digest = (digest ^ hs) * 0x100000001B3ull;
    }
    return digest;
}

static int det_check() {
    const std::uint64_t a = det_run();
    const std::uint64_t b = det_run();
    std::printf("det-check: run A = %016llx  run B = %016llx  %s\n",
                (unsigned long long)a, (unsigned long long)b,
                a == b ? "MATCH" : "DIVERGED");
    return a == b ? 0 : 1;
}

// --- Part B8: cross-platform BitExact golden digest -----------------------
//
// The same idea as det_run(), but with Determinism::BitExact: the flight runs
// entirely on the Q32.32 fixed-point core (deterministic sqrt + transcendental
// LUTs + fixed-point drag sampling), so the folded stateHash() digest is
// INTEGER-DERIVED and must be identical on every OS / compiler / optimisation
// level — not just repeatable on one build like det_run().
//
// kBitExactGolden was captured on x86-64 / MSVC and is committed. CI runs this
// on Linux + macOS too; a mismatch is either a real regression in the
// fixed-point path or a genuine portability bug to chase down — never a
// tolerance to relax. Scenario still avoids PrecisionFlag::AdaptiveRKF45 — not
// because of its step-size controller's std::pow anymore (that's a fixed-point
// LUT now, see integrate.cpp's pow_ratio02 / bitexact_rkf45_check below), but
// because adding a 6th shot here would change this committed golden's value
// and there's no 3-OS-verified reference for that combination yet.
// Diagnostic (added chasing the ARM64 macOS BitExact mismatch, 2026-09-11):
// the folded digest tells you THAT two runs/platforms disagree, not WHICH of
// the 5 shots. Pass a PerShotDigests* to bitexact_run() to also fold each
// shot's own Sim::stateHash(StateId) independently, so `--bitexact-per-shot`
// can print all 5 and a diff against another platform's output pinpoints the
// still-divergent shot in one round trip instead of guessing again.
struct PerShotDigests {
    std::array<StateId, 5>      ids{};
    std::array<std::uint64_t, 5> digest{
        0xCBF29CE484222325ull, 0xCBF29CE484222325ull, 0xCBF29CE484222325ull,
        0xCBF29CE484222325ull, 0xCBF29CE484222325ull};
};

static std::uint64_t bitexact_run(PerShotDigests* perShot = nullptr) {
    Environment env;
    SimConfig cfg;
    cfg.determinism = config::Determinism::BitExact;
    cfg.trajectoryCacheFrames = 8;
    Sim sim(env, cfg);

    ProjectileType bullet;
    bullet.id = "bx_308"; bullet.klass = ProjectileClass::Bullet;
    bullet.dragModel = DragModel::G7; bullet.ballisticCoefficient = 0.243;
    bullet.mass_kg = 0.0113; bullet.refDiameter_m = 0.00782;
    bullet.twistRate_m = 0.254;
    const TypeId bid = sim.registerType(bullet);

    ProjectileType arrow;
    arrow.id = "bx_arrow"; arrow.klass = ProjectileClass::Arrow;
    arrow.dragModel = DragModel::ConstantCd; arrow.dragCoefficient = 2.0;
    arrow.mass_kg = 0.025; arrow.refDiameter_m = 0.008;
    const TypeId aid = sim.registerType(arrow);

    auto fire = [&](TypeId id, Vec3 dir, double spd, FidelityTier tier, PrecisionFlag pf) {
        LaunchParams lp;
        lp.position = {0, 2.0, 0};
        lp.direction = dir; lp.speed = spd; lp.tier = tier; lp.precision = pf;
        return sim.spawn(id, lp);
    };
    const StateId h0 = fire(bid, {1, 0.03, 0.00}, 820.0, FidelityTier::Integrated,   PrecisionFlag::None);
    const StateId h1 = fire(bid, {1, 0.05, 0.02}, 790.0, FidelityTier::AnalyticDrag, PrecisionFlag::None);
    const StateId h2 = fire(bid, {1, 0.02, 0.01}, 800.0, FidelityTier::Integrated,   PrecisionFlag::SixDOF);
    const StateId h3 = fire(bid, {1, 0.04, 0.00}, 810.0, FidelityTier::Integrated,
         PrecisionFlag::SpinDrift | PrecisionFlag::Coriolis);
    const StateId h4 = fire(aid, {1, 0.15, 0.00}, 70.0,  FidelityTier::Integrated,   PrecisionFlag::None);
    if (perShot) perShot->ids = {h0, h1, h2, h3, h4};

    EmptyWorld world;
    VectorEventSink sink;
    std::uint64_t digest = 0xCBF29CE484222325ull;
    for (int f = 0; f < 400; ++f) {
        sim.step(1.0 / 200.0, world, sink);
        digest = (digest ^ sim.stateHash()) * 0x100000001B3ull;
        if (perShot)
            for (int i = 0; i < 5; ++i)
                perShot->digest[i] = (perShot->digest[i] ^ sim.stateHash(perShot->ids[i]))
                                    * 0x100000001B3ull;
    }
    return digest;
}

// Captured on x86-64 / MSVC 19.44, verified identical across Debug and Release.
// Re-captured 2026-09-11 after fixing a real cross-platform BitExact bug (a
// real 3-OS CI run caught it): the 6-DOF roll->Quat write-back in sim.cpp
// used plain std::cos/std::sin unconditionally, which isn't bit-identical
// across platforms for a general input; it now routes through
// fx_cos_full/fx_sin_full (fixed_lut.hpp) when bitExact_. This changes the
// digest (the fixed-point trig table is a lossy-but-deterministic
// approximation of the libm functions it replaces) but not the physical
// meaning — see fixed_lut.hpp's fx_sin_full/fx_cos_full comment and
// sim.cpp's writeBack for the full story.
static constexpr std::uint64_t kBitExactGolden = 0x21525237f2e246b2ull;

static int bitexact_golden() {
    const std::uint64_t a = bitexact_run();
    const std::uint64_t b = bitexact_run();
    const bool repeat = a == b;
    const bool match  = a == kBitExactGolden;
    std::printf("bitexact-golden: digest = %016llx  golden = %016llx  %s%s\n",
                (unsigned long long)a, (unsigned long long)kBitExactGolden,
                match ? "MATCH" : "MISMATCH",
                repeat ? "" : "  (NOT EVEN REPEATABLE)");
    return (match && repeat) ? 0 : 1;
}

// AdaptiveRKF45's step-size controller used a bare std::pow(ratio, 0.2)
// regardless of Determinism::BitExact — the golden scenario above
// deliberately avoids PrecisionFlag::AdaptiveRKF45 for exactly that reason
// (see bitexact_run()'s comment). Now that the controller routes through a
// fixed-point LUT under BitExact (integrate.cpp's pow_ratio02, wired in by
// sim.cpp's advanceIntegratedAdaptive), this checks the one thing a single
// machine actually CAN prove locally: the same scene run twice folds to the
// same digest. That's the same bar poncelet_det_check holds PlatformStable
// to — not a claim of cross-platform bit-exactness on its own (that would
// need the same real-CI-round-trip verification the golden digest above
// got), but it is a genuine regression gate: this would have caught the
// original bare-libm-pow gap (a machine where std::pow itself isn't
// perfectly repeatable run-to-run would fail it) and will catch this fix
// ever regressing.
static std::uint64_t bitexact_rkf45_run() {
    Environment env;
    SimConfig cfg;
    cfg.determinism = config::Determinism::BitExact;
    Sim sim(env, cfg);

    ProjectileType bullet;
    bullet.id = "bx_rkf45_308"; bullet.klass = ProjectileClass::Bullet;
    bullet.dragModel = DragModel::G7; bullet.ballisticCoefficient = 0.243;
    bullet.mass_kg = 0.0113; bullet.refDiameter_m = 0.00782;
    const TypeId bid = sim.registerType(bullet);

    LaunchParams lp;
    lp.position = {0, 2.0, 0};
    lp.direction = {1, 0.03, 0.01}; lp.speed = 820.0;
    lp.tier = FidelityTier::Integrated; lp.precision = PrecisionFlag::AdaptiveRKF45;
    sim.spawn(bid, lp);

    EmptyWorld world;
    VectorEventSink sink;
    std::uint64_t digest = 0xCBF29CE484222325ull;
    for (int f = 0; f < 400; ++f) {
        sim.step(1.0 / 200.0, world, sink);
        digest = (digest ^ sim.stateHash()) * 0x100000001B3ull;
    }
    return digest;
}

static int bitexact_rkf45_check() {
    const std::uint64_t a = bitexact_rkf45_run();
    const std::uint64_t b = bitexact_rkf45_run();
    std::printf("bitexact-rkf45-check: run A = %016llx  run B = %016llx  %s\n",
                (unsigned long long)a, (unsigned long long)b,
                a == b ? "MATCH" : "DIVERGED");
    return a == b ? 0 : 1;
}

// Chasing the macOS-Debug-only residual mismatch (Release is now fixed by
// the sim.cpp FMA-contraction-off flag; Debug's shot2/shot3 digests are
// unaffected by that same flag — so it's a second, distinct cause). The
// folded per-shot digest (bitexact_per_shot) says THAT shot3 diverges over
// 400 frames, not WHEN it starts: this prints shot2/shot3's raw
// Sim::stateHash(id) (not folded/XORed — the actual per-frame value) for
// each of the first 8 frames, so a diff against another platform's trace
// pinpoints the exact frame the divergence begins at — frame 0 implicates
// the spawn-time / first-force-model setup, a later frame implicates
// accumulation.
static int bitexact_frame_trace() {
    Environment env;
    SimConfig cfg;
    cfg.determinism = config::Determinism::BitExact;
    cfg.trajectoryCacheFrames = 8;
    Sim sim(env, cfg);

    ProjectileType bullet;
    bullet.id = "bx_308"; bullet.klass = ProjectileClass::Bullet;
    bullet.dragModel = DragModel::G7; bullet.ballisticCoefficient = 0.243;
    bullet.mass_kg = 0.0113; bullet.refDiameter_m = 0.00782;
    bullet.twistRate_m = 0.254;
    const TypeId bid = sim.registerType(bullet);

    auto fire = [&](TypeId id, Vec3 dir, double spd, FidelityTier tier, PrecisionFlag pf) {
        LaunchParams lp;
        lp.position = {0, 2.0, 0};
        lp.direction = dir; lp.speed = spd; lp.tier = tier; lp.precision = pf;
        return sim.spawn(id, lp);
    };
    const StateId h2 = fire(bid, {1, 0.02, 0.01}, 800.0, FidelityTier::Integrated,
                            PrecisionFlag::SixDOF);
    const StateId h3 = fire(bid, {1, 0.04, 0.00}, 810.0, FidelityTier::Integrated,
                            PrecisionFlag::SpinDrift | PrecisionFlag::Coriolis);

    auto bits = [](double v) {
        static_assert(sizeof(double) == sizeof(std::uint64_t));
        std::uint64_t u;
        std::memcpy(&u, &v, sizeof(u));
        return u;
    };

    // Both prior BitExact mismatches (shot3's distanceTravelled_m, shot2's
    // writeBack orientation Quat — see dist_no_fma/quat_mul_no_fma in
    // sim.cpp) were macOS/Apple-Clang-(-O0)-only and found by diffing this
    // per-frame trace against another platform's. Kept small (8 frames) as
    // a standing diagnostic for the next one.
    EmptyWorld world;
    VectorEventSink sink;
    for (int f = 0; f < 8; ++f) {
        sim.step(1.0 / 200.0, world, sink);
        std::printf("bitexact-frame-trace: frame %d  shot2 = %016llx  shot3 = %016llx\n",
                    f, (unsigned long long)sim.stateHash(h2),
                    (unsigned long long)sim.stateHash(h3));
    }
    (void)bits;
    return 0;
}

// See PerShotDigests' comment. Labels match bitexact_run()'s fire() order.
static int bitexact_per_shot() {
    static const char* const kLabel[5] = {
        "shot0 Integrated/None", "shot1 AnalyticDrag/None", "shot2 Integrated/SixDOF",
        "shot3 Integrated/SpinDrift+Coriolis", "shot4 Integrated/None(arrow)"};
    PerShotDigests ps;
    (void)bitexact_run(&ps);
    for (int i = 0; i < 5; ++i)
        std::printf("bitexact-per-shot: %-38s digest = %016llx\n",
                    kLabel[i], (unsigned long long)ps.digest[i]);
    return 0;
}

int main(int argc, char** argv) {
    constexpr int kFrames = 120;
    const bool check = argc > 1 && std::strcmp(argv[1], "--check") == 0;
    if (argc > 1 && std::strcmp(argv[1], "--det-check") == 0) return det_check();
    if (argc > 1 && std::strcmp(argv[1], "--bitexact-golden") == 0) return bitexact_golden();
    if (argc > 1 && std::strcmp(argv[1], "--bitexact-rkf45-check") == 0) return bitexact_rkf45_check();
    if (argc > 1 && std::strcmp(argv[1], "--bitexact-per-shot") == 0) return bitexact_per_shot();
    if (argc > 1 && std::strcmp(argv[1], "--bitexact-frame-trace") == 0) return bitexact_frame_trace();
    if (argc > 1 && std::strcmp(argv[1], "--bitexact-print") == 0) {
        std::printf("0x%016llxull\n", (unsigned long long)bitexact_run());
        return 0;
    }

    // Ceilings are deliberately loose — they gate a gross regression (roughly a
    // 10x blow-up), not a few-percent drift, and pass in a Debug CI build too.
    struct Row { const char* name; double ms; double ceil_ms; };
    const Row rows[] = {
        {"AnalyticDrag  10000", run(FidelityTier::AnalyticDrag, 10000, kFrames),        60.0},
        {"Integrated     1000", run(FidelityTier::Integrated,    1000, kFrames),       120.0},
        {"Integrated/SoA 1000", run(FidelityTier::Integrated,    1000, kFrames, true), 120.0},
        {"Hitscan       10000", run(FidelityTier::Hitscan,       10000, kFrames),       20.0},
    };

    int rc = 0;
    for (const Row& r : rows) {
        std::printf("%-20s %9.4f ms/frame", r.name, r.ms);
        if (check) {
            const bool over = r.ms > r.ceil_ms;
            std::printf("   (ceiling %.0f%s)", r.ceil_ms, over ? " — OVER BUDGET" : "");
            if (over) rc = 1;
        }
        std::printf("\n");
    }
    return rc;
}
