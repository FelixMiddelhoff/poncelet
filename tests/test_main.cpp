// poncelet — smoke tests (item 1) + exterior-ballistics core validation (item 2).
// SPDX-License-Identifier: MIT
#include "check.hpp"

#include "poncelet/poncelet.hpp"
#include "poncelet/poncelet.h"
#include "poncelet/atmosphere.hpp"

#include "drag_tables.hpp" // item 3: G1/G7 standard-curve unit checks

#include "fixed_point.hpp" // Part B2: Q32.32 fixed-point primitive
#include "fixed_lut.hpp"   // Part B4: fixed-point transcendental LUTs
#include "drag_fx.hpp"     // Part B5: fixed-point drag/lift LUT sampling
#include "avec3.hpp"       // Part B6: accumulator-templated vector
#include "accum_ops.hpp"   // Part B6: accumulator-generic math
#include "integrate.hpp"   // Part B6b: integrator entry points (bitExact dispatch)

#include "poncelet/catalog.hpp" // item 9: named catalog + material table
#include "poncelet/worlds.hpp"  // Tier D1: stock World primitives

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

using namespace pon;

namespace {

// An independent reference solver: plain forward Euler at a tiny fixed step,
// quadratic constant-Cd drag + constant gravity, no wind. Deliberately a
// different scheme from the production RK4 / semi-implicit core so an agreement
// check actually means something. Returns the state after `flight_s` seconds
// (or when y drops below `yFloor`).
struct RefState { Vec3 pos, vel; double t; };

RefState reference_flight(Vec3 x0, Vec3 v0, double mass, double cd, double dia,
                          double rho, Vec3 g, double flight_s,
                          double yFloor = -1e30, double dtRef = 2.0e-6) {
    const double area = 0.25 * 3.14159265358979323846 * dia * dia;
    const double k2 = 0.5 * rho * cd * area / mass; // |a_drag| = k2 * |v|^2
    RefState s{x0, v0, 0.0};
    const long steps = static_cast<long>(flight_s / dtRef + 0.5);
    for (long i = 0; i < steps; ++i) {
        const double sp = std::sqrt(dot(s.vel, s.vel));
        Vec3 a = g;
        if (sp > 0.0) a = a - s.vel * (k2 * sp);
        s.vel = s.vel + a * dtRef;
        s.pos = s.pos + s.vel * dtRef;
        s.t += dtRef;
        if (s.pos.y < yFloor) break;
    }
    return s;
}

} // namespace

PON_TEST(vec3_math) {
    Vec3 a{1, 2, 3}, b{4, 5, 6};
    CHECK_NEAR(dot(a, b), 32.0, 1e-12);
    Vec3 c = cross({1, 0, 0}, {0, 1, 0});
    CHECK_NEAR(c.z, 1.0, 1e-12);
    CHECK_NEAR(length({3, 4, 0}), 5.0, 1e-12);
}

PON_TEST(vacuum_range_matches_analytic) {
    // No drag: a 45-degree shot at v0 in vacuum has range v0^2 sin(2t)/g.
    Environment env;
    env.airDensity_kgm3 = 0.0; // vacuum -> drag term vanishes for any model
    Sim sim(env);
    ProjectileType t;
    t.id = "test_rock";
    t.klass = ProjectileClass::Rock;
    t.dragModel = DragModel::G1;
    TypeId id = sim.registerType(t);
    CHECK(id != kInvalidType);

    const double v0 = 40.0;
    const double s = std::sqrt(0.5);
    LaunchParams lp;
    lp.direction = {s, s, 0};
    lp.speed = v0;
    lp.tier = FidelityTier::Integrated;
    StateId h = sim.spawn(id, lp);
    CHECK(h != kInvalidState);

    EmptyWorld world;
    VectorEventSink sink;
    for (int i = 0; i < 2000 && sim.state(h).position.y >= 0.0; ++i)
        sim.step(1.0 / 240.0, world, sink);

    const double g = 9.80665;
    const double expected = v0 * v0 / g; // sin(90 deg) = 1
    CHECK_NEAR(sim.state(h).position.x, expected, expected * 0.02);
}

PON_TEST(constant_cd_drag_shortens_range) {
    Sim sim;
    ProjectileType t;
    t.id = "arrow";
    t.klass = ProjectileClass::Arrow; // ConstantCd
    TypeId id = sim.registerType(t);

    LaunchParams lp;
    const double s = std::sqrt(0.5);
    lp.direction = {s, s, 0};
    lp.speed = 60.0;
    StateId h = sim.spawn(id, lp);

    EmptyWorld world;
    VectorEventSink sink;
    for (int i = 0; i < 4000 && sim.state(h).position.y >= 0.0; ++i)
        sim.step(1.0 / 240.0, world, sink);

    const double vacuum = 60.0 * 60.0 / 9.80665;
    CHECK(sim.state(h).position.x < vacuum); // drag ate some range
    CHECK(sim.state(h).position.x > 0.0);
}

PON_TEST(bitexact_is_supported_and_runs) {
    // Part B: the Q32.32 fixed-point core makes BitExact a live guarantee.
    Sim sim(Environment{}, SimConfig{config::Determinism::BitExact});
    CHECK(sim.status() == Status::Ok);

    ProjectileType t;
    t.id = "bx"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 300; ++i) sim.step(1.0 / 600.0, w, sink);
    const auto& s = sim.state(h);
    CHECK(std::isfinite(s.position.x) && s.position.x > 100.0);
    CHECK(length(s.velocity) < 800.0 && length(s.velocity) > 200.0);
}

PON_TEST(bitexact_two_runs_fold_to_the_same_state_hash) {
    auto run = [] {
        Sim sim(Environment{}, SimConfig{config::Determinism::BitExact});
        ProjectileType t;
        t.id = "bx"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
        t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0.02, 0};
        lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
        sim.spawn(id, lp);
        EmptyWorld w; VectorEventSink sink;
        for (int i = 0; i < 400; ++i) sim.step(1.0 / 500.0, w, sink);
        return sim.stateHash();
    };
    CHECK(run() == run());
}

PON_TEST(per_shot_state_hash_pinpoints_the_desynced_shot) {
    Sim sim;
    ProjectileType t;
    t.id = "psh"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009;
    const TypeId tid = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0, 0}; lp.speed = 360.0;
    const StateId a = sim.spawn(tid, lp);
    lp.direction = {1, 0.1, 0}; // a different shot -> a different digest
    const StateId b = sim.spawn(tid, lp);

    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 30; ++i) sim.step(1.0 / 240.0, w, sink);

    const std::uint64_t ha = sim.stateHash(a);
    const std::uint64_t hb = sim.stateHash(b);
    CHECK(ha != 0 && hb != 0 && ha != hb);

    // The whole-sim hash changes if EITHER shot's per-shot hash would change
    // (sanity: the two aren't computed from disjoint, meaningless data).
    sim.despawn(b);
    CHECK(sim.stateHash(a) == ha); // a's own digest is unaffected by b's despawn
    CHECK(sim.stateHash(b) == 0);  // dead / recycled slot -> 0

    // Out-of-range id -> 0, not a crash.
    CHECK(sim.stateHash(9999) == 0);
}

namespace {
TypeId register_snapshot_test_type(Sim& sim) {
    ProjectileType t;
    t.id = "snaptest"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.25;
    t.mass_kg = 0.011; t.refDiameter_m = 0.0078; t.twistRate_m = 0.254;
    return sim.registerType(t);
}

void spawn_snapshot_test_scene(Sim& sim, TypeId tid) {
    LaunchParams lp; lp.position = {0, 2, 0};
    lp.direction = {1, 0.04, 0.01}; lp.speed = 810.0;
    lp.tier = FidelityTier::Integrated; sim.spawn(tid, lp);
    lp.tier = FidelityTier::AnalyticDrag; lp.direction = {1, 0.06, -0.02};
    sim.spawn(tid, lp);
    lp.tier = FidelityTier::Integrated; lp.precision = PrecisionFlag::SixDOF;
    lp.direction = {1, 0.02, 0.0}; lp.spin = 1600.0; lp.initialYaw = 0.02;
    sim.spawn(tid, lp);
}
} // namespace

PON_TEST(snapshot_restore_round_trip_matches_state_hash) {
    // snapshot -> step N -> restore -> step N again must fold to the same
    // stateHash() every frame as continuing the original run uninterrupted.
    SimConfig cfg; cfg.trajectoryCacheFrames = 4;
    Sim sim(Environment{}, cfg);
    const TypeId tid = register_snapshot_test_type(sim);
    spawn_snapshot_test_scene(sim, tid);

    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 50; ++i) sim.step(1.0 / 250.0, w, sink);

    const std::vector<std::byte> snap = sim.snapshot();
    CHECK(!snap.empty());

    std::vector<std::uint64_t> continued;
    for (int i = 0; i < 200; ++i) { sim.step(1.0 / 250.0, w, sink); continued.push_back(sim.stateHash()); }

    CHECK(sim.restore(snap.data(), snap.size()));
    std::vector<std::uint64_t> replayed;
    for (int i = 0; i < 200; ++i) { sim.step(1.0 / 250.0, w, sink); replayed.push_back(sim.stateHash()); }

    CHECK(continued.size() == replayed.size());
    bool same = continued.size() == replayed.size();
    for (std::size_t i = 0; same && i < continued.size(); ++i) same = continued[i] == replayed[i];
    CHECK(same);
}

PON_TEST(snapshot_restore_across_separate_sim_instances) {
    // A snapshot taken from one Sim restores cleanly into a different Sim that
    // has the same types registered in the same order — the rollback-netcode
    // case (late-join / a fresh Sim resynced from a host snapshot).
    SimConfig cfg; cfg.trajectoryCacheFrames = 2;

    Sim src(Environment{}, cfg);
    const TypeId srcTid = register_snapshot_test_type(src);
    spawn_snapshot_test_scene(src, srcTid);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 30; ++i) src.step(1.0 / 250.0, w, sink);
    const std::vector<std::byte> snap = src.snapshot();

    Sim dst(Environment{}, cfg);
    register_snapshot_test_type(dst); // same id, same order -> same type digest
    CHECK(dst.restore(snap.data(), snap.size()));
    CHECK(dst.stateHash() == src.stateHash());

    for (int i = 0; i < 40; ++i) {
        src.step(1.0 / 250.0, w, sink);
        dst.step(1.0 / 250.0, w, sink);
        CHECK(dst.stateHash() == src.stateHash());
    }
}

PON_TEST(snapshot_restore_rejects_type_count_mismatch) {
    Sim src(Environment{}, SimConfig{});
    const TypeId tid = register_snapshot_test_type(src);
    spawn_snapshot_test_scene(src, tid);
    const std::vector<std::byte> snap = src.snapshot();

    Sim dst(Environment{}, SimConfig{});
    register_snapshot_test_type(dst);
    ProjectileType extra; extra.id = "extra"; extra.klass = ProjectileClass::Bullet;
    extra.mass_kg = 0.01; extra.refDiameter_m = 0.008; extra.dragCoefficient = 0.3;
    dst.registerType(extra); // one more type than src -> registry digest can't match
    LaunchParams lp; lp.position = {5, 5, 5}; lp.direction = {0, 1, 0}; lp.speed = 1.0;
    dst.spawn(0, lp);
    const std::uint64_t before = dst.stateHash();

    CHECK(!dst.restore(snap.data(), snap.size()));
    CHECK(dst.stateHash() == before); // rejected restore leaves the Sim untouched

    // A truncated buffer is rejected the same way, not read out of bounds.
    CHECK(!dst.restore(snap.data(), snap.size() > 4 ? snap.size() - 4 : 0));
}

PON_TEST(bitexact_snapshot_round_trips_byte_identically) {
    Sim sim(Environment{}, SimConfig{config::Determinism::BitExact});
    const TypeId tid = register_snapshot_test_type(sim);
    spawn_snapshot_test_scene(sim, tid);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 40; ++i) sim.step(1.0 / 250.0, w, sink);

    const std::vector<std::byte> a = sim.snapshot();
    CHECK(sim.restore(a.data(), a.size()));
    const std::vector<std::byte> b = sim.snapshot();
    CHECK(a.size() == b.size());
    CHECK(a.size() > 0 && std::memcmp(a.data(), b.data(), a.size()) == 0);
}

PON_TEST(c_abi_snapshot_restore_roundtrip) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0) != 0xFFFFFFFFu);
    for (int i = 0; i < 20; ++i) pon_step(sim, 1.0 / 240.0);

    const size_t need = pon_sim_snapshot(sim, nullptr, 0);
    CHECK(need > 0);
    std::vector<unsigned char> buf(need);
    CHECK(pon_sim_snapshot(sim, buf.data(), buf.size()) == need);

    for (int i = 0; i < 10; ++i) pon_step(sim, 1.0 / 240.0);
    CHECK(pon_sim_restore(sim, buf.data(), buf.size()) == 1);

    pon_state st{};
    CHECK(pon_get_state(sim, 0, &st) == PON_OK);
    // After 20 restored steps the round is well downrange but has not yet hit
    // the ground; a bad restore (e.g. reading the pre-step frame) still passes
    // this loosely, so it mainly guards against a crash / obviously wrong state.
    CHECK(st.position.x > 0.0);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_roundtrip) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    CHECK(sim != nullptr);

    pon_projectile_desc d{};
    d.id = "9mm";
    d.klass = 0; // Bullet
    d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008;
    d.ref_diameter_m = 0.009;
    d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);

    uint32_t sid = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);
    CHECK(sid != 0xFFFFFFFFu);
    CHECK(pon_live_count(sim) == 1);

    for (int i = 0; i < 30; ++i) pon_step(sim, 1.0 / 240.0);

    pon_state st{};
    CHECK(pon_get_state(sim, sid, &st) == PON_OK);
    CHECK(st.position.x > 0.0);      // moved downrange
    CHECK(st.velocity.x < 360.0);    // drag decelerated it
    CHECK(st.position.y < 2.0);      // and it dropped

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_preview_arc_measure_then_fill) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);

    const size_t n = pon_preview_arc(sim, tid, {0, 2, 0}, {1, 0, 0},
                                     1.0 / 60.0, 2.0, -1e30, nullptr, 0);
    CHECK(n > 1);

    std::vector<pon_vec3> pts(n);
    const size_t nw = pon_preview_arc(sim, tid, {0, 2, 0}, {1, 0, 0},
                                      1.0 / 60.0, 2.0, -1e30, pts.data(), pts.size());
    CHECK(nw == n);
    CHECK(pts.front().x == 0.0); // muzzle first
    CHECK(pts.back().x > pts.front().x); // arc went downrange

    // A too-small buffer truncates to `max` and still reports the full count.
    std::vector<pon_vec3> small(2);
    const size_t truncated = pon_preview_arc(sim, tid, {0, 2, 0}, {1, 0, 0},
                                             1.0 / 60.0, 2.0, -1e30, small.data(), 2);
    CHECK(truncated == 2);

    // An out-of-range type id writes / returns nothing rather than crashing.
    CHECK(pon_preview_arc(sim, 9999, {0, 0, 0}, {1, 0, 0}, 0.01, 1.0, -1e30, nullptr, 0) == 0);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_describe_and_last_error) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    const uint32_t sid = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);
    for (int i = 0; i < 30; ++i) pon_step(sim, 1.0 / 240.0);

    const size_t need = pon_describe(sim, sid, nullptr, 0);
    CHECK(need > 0);
    std::string text(need + 1, '\0');
    const size_t got = pon_describe(sim, sid, text.data(), text.size());
    CHECK(got == need);
    CHECK(text.find("v=") != std::string::npos);
    CHECK(text.find("E=") != std::string::npos); // mass known -> energy shown

    // A too-small buffer truncates but still NUL-terminates and reports the
    // untruncated length.
    char tiny[8];
    const size_t full = pon_describe(sim, sid, tiny, sizeof tiny);
    CHECK(full == need);
    CHECK(tiny[sizeof(tiny) - 1] == '\0');

    // lastError(): "" after a successful call, non-empty after a rejected one.
    CHECK(std::string(pon_last_error(sim)).empty());
    pon_projectile_desc bad{}; // empty id -> registerType rejects it
    CHECK(pon_register_type(sim, &bad) == 0xFFFFFFFFu);
    CHECK(!std::string(pon_last_error(sim)).empty());

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_describe_type) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);

    const size_t need = pon_describe_type(sim, tid, nullptr, 0);
    CHECK(need > 0);
    std::string text(need + 1, '\0');
    CHECK(pon_describe_type(sim, tid, text.data(), text.size()) == need);
    CHECK(text.find("9mm:") != std::string::npos);
    CHECK(text.find("Cd 0.30") != std::string::npos);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_live_ids) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    const uint32_t a = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);
    const uint32_t b = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);
    pon_despawn(sim, a);

    const size_t n = pon_live_ids(sim, nullptr, 0);
    CHECK(n == 1);
    CHECK(n == pon_live_count(sim));

    uint32_t out[4] = {};
    CHECK(pon_live_ids(sim, out, 4) == 1);
    CHECK(out[0] == b);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_shot_mach_and_energy) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    const uint32_t sid = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);

    const double m = pon_shot_mach(sim, sid);
    const double e = pon_shot_energy_j(sim, sid);
    CHECK(m > 0.5);
    CHECK(e > 0.0);

    char line[192];
    pon_describe(sim, sid, line, sizeof line);
    char buf[16];
    std::snprintf(buf, sizeof buf, "M=%.2f", m);
    CHECK(std::string(line).find(buf) != std::string::npos);

    // An out-of-range state id is inert, not a crash.
    CHECK(pon_shot_mach(sim, 9999) == 0.0);
    CHECK(pon_shot_energy_j(sim, 9999) == 0.0);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_preview_arc_ex_reports_velocity_and_time) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);

    const size_t n = pon_preview_arc_ex(sim, tid, {0, 2, 0}, {1, 0, 0},
                                        1.0 / 60.0, 2.0, -1e30, nullptr, 0);
    CHECK(n > 1);
    std::vector<pon_trajectory_sample> pts(n);
    const size_t nw = pon_preview_arc_ex(sim, tid, {0, 2, 0}, {1, 0, 0},
                                         1.0 / 60.0, 2.0, -1e30, pts.data(), pts.size());
    CHECK(nw == n);
    CHECK(pts.front().time_s == 0.0);
    CHECK(pts.back().time_s > 0.0);
    CHECK(pts.front().velocity.x > 300.0); // ~muzzle speed

    CHECK(pon_preview_arc_ex(sim, 9999, {0, 0, 0}, {1, 0, 0}, 0.01, 1.0, -1e30, nullptr, 0) == 0);
    pon_sim_destroy(sim);
}

PON_TEST(c_abi_trajectory_cache) {
    pon_sim_config cfg{};
    cfg.trajectory_cache_frames = 8;
    pon_sim* sim = pon_sim_create_ex(PON_DET_PLATFORM_STABLE, &cfg);

    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    const uint32_t sid = pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0);

    for (int i = 0; i < 5; ++i) pon_step(sim, 1.0 / 60.0);
    CHECK(pon_trajectory_size(sim, sid) == 6); // spawn stamp + 5 steps

    const size_t need = pon_trajectory(sim, sid, nullptr, 0);
    CHECK(need == pon_trajectory_size(sim, sid));
    std::vector<pon_trajectory_sample> buf(need);
    CHECK(pon_trajectory(sim, sid, buf.data(), buf.size()) == need);
    CHECK(buf.front().time_s == 0.0);
    CHECK(buf.back().time_s > buf.front().time_s);
    CHECK(buf.back().position.x > buf.front().position.x);

    // A too-small buffer truncates to `max`, oldest-first.
    std::vector<pon_trajectory_sample> small(2);
    CHECK(pon_trajectory(sim, sid, small.data(), 2) == 2);
    CHECK(small[0].time_s == buf[0].time_s);
    CHECK(small[1].time_s == buf[1].time_s);

    pon_vec3 pos{}, vel{};
    const double midT = (buf.front().time_s + buf.back().time_s) * 0.5;
    CHECK(pon_sample_trajectory(sim, sid, midT, &pos, &vel) == 1);
    CHECK(pos.x > buf.front().position.x && pos.x < buf.back().position.x);

    // Cache off / bad id -> empty / rejected, not a crash.
    CHECK(pon_trajectory_size(sim, 9999) == 0);
    CHECK(pon_sample_trajectory(sim, 9999, 0.0, &pos, &vel) == 0);

    pon_sim_destroy(sim);
}

PON_TEST(c_abi_sim_create_ex_applies_config) {
    pon_sim_config cfg{};
    cfg.fixed_step_s = 1.0 / 2000.0;
    cfg.integrator = PON_INTEGRATOR_SEMI_IMPLICIT;
    cfg.pos_tolerance_m = 0.05;
    cfg.batch_integrator = 1;
    cfg.trajectory_cache_frames = 8;
    pon_sim* sim = pon_sim_create_ex(PON_DET_PLATFORM_STABLE, &cfg);
    CHECK(sim != nullptr);

    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(pon_spawn(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0) != 0xFFFFFFFFu);
    for (int i = 0; i < 10; ++i) pon_step(sim, 1.0 / 60.0); // exercises the config, not asserted bit-for-bit
    CHECK(pon_live_count(sim) == 1);
    pon_sim_destroy(sim);

    // cfg = NULL is exactly pon_sim_create().
    pon_sim* plain = pon_sim_create_ex(PON_DET_PLATFORM_STABLE, nullptr);
    CHECK(plain != nullptr);
    pon_sim_destroy(plain);
}

PON_TEST(version_string) {
    CHECK(std::string(pon_version_string()) == PONCELET_VERSION_STRING);
    CHECK(std::string(library_version_string()) == PONCELET_VERSION_STRING);
}

// --- Item 2: exterior-ballistics core -------------------------------------

PON_TEST(rk4_matches_reference_quadratic_drag) {
    // A .308-class round, constant-Cd, fired flat. The Integrated tier (RK4,
    // 1 kHz nominal) must track an independent 2 µs forward-Euler reference.
    Sim sim;
    ProjectileType t;
    t.id = "ref_308"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    t.maxRange_m = 5000.0; t.maxLifetime_s = 20.0;
    const TypeId id = sim.registerType(t);

    const Vec3 x0{0, 2.0, 0}, dir{1, 0, 0};
    const double v0 = 800.0, flight = 1.5;
    LaunchParams lp; lp.position = x0; lp.direction = dir; lp.speed = v0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    for (double tt2 = 0; tt2 < flight - 1e-9; tt2 += 1.0 / 240.0)
        sim.step(1.0 / 240.0, world, sink);

    const RefState r = reference_flight(x0, dir * v0, t.mass_kg, 0.30,
                                        t.refDiameter_m, 1.225,
                                        {0, -9.80665, 0}, flight);
    const ProjectileState& s = sim.state(h);
    CHECK(s.alive);
    CHECK_NEAR(s.position.x, r.pos.x, 0.5);   // ~500 m downrange, < 0.5 m error
    CHECK_NEAR(s.position.y, r.pos.y, 0.05);  // drop within 5 cm
    CHECK_NEAR(length(s.velocity), length(r.vel), 1.0);
}

PON_TEST(vertical_drop_reaches_terminal_velocity) {
    // A sphere dropped from rest converges to v_t = sqrt(m g / (0.5 rho Cd A)).
    Sim sim;
    ProjectileType t;
    t.id = "ball"; t.klass = ProjectileClass::Custom;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.47;
    t.mass_kg = 0.145; t.refDiameter_m = 0.073;
    t.maxLifetime_s = 60.0; t.maxRange_m = 1e9;
    const TypeId id = sim.registerType(t);

    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {0, -1, 0};
    lp.speed = 0.01; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 1200; ++i) sim.step(1.0 / 120.0, world, sink); // 10 s

    const double area = 0.25 * 3.14159265358979323846 * 0.073 * 0.073;
    const double vt = std::sqrt(0.145 * 9.80665 / (0.5 * 1.225 * 0.47 * area));
    CHECK_NEAR(-sim.state(h).velocity.y, vt, vt * 0.01);
}

PON_TEST(analytic_tier_tracks_integrated) {
    // The closed-form fast path is an approximation of quadratic drag; over an
    // arrow's flight it should stay within a few percent of the integrator.
    auto run = [](FidelityTier tier) {
        Sim sim;
        ProjectileType t;
        t.id = "arrow"; t.klass = ProjectileClass::Arrow;
        t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.42;
        t.mass_kg = 0.026; t.refDiameter_m = 0.0079;
        t.maxRange_m = 1000.0; t.maxLifetime_s = 30.0;
        const TypeId id = sim.registerType(t);
        const double s2 = std::sqrt(0.5);
        LaunchParams lp; lp.position = {0, 1.6, 0};
        lp.direction = {s2, s2, 0}; lp.speed = 60.0; lp.tier = tier;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld world; VectorEventSink sink;
        for (int i = 0; i < 2000 && sim.state(h).position.y > 0.0; ++i)
            sim.step(1.0 / 240.0, world, sink);
        return sim.state(h).position;
    };
    const Vec3 a = run(FidelityTier::AnalyticDrag);
    const Vec3 b = run(FidelityTier::Integrated);
    CHECK_NEAR(a.x, b.x, b.x * 0.05); // range within 5%
}

PON_TEST(arrow_drop_is_physically_sane) {
    // Traditional-bow arrow, ~55 m/s, launched horizontally from 1.6 m.
    // Cross-checked against the independent reference solver; the absolute
    // drop over 30 m of downrange travel is ~1.7 m (a bit more than the
    // drag-free 0.5 g (x/v0)^2 = 1.46 m because drag stretches the flight
    // time), which matches published traditional-archery drop charts.
    Sim sim;
    ProjectileType t;
    t.id = "trad_arrow"; t.klass = ProjectileClass::Arrow;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.42;
    t.mass_kg = 0.033; t.refDiameter_m = 0.0079;
    t.maxRange_m = 500.0; t.maxLifetime_s = 30.0;
    const TypeId id = sim.registerType(t);

    const Vec3 x0{0, 1.6, 0}, dir{1, 0, 0};
    const double v0 = 55.0;
    LaunchParams lp; lp.position = x0; lp.direction = dir; lp.speed = v0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    while (sim.state(h).position.x < 30.0 && sim.state(h).alive)
        sim.step(1.0 / 480.0, world, sink);

    const double drop = x0.y - sim.state(h).position.y;
    const double tof  = sim.state(h).timeAlive_s;
    const RefState r = reference_flight(x0, dir * v0, t.mass_kg, 0.42,
                                        t.refDiameter_m, 1.225,
                                        {0, -9.80665, 0}, tof);
    CHECK_NEAR(sim.state(h).position.y, r.pos.y, 0.03);
    CHECK(drop > 1.46);   // more than the drag-free drop
    CHECK(drop < 2.2);    // but not wildly so
}

PON_TEST(platform_stable_is_repeatable) {
    // Same inputs, two Sims -> bit-identical trajectory (PlatformStable).
    auto fire = [](FidelityTier tier) {
        Sim sim;
        ProjectileType t;
        t.id = "x"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.31;
        t.mass_kg = 0.008; t.refDiameter_m = 0.009;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 1.7, 0};
        lp.direction = {0.99, 0.12, 0.03}; lp.speed = 420.0; lp.tier = tier;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld world; VectorEventSink sink;
        for (int i = 0; i < 300; ++i) sim.step(1.0 / 200.0, world, sink);
        return sim.state(h).position;
    };
    for (FidelityTier tier : {FidelityTier::AnalyticDrag, FidelityTier::Integrated}) {
        const Vec3 p = fire(tier), q = fire(tier);
        CHECK(p.x == q.x && p.y == q.y && p.z == q.z);
    }
}

PON_TEST(validation_state_hash_is_bit_repeatable) {
    // Sim::stateHash() over a mixed-tier scene must be identical across two
    // from-scratch runs, every frame (PlatformStable). This is the digest the
    // `poncelet_bench --det-check` CI gate and a rollback-netcode desync check
    // both rely on.
    auto run = [](std::vector<std::uint64_t>& hs) {
        SimConfig cfg;
        cfg.trajectoryCacheFrames = 4;
        Sim sim(Environment{}, cfg);

        ProjectileType b;
        b.id = "h"; b.klass = ProjectileClass::Bullet;
        b.dragModel = DragModel::G7; b.ballisticCoefficient = 0.25;
        b.mass_kg = 0.011; b.refDiameter_m = 0.0078; b.twistRate_m = 0.254;
        const TypeId bid = sim.registerType(b);

        LaunchParams lp; lp.position = {0, 2, 0};
        lp.direction = {1, 0.04, 0.01}; lp.speed = 810.0;
        lp.tier = FidelityTier::Integrated; sim.spawn(bid, lp);
        lp.tier = FidelityTier::AnalyticDrag; lp.direction = {1, 0.06, -0.02};
        sim.spawn(bid, lp);
        lp.tier = FidelityTier::Integrated; lp.precision = PrecisionFlag::SixDOF;
        lp.direction = {1, 0.02, 0.0}; lp.spin = 1600.0; lp.initialYaw = 0.02;
        sim.spawn(bid, lp);

        EmptyWorld w; VectorEventSink sink;
        for (int i = 0; i < 500; ++i) {
            sim.step(1.0 / 250.0, w, sink);
            hs.push_back(sim.stateHash());
        }
    };
    std::vector<std::uint64_t> a, c;
    run(a); run(c);
    CHECK(a.size() == c.size() && !a.empty());
    bool same = a.size() == c.size();
    for (std::size_t i = 0; same && i < a.size(); ++i) same = a[i] == c[i];
    CHECK(same);
    // A moving sim must not be hashing to a constant (would mask a frozen state).
    CHECK(a.front() != a.back());
}

PON_TEST(semi_implicit_and_rk4_converge) {
    // Both integrators must agree with the reference as the step shrinks;
    // check they land close to each other and to the reference at 4 kHz.
    auto run = [](Integrator scheme) {
        SimConfig cfg; cfg.integrator = scheme; cfg.fixedStep_s = 1.0 / 4000.0;
        Sim sim(Environment{}, cfg);
        ProjectileType t;
        t.id = "p"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
        t.mass_kg = 0.01; t.refDiameter_m = 0.008; t.maxRange_m = 1e4;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0.2, 0};
        lp.speed = 300.0; lp.tier = FidelityTier::Integrated;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld world; VectorEventSink sink;
        for (int i = 0; i < 240; ++i) sim.step(1.0 / 240.0, world, sink); // 1 s
        return sim.state(h).position;
    };
    const Vec3 si = run(Integrator::SemiImplicit);
    const Vec3 rk = run(Integrator::RK4);
    const Vec3 dn{1.0, 0.2, 0.0};
    const RefState r = reference_flight({0,0,0}, normalized(dn) * 300.0,
                                        0.01, 0.30, 0.008, 1.225,
                                        {0, -9.80665, 0}, 1.0);
    CHECK_NEAR(si.x, rk.x, 0.05);
    CHECK_NEAR(rk.x, r.pos.x, 0.10);
    CHECK_NEAR(rk.y, r.pos.y, 0.02);
}

// --- Item 3: G1/G7 drag models + BC scaling + custom curves + ISA ----------

namespace {
// Free-flight until the projectile passes `x_target` downrange (fired from a
// high start so it never reaches the ground). Returns speed + time there.
struct DownrangeSample { double speed, tof, drop; };
DownrangeSample fly_to(Sim& sim, StateId h, double x_target) {
    EmptyWorld world; VectorEventSink sink;
    const double y0 = sim.state(h).position.y;
    for (int i = 0; i < 200000 && sim.state(h).alive &&
                    sim.state(h).position.x < x_target; ++i)
        sim.step(1.0 / 1000.0, world, sink);
    const ProjectileState& s = sim.state(h);
    return {length(s.velocity), s.timeAlive_s, y0 - s.position.y};
}
} // namespace

PON_TEST(g1_g7_table_shapes) {
    using namespace pon::detail;
    // BRL/McCoy standard curves (JBM mcg1.txt / mcg7.txt). Subsonic: G1 sits
    // ~0.20, the sleek G7 boat-tail ~0.12.
    CHECK(g1_cd(0.5) > 0.19 && g1_cd(0.5) < 0.21);
    CHECK(g7_cd(0.5) > 0.11 && g7_cd(0.5) < 0.13);
    // Transonic rise to a peak, then a slow supersonic decline. G1 peaks higher
    // (~0.66 near Mach 1.4) and later; G7 peaks ~0.40 near Mach 1.05.
    CHECK(g1_cd(1.4) > 0.64 && g1_cd(1.4) < 0.67);
    CHECK(g7_cd(1.05) > 0.39 && g7_cd(1.05) < 0.41);
    CHECK(g1_cd(1.4) > g7_cd(1.4));
    CHECK(g1_cd(3.0) < g1_cd(1.4));
    CHECK(g7_cd(3.0) < g7_cd(1.05));
    // Clamped outside the tabulated range.
    CHECK_NEAR(g7_cd(-1.0), g7_cd(0.0), 1e-12);
    CHECK_NEAR(g1_cd(99.0), g1_cd(5.0), 1e-12);
}

PON_TEST(g7_308_175smk_matches_published_dope) {
    // .308 Win, 175 gr Sierra MatchKing, published G7 BC 0.243, MV 792 m/s,
    // ICAO sea level. Published dope (Berger / JBM): ~342 m/s (1120 fps),
    // near-transonic at 914 m, bore-line drop ~12 m there. With the BRL/McCoy
    // G7 table the library produces ~336 m/s / 1.75 s / 11.5 m drop at 900 m —
    // inside 2 % of the published retained velocity.
    Sim sim;
    ProjectileType t;
    t.id = "762x51_175smk"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 792.0; lp.tier = FidelityTier::Integrated;
    const DownrangeSample d = fly_to(sim, sim.spawn(id, lp), 900.0);
    CHECK(d.speed > 315.0 && d.speed < 360.0); // ~336 m/s, still ~transonic
    CHECK(d.tof   > 1.55  && d.tof   < 1.95);
    CHECK(d.drop  > 9.0   && d.drop  < 15.0);
}

PON_TEST(g1_9mm_124fmj_retained_velocity_100m) {
    // 9x19, 124 gr FMJ, G1 BC ~0.15, MV 360 m/s. Published ~300 m/s at 100 m.
    Sim sim;
    ProjectileType t;
    t.id = "9x19_124fmj"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G1; t.ballisticCoefficient = 0.15;
    t.mass_kg = 0.00804; t.refDiameter_m = 0.00902;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 2000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 360.0; lp.tier = FidelityTier::Integrated;
    const DownrangeSample d = fly_to(sim, sim.spawn(id, lp), 100.0);
    CHECK(d.speed > 255.0 && d.speed < 330.0); // ~300 m/s, transonic-band G1
    CHECK(d.drop  < 0.6);
}

PON_TEST(higher_bc_shoots_flatter) {
    auto shoot = [](double bc) {
        Sim sim;
        ProjectileType t;
        t.id = "x"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = bc;
        t.mass_kg = 0.0113; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
        return fly_to(sim, sim.spawn(id, lp), 800.0);
    };
    const DownrangeSample lo = shoot(0.20), hi = shoot(0.45);
    CHECK(hi.speed > lo.speed);   // retains more velocity
    CHECK(hi.drop  < lo.drop);    // and drops less over the same distance
    CHECK(hi.tof   < lo.tof);
}

PON_TEST(custom_curve_equals_constant_cd) {
    // A flat custom Cd(Mach) curve must reproduce the ConstantCd path exactly.
    auto shoot = [](DragModel model, const std::vector<DragCurvePoint>& curve) {
        Sim sim;
        ProjectileType t;
        t.id = "c"; t.klass = ProjectileClass::Bullet;
        t.dragModel = model;
        t.dragCoefficient = 0.3;
        t.customDragCurve = curve;
        t.mass_kg = 0.01; t.refDiameter_m = 0.008;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0.05, 0};
        lp.speed = 500.0; lp.tier = FidelityTier::Integrated;
        return fly_to(sim, sim.spawn(id, lp), 600.0);
    };
    const DownrangeSample a = shoot(DragModel::ConstantCd, {});
    const DownrangeSample b = shoot(DragModel::CustomCurve,
                                    {{0.0, 0.3}, {2.0, 0.3}, {5.0, 0.3}});
    CHECK_NEAR(a.speed, b.speed, 0.2);
    CHECK_NEAR(a.drop,  b.drop,  0.02);
}

PON_TEST(isa_atmosphere_reference_points) {
    const AtmoState sea = isa_atmosphere({});
    CHECK_NEAR(sea.density_kgm3, 1.225, 0.005);
    CHECK_NEAR(sea.speedOfSound_mps, 340.29, 1.0);

    IsaConditions high; high.altitude_m = 5000.0;
    const AtmoState a5 = isa_atmosphere(high);
    CHECK_NEAR(a5.density_kgm3, 0.7364, 0.02);   // ISA 5 km
    CHECK(a5.speedOfSound_mps < sea.speedOfSound_mps - 10.0);

    // Hot day at the coast: thinner air than the 15 C standard.
    IsaConditions hot; hot.temperature_K = 308.15; // 35 C
    CHECK(isa_atmosphere(hot).density_kgm3 < sea.density_kgm3);

    // Humidity displaces heavier dry air -> slightly lower density.
    IsaConditions humid; humid.temperature_K = 303.15; humid.relativeHumidity = 1.0;
    IsaConditions dry;   dry.temperature_K   = 303.15; dry.relativeHumidity   = 0.0;
    CHECK(isa_atmosphere(humid).density_kgm3 < isa_atmosphere(dry).density_kgm3);
}

PON_TEST(altitude_flattens_the_trajectory) {
    auto shoot = [](double altitude_m) {
        Environment env;
        IsaConditions c; c.altitude_m = altitude_m;
        env.setAtmosphere(c);
        Sim sim(env);
        ProjectileType t;
        t.id = "x"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.24;
        t.mass_kg = 0.0113; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 5000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
        return fly_to(sim, sim.spawn(id, lp), 800.0);
    };
    const DownrangeSample low  = shoot(0.0);
    const DownrangeSample high = shoot(2500.0);
    CHECK(high.speed > low.speed); // thinner air -> less drag
    CHECK(high.drop  < low.drop);
}

// --- Item 4: spin, Magnus & the BallProfile system ------------------------

namespace {
struct BallShot { Vec3 pos, vel; double tof; };

// Free-flight until the ball falls below `floorY` (or the step budget runs out).
BallShot fly_ball(Sim& sim, StateId h, double floorY,
                  double dt = 1.0 / 500.0, int maxSteps = 40000) {
    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < maxSteps && sim.state(h).alive &&
                    sim.state(h).position.y > floorY; ++i)
        sim.step(dt, world, sink);
    const ProjectileState& s = sim.state(h);
    return {s.position, s.velocity, s.timeAlive_s};
}

TypeId reg_ball(Sim& sim, const char* profile,
                SpinAxisMode mode = SpinAxisMode::Fixed) {
    ProjectileType t;
    t.id = profile;
    t.klass = ProjectileClass::SportsBall;
    t.dragModel = DragModel::BallProfile;
    t.ballProfile = profile;
    t.spinAxisMode = mode;
    return sim.registerType(t);
}
} // namespace

PON_TEST(ball_profile_fills_dimensions) {
    Sim sim;
    const TypeId id = reg_ball(sim, "golf_ball");
    CHECK(id != kInvalidType);
    const ProjectileType& t = sim.type(id);
    CHECK_NEAR(t.refDiameter_m, 0.0427, 1e-4);
    CHECK_NEAR(t.mass_kg, 0.0459, 1e-3);
    CHECK(t.muzzleSpeed_mps && *t.muzzleSpeed_mps > 40.0); // profile typical speed
}

PON_TEST(unknown_ball_profile_falls_back_to_sphere) {
    Sim sim;
    const TypeId id = reg_ball(sim, "banana_ball");
    CHECK(id != kInvalidType);
    LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0.3, 0};
    lp.speed = 25.0; lp.tier = FidelityTier::Integrated;
    const BallShot d = fly_ball(sim, sim.spawn(id, lp), -0.5);
    CHECK(d.pos.x > 5.0 && d.pos.x < 80.0); // a plausible thrown-ball range
}

PON_TEST(golf_backspin_extends_carry) {
    // Same launch, with and without backspin. The default sphere spin axis is
    // horizontal and square to the shot, so a positive spin rate is backspin —
    // upward Magnus lift that stretches the carry.
    auto carry = [](double spin_radps) {
        Sim sim;
        const TypeId id = reg_ball(sim, "golf_ball");
        LaunchParams lp; lp.position = {0, 0.0, 0};
        const double c = std::cos(0.20), s = std::sin(0.20); // ~11.5 deg
        lp.direction = {c, s, 0}; lp.speed = 68.0;
        lp.spin = spin_radps; lp.tier = FidelityTier::Integrated;
        return fly_ball(sim, sim.spawn(id, lp), -0.5).pos.x;
    };
    const double dead = carry(0.0);
    const double spun = carry(300.0); // ~2900 rpm, a solid tee shot
    CHECK(dead > 70.0 && dead < 185.0);
    CHECK(spun > dead * 1.2); // lift buys real distance
    CHECK(spun < 320.0);      // but not a moon shot
}

PON_TEST(baseball_curveball_breaks_sideways) {
    // A pitch thrown flat with a vertical spin axis. s_hat x v = y_hat x x_hat
    // = -z_hat, so it must break toward -z, by tens of centimetres over the
    // ~18 m to the plate.
    Sim sim;
    const TypeId id = reg_ball(sim, "baseball");
    LaunchParams lp; lp.position = {0, 1.8, 0}; lp.direction = {1, 0, 0};
    lp.speed = 38.0; lp.spin = 190.0;          // ~1800 rpm
    lp.spinAxis = Vec3{0, 1, 0};
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld world; VectorEventSink sink;
    while (sim.state(h).position.x < 18.0 && sim.state(h).alive)
        sim.step(1.0 / 2000.0, world, sink);
    const double z = sim.state(h).position.z;
    CHECK(z < -0.05);
    CHECK(z > -1.5);
}

PON_TEST(soccer_free_kick_bends_and_low_spin_reverses) {
    // fifa_football profile carries a signed Cl(S) curve: a brief negative
    // (reverse-Magnus) regime at very low spin parameter, then a strong
    // positive bend. Fire two kicks with the same vertical spin axis, one
    // heavily spun, one barely — they must curve to opposite sides.
    auto sideways = [](double spin_radps) {
        Sim sim;
        const TypeId id = reg_ball(sim, "fifa_football");
        LaunchParams lp; lp.position = {0, 0.3, 0};
        const double c = std::cos(0.12), s = std::sin(0.12);
        lp.direction = {c, s, 0}; lp.speed = 30.0;
        lp.spin = spin_radps; lp.spinAxis = Vec3{0, 1, 0};
        lp.tier = FidelityTier::Integrated;
        return fly_ball(sim, sim.spawn(id, lp), -0.5).pos.z;
    };
    const double hard = sideways(70.0); // S ~ 0.26 -> strong positive Cl -> -z
    const double soft = sideways(3.0);  // S ~ 0.011 -> slight negative Cl -> +z
    CHECK(hard < -0.15);
    CHECK(soft > 0.0);
}

PON_TEST(spiral_punt_outflies_end_over_end) {
    // ProlateSpheroid: a clean spiral (spin axis along the launch velocity)
    // sees the low spiral Cd; an end-over-end kick (axis square to velocity)
    // sees the high tumble Cd and falls well short.
    const double c = std::cos(0.70), s = std::sin(0.70); // ~40 deg
    auto range = [&](SpinAxisMode mode, Vec3 axis) {
        Sim sim;
        const TypeId id = reg_ball(sim, "american_football", mode);
        LaunchParams lp; lp.position = {0, 0.0, 0};
        lp.direction = {c, s, 0}; lp.speed = 26.0;
        lp.spin = 90.0;
        if (axis.x || axis.y || axis.z) lp.spinAxis = axis;
        lp.tier = FidelityTier::Integrated;
        return fly_ball(sim, sim.spawn(id, lp), -0.5).pos.x;
    };
    // Spiral: AlongVelocity, nose tracks the trajectory. Tumble: a fixed axis
    // square to the launch velocity (end-over-end in the plane of flight).
    const double spiral = range(SpinAxisMode::AlongVelocity, Vec3{0, 0, 0});
    const double tumble = range(SpinAxisMode::Fixed, Vec3{-s, c, 0});
    CHECK(spiral > tumble * 1.2);
}

PON_TEST(rifle_spin_produces_no_magnus) {
    // A spin-stabilised bullet: spinAxisMode AlongVelocity. Magnus is off (the
    // gyroscopic spin drift of item 6 is a separate, smaller effect); a flat
    // shot must stay in its firing plane.
    Sim sim;
    ProjectileType t;
    t.id = "spun_bullet"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.24;
    t.mass_kg = 0.0113; t.refDiameter_m = 0.00782;
    t.spinRate_radps = 2000.0; // ~19000 rpm
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 1.8, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld world; VectorEventSink sink;
    while (sim.state(h).position.x < 300.0 && sim.state(h).alive)
        sim.step(1.0 / 2000.0, world, sink);
    CHECK(std::fabs(sim.state(h).position.z) < 0.01);
}

PON_TEST(ball_drag_crisis_lowers_the_coefficient) {
    // Above its drag crisis a fifa football has a much lower Cd. Fire it slow
    // (subcritical) and fast (supercritical), no spin, and compare the drag
    // deceleration normalised by v^2 over the first slice of flight — the
    // effective Cd must drop.
    auto k_over_v2 = [](double v0) {
        Sim sim;
        const TypeId id = reg_ball(sim, "fifa_football",
                                   SpinAxisMode::AlongVelocity); // no Magnus
        LaunchParams lp; lp.position = {0, 100, 0}; lp.direction = {1, 0, 0};
        lp.speed = v0; lp.tier = FidelityTier::Integrated;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld world; VectorEventSink sink;
        const double vx0 = sim.state(h).velocity.x;
        for (int i = 0; i < 20; ++i) sim.step(1.0 / 2000.0, world, sink); // 10 ms
        const ProjectileState& s = sim.state(h);
        const double dvx = vx0 - s.velocity.x;       // lost to drag (gravity is +y)
        const double vmean = 0.5 * (vx0 + s.velocity.x);
        return (dvx / (20.0 / 2000.0)) / (vmean * vmean); // ~ 0.5 rho Cd A / m
    };
    const double slow = k_over_v2(8.0);   // Re ~ 1.2e5, subcritical
    const double fast = k_over_v2(35.0);  // Re ~ 5e5, past the crisis
    CHECK(fast < slow * 0.8);
}

// --- Item 5: wind field + medium registry ----------------------------------

namespace {
// A plain drag round used by the wind / medium tests.
TypeId reg_slug(Sim& sim) {
    ProjectileType t;
    t.id = "slug"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.010; t.refDiameter_m = 0.0095;
    return sim.registerType(t);
}
double range_to_ground(Sim& sim, StateId h) {
    EmptyWorld world; VectorEventSink sink;
    // Step once so a shot launched from y == 0 is airborne before the y-test.
    for (int i = 0; i < 20000 && sim.state(h).alive &&
                    (i == 0 || sim.state(h).position.y >= 0.0); ++i)
        sim.step(1.0 / 1000.0, world, sink);
    return sim.state(h).position.x;
}

// A World that hits nothing and reports one fixed medium everywhere — lets a
// test hold a projectile in water without an EmptyWorld snapping it back to air.
struct FixedMediumWorld final : World {
    MediumId medium;
    explicit FixedMediumWorld(MediumId m) : medium(m) {}
    bool raycast(Vec3, Vec3, HitResult&) const override { return false; }
    MediumId mediumAt(Vec3) const override { return medium; }
    const Material& material(SurfaceId) const override {
        static const Material air = material_air();
        return air;
    }
};
} // namespace

PON_TEST(crosswind_pushes_the_shot_downrange_sideways) {
    // A steady 10 m/s wind along +z must deflect a shot fired along +x toward
    // +z; still air leaves it in plane.
    auto lateral = [](Vec3 w) {
        Sim sim; const TypeId id = reg_slug(sim);
        sim.environment().wind = [w](Vec3, Seconds) { return w; };
        LaunchParams lp; lp.position = {0, 1.8, 0}; lp.direction = {1, 0, 0};
        lp.speed = 300.0; lp.tier = FidelityTier::Integrated;
        const StateId h = sim.spawn(id, lp);
        range_to_ground(sim, h);
        return sim.state(h).position.z;
    };
    CHECK(std::fabs(lateral({0, 0, 0})) < 1e-3);
    const double z = lateral({0, 0, 10.0});
    CHECK(z > 0.3);          // pushed with the wind
    CHECK(z < 20.0);         // but it is a bullet, not a leaf
}

PON_TEST(head_and_tailwind_change_range) {
    auto shoot = [](double wx) {
        Sim sim; const TypeId id = reg_slug(sim);
        sim.environment().wind = [wx](Vec3, Seconds) { return Vec3{wx, 0, 0}; };
        const double s = std::sqrt(0.5);
        LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {s, s, 0};
        lp.speed = 120.0; lp.tier = FidelityTier::Integrated;
        return range_to_ground(sim, sim.spawn(id, lp));
    };
    const double still = shoot(0.0);
    CHECK(shoot(-15.0) < still - 1.0); // headwind: more drag, shorter
    CHECK(shoot(+15.0) > still + 1.0); // tailwind: less relative airspeed, longer
}

PON_TEST(updraft_extends_time_of_flight) {
    Sim sim; const TypeId id = reg_slug(sim);
    sim.environment().wind = [](Vec3, Seconds) { return Vec3{0, 6.0, 0}; };
    const double s = std::sqrt(0.5);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {s, s, 0};
    lp.speed = 120.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    range_to_ground(sim, h);
    CHECK(sim.state(h).timeAlive_s > 0.0);

    Sim ref; const TypeId rid = reg_slug(ref);
    const StateId rh = ref.spawn(rid, lp);
    range_to_ground(ref, rh);
    CHECK(sim.state(h).timeAlive_s > ref.state(rh).timeAlive_s + 0.1);
}

PON_TEST(water_medium_bleeds_a_rifle_round_over_about_a_metre) {
    // Submerged shot (a FixedMediumWorld keeps it in water). Water is ~815x the
    // density of air, so the round sheds most of its 800 m/s inside the first
    // ~2 m — the real "sub-wounding past ~1 m underwater" behaviour.
    Sim sim; const TypeId id = reg_slug(sim);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    lp.medium = kMediumWater;
    const StateId h = sim.spawn(id, lp);
    FixedMediumWorld water(kMediumWater); VectorEventSink sink;
    double vAt2 = -1.0;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i) {
        sim.step(1.0 / 4000.0, water, sink);
        if (vAt2 < 0.0 && sim.state(h).position.x >= 2.0)
            vAt2 = length(sim.state(h).velocity);
    }
    CHECK(sim.state(h).mediumId == kMediumWater);
    CHECK(vAt2 > 0.0 && vAt2 < 150.0);   // shed >80% of 800 m/s inside 2 m

    // The same shot in air still has almost all of its speed at 2 m.
    Sim air; const TypeId aid = reg_slug(air);
    LaunchParams la = lp; la.medium.reset();
    const StateId ah = air.spawn(aid, la);
    EmptyWorld world;
    for (int i = 0; i < 200 && air.state(ah).position.x < 2.0; ++i)
        air.step(1.0 / 4000.0, world, sink);
    CHECK(length(air.state(ah).velocity) > 700.0);
}

PON_TEST(custom_medium_registers_and_drives_drag) {
    Sim sim; const TypeId id = reg_slug(sim);
    MediumDesc mud;
    mud.name = "mud"; mud.density_kgm3 = 1600.0; mud.dragScale = 3.0;
    mud.buoyancy = 0.2;
    const MediumId kMud = sim.environment().media.add(mud);
    CHECK(kMud >= 2);
    CHECK(sim.environment().media.get(kMud).name == "mud");

    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 400.0; lp.tier = FidelityTier::Integrated; lp.medium = kMud;
    const StateId h = sim.spawn(id, lp);
    FixedMediumWorld thick(kMud); VectorEventSink sink;
    for (int i = 0; i < 4000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 4000.0, thick, sink);
    CHECK(sim.state(h).mediumId == kMud);
    CHECK(sim.state(h).position.x < 2.0);        // thicker than water: crawls
    CHECK(length(sim.state(h).velocity) < 20.0);
}

PON_TEST(c_abi_wind_and_medium) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    CHECK(sim != nullptr);

    pon_projectile_desc d{};
    d.id = "cabi_slug"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.010; d.ref_diameter_m = 0.0095; d.drag_coefficient = 0.30;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);

    // Wind callback via the C function pointer + user data.
    static int calls = 0; calls = 0;
    struct W { static pon_vec3 fn(pon_vec3, double, void* u) {
        ++*static_cast<int*>(u); return pon_vec3{0, 0, 12.0};
    } };
    pon_sim_set_wind(sim, &W::fn, &calls);

    const uint32_t sid = pon_spawn(sim, tid, {0, 1.8, 0}, {1, 0, 0}, 300.0);
    for (int i = 0; i < 400; ++i) pon_step(sim, 1.0 / 1000.0);
    pon_state st{};
    CHECK(pon_get_state(sim, sid, &st) == PON_OK);
    CHECK(calls > 0);          // the wind fn was actually invoked
    CHECK(st.position.z > 0.1); // and it deflected the shot

    // Custom medium registers and a submerged spawn reports it. (Stepping it
    // through the built-in EmptyWorld would snap it back to air — the World
    // callback reaches the C ABI in item 6.)
    pon_medium_desc md{};
    md.name = "syrup"; md.density_kgm3 = 1400.0; md.drag_scale = 2.0;
    md.buoyancy = 0.3;
    const int32_t mid = pon_sim_add_medium(sim, &md);
    CHECK(mid >= 2);
    const uint32_t sid2 = pon_spawn_in_medium(sim, tid, {0, 0, 0}, {1, 0, 0},
                                              500.0, mid);
    CHECK(pon_get_state(sim, sid2, &st) == PON_OK);
    CHECK(st.medium_id == mid);

    pon_sim_set_wind(sim, nullptr, nullptr); // clears back to still air
    pon_sim_destroy(sim);
}

// --- Item 6: precision effects (§3.7) --------------------------------------

namespace {
// .308 Win / 175 gr SMK, G7 BC 0.243 — the long-range reference round.
TypeId reg_match308(Sim& sim) {
    ProjectileType t;
    t.id = "762x51_175smk"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    return sim.registerType(t);
}
// Fire from a high start along +x to `x_target`; return the full end state.
ProjectileState fly_xy(Sim& sim, StateId h, double x_target) {
    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 400000 && sim.state(h).alive &&
                    sim.state(h).position.x < x_target; ++i)
        sim.step(1.0 / 1000.0, world, sink);
    return sim.state(h);
}
} // namespace

PON_TEST(spin_drift_goes_right_for_right_hand_twist) {
    auto drift_z = [](PrecisionFlag pf, double twist) {
        Sim sim; const TypeId id = reg_match308(sim);
        // twist only sets the sign; register a second type carrying it.
        ProjectileType t = sim.type(id); t.twistRate_m = twist;
        const TypeId id2 = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 792.0; lp.tier = FidelityTier::Integrated; lp.precision = pf;
        return fly_xy(sim, sim.spawn(id2, lp), 1000.0).position.z;
    };
    // Right of a +x heading (world up +y) is -z, so a right-hand twist drifts -z.
    const double none  = drift_z(PrecisionFlag::None, 0.0);
    const double right = drift_z(PrecisionFlag::SpinDrift, +0.254);
    const double left  = drift_z(PrecisionFlag::SpinDrift, -0.254);
    CHECK(std::fabs(none) < 0.02);
    CHECK(right < -0.05 && right > -0.45);   // ~0.2 m at 1000 m (Litz)
    CHECK(left  >  0.05 && left  <  0.45);
    CHECK_NEAR(right, -left, 0.03);
}

PON_TEST(coriolis_flips_with_hemisphere_and_lifts_an_eastward_shot) {
    auto shoot = [](double lat) {
        Sim sim; const TypeId id = reg_match308(sim);
        sim.environment().setCoriolis(lat, 3.14159265358979 / 2.0); // fire east
        const double c = std::sqrt(0.5);
        LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {c, c, 0};
        lp.speed = 300.0; lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::Coriolis;
        EmptyWorld world; VectorEventSink sink;
        const StateId h = sim.spawn(id, lp);
        for (int i = 0; i < 200000 && sim.state(h).alive &&
                        (i == 0 || sim.state(h).position.y >= 0.0); ++i)
            sim.step(1.0 / 500.0, world, sink);
        return sim.state(h).position;
    };
    const Vec3 north = shoot(+0.785), south = shoot(-0.785);
    CHECK(north.z * south.z < 0.0);          // horizontal deflection flips sign
    CHECK(std::fabs(north.z) > 1.0);

    // Eötvös: an eastward shot at either latitude flies marginally farther than
    // the same shot with Coriolis off (the vertical term fights gravity).
    Sim ref; const TypeId rid = reg_match308(ref);
    const double c = std::sqrt(0.5);
    LaunchParams lp; lp.direction = {c, c, 0}; lp.speed = 300.0;
    lp.tier = FidelityTier::Integrated;
    EmptyWorld world; VectorEventSink sink;
    const StateId rh = ref.spawn(rid, lp);
    for (int i = 0; i < 200000 && ref.state(rh).alive &&
                    (i == 0 || ref.state(rh).position.y >= 0.0); ++i)
        ref.step(1.0 / 500.0, world, sink);
    CHECK(north.x > ref.state(rh).position.x);
}

PON_TEST(aero_jump_kicks_vertically_with_the_crosswind) {
    auto impact_y = [](bool jump, Vec3 wind) {
        Sim sim; const TypeId id = reg_match308(sim);
        sim.environment().wind = [wind](Vec3, Seconds) { return wind; };
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
        lp.precision = jump ? PrecisionFlag::AeroJump : PrecisionFlag::None;
        return fly_xy(sim, sim.spawn(id, lp), 600.0).position.y;
    };
    // Wind toward -z (the shooter's right): a right-twist round jumps low.
    CHECK(impact_y(true,  {0, 0, -4.0}) < impact_y(false, {0, 0, -4.0}) - 0.01);
    // Wind toward +z (from the right): jumps high.
    CHECK(impact_y(true,  {0, 0,  4.0}) > impact_y(false, {0, 0,  4.0}) + 0.01);
}

PON_TEST(rkf45_tracks_the_fixed_step_integrator) {
    auto shoot = [](PrecisionFlag pf) {
        Sim sim; const TypeId id = reg_match308(sim);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 850.0; lp.tier = FidelityTier::Integrated; lp.precision = pf;
        return fly_xy(sim, sim.spawn(id, lp), 900.0).position;
    };
    const Vec3 fixed = shoot(PrecisionFlag::None);
    const Vec3 rkf45 = shoot(PrecisionFlag::AdaptiveRKF45);
    CHECK(std::fabs(fixed.x - rkf45.x) < 0.5);
    CHECK(std::fabs(fixed.y - rkf45.y) < 0.15); // both converge on the same drop
    CHECK(3000.0 - rkf45.y > 5.0 && 3000.0 - rkf45.y < 20.0);
}

PON_TEST(transonic_window_event_fires_once) {
    Sim sim; const TypeId id = reg_match308(sim);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 850.0; lp.tier = FidelityTier::Integrated;
    lp.precision = PrecisionFlag::TransonicFlag;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 200000 && sim.state(h).alive &&
                    length(sim.state(h).velocity) > 250.0; ++i)
        sim.step(1.0 / 1000.0, world, sink);

    int transonic = 0;
    for (const Event& e : sink.events)
        if (e.type == EventType::TransonicWindow) {
            ++transonic;
            CHECK(e.residualSpeed_mps > 250.0 && e.residualSpeed_mps < 420.0);
        }
    CHECK(transonic == 1);
    CHECK((sim.state(h).flags & kFlagPastTransonic) != 0);
}

PON_TEST(mv_from_powder_temp_is_linear) {
    CHECK_NEAR(mv_from_powder_temp(800.0, 288.15, 0.5, 298.15), 805.0, 1e-9);
    CHECK_NEAR(mv_from_powder_temp(800.0, 288.15, 0.5, 278.15), 795.0, 1e-9);
    CHECK_NEAR(pon_mv_from_powder_temp(800.0, 288.15, 0.5, 288.15), 800.0, 1e-9);
}

PON_TEST(c_abi_precision) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_sim_set_coriolis(sim, 0.9, 0.0);

    pon_projectile_desc d{};
    d.id = "precise_308"; d.klass = 0; d.drag_model = PON_DRAG_G7;
    d.mass_kg = 0.01134; d.ref_diameter_m = 0.00782;
    d.ballistic_coefficient = 0.243; d.muzzle_speed_mps = 850.0;
    d.twist_rate_m = 0.254; d.miller_stability = 1.9;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);

    const uint32_t sid = pon_spawn_precise(
        sim, tid, {0, 3000, 0}, {1, 0, 0}, 0.0, /*Integrated*/ 2,
        PON_PRECISION_SPIN_DRIFT | PON_PRECISION_CORIOLIS |
        PON_PRECISION_TRANSONIC_FLAG);
    CHECK(sid != 0xFFFFFFFFu);

    pon_state st{};
    for (int i = 0; i < 4000 && (pon_get_state(sim, sid, &st), st.alive) &&
                    st.position.x < 900.0; ++i)
        pon_step(sim, 1.0 / 1000.0);
    pon_get_state(sim, sid, &st);
    CHECK(st.position.z < 0.0);                     // right-hand twist drift
    CHECK((st.flags & (PON_FLAG_IN_TRANSONIC | PON_FLAG_PAST_TRANSONIC)) != 0);
    pon_sim_destroy(sim);
}

// --- Phase 19 item 1: 6-DOF rigid-body flight (PrecisionFlag::SixDOF) -------

namespace {
// Fly a 6-DOF shot to x_target (or death); return the end state. Records the
// max angle of attack and the angle of attack near a given downrange distance.
struct SixDofRun {
    ProjectileState end;
    double maxAlpha = 0.0;
    double alphaEarly = 0.0;   // at ~x=60 m
    double alphaLate  = 0.0;   // at ~x=x_target-40 m
    double p0 = 0.0, pEnd = 0.0;
};
SixDofRun fly_6dof(Sim& sim, StateId h, double x_target) {
    EmptyWorld world; VectorEventSink sink;
    SixDofRun r;
    r.p0 = sim.state(h).angVel_radps.x;
    bool gotEarly = false;
    for (int i = 0; i < 120000 && sim.state(h).alive &&
                    sim.state(h).position.x < x_target; ++i) {
        sim.step(1.0 / 2000.0, world, sink);
        const ProjectileState& s = sim.state(h);
        r.maxAlpha = std::max(r.maxAlpha, s.angleOfAttack_rad);
        if (!gotEarly && s.position.x > 60.0) { r.alphaEarly = s.angleOfAttack_rad; gotEarly = true; }
        if (s.position.x > x_target - 40.0)   r.alphaLate = s.angleOfAttack_rad;
    }
    r.end  = sim.state(h);
    r.pEnd = r.end.angVel_radps.x;
    return r;
}
// A .308-class spin rate for a 1:10" twist at ~790 m/s: 2*pi*v / twist.
constexpr double kSpin308 = 2.0 * 3.14159265358979 * 790.0 / 0.254; // ~19.5 krad/s
} // namespace

PON_TEST(sixdof_spin_stabilised_bullet_holds_its_nose) {
    Sim sim; ProjectileType t = sim.type(reg_match308(sim));
    t.twistRate_m = 0.254;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
    lp.precision = PrecisionFlag::SixDOF;
    lp.spin = kSpin308;
    lp.initialYaw = 0.035; // ~2 deg muzzle tip-off
    const SixDofRun r = fly_6dof(sim, sim.spawn(id, lp), 700.0);
    CHECK(r.end.alive);
    CHECK(r.maxAlpha < 0.5);                        // stays point-first, never tumbles
    CHECK((r.end.flags & kFlagTumbling) == 0);
    CHECK(r.alphaLate < r.alphaEarly + 0.05);       // coning damps, does not grow
}

PON_TEST(sixdof_unspun_bullet_tumbles) {
    Sim sim; const TypeId id = reg_match308(sim);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
    lp.precision = PrecisionFlag::SixDOF;
    lp.spin = 0.0;
    lp.initialYaw = 0.035;
    const SixDofRun r = fly_6dof(sim, sim.spawn(id, lp), 400.0);
    CHECK(r.maxAlpha > 1.05);                       // > 60 deg — end over end
    CHECK((r.end.flags & kFlagTumbling) != 0);
}

PON_TEST(sixdof_spin_drift_sign_and_order_of_magnitude) {
    auto drift_z = [](double twist) {
        Sim sim; ProjectileType t = sim.type(reg_match308(sim));
        t.twistRate_m = twist;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::SixDOF;
        lp.spin = kSpin308;
        return fly_6dof(sim, sim.spawn(id, lp), 1000.0).end.position.z;
    };
    const double right = drift_z(+0.254);  // right-hand twist
    const double left  = drift_z(-0.254);
    CHECK(right < -0.02 && right > -3.0);   // yaw of repose ⇒ drift right (-z)
    CHECK(left  >  0.02 && left  <  3.0);
    CHECK_NEAR(right, -left, 0.20);         // symmetric in twist sign
}

PON_TEST(sixdof_spin_decays_over_the_flight) {
    Sim sim; ProjectileType t = sim.type(reg_match308(sim));
    t.twistRate_m = 0.254;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
    lp.precision = PrecisionFlag::SixDOF; lp.spin = kSpin308;
    const SixDofRun r = fly_6dof(sim, sim.spawn(id, lp), 900.0);
    CHECK(r.p0 > 1.0e4);
    CHECK(r.pEnd < r.p0 && r.pEnd > r.p0 * 0.5); // bleeds, but only a fraction
}

PON_TEST(sixdof_determinism_same_platform_bit_repeatable) {
    auto run_one = [] {
        Sim sim; ProjectileType t = sim.type(reg_match308(sim));
        t.twistRate_m = 0.254;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0.02, 0.01};
        lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::SixDOF; lp.spin = kSpin308;
        lp.initialYaw = 0.03;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld w; VectorEventSink sink;
        for (int i = 0; i < 2000; ++i) sim.step(1.0 / 1000.0, w, sink);
        return sim.state(h);
    };
    const ProjectileState a = run_one(), b = run_one();
    CHECK(a.position.x == b.position.x && a.position.y == b.position.y &&
          a.position.z == b.position.z);
    CHECK(a.angVel_radps.x == b.angVel_radps.x);
    CHECK(a.orientation.w == b.orientation.w && a.orientation.x == b.orientation.x);
}

PON_TEST(sixdof_c_abi_reports_orientation) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_aero_angular aero{}; aero.overturning_moment_slope = 2.5;
    pon_projectile_desc d{};
    d.id = "6dof_308"; d.klass = 0; d.drag_model = PON_DRAG_G7;
    d.mass_kg = 0.01134; d.ref_diameter_m = 0.00782;
    d.ballistic_coefficient = 0.243; d.muzzle_speed_mps = 790.0;
    d.twist_rate_m = 0.254; d.aero = &aero;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);
    const uint32_t sid = pon_spawn_precise(sim, tid, {0, 3000, 0}, {1, 0, 0},
                                           0.0, 2, PON_PRECISION_SIX_DOF);
    CHECK(sid != 0xFFFFFFFFu);
    pon_state st{};
    for (int i = 0; i < 3000 && (pon_get_state(sim, sid, &st), st.alive) &&
                    st.position.x < 500.0; ++i)
        pon_step(sim, 1.0 / 2000.0);
    pon_get_state(sim, sid, &st);
    const double* q = st.orientation_quat;
    const double qn = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    CHECK_NEAR(qn, 1.0, 1e-6);                    // stays a unit quaternion
    CHECK(std::isfinite(st.angle_of_attack_rad));
    CHECK(std::isfinite(st.ang_vel_radps.x));
    pon_sim_destroy(sim);
}

PON_TEST(orientation_helpers_are_identity_for_non_sixdof) {
    // Every other fidelity tier leaves `orientation` at identity — the
    // helpers must return the unrotated body axes / 0, not garbage, so a
    // renderer doesn't have to special-case the tier.
    ProjectileState s;
    CHECK_NEAR(nose_direction(s).x, 1.0, 1e-12);
    CHECK_NEAR(length(nose_direction(s) - Vec3{1, 0, 0}), 0.0, 1e-12);
    CHECK_NEAR(length(up_direction(s) - Vec3{0, 1, 0}), 0.0, 1e-12);
    CHECK_NEAR(spin_phase(s), 0.0, 1e-12);
}

PON_TEST(orientation_helpers_track_a_live_sixdof_shot) {
    Sim sim; ProjectileType t = sim.type(reg_match308(sim));
    t.twistRate_m = 0.254;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
    lp.precision = PrecisionFlag::SixDOF; lp.spin = kSpin308;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 400; ++i) sim.step(1.0 / 2000.0, world, sink);

    const ProjectileState& s = sim.state(h);
    CHECK_NEAR(length(nose_direction(s)), 1.0, 1e-6);   // stays unit
    CHECK_NEAR(length(up_direction(s)), 1.0, 1e-6);
    CHECK(std::fabs(dot(nose_direction(s), up_direction(s))) < 0.05); // ~orthogonal
    // A spin-stabilised round still flies point-first at 790 m/s: nose tracks
    // the velocity direction closely.
    CHECK(dot(nose_direction(s), normalized(s.velocity)) > 0.99);
    // Barrel-twist spin (kSpin308 rad/s) over this flight time accumulates a
    // large, non-zero roll phase — same field as ProjectileState::spinPhase_rad.
    CHECK(spin_phase(s) == s.spinPhase_rad);
    CHECK(std::fabs(spin_phase(s)) > 1.0);
}

PON_TEST(c_abi_orientation_helpers) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    pon_aero_angular aero{}; aero.overturning_moment_slope = 2.5;
    pon_projectile_desc d{};
    d.id = "6dof_308"; d.klass = 0; d.drag_model = PON_DRAG_G7;
    d.mass_kg = 0.01134; d.ref_diameter_m = 0.00782;
    d.ballistic_coefficient = 0.243; d.muzzle_speed_mps = 790.0;
    d.twist_rate_m = 0.254; d.aero = &aero;
    d.spin_rate_radps = 2.0 * 3.14159265358979 * 790.0 / 0.254; // barrel-twist spin
    const uint32_t tid = pon_register_type(sim, &d);
    const uint32_t sid = pon_spawn_precise(sim, tid, {0, 3000, 0}, {1, 0, 0},
                                           0.0, 2, PON_PRECISION_SIX_DOF);
    for (int i = 0; i < 400; ++i) pon_step(sim, 1.0 / 2000.0);

    const pon_vec3 nose = pon_shot_nose(sim, sid);
    const pon_vec3 up   = pon_shot_up(sim, sid);
    const double len2 = nose.x*nose.x + nose.y*nose.y + nose.z*nose.z;
    CHECK_NEAR(len2, 1.0, 1e-6);
    const double dotp = nose.x*up.x + nose.y*up.y + nose.z*up.z;
    CHECK(std::fabs(dotp) < 0.05);
    CHECK(std::fabs(pon_shot_spin_phase(sim, sid)) > 1.0);

    // Out-of-range id -> identity axes / 0, not a crash.
    const pon_vec3 badNose = pon_shot_nose(sim, 9999);
    CHECK_NEAR(badNose.x, 1.0, 1e-12);
    CHECK_NEAR(pon_shot_spin_phase(sim, 9999), 0.0, 1e-12);

    pon_sim_destroy(sim);
}

PON_TEST(validation_sixdof_stability_follows_the_spin_rate) {
    // The classic gyroscopic result: a round with barrel-twist spin flies
    // point-first; the same round fired without spin tumbles. Both from the
    // same 2 deg muzzle yaw.
    auto maxYaw = [](double spin) {
        Sim sim; ProjectileType t = sim.type(reg_match308(sim));
        t.twistRate_m = 0.254;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 2000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 790.0; lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::SixDOF; lp.spin = spin;
        lp.initialYaw = 0.035;
        return fly_6dof(sim, sim.spawn(id, lp), 300.0).maxAlpha;
    };
    CHECK(maxYaw(kSpin308) < 0.35);
    CHECK(maxYaw(0.0)      > 1.05);
}

PON_TEST(validation_sixdof_fuzz_random_angular_shots_stay_finite) {
    // Local LCG — the shared Rng helper is defined later in the file.
    std::uint64_t st = 0x6D0F6D0Full;
    auto uni = [&](double lo, double hi) {
        st = st * 6364136223846793005ull + 1442695040888963407ull;
        return lo + (hi - lo) * ((st >> 11) * (1.0 / 9007199254740992.0));
    };
    const ProjectileClass classes[] = {ProjectileClass::Bullet, ProjectileClass::Arrow,
                                       ProjectileClass::Bolt, ProjectileClass::Spear};
    int bad = 0, exercised = 0;
    for (int k = 0; k < 800; ++k) {
        Sim sim;
        ProjectileType t;
        t.id = "6dof_fuzz";
        t.klass = classes[static_cast<int>(uni(0.0, 3.999))];
        t.dragModel = DragModel::ConstantCd;
        t.dragCoefficient = uni(0.1, 0.8);
        t.mass_kg = uni(0.01, 2.0);
        t.refDiameter_m = uni(0.005, 0.05);
        t.twistRate_m = uni(0.0, 1.0) < 0.5 ? 0.3 : -0.3;
        t.maxLifetime_s = 6.0;
        const TypeId id = sim.registerType(t);
        if (id == kInvalidType) continue;
        LaunchParams lp;
        lp.position = {0.0, uni(0.0, 3.0), 0.0};
        lp.direction = {1.0, uni(-0.2, 0.5), uni(-0.2, 0.2)};
        lp.speed = uni(30.0, 1000.0);
        lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::SixDOF;
        lp.spin = uni(0.0, 25000.0);
        lp.initialYaw = uni(0.0, 0.3);
        const StateId h = sim.spawn(id, lp);
        if (h == kInvalidState) continue;
        EmptyWorld world; VectorEventSink sink;
        int steps = 0;
        for (; steps < 40000 && sim.state(h).alive; ++steps)
            sim.step(1.0 / 2000.0, world, sink);
        ++exercised;
        const ProjectileState& s = sim.state(h);
        const Quat& q = s.orientation;
        const double qn = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        const bool ok = std::isfinite(s.position.x) && std::isfinite(s.position.y) &&
                        std::isfinite(s.position.z) && std::isfinite(s.velocity.x) &&
                        std::isfinite(s.angVel_radps.x) && std::isfinite(s.angleOfAttack_rad) &&
                        std::fabs(qn - 1.0) < 1e-3 && steps < 40000;
        if (!ok) ++bad;
    }
    CHECK(exercised > 700);
    CHECK(bad == 0);
}

// --- Item 8: hit detection — swept queries, exact TOI, moving targets --------

namespace {

// A single plane perpendicular to +x at `wallX`, infinite in y/z. `wallVel` is
// reported as the surface velocity so the stepper's swept-vs-swept correction
// runs; the test animates `wallX` between frames itself (as a real engine's
// broad-phase would).
struct WallWorld final : World {
    Real wallX;
    Vec3 wallVel{0, 0, 0};
    bool reportVel = true; // false ⇒ hide the surface velocity (naive sweep)
    explicit WallWorld(Real x, Vec3 v = {0, 0, 0}) : wallX(x), wallVel(v) {}

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        const Real da = a.x - wallX, db = b.x - wallX;
        if ((da > 0.0) == (db > 0.0)) return false; // no sign change ⇒ no crossing
        const Real t = da / (da - db);
        if (t < 0.0 || t > 1.0) return false;
        out.t               = t;
        out.point           = a + (b - a) * t;
        out.normal          = {da > 0.0 ? Real(1) : Real(-1), 0, 0};
        out.surface         = 0;
        out.surfaceVelocity = reportVel ? wallVel : Vec3{0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override {
        // Bulk brittle block: every hit is a clean Stopped at the face, so these
        // tests stay about the swept hit (TOI / point / closing energy), not the
        // terminal pipeline (that has its own tests).
        static const Material block = [] {
            Material m = material_air();
            m.name = "block"; m.behaviour = MaterialBehaviour::Brittle;
            m.density_kgm3 = 2400; m.strength_Pa = 4.0e7; m.thickness_m = -1.0;
            return m;
        }();
        return block;
    }
};

// A plain constant-Cd round for the hit-detection tests.
TypeId reg_hd_round(Sim& sim) {
    ProjectileType t;
    t.id = "hd_round"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.010; t.refDiameter_m = 0.0095;
    return sim.registerType(t);
}

// Run to the wall, return the first Stopped event (type set to Expired on miss).
Event run_to_wall(FidelityTier tier, WallWorld& world, Seconds dt,
                  int maxSteps = 400000) {
    Sim sim; const TypeId id = reg_hd_round(sim);
    LaunchParams lp; lp.position = {0, 100, 0}; lp.direction = {1, 0, 0};
    lp.speed = 300.0; lp.tier = tier;
    const StateId h = sim.spawn(id, lp);
    VectorEventSink sink;
    for (int i = 0; i < maxSteps && sim.state(h).alive; ++i) {
        sim.step(dt, world, sink);
        world.wallX += world.wallVel.x * dt; // animate between frames
    }
    for (const Event& e : sink.events)
        if (e.type == EventType::Stopped) return e;
    Event miss; miss.type = EventType::Expired; return miss;
}
Event run_to_wall(FidelityTier tier, Real x0, Vec3 vel, Seconds dt,
                  bool reportVel = true) {
    WallWorld w(x0, vel);
    w.reportVel = reportVel;
    return run_to_wall(tier, w, dt);
}

} // namespace

PON_TEST(hit_toi_and_point_agree_across_tiers) {
    // Static wall at x = 150. All three tiers must stop at the plane and report
    // the same time of impact.
    auto shot = [](FidelityTier tier) {
        WallWorld w(150.0);
        return run_to_wall(tier, w, 1.0 / 240.0);
    };
    const Event hs = shot(FidelityTier::Hitscan);
    const Event an = shot(FidelityTier::AnalyticDrag);
    const Event in = shot(FidelityTier::Integrated);

    for (const Event& e : {hs, an, in}) {
        CHECK(e.type == EventType::Stopped);
        CHECK_NEAR(e.point.x, 150.0, 1e-3);
    }
    // Hitscan is drag-free so it arrives earliest; the drag tiers agree tightly.
    CHECK(hs.time_s < an.time_s);
    CHECK_NEAR(an.time_s, in.time_s, 5e-3);
    // Sub-step TOI, not a whole-step snap: not a multiple of the 1/240 s step.
    const double frac = in.time_s * 240.0;
    CHECK(std::fabs(frac - std::floor(frac + 0.5)) > 1e-4);
}

PON_TEST(hit_no_flythrough_at_coarse_step) {
    // 1200 m/s into a wall at x = 100 with a 60 Hz step (~20 m of travel per
    // step) — the swept segment query must catch it at the plane, not step past.
    for (FidelityTier tier : {FidelityTier::AnalyticDrag, FidelityTier::Integrated}) {
        Sim sim; const TypeId id = reg_hd_round(sim);
        LaunchParams lp; lp.position = {0, 100, 0}; lp.direction = {1, 0, 0};
        lp.speed = 1200.0; lp.tier = tier;
        const StateId h = sim.spawn(id, lp);
        WallWorld world(100.0); VectorEventSink sink;
        int steps = 0;
        for (; steps < 100 && sim.state(h).alive; ++steps)
            sim.step(1.0 / 60.0, world, sink);
        CHECK(!sim.state(h).alive);
        CHECK(steps < 12);                        // ~100 m / ~1150 m/s ≈ 5 frames
        // Caught at the wall (embeds a few cm), not tunnelled downrange.
        CHECK(sim.state(h).position.x >= 100.0);
        CHECK(sim.state(h).position.x < 100.2);
    }
}

PON_TEST(hit_moving_target_swept_vs_swept) {
    // Wall nominally at x = 150, animated between frames. The swept-vs-swept
    // correction (surfaceVelocity applied over the sub-step) makes a coarse
    // 60 Hz step land where a 600 Hz step does; without it the coarse step
    // over-shoots into the frame the wall has already vacated.
    const Event closeCoarse = run_to_wall(FidelityTier::Integrated, 150.0, {-40, 0, 0}, 1.0 / 60.0);
    const Event closeFiner  = run_to_wall(FidelityTier::Integrated, 150.0, {-40, 0, 0}, 1.0 / 600.0);
    const Event closeNaive  = run_to_wall(FidelityTier::Integrated, 150.0, {-40, 0, 0}, 1.0 / 60.0, /*reportVel=*/false);
    const Event still        = run_to_wall(FidelityTier::Integrated, 150.0, {0, 0, 0},   1.0 / 60.0);
    const Event recede       = run_to_wall(FidelityTier::Integrated, 150.0, {+40, 0, 0}, 1.0 / 60.0);

    CHECK(closeCoarse.type == EventType::Stopped);
    // Corrected coarse step tracks the finer step well inside one frame of wall
    // travel (0.67 m); the residual is the frozen-pose gate lateness (§3.4).
    CHECK_NEAR(closeCoarse.point.x, closeFiner.point.x, 0.35);
    CHECK_NEAR(closeCoarse.time_s,  closeFiner.time_s,  3e-3);
    // and clearly beats the uncorrected coarse step.
    CHECK(std::fabs(closeCoarse.point.x - closeFiner.point.x) <
          std::fabs(closeNaive.point.x - closeFiner.point.x) - 0.1);

    // A closing wall is met sooner and nearer; a receding wall later and further.
    CHECK(closeCoarse.point.x < still.point.x - 5.0);
    CHECK(closeCoarse.time_s  < still.time_s);
    CHECK(recede.point.x      > still.point.x + 5.0);
    CHECK(recede.time_s       > still.time_s);

    // Impact energy is the closing energy: higher into a closing wall, lower
    // into a receding one (same round, ½m·v_rel²).
    CHECK(closeCoarse.energy_J > still.energy_J * 1.15);
    CHECK(recede.energy_J      < still.energy_J * 0.85);
}

PON_TEST(hit_moving_target_applies_to_hitscan) {
    // The swept-vs-swept correction is tier-independent.
    const Event e = run_to_wall(FidelityTier::Hitscan, 150.0, {-40, 0, 0}, 1.0 / 60.0);
    CHECK(e.type == EventType::Stopped);
    CHECK(e.point.x < 145.0); // met before reaching the frozen plane
}

// --- Item 8: material system + terminal ballistics --------------------------

namespace {

Material mk_mat(MaterialBehaviour b, Real rho, Real strength_Pa, Real thick_m) {
    Material m = material_air();
    m.name         = "test";
    m.behaviour    = b;
    m.density_kgm3  = rho;
    m.strength_Pa   = strength_Pa;
    m.thickness_m   = thick_m;   // <= 0 ⇒ bulk (never perforates)
    return m;
}

// A stack of planes perpendicular to +x, each with its own material. Static.
struct SlabWorld final : World {
    struct Slab { Real x; Material mat; };
    std::vector<Slab> slabs;

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        Real best = 1.0; int idx = -1;
        for (std::size_t i = 0; i < slabs.size(); ++i) {
            const Real da = a.x - slabs[i].x, db = b.x - slabs[i].x;
            if ((da > 0.0) == (db > 0.0)) continue;
            const Real t = da / (da - db);
            if (t >= 0.0 && t < best) { best = t; idx = static_cast<int>(i); }
        }
        if (idx < 0) return false;
        out.t = best;
        out.point = a + (b - a) * best;
        out.normal = {a.x < slabs[idx].x ? Real(-1) : Real(1), 0, 0};
        out.surface = static_cast<SurfaceId>(idx);
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId s) const override {
        return slabs[s < slabs.size() ? s : 0].mat;
    }
};

// A ground plane at y=0 with a chosen material (for the ricochet tests).
struct FloorWorld final : World {
    Material mat;
    explicit FloorWorld(Material m) : mat(std::move(m)) {}
    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        if ((a.y > 0.0) == (b.y > 0.0)) return false;
        const Real t = a.y / (a.y - b.y);
        if (t < 0.0 || t > 1.0) return false;
        out.t = t;
        out.point = a + (b - a) * t;
        out.normal = {0, 1, 0};
        out.surface = 0;
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override { return mat; }
};

TypeId reg_arrow(Sim& sim) {
    ProjectileType t;
    t.id = "war_arrow"; t.klass = ProjectileClass::Arrow; // nose 0.35 defaulted
    return sim.registerType(t);
}
TypeId reg_rifle(Sim& sim) {
    ProjectileType t;
    t.id = "308"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    return sim.registerType(t);
}
TypeId reg_pistol(Sim& sim) {
    ProjectileType t;
    t.id = "9mm"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009;
    return sim.registerType(t);
}

// Fire one shot along +x from x=0 and collect every event; `pos`/`state`
// out-params report where it ended.
std::vector<Event> shoot(Sim& sim, TypeId id, const World& world, Real speed,
                         Vec3 dir = {1, 0, 0}, Vec3 from = {0, 0, 0},
                         FidelityTier tier = FidelityTier::Integrated) {
    LaunchParams lp; lp.position = from; lp.direction = dir; lp.speed = speed;
    lp.tier = tier;
    const StateId h = sim.spawn(id, lp);
    VectorEventSink sink;
    for (int i = 0; i < 8000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 1000.0, world, sink);
    return sink.events;
}

const Event* first_of(const std::vector<Event>& ev, EventType ty) {
    for (const Event& e : ev) if (e.type == ty) return &e;
    return nullptr;
}
int count_of(const std::vector<Event>& ev, EventType ty) {
    int n = 0; for (const Event& e : ev) if (e.type == ty) ++n; return n;
}

} // namespace

PON_TEST(terminal_poncelet_depth_is_physical) {
    // Arrow into a thick oak block embeds a few cm; a rifle round drives much
    // deeper into the same wood. Both stay in a sane range (Poncelet closed
    // form, §3.5.3).
    SlabWorld oak; oak.slabs = {{20.0, mk_mat(MaterialBehaviour::Fibrous, 750, 9.0e7, -1.0)}};

    Sim s1; const std::vector<Event> a = shoot(s1, reg_arrow(s1), oak, 55.0);
    const Event* ea = first_of(a, EventType::Embedded);
    CHECK(ea != nullptr);
    if (ea) { CHECK(ea->channelDepth_m > 0.01); CHECK(ea->channelDepth_m < 0.30); }

    Sim s2; const std::vector<Event> r = shoot(s2, reg_rifle(s2), oak, 800.0);
    const Event* er = first_of(r, EventType::Embedded);
    CHECK(er != nullptr);
    if (er && ea) {
        CHECK(er->channelDepth_m > ea->channelDepth_m);
        CHECK(er->channelDepth_m < 1.5);
    }
}

PON_TEST(terminal_arrow_bounces_or_stops_on_concrete) {
    // "Arrow vs concrete": no meaningful penetration.
    SlabWorld wall; wall.slabs = {{20.0, mk_mat(MaterialBehaviour::Brittle, 2400, 3.5e7, -1.0)}};
    Sim s; const std::vector<Event> ev = shoot(s, reg_arrow(s), wall, 60.0);
    const Event* stop = first_of(ev, EventType::Stopped);
    CHECK(stop != nullptr);
    CHECK(first_of(ev, EventType::Embedded) == nullptr);
    if (stop) CHECK(stop->point.x < 20.05); // stopped at the face
}

PON_TEST(terminal_thin_steel_perforate_vs_stop) {
    // A rifle round punches 6 mm of mild steel and exits slower; a pistol round
    // does not make the ballistic limit and stays in the plate.
    auto steel6 = [] { SlabWorld w; w.slabs = {
        {20.0, mk_mat(MaterialBehaviour::Ductile, 7850, 2.5e8, 0.006)}}; return w; };

    SlabWorld a = steel6();
    Sim s1; const std::vector<Event> rifle = shoot(s1, reg_rifle(s1), a, 800.0);
    const Event* perf = first_of(rifle, EventType::Perforated);
    CHECK(perf != nullptr);
    if (perf) {
        CHECK(perf->residualSpeed_mps > 150.0);
        CHECK(perf->residualSpeed_mps < 800.0);
        CHECK(perf->energy_J > 0.0);
    }

    SlabWorld b = steel6();
    Sim s2; const std::vector<Event> pist = shoot(s2, reg_pistol(s2), b, 360.0);
    CHECK(first_of(pist, EventType::Perforated) == nullptr);
    CHECK(first_of(pist, EventType::Embedded) != nullptr ||
          first_of(pist, EventType::Stopped)  != nullptr);
}

PON_TEST(terminal_ricochet_at_a_shallow_angle) {
    FloorWorld steel(mk_mat(MaterialBehaviour::Ductile, 7850, 2.5e8, 0.02));

    // ~8° below horizontal onto the steel floor → skips off, upward, slower.
    Sim s1; const std::vector<Event> graze =
        shoot(s1, reg_pistol(s1), steel, 380.0, {1, -0.14, 0}, {0, 2, 0});
    const Event* ric = first_of(graze, EventType::Ricochet);
    CHECK(ric != nullptr);
    if (ric) {
        CHECK(ric->residualSpeed_mps > 60.0);
        CHECK(ric->residualSpeed_mps < 380.0);
    }

    // ~63° below horizontal → bites in, no ricochet.
    Sim s2; const std::vector<Event> steep =
        shoot(s2, reg_pistol(s2), steel, 380.0, {0.5, -1.0, 0}, {0, 2, 0});
    CHECK(first_of(steep, EventType::Ricochet) == nullptr);
}

PON_TEST(terminal_layered_targets_chain) {
    // Two thin pine boards in series: perforate both, residual speed drops each
    // time, projectile ends up past both.
    SlabWorld boards; boards.slabs = {
        {10.0, mk_mat(MaterialBehaviour::Fibrous, 500, 4.0e7, 0.02)},
        {10.6, mk_mat(MaterialBehaviour::Fibrous, 500, 4.0e7, 0.02)},
    };
    Sim s; const TypeId id = reg_rifle(s);
    const std::vector<Event> ev = shoot(s, id, boards, 800.0);
    CHECK(count_of(ev, EventType::Perforated) == 2);
    std::vector<Real> vres;
    for (const Event& e : ev)
        if (e.type == EventType::Perforated) vres.push_back(e.residualSpeed_mps);
    if (vres.size() == 2) {
        CHECK(vres[0] < 800.0);
        CHECK(vres[1] < vres[0]);
    }
}

PON_TEST(terminal_membrane_always_perforates) {
    SlabWorld balloon; balloon.slabs = {
        {5.0, mk_mat(MaterialBehaviour::Membrane, 1100, 1.5e7, 0.001)}};
    Sim s; const std::vector<Event> ev = shoot(s, reg_pistol(s), balloon, 120.0);
    const Event* p = first_of(ev, EventType::Perforated);
    CHECK(p != nullptr);
    if (p) CHECK(p->residualSpeed_mps > 0.9 * 120.0); // barely slowed
}

PON_TEST(terminal_fragile_projectile_shatters) {
    ProjectileType t;
    t.id = "clay_pot"; t.klass = ProjectileClass::Rock;
    t.mass_kg = 0.3; t.refDiameter_m = 0.08;
    t.terminal.fragile = true;
    Sim s; const TypeId id = s.registerType(t);
    SlabWorld wall; wall.slabs = {{6.0, mk_mat(MaterialBehaviour::Brittle, 2000, 2.0e7, 0.2)}};
    const std::vector<Event> ev = shoot(s, id, wall, 25.0);
    CHECK(first_of(ev, EventType::Shattered) != nullptr);
}

PON_TEST(terminal_deformable_round_expands_between_layers) {
    ProjectileType t;
    t.id = "jhp"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.0095; t.refDiameter_m = 0.009;
    t.terminal.deformable = true;
    Sim s; const TypeId id = s.registerType(t);
    // A thin soft board it will punch through, then a wall behind.
    SlabWorld tgt; tgt.slabs = {
        {8.0,  mk_mat(MaterialBehaviour::Fibrous, 500, 2.5e7, 0.015)},
        {8.5,  mk_mat(MaterialBehaviour::Fibrous, 900, 9.0e7, -1.0)},
    };
    LaunchParams lp; lp.position = {0,0,0}; lp.direction = {1,0,0}; lp.speed = 400.0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = s.spawn(id, lp);
    VectorEventSink sink;
    for (int i = 0; i < 8000 && s.state(h).alive; ++i) s.step(1.0/1000.0, tgt, sink);
    CHECK(count_of(sink.events, EventType::Perforated) >= 1);
    CHECK((s.state(h).flags & kFlagExpanded) != 0);
    CHECK(s.state(h).expandedDiameter_m > 0.009);
}

PON_TEST(terminal_obliquity_costs_more_velocity) {
    auto slab = [] { SlabWorld w; w.slabs = {
        {20.0, mk_mat(MaterialBehaviour::Ductile, 7850, 2.0e8, 0.004)}}; return w; };

    SlabWorld a = slab();
    Sim s1; const std::vector<Event> head = shoot(s1, reg_rifle(s1), a, 850.0, {1, 0, 0});
    SlabWorld b = slab();
    Sim s2; const std::vector<Event> obl = shoot(s2, reg_rifle(s2), b, 850.0, {1, 0, 1});

    const Event* ph = first_of(head, EventType::Perforated);
    const Event* po = first_of(obl,  EventType::Perforated);
    CHECK(ph != nullptr);
    CHECK(po != nullptr);
    if (ph && po) CHECK(po->residualSpeed_mps < ph->residualSpeed_mps);
}

// --- Phase 19 item 6: destruction coupling — impulse on the struck body ----

PON_TEST(terminal_impact_reports_delivered_impulse) {
    // Kinetic events carry m·Δv handed to the struck body, pointing into the
    // surface. This is the seam an engine's destruction / rigid-body response
    // reads (WORKPLAN Phase 19 item 6). reg_slug mass = 0.010 kg.
    const Real m = 0.010;

    // Thin ductile plate: perforates, keeps most speed → deposits only the
    // small momentum difference.
    SlabWorld plate; plate.slabs = {
        {12.0, mk_mat(MaterialBehaviour::Ductile, 7850, 3.0e8, 0.006)}};
    Sim sp; const std::vector<Event> pev = shoot(sp, reg_slug(sp), plate, 900.0);
    const Event* pf = first_of(pev, EventType::Perforated);
    CHECK(pf != nullptr);
    if (pf) {
        CHECK(pf->impulse_Ns.x > 0.0);                          // +x travel → into plate
        CHECK(std::fabs(pf->impulse_Ns.y) < pf->impulse_Ns.x);  // mostly axial
        const Real lost = m * (900.0 - pf->residualSpeed_mps);
        CHECK_NEAR(length(pf->impulse_Ns), lost, 0.20 * lost + 1e-4);
        CHECK(length(pf->impulse_Ns) < m * 900.0 * 1.02);       // ≤ incoming p
    }

    // Deep soft medium: embeds, absorbs the whole incoming momentum.
    SlabWorld clay; clay.slabs = {
        {8.0, mk_mat(MaterialBehaviour::Granular, 1800, 8.0e6, -1.0)}};
    Sim sc; const std::vector<Event> cev = shoot(sc, reg_slug(sc), clay, 400.0);
    const Event* em = first_of(cev, EventType::Embedded);
    CHECK(em != nullptr);
    if (em) CHECK_NEAR(length(em->impulse_Ns), m * 400.0, 0.06 * m * 400.0);
}

PON_TEST(validation_impact_impulse_bounded_by_incoming_momentum) {
    // Across a speed / material / obliquity sweep no kinetic event reports more
    // than 2× the incoming momentum (a perfect bounce is the ceiling).
    static const MaterialBehaviour behs[4] = {
        MaterialBehaviour::Ductile, MaterialBehaviour::Brittle,
        MaterialBehaviour::Fibrous, MaterialBehaviour::Granular};
    int checked = 0;
    for (int si = 0; si < 6; ++si)
    for (int mi = 0; mi < 4; ++mi)
    for (int ai = 0; ai < 3; ++ai) {
        const Real speed = 150.0 + si * 180.0;
        SlabWorld w; w.slabs = {
            {6.0, mk_mat(behs[mi], 3000, 5.0e7, 0.01 + 0.02 * mi)}};
        Sim s; const TypeId id = reg_slug(s);
        const std::vector<Event> ev =
            shoot(s, id, w, speed, normalized(Vec3{1, 0, Real(ai) * 0.4}));
        const Real pIn = 0.010 * speed;
        for (const Event& e : ev) {
            if (e.type == EventType::SurfaceCrossed) continue;
            CHECK(length(e.impulse_Ns) <= pIn * 2.05 + 1e-3);
            ++checked;
        }
    }
    CHECK(checked > 10);
}

// --- Item: special media & behaviours --------------------------------------

namespace {
// Air above a water surface at x = surfaceX (shot travels +x). No geometry —
// mediumAt alone drives the transition, so the surface impulse + drag-down are
// exercised through check_medium_change.
struct WaterHalfSpace final : World {
    Real surfaceX;
    explicit WaterHalfSpace(Real x) : surfaceX(x) {}
    bool raycast(Vec3, Vec3, HitResult&) const override { return false; }
    MediumId mediumAt(Vec3 p) const override {
        return p.x >= surfaceX ? kMediumWater : kMediumAir;
    }
    const Material& material(SurfaceId) const override {
        static const Material w = material_water();
        return w;
    }
};
} // namespace

PON_TEST(water_entry_surface_impulse_bleeds_speed_at_the_interface) {
    // A rifle round crossing the air/water boundary drops a single-digit % right
    // at the surface (MediumChanged carries the post-impulse speed), then the
    // water drag-down does the metre-scale work.
    Sim sim; const TypeId id = reg_slug(sim);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 600.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    WaterHalfSpace world(1.0); VectorEventSink sink;
    for (int i = 0; i < 20000 && sim.state(h).alive &&
                    sim.state(h).position.x < 3.0; ++i)
        sim.step(1.0 / 8000.0, world, sink);

    const Event* mc = first_of(sink.events, EventType::MediumChanged);
    CHECK(mc != nullptr);
    if (mc) {
        CHECK(mc->residualSpeed_mps < 600.0);   // lost something at the surface
        CHECK(mc->residualSpeed_mps > 480.0);   // but only the interface step
    }
    CHECK(sim.state(h).mediumId == kMediumWater);
}

PON_TEST(supercavitation_extends_underwater_reach) {
    // The same submerged shot in plain water vs a supercavitation-flagged medium
    // of identical density: the cavity round keeps lethal speed far longer.
    auto reach_to_150 = [](bool cavity) {
        Sim sim; const TypeId id = reg_slug(sim);
        MediumDesc m; m.name = "w2"; m.density_kgm3 = 1000.0; m.buoyancy = 0.1;
        m.supercavitation = cavity;
        const MediumId mid = sim.environment().media.add(m);
        LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
        lp.speed = 700.0; lp.tier = FidelityTier::Integrated; lp.medium = mid;
        const StateId h = sim.spawn(id, lp);
        FixedMediumWorld world(mid); VectorEventSink sink;
        for (int i = 0; i < 40000 && sim.state(h).alive &&
                        length(sim.state(h).velocity) > 150.0; ++i)
            sim.step(1.0 / 8000.0, world, sink);
        return sim.state(h).position.x;
    };
    const double plain  = reach_to_150(false);
    const double cavity = reach_to_150(true);
    CHECK(plain  < 3.0);         // plain water: sub-wounding inside a couple m
    CHECK(cavity > 4.0 * plain); // cavity: many times the reach
}

PON_TEST(brittle_sharp_low_sd_round_stops_at_the_face) {
    // A crossbow bolt into a thin concrete panel: no channel, stops at the
    // surface (spall), even though a thin slab could in principle be perforated.
    SlabWorld panel; panel.slabs = {
        {20.0, mk_mat(MaterialBehaviour::Brittle, 2400, 3.0e7, 0.04)}};
    Sim s; const std::vector<Event> ev = shoot(s, reg_arrow(s), panel, 120.0);
    const Event* stop = first_of(ev, EventType::Stopped);
    CHECK(stop != nullptr);
    CHECK(first_of(ev, EventType::Perforated) == nullptr);
    CHECK(first_of(ev, EventType::Embedded) == nullptr);
    if (stop) CHECK(stop->point.x < 20.05);

    // A rifle round (high SD, not sharp) still punches the same panel.
    SlabWorld panel2 = panel;
    Sim s2; const std::vector<Event> ev2 = shoot(s2, reg_rifle(s2), panel2, 800.0);
    CHECK(first_of(ev2, EventType::Perforated) != nullptr);
}

// --- Item 9: default data tables (catalog + materials) --------------------

PON_TEST(catalog_get_returns_a_populated_type) {
    CHECK(catalog::has("9x19_124gr_fmj"));
    const ProjectileType t = catalog::get("9x19_124gr_fmj");
    CHECK(t.id == "9x19_124gr_fmj");
    CHECK(t.klass == ProjectileClass::Bullet);
    CHECK(t.dragModel == DragModel::G1);
    CHECK(t.mass_kg > 0.0079 && t.mass_kg < 0.0082);
    CHECK(t.refDiameter_m > 0.0089 && t.refDiameter_m < 0.0091);
    CHECK(t.ballisticCoefficient.has_value());
    CHECK(*t.ballisticCoefficient > 0.14 && *t.ballisticCoefficient < 0.16);
    CHECK(t.muzzleSpeed_mps.has_value());
    CHECK(t.twistRate_m > 0.0); // right-hand twist
}

PON_TEST(catalog_unknown_id_is_reported) {
    CHECK(!catalog::has("no_such_round"));
    CHECK(!catalog::find("no_such_round").has_value());
    const ProjectileType fb = catalog::get("no_such_round");
    CHECK(fb.klass == ProjectileClass::Custom);
    CHECK(fb.id == "no_such_round");
}

PON_TEST(catalog_arrow_and_ball_entries_resolve) {
    const ProjectileType a = catalog::get("longbow_bodkin_60g");
    CHECK(a.klass == ProjectileClass::Arrow);
    CHECK(a.dragModel == DragModel::ConstantCd);
    CHECK(a.dragCoefficient.has_value());

    const ProjectileType b = catalog::get("fifa_matchball");
    CHECK(b.dragModel == DragModel::BallProfile);
    CHECK(b.ballProfile == "fifa_football");
    // geometry filled from the ball profile at registerType()
    Sim sim; const TypeId id = sim.registerType(b);
    CHECK(id != kInvalidType);
    CHECK(sim.type(id).refDiameter_m > 0.2);
    CHECK(sim.type(id).mass_kg > 0.4);
}

PON_TEST(catalog_entry_flies_like_a_hand_built_type) {
    // catalog::get + registerType must match the equivalent explicit type.
    Sim a; const TypeId ca = a.registerType(catalog::get("762x51_175gr_smk"));

    ProjectileType m;
    m.id = "manual"; m.klass = ProjectileClass::Bullet; m.dragModel = DragModel::G7;
    m.mass_kg = 0.01134; m.refDiameter_m = 0.00782;
    m.ballisticCoefficient = 0.243; m.muzzleSpeed_mps = 800.0;
    Sim b; const TypeId cb = b.registerType(m);

    LaunchParams lp; lp.position = {0, 1.8, 0}; lp.direction = {1, 0, 0};
    lp.tier = FidelityTier::Integrated;
    const StateId ha = a.spawn(ca, lp), hb = b.spawn(cb, lp);
    EmptyWorld w; VectorEventSink sink;
    for (int f = 0; f < 90; ++f) { a.step(1.0 / 60.0, w, sink); b.step(1.0 / 60.0, w, sink); }
    CHECK_NEAR(a.state(ha).position.x, b.state(hb).position.x, 1.0);
    CHECK_NEAR(length(a.state(ha).velocity), length(b.state(hb).velocity), 2.0);
}

PON_TEST(catalog_9mm_retained_velocity_is_in_the_published_band) {
    // 124 gr FMJ, G1 BC 0.15, MV 360 m/s: published dope has ~330 m/s left at
    // 50 m, ~305 m/s at 100 m.
    Sim sim; const TypeId id = sim.registerType(catalog::get("9x19_124gr_fmj"));
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld w; VectorEventSink sink;
    double v50 = 0.0, v100 = 0.0;
    for (int i = 0; i < 5000 && sim.state(h).position.x < 100.0; ++i) {
        sim.step(1.0 / 2000.0, w, sink);
        const auto& s = sim.state(h);
        if (v50 == 0.0 && s.position.x >= 50.0)  v50 = length(s.velocity);
        if (v100 == 0.0 && s.position.x >= 100.0) v100 = length(s.velocity);
    }
    CHECK(v50 > 315.0 && v50 < 345.0);
    CHECK(v100 > 285.0 && v100 < 320.0);
}

PON_TEST(catalog_overlay_replaces_a_baked_entry) {
    catalog::clear_overlay();
    const double baseBc = *catalog::get("762x51_175gr_smk").ballisticCoefficient;

    const char* csv =
        "id,klass,drag_model,mass_kg,ref_diameter_m,ballistic_coefficient,muzzle_speed_mps\n"
        "762x51_175gr_smk,Bullet,G7,0.01134,0.00782,0.300,810\n"
        "test_only_bolt,Custom,ConstantCd,0.04,0.02,,900\n";
    std::string err;
    const int n = catalog::load_csv_string(csv, &err);
    CHECK(n == 2);
    CHECK(err.empty());
    CHECK_NEAR(*catalog::get("762x51_175gr_smk").ballisticCoefficient, 0.300, 1e-9);
    CHECK(catalog::has("test_only_bolt"));

    catalog::clear_overlay();
    CHECK_NEAR(*catalog::get("762x51_175gr_smk").ballisticCoefficient, baseBc, 1e-9);
    CHECK(!catalog::has("test_only_bolt"));
}

PON_TEST(catalog_to_csv_round_trips_through_load_csv_string) {
    catalog::clear_overlay();
    const std::string csv = catalog::to_csv();
    CHECK(csv.substr(0, 3) == "id,");
    CHECK(csv.find("9x19_124gr_fmj,") != std::string::npos);

    // Every id from the shipped catalog appears exactly once.
    const std::vector<std::string> ids = catalog::ids();
    CHECK(!ids.empty());
    std::size_t lines = 0;
    for (char c : csv) if (c == '\n') ++lines;
    CHECK(lines == ids.size() + 1); // + the header row

    // Feed it back in as an overlay: a known field must survive the round trip.
    const ProjectileType before = catalog::get("9x19_124gr_fmj");
    std::string err;
    const int n = catalog::load_csv_string(csv, &err);
    CHECK(n == static_cast<int>(ids.size()));
    CHECK(err.empty());
    const ProjectileType after = catalog::get("9x19_124gr_fmj");
    CHECK_NEAR(after.mass_kg, before.mass_kg, 1e-9);
    CHECK_NEAR(after.refDiameter_m, before.refDiameter_m, 1e-9);
    CHECK_NEAR(*after.ballisticCoefficient, *before.ballisticCoefficient, 1e-9);
    CHECK(after.dragModel == before.dragModel);
    catalog::clear_overlay();
}

PON_TEST(materials_to_csv_round_trips_through_load_csv_string) {
    materials::clear_overlay();
    const std::string csv = materials::to_csv();
    CHECK(csv.substr(0, 5) == "name,");
    CHECK(csv.find("oak,") != std::string::npos || csv.find("\noak,") != std::string::npos);

    const Material before = *materials::find("oak");
    std::string err;
    const int n = materials::load_csv_string(csv, &err);
    CHECK(n == static_cast<int>(materials::names().size()));
    CHECK(err.empty());
    const Material after = *materials::find("oak");
    CHECK_NEAR(after.density_kgm3, before.density_kgm3, 1e-6);
    CHECK_NEAR(after.strength_Pa, before.strength_Pa, 1e-6);
    CHECK(after.behaviour == before.behaviour);
    materials::clear_overlay();
}

PON_TEST(catalog_save_csv_writes_a_loadable_file) {
    catalog::clear_overlay();
    const std::string path = "test_catalog_to_csv_tmp.csv";
    std::string err;
    CHECK(catalog::save_csv(path, &err));
    CHECK(err.empty());

    std::ifstream in(path, std::ios::binary);
    CHECK(in.good());
    std::stringstream ss; ss << in.rdbuf();
    in.close();
    CHECK(ss.str() == catalog::to_csv());
    std::remove(path.c_str());

    CHECK(!catalog::save_csv("no/such/directory/x.csv", &err));
    CHECK(!err.empty());
}

PON_TEST(catalog_c_abi_registers_a_named_round) {
    CHECK(pon_catalog_has("9x19_124gr_fmj") == 1);
    CHECK(pon_catalog_has("no_such_round") == 0);
    pon_sim* s = pon_sim_create(PON_DET_PLATFORM_STABLE);
    const uint32_t id = pon_register_catalog_type(s, "9x19_124gr_fmj");
    CHECK(id != 0xFFFFFFFFu);
    CHECK(pon_register_catalog_type(s, "no_such_round") == 0xFFFFFFFFu);
    const uint32_t h = pon_spawn(s, id, {0, 0, 0}, {1, 0, 0}, 0.0);
    CHECK(h != 0xFFFFFFFFu);
    pon_sim_destroy(s);
}

PON_TEST(materials_table_has_the_seed_entries) {
    for (const char* n : {"air", "water", "oak", "pine", "concrete",
                          "glass", "mild_steel", "sand"})
        CHECK(catalog::has(n) || materials::has(n));

    const auto oak = materials::find("oak");
    CHECK(oak.has_value());
    if (oak) {
        CHECK(oak->behaviour == MaterialBehaviour::Fibrous);
        CHECK(oak->density_kgm3 > 700 && oak->density_kgm3 < 800);
        CHECK(std::string(oak->name) == "oak");
    }
    const auto water = materials::find("water");
    CHECK(water.has_value());
    if (water) CHECK(water->entersMedium == kMediumWater);
}

PON_TEST(materials_overlay_adds_a_game_material) {
    materials::clear_overlay();
    CHECK(!materials::has("kevlar_panel"));
    const char* csv =
        "name,behaviour,density_kgm3,strength_pa,toughness_j_m2,thickness_m,elasticity,enters_medium\n"
        "kevlar_panel,Fibrous,1440,3.0e9,4.0e4,0.008,0.1,-1\n";
    std::string err;
    CHECK(materials::load_csv_string(csv, &err) == 1);
    const auto k = materials::find("kevlar_panel");
    CHECK(k.has_value());
    if (k) {
        CHECK(std::string(k->name) == "kevlar_panel");
        CHECK(k->density_kgm3 > 1400);
    }
    materials::clear_overlay();
    CHECK(!materials::has("kevlar_panel"));
}

// --- v1 perf pass (WORKPLAN item 12): trajectory cache + SoA batch integrator -

namespace {
ProjectileType perf_bullet() {
    ProjectileType t;
    t.id = "perf_bullet";
    t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd;
    t.dragCoefficient = 0.30;
    t.mass_kg = 0.010;
    t.refDiameter_m = 0.0090;
    return t;
}
} // namespace

PON_TEST(trajectory_cache_records_and_replays) {
    Environment env;
    SimConfig cfg;
    cfg.trajectoryCacheFrames = 256;
    Sim sim(env, cfg);
    const TypeId id = sim.registerType(perf_bullet());

    LaunchParams lp;
    lp.position  = {0, 2, 0};
    lp.direction = {1, 0.05, 0};
    lp.speed     = 800.0;
    lp.tier      = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    for (int f = 0; f < 120; ++f) sim.step(1.0 / 120.0, world, sink); // 1.0 s

    CHECK(sim.trajectorySize(h) >= 100);

    Vec3 p, v;
    // Before the first sample clamps to the launch point.
    CHECK(sim.sampleTrajectory(h, -1.0, p, v));
    CHECK_NEAR(p.x, 0.0, 1e-9);
    CHECK_NEAR(p.y, 2.0, 1e-9);
    // After the last clamps to the newest cached pose.
    CHECK(sim.sampleTrajectory(h, 100.0, p, v));
    CHECK_NEAR(p.x, sim.state(h).position.x, 1e-6);

    // Mid-flight replay matches an independent re-sim stepped to exactly 0.5 s.
    CHECK(sim.sampleTrajectory(h, 0.5, p, v));
    Sim ref(env, SimConfig{});
    const TypeId rid = ref.registerType(perf_bullet());
    const StateId rh = ref.spawn(rid, lp);
    for (int f = 0; f < 60; ++f) ref.step(1.0 / 120.0, world, sink);
    CHECK_NEAR(p.x, ref.state(rh).position.x, 0.5);
    CHECK_NEAR(p.y, ref.state(rh).position.y, 0.05);
}

PON_TEST(trajectory_cache_ring_caps_at_capacity) {
    SimConfig cfg;
    cfg.trajectoryCacheFrames = 16;
    Sim sim({}, cfg);
    const TypeId id = sim.registerType(perf_bullet());
    LaunchParams lp;
    lp.direction = {1, 0.1, 0};
    lp.speed     = 300.0;
    lp.tier      = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    for (int f = 0; f < 100; ++f) sim.step(1.0 / 60.0, world, sink);

    CHECK(sim.trajectorySize(h) == 16);
    TrajectorySample buf[64];
    const std::size_t n = sim.trajectory(h, buf, 64);
    CHECK(n == 16);
    for (std::size_t k = 1; k < n; ++k) CHECK(buf[k].time_s > buf[k - 1].time_s);
}

PON_TEST(batch_integrator_matches_per_shot) {
    Environment env;
    SimConfig sc; sc.batchIntegrator = false;
    SimConfig bc; bc.batchIntegrator = true; bc.batchMinGroup = 4;
    Sim a(env, sc), b(env, bc);
    const TypeId ia = a.registerType(perf_bullet());
    const TypeId ib = b.registerType(perf_bullet());

    std::vector<StateId> ha, hb;
    LaunchParams lp;
    lp.position = {0, 1.6, 0};
    lp.speed    = 850.0;
    lp.tier     = FidelityTier::Integrated;
    for (int k = 0; k < 16; ++k) {
        lp.direction = {1, 0.010 + 0.003 * k, 0.002 * k};
        ha.push_back(a.spawn(ia, lp));
        hb.push_back(b.spawn(ib, lp));
    }

    EmptyWorld world; VectorEventSink sa, sb;
    for (int f = 0; f < 90; ++f) { // 1.5 s
        a.step(1.0 / 60.0, world, sa);
        b.step(1.0 / 60.0, world, sb);
    }

    for (int k = 0; k < 16; ++k) {
        const Vec3 pa = a.state(ha[k]).position;
        const Vec3 pb = b.state(hb[k]).position;
        // Group-max sub-step count is the only source of divergence — sub-mm
        // over a 1.5 s flight.
        CHECK_NEAR(pa.x, pb.x, 0.02);
        CHECK_NEAR(pa.y, pb.y, 0.02);
        CHECK_NEAR(pa.z, pb.z, 0.02);
    }
}

PON_TEST(sim_stats_tallies_live_by_tier_and_substeps) {
    Sim sim;
    const TypeId id = sim.registerType(perf_bullet());
    LaunchParams lp; lp.position = {0, 1.6, 0}; lp.direction = {1, 0, 0}; lp.speed = 800.0;

    lp.tier = FidelityTier::Hitscan;      sim.spawn(id, lp);
    lp.tier = FidelityTier::AnalyticDrag; sim.spawn(id, lp);
    lp.tier = FidelityTier::Integrated;   sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    sim.step(1.0 / 60.0, world, sink);

    const SimStats& st = sim.stats();
    CHECK(st.liveHitscan == 1);
    CHECK(st.liveAnalytic == 1);
    CHECK(st.liveIntegrated == 1);
    CHECK(st.subSteps > 0);       // only the Integrated shot sub-steps
    CHECK(st.batchGroups == 0);   // batchIntegrator is off
    CHECK(st.sweptQueries > 0);   // every tier issues at least one swept query
}

PON_TEST(sim_stats_counts_batch_groups) {
    SimConfig c; c.batchIntegrator = true; c.batchMinGroup = 4;
    Sim sim({}, c);
    const TypeId id = sim.registerType(perf_bullet());
    LaunchParams lp; lp.position = {0, 1.6, 0}; lp.speed = 850.0; lp.tier = FidelityTier::Integrated;
    for (int k = 0; k < 8; ++k) {
        lp.direction = {1, 0.01 + 0.002 * k, 0};
        sim.spawn(id, lp);
    }
    EmptyWorld world; VectorEventSink sink;
    sim.step(1.0 / 60.0, world, sink);

    const SimStats& st = sim.stats();
    CHECK(st.batchGroups == 1);       // one type, all 8 eligible -> one group
    CHECK(st.liveIntegrated == 8);
    CHECK(st.subSteps > 0);
}

PON_TEST(sim_stats_counts_events_emitted) {
    Sim sim;
    ProjectileType t = perf_bullet();
    t.maxLifetime_s = 1.0 / 120.0; // expires on the very next step
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 1.6, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    sim.spawn(id, lp);

    EmptyWorld world; VectorEventSink sink;
    sim.step(1.0 / 60.0, world, sink);

    CHECK(!sink.events.empty());
    CHECK(sim.stats().eventsEmitted == sink.events.size());
}

PON_TEST(c_abi_sim_get_stats) {
    pon_sim_config cfg{};
    pon_sim* sim = pon_sim_create_ex(PON_DET_PLATFORM_STABLE, &cfg);
    pon_projectile_desc d{};
    d.id = "9mm"; d.klass = 0; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.mass_kg = 0.008; d.ref_diameter_m = 0.009; d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;
    const uint32_t tid = pon_register_type(sim, &d);
    pon_spawn_precise(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0, 2 /*Integrated*/, 0);
    pon_step(sim, 1.0 / 60.0);

    pon_sim_stats st{};
    pon_sim_get_stats(sim, &st);
    CHECK(st.live_integrated == 1);
    CHECK(st.sub_steps > 0);
    CHECK(st.batch_groups == 0);

    pon_sim_destroy(sim);

    // NULL sim -> zeroed struct, not a crash.
    pon_sim_stats zeroed;
    zeroed.live_integrated = 42; // pre-poison to prove it gets overwritten
    pon_sim_get_stats(nullptr, &zeroed);
    CHECK(zeroed.live_integrated == 0);
}

PON_TEST(batch_group_resolves_mixed_hits_and_misses) {
    SimConfig c; c.batchIntegrator = true; c.batchMinGroup = 4;
    Sim sim({}, c);
    const TypeId id = sim.registerType(perf_bullet());

    LaunchParams lp;
    lp.position = {0, 2, 0};
    lp.speed    = 400.0;
    lp.tier     = FidelityTier::Integrated;

    std::vector<StateId> hitShots, missShots;
    for (int k = 0; k < 4; ++k) { lp.direction = {1, 0.0, 0}; hitShots.push_back(sim.spawn(id, lp)); }
    for (int k = 0; k < 4; ++k) { lp.direction = {0, 1, 0.02}; missShots.push_back(sim.spawn(id, lp)); }

    WallWorld wall(40.0);
    VectorEventSink sink;
    for (int f = 0; f < 60; ++f) sim.step(1.0 / 60.0, wall, sink);

    int stopped = 0;
    for (const auto& e : sink.events) if (e.type == EventType::Stopped) ++stopped;
    CHECK(stopped == 4);
    for (const auto& e : sink.events)
        if (e.type == EventType::Stopped) CHECK_NEAR(e.point.x, 40.0, 0.5);

    for (const StateId h : hitShots) CHECK(!sim.state(h).alive);
    for (const StateId h : missShots) {
        CHECK(sim.state(h).alive);
        CHECK(sim.state(h).position.y > 2.0); // still climbing / aloft
    }
}

// ===========================================================================
// Validation suite (WORKPLAN item 13 / §6). These reproduce published exterior
// and terminal data within documented tolerances, plus a determinism check and
// a fuzz sweep. The `poncelet_validation` ctest entry runs just this group
// (name substring `validation_`); they are also part of the full run.
// ===========================================================================

namespace {
// splitmix64 — a tiny deterministic PRNG for the fuzz sweep (no <random>, so
// the sweep is bit-reproducible across platforms and runs).
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed) {}
    std::uint64_t next() {
        s += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * (double(next() >> 11) * (1.0 / 9007199254740992.0));
    }
    int range(int n) { return int(next() % std::uint64_t(n)); }
};
} // namespace

PON_TEST(validation_exterior_308_168_match_dope_1000m) {
    // .308 Win, 168 gr HPBT Match (catalog `3006_168gr_hpbt`, G7 BC 0.223,
    // MV 850 m/s), ICAO sea level. The 168 SMK is a famously transonic round at
    // this distance — real Sierra / JBM dope has it at ~1120-1150 fps
    // (~340-350 m/s) at ~900-1000 m, dropping through Mach 1 near there. With
    // the BRL/McCoy G7 table the library produces ~315 m/s / ~2.0 s / ~14 m
    // drop at 1000 m, matching that published behaviour (an independent RK4
    // integration of the same table lands on the same number). A single-BC G7
    // model can't track the bullet's real post-transonic drag rise, so treat
    // the sub-Mach-1 tail as indicative, not exact.
    Sim sim;
    const TypeId id = sim.registerType(catalog::get("3006_168gr_hpbt"));
    CHECK(id != kInvalidType);
    LaunchParams lp; lp.position = {0, 4000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 850.0; lp.tier = FidelityTier::Integrated;
    const DownrangeSample d = fly_to(sim, sim.spawn(id, lp), 1000.0);
    std::printf("    [308 168gr @1000m] speed=%.1f tof=%.3f drop=%.2f "
                "(Sierra dope ~340 / ~2.0)\n", d.speed, d.tof, d.drop);
    CHECK(d.speed > 285.0 && d.speed < 360.0);
    CHECK(d.tof   > 1.80  && d.tof   < 2.20);
    CHECK(d.drop  > 10.0  && d.drop  < 19.0);
}

PON_TEST(validation_exterior_556_55gr_m193_300m) {
    // 5.56x45 M193, 55 gr (catalog `556x45_55gr_m193`, MV 990 m/s). Published:
    // ~640 m/s remaining at 300 m.
    Sim sim;
    const TypeId id = sim.registerType(catalog::get("556x45_55gr_m193"));
    CHECK(id != kInvalidType);
    LaunchParams lp; lp.position = {0, 2000, 0}; lp.direction = {1, 0, 0};
    lp.speed = 990.0; lp.tier = FidelityTier::Integrated;
    const DownrangeSample d = fly_to(sim, sim.spawn(id, lp), 300.0);
    CHECK(d.speed > 560.0 && d.speed < 740.0);
    CHECK(d.drop  < 0.9);
}

PON_TEST(validation_exterior_analytic_tier_tracks_integrated) {
    // The AnalyticDrag fast path must stay within a few percent of the full
    // integrator over a long flat shot (it is what most tracers run on).
    auto fly = [](FidelityTier tier) {
        Sim sim;
        ProjectileType t;
        t.id = "v"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
        t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 4000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 800.0; lp.tier = tier;
        return fly_to(sim, sim.spawn(id, lp), 800.0);
    };
    const DownrangeSample a = fly(FidelityTier::AnalyticDrag);
    const DownrangeSample i = fly(FidelityTier::Integrated);
    CHECK_NEAR(a.speed, i.speed, i.speed * 0.05);
    CHECK_NEAR(a.drop,  i.drop,  0.30);
}

PON_TEST(validation_terminal_oak_penetration_ordering) {
    // Poncelet's oak-penetration dataset (the closed form fits it R²≈0.98). We
    // check the qualitative envelope: an arrow embeds shallow, a rifle round
    // much deeper into the same block, both bounded and both `Embedded`.
    SlabWorld oak; oak.slabs = {{20.0, mk_mat(MaterialBehaviour::Fibrous, 750, 9.0e7, -1.0)}};
    Sim sa; const std::vector<Event> arrowEv = shoot(sa, reg_arrow(sa), oak, 55.0);
    Sim sr; const std::vector<Event> rifleEv = shoot(sr, reg_rifle(sr), oak, 800.0);
    const Event* ea = first_of(arrowEv, EventType::Embedded);
    const Event* er = first_of(rifleEv, EventType::Embedded);
    CHECK(ea != nullptr);
    CHECK(er != nullptr);
    if (ea && er) {
        CHECK(ea->channelDepth_m > 0.01 && ea->channelDepth_m < 0.30);
        CHECK(er->channelDepth_m > ea->channelDepth_m);
        CHECK(er->channelDepth_m < 1.5);
    }
}

PON_TEST(validation_terminal_rifle_round_stopped_by_water_within_a_couple_of_metres) {
    // "A rifle round is stopped within roughly a metre or two of water." Fire a
    // slug into a water half-space and measure how far it travels before
    // dropping below 100 m/s (sub-lethal). Real high-speed footage: a couple of
    // metres, and not instantaneous.
    Sim sim; const TypeId id = reg_slug(sim);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0};
    lp.speed = 600.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    WaterHalfSpace world(0.0); VectorEventSink sink;
    for (int i = 0; i < 200000 && sim.state(h).alive &&
           length(sim.state(h).velocity) > 100.0; ++i)
        sim.step(1.0 / 20000.0, world, sink);
    const double x = sim.state(h).position.x;
    CHECK(sim.state(h).mediumId == kMediumWater);
    CHECK(x > 0.2);   // not an instant stop
    CHECK(x < 3.0);   // pulled down within a couple of metres
}

PON_TEST(validation_terminal_arrow_off_concrete) {
    SlabWorld wall; wall.slabs = {{20.0, mk_mat(MaterialBehaviour::Brittle, 2400, 3.5e7, -1.0)}};
    Sim s; const std::vector<Event> ev = shoot(s, reg_arrow(s), wall, 65.0);
    const Event* stop = first_of(ev, EventType::Stopped);
    CHECK(stop != nullptr);
    CHECK(first_of(ev, EventType::Perforated) == nullptr);
    if (stop) CHECK(stop->point.x < 20.05); // spalls at the face, no channel
}

PON_TEST(validation_terminal_bulk_penetration_depth_bands) {
    // Poncelet closed form vs published ordnance envelopes for a .308-class
    // rifle bullet (11.3 g, 7.82 mm) at ~800 m/s into bulk targets:
    //   * mild steel — a non-AP rifle bullet embeds ~10–25 mm (Recht–Ipson /
    //     ordnance data; .30 M2 AP is 10.7–12.7 mm into the harder RHA @ 100 yd).
    //   * 35 MPa concrete — 7.62-class embeds ~50–160 mm (Forrestal–Frew–Hanchak).
    //   * dry earth — a tumbling rifle bullet stops within ~0.2–0.7 m.
    auto depth_into = [](MaterialBehaviour b, Real rho, Real strength) {
        SlabWorld w; w.slabs = {{20.0, mk_mat(b, rho, strength, -1.0)}}; // bulk
        Sim s; const std::vector<Event> ev = shoot(s, reg_rifle(s), w, 800.0);
        const Event* e = first_of(ev, EventType::Embedded);
        if (!e) e = first_of(ev, EventType::Stopped);
        return e ? e->channelDepth_m : -1.0;
    };
    const Real steel = depth_into(MaterialBehaviour::Ductile, 7850, 2.5e8);
    CHECK(steel > 0.006); CHECK(steel < 0.040);
    const Real conc = depth_into(MaterialBehaviour::Brittle, 2400, 3.5e7);
    CHECK(conc > 0.030); CHECK(conc < 0.250);
    const Real earth = depth_into(MaterialBehaviour::Granular, 1500, 3.0e5);
    CHECK(earth > 0.10); CHECK(earth < 0.90);
}

// --- Part B7: validation retune under the Q32.32 fixed-point core ---------

namespace {
DownrangeSample fly_to_det(config::Determinism det, const ProjectileType& t,
                           Vec3 pos, double speed, double x_target) {
    Sim sim(Environment{}, SimConfig{det});
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = pos; lp.direction = {1, 0, 0};
    lp.speed = speed; lp.tier = FidelityTier::Integrated;
    return fly_to(sim, sim.spawn(id, lp), x_target);
}
ProjectileType g7_308_168() {
    ProjectileType t;
    t.id = "b7"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.223;
    t.mass_kg = 0.01089; t.refDiameter_m = 0.00782;
    return t;
}
} // namespace

PON_TEST(validation_bitexact_308_168_lands_in_the_published_band) {
    // The same transonic .308 shot the double core validates against Sierra/JBM
    // dope must land in the same band when run through the Q32.32 core.
    const DownrangeSample d =
        fly_to_det(config::Determinism::BitExact, g7_308_168(), {0, 4000, 0}, 850.0, 1000.0);
    std::printf("    [BitExact 308 168gr @1000m] speed=%.2f tof=%.4f drop=%.3f\n",
                d.speed, d.tof, d.drop);
    CHECK(d.speed > 285.0 && d.speed < 360.0);
    CHECK(d.tof   > 1.80  && d.tof   < 2.20);
    CHECK(d.drop  > 10.0  && d.drop  < 19.0);
}

PON_TEST(validation_bitexact_tracks_the_double_core) {
    // The retune measurement: what does Q32.32 rounding cost over a real flight?
    struct Case { const char* name; ProjectileType t; Vec3 p; double v, x; };
    ProjectileType m193;
    m193.id = "m193"; m193.klass = ProjectileClass::Bullet;
    m193.dragModel = DragModel::G1; m193.ballisticCoefficient = 0.243;
    m193.mass_kg = 0.00356; m193.refDiameter_m = 0.00570;

    const Case cases[] = {
        {"308_168_1000m", g7_308_168(), {0, 4000, 0}, 850.0, 1000.0},
        {"m193_300m",     m193,         {0, 2000, 0}, 990.0, 300.0},
    };
    for (const auto& c : cases) {
        const DownrangeSample dbl = fly_to_det(config::Determinism::PlatformStable, c.t, c.p, c.v, c.x);
        const DownrangeSample fx  = fly_to_det(config::Determinism::BitExact,       c.t, c.p, c.v, c.x);
        std::printf("    [%s] dspeed=%.4f m/s  dtof=%.5f s  ddrop=%.5f m\n",
                    c.name, fx.speed - dbl.speed, fx.tof - dbl.tof, fx.drop - dbl.drop);
        // Measured cost of Q32.32 over these flights: < 1e-3 m/s, < 1e-4 s,
        // < 1e-4 m. Gate with ~50-100x margin — tight enough to catch a real
        // regression in the fixed-point path, loose enough for other rounds.
        CHECK(std::fabs(fx.speed - dbl.speed) <= 0.05);     // m/s
        CHECK(std::fabs(fx.tof   - dbl.tof)   <= 5.0e-4);   // s
        CHECK(std::fabs(fx.drop  - dbl.drop)  <= 5.0e-3);   // m
    }
}

PON_TEST(validation_bitexact_rk4_self_consistency_on_step_halving) {
    // No external reference: the fixed-point RK4 at h and h/2 must converge on
    // each other (4th-order → the h/2 run is the better estimate; the gap
    // bounds the truncation error, which Q32.32 rounding must not swamp).
    auto fly = [](double h, int steps) {
        Sim sim(Environment{}, SimConfig{config::Determinism::BitExact});
        const TypeId id = sim.registerType(g7_308_168());
        LaunchParams lp; lp.position = {0, 3000, 0}; lp.direction = {1, 0, 0};
        lp.speed = 820.0; lp.tier = FidelityTier::Integrated;
        const StateId hd = sim.spawn(id, lp);
        EmptyWorld w; VectorEventSink sink;
        for (int i = 0; i < steps; ++i) sim.step(h, w, sink);
        return sim.state(hd).position;
    };
    const Vec3 a = fly(1.0 / 500.0,  750);   // 1.5 s
    const Vec3 b = fly(1.0 / 1000.0, 1500);
    CHECK(std::fabs(a.x - b.x) <= 0.20);
    CHECK(std::fabs(a.y - b.y) <= 0.05);
}

PON_TEST(validation_determinism_same_platform_bit_repeatable) {
    // PlatformStable: identical inputs → bit-identical trajectory on this build.
    auto run_one = [] {
        Sim sim;
        ProjectileType t;
        t.id = "d"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.24;
        t.mass_kg = 0.0113; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp; lp.position = {0, 1000, 0}; lp.direction = {1, 0.05, 0.01};
        lp.speed = 830.0; lp.tier = FidelityTier::Integrated;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld w; VectorEventSink sink;
        for (int i = 0; i < 1500; ++i) sim.step(1.0 / 1000.0, w, sink);
        return sim.state(h).position;
    };
    const Vec3 a = run_one();
    const Vec3 b = run_one();
    CHECK(a.x == b.x && a.y == b.y && a.z == b.z);
}

PON_TEST(validation_fuzz_random_shots_stay_finite) {
    // Random projectile class / drag model / speed / launch angle / target
    // material, stepped through impact. Nothing NaNs, nothing runs away.
    Rng rng(0xF022F025ull);
    const MaterialBehaviour behs[] = {
        MaterialBehaviour::Brittle, MaterialBehaviour::Ductile,
        MaterialBehaviour::Fibrous, MaterialBehaviour::Membrane,
        MaterialBehaviour::Granular, MaterialBehaviour::Fluid};
    const ProjectileClass classes[] = {
        ProjectileClass::Bullet, ProjectileClass::Arrow, ProjectileClass::Bolt,
        ProjectileClass::Spear, ProjectileClass::Pellet, ProjectileClass::Rock};
    const DragModel drags[] = {DragModel::ConstantCd, DragModel::G1, DragModel::G7};
    const FidelityTier tiers[] = {FidelityTier::Hitscan, FidelityTier::AnalyticDrag,
                                  FidelityTier::Integrated};

    int bad = 0, exercised = 0;
    for (int k = 0; k < 3000; ++k) {
        Sim sim;
        ProjectileType t;
        t.id = "fuzz";
        t.klass = classes[rng.range(6)];
        t.dragModel = drags[rng.range(3)];
        t.dragCoefficient = rng.uniform(0.05, 1.2);
        t.ballisticCoefficient = rng.uniform(0.05, 0.6);
        t.mass_kg = rng.uniform(0.005, 5.0);
        t.refDiameter_m = rng.uniform(0.004, 0.15);
        t.maxLifetime_s = 3.0; // every shot must terminate within the step budget
        const TypeId id = sim.registerType(t);
        if (id == kInvalidType) continue;

        SlabWorld world;
        world.slabs = {{rng.uniform(3.0, 40.0),
                        mk_mat(behs[rng.range(6)], rng.uniform(50.0, 8000.0),
                               rng.uniform(1.0e6, 4.0e8),
                               rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : rng.uniform(0.002, 0.2))}};

        LaunchParams lp;
        lp.position  = {0, rng.uniform(0.0, 3.0), 0};
        lp.direction = {1, rng.uniform(-0.3, 0.6), rng.uniform(-0.3, 0.3)};
        lp.speed     = rng.uniform(20.0, 1400.0);
        lp.tier      = tiers[rng.range(3)];
        const StateId h = sim.spawn(id, lp);
        if (h == kInvalidState) continue;

        VectorEventSink sink;
        int steps = 0;
        for (; steps < 20000 && sim.state(h).alive; ++steps)
            sim.step(1.0 / 2000.0, world, sink);
        ++exercised;

        const ProjectileState& s = sim.state(h);
        const bool finite = std::isfinite(s.position.x) && std::isfinite(s.position.y) &&
                            std::isfinite(s.position.z) && std::isfinite(s.velocity.x) &&
                            std::isfinite(s.velocity.y) && std::isfinite(s.velocity.z) &&
                            std::isfinite(s.timeAlive_s);
        if (!finite || steps >= 20000) { ++bad; }
        for (const Event& e : sink.events)
            if (!std::isfinite(e.point.x) || !std::isfinite(e.time_s) ||
                !std::isfinite(e.energy_J)) { ++bad; break; }
    }
    CHECK(exercised > 2500); // most random descriptions are usable
    CHECK(bad == 0);
}

// ---------------------------------------------------------------------------
// Phase 19 item 2 — explosive warheads: Friedlander blast field, fuze wiring,
// impulse on bodies, underwater / thermobaric variants.
// ---------------------------------------------------------------------------

namespace {
WarheadDesc tnt(double kg, FuzeMode f = FuzeMode::Contact) {
    WarheadDesc w;
    w.chargeMass_kg = kg;
    w.fuze          = f;
    return w;
}
} // namespace

PON_TEST(validation_blast_kinney_graham_reference_overpressure) {
    // 1 kg TNT free-air, sea-level ISA. Kinney & Graham: ~9 kPa incident at
    // 10 m (Z = 10), rising past 1 bar inside ~1.5 m.
    Environment env;
    Burst b({0, 0, 0}, tnt(1.0), env, 0.0);

    const double p10 = b.sampleAt({10, 0, 0}).peakOverpressure_Pa;
    CHECK(p10 > 6.0e3 && p10 < 14.0e3);

    const double p2 = b.sampleAt({2, 0, 0}).peakOverpressure_Pa;
    CHECK(p2 > 5.0e4);                       // strong shock in the near field
    CHECK(p2 > p10 * 3.0);                   // falls off hard with distance

    double prev = 1e30;
    for (double r = 1.0; r <= 40.0; r += 1.0) {
        const double p = b.sampleAt({r, 0, 0}).peakOverpressure_Pa;
        CHECK(p < prev);
        prev = p;
    }
}

PON_TEST(validation_blast_cube_root_scaling_law) {
    // Same scaled distance Z = R / W^(1/3) -> same overpressure. 8 kg at 20 m
    // and 1 kg at 10 m are both Z = 10.
    Environment env;
    const double a = Burst({0,0,0}, tnt(1.0), env).sampleAt({10, 0, 0}).peakOverpressure_Pa;
    const double c = Burst({0,0,0}, tnt(8.0), env).sampleAt({20, 0, 0}).peakOverpressure_Pa;
    CHECK_NEAR(a, c, a * 0.02);
    const double ia = Burst({0,0,0}, tnt(1.0), env).sampleAt({10,0,0}).specificImpulse_Pa_s;
    const double ic = Burst({0,0,0}, tnt(8.0), env).sampleAt({20,0,0}).specificImpulse_Pa_s;
    CHECK_NEAR(ic, ia * 2.0, ia * 0.05);
}

PON_TEST(validation_blast_reflected_ratio_between_2_and_8) {
    Environment env;
    Burst b({0, 0, 0}, tnt(5.0), env);
    for (double r : {1.0, 3.0, 8.0, 20.0, 60.0}) {
        const BlastSample s = b.sampleAt({r, 0, 0});
        const double ratio = s.reflectedOverpressure_Pa / s.peakOverpressure_Pa;
        CHECK(ratio >= 1.99 && ratio <= 8.05);
    }
    const BlastSample nearS = b.sampleAt({1.5, 0, 0});
    const BlastSample farS  = b.sampleAt({50.0, 0, 0});
    CHECK(nearS.reflectedOverpressure_Pa / nearS.peakOverpressure_Pa >
          farS.reflectedOverpressure_Pa / farS.peakOverpressure_Pa);
}

PON_TEST(validation_blast_friedlander_waveform_and_impulse) {
    Environment env;
    Burst b({0, 0, 0}, tnt(2.0), env, 0.0);
    const Vec3 p{12, 0, 0};
    const BlastSample s = b.sampleAt(p);

    CHECK(b.overpressureAt(p, s.arrivalTime_s - 1e-4) == 0.0);
    CHECK_NEAR(b.overpressureAt(p, s.arrivalTime_s), s.peakOverpressure_Pa,
              s.peakOverpressure_Pa * 1e-6);
    CHECK(b.overpressureAt(p, s.arrivalTime_s + s.positiveDuration_s * 1.01) == 0.0);
    for (double t = 0; t < s.arrivalTime_s + s.positiveDuration_s * 1.5; t += 1e-4)
        CHECK(b.overpressureAt(p, t) >= 0.0);

    double integ = 0.0;
    const double dt = 1e-6;
    for (double t = s.arrivalTime_s; t < s.arrivalTime_s + s.positiveDuration_s; t += dt)
        integ += b.overpressureAt(p, t) * dt;
    CHECK_NEAR(integ, s.specificImpulse_Pa_s, s.specificImpulse_Pa_s * 0.05);
}

PON_TEST(validation_blast_underwater_shock_far_exceeds_air) {
    Environment env;
    WarheadDesc uw = tnt(5.0); uw.underwater = true;
    const double pw = Burst({0,0,0}, uw,      env).sampleAt({5, 0, 0}).peakOverpressure_Pa;
    const double pa = Burst({0,0,0}, tnt(5.0), env).sampleAt({5, 0, 0}).peakOverpressure_Pa;
    CHECK(pw > pa * 10.0);
    CHECK(pw > 1.0e7);                       // tens of MPa near field (Cole)
    const BlastSample s = Burst({0,0,0}, uw, env).sampleAt({5, 0, 0});
    CHECK(s.positiveDuration_s > 0.0 && s.positiveDuration_s < 0.02);
    CHECK(s.arrivalTime_s > 5.0 / 1500.0);   // water sound speed ~1481 m/s
}

PON_TEST(validation_blast_thermobaric_boosts_impulse_and_duration) {
    Environment env;
    WarheadDesc conv = tnt(3.0);
    WarheadDesc thermo = tnt(3.0); thermo.thermobaric = 1.5;
    const BlastSample c = Burst({0,0,0}, conv,   env).sampleAt({15, 0, 0});
    const BlastSample t = Burst({0,0,0}, thermo, env).sampleAt({15, 0, 0});
    CHECK(t.positiveDuration_s > c.positiveDuration_s * 1.5);
    CHECK(t.specificImpulse_Pa_s > c.specificImpulse_Pa_s * 1.5);
    CHECK(Burst({0,0,0}, thermo, env).effectiveCharge_kg() >
          Burst({0,0,0}, conv, env).effectiveCharge_kg());
}

PON_TEST(validation_blast_surface_burst_enhances_yield) {
    Environment env;
    WarheadDesc air = tnt(4.0);
    WarheadDesc ground = tnt(4.0); ground.surfaceBurst = true;
    const double pa = Burst({0,0,0}, air,    env).sampleAt({12, 0, 0}).peakOverpressure_Pa;
    const double pg = Burst({0,0,0}, ground, env).sampleAt({12, 0, 0}).peakOverpressure_Pa;
    CHECK(pg > pa * 1.1 && pg < pa * 1.6);
}

PON_TEST(warhead_impulse_points_away_and_scales_with_mass) {
    Environment env;
    Burst b({0, 0, 0}, tnt(10.0), env);
    BlastTarget lite; lite.centroid = {8, 0, 0}; lite.area_m2 = 1.0; lite.mass_kg = 20.0;
    BlastTarget heavy = lite; heavy.mass_kg = 200.0;

    const BlastLoad l = b.loadOnBody(lite);
    const BlastLoad h = b.loadOnBody(heavy);
    CHECK(l.impulse_Ns.x > 0.0);
    CHECK(std::fabs(l.impulse_Ns.y) < l.impulse_Ns.x * 1e-6);
    CHECK_NEAR(l.impulse_Ns.x, h.impulse_Ns.x, l.impulse_Ns.x * 1e-6);
    CHECK_NEAR(length(l.deltaVelocity_mps), length(h.deltaVelocity_mps) * 10.0,
              length(h.deltaVelocity_mps) * 0.1);
    CHECK(length(b.loadOnBody(lite, 1.0, {0, 0, 0}).torqueImpulse_Nms) == 0.0);
    CHECK(length(b.loadOnBody(lite, 1.0, {0, 0.5, 0}).torqueImpulse_Nms) > 0.0);
}

PON_TEST(warhead_line_of_sight_cuts_the_load) {
    struct Wall final : World {
        Material m = material_water();
        bool raycast(Vec3 a, Vec3 bb, HitResult& o) const override {
            if ((a.x - 5.0) * (bb.x - 5.0) < 0.0) {
                o.t = (5.0 - a.x) / (bb.x - a.x); o.point = a + (bb - a) * o.t;
                o.normal = {-1, 0, 0}; o.surface = 0; return true;
            }
            return false;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } wall;
    Environment env;
    Burst b({0, 0, 0}, tnt(6.0), env);
    const double los = blast_line_of_sight(wall, {0, 0, 0}, {10, 0, 0});
    CHECK(los > 0.0 && los < 1.0);
    const double clear = blast_line_of_sight(wall, {0, 0, 0}, {4, 0, 0});
    CHECK(clear == 1.0);

    BlastTarget t; t.centroid = {10, 0, 0}; t.area_m2 = 1.0; t.mass_kg = 40.0;
    const double exposed = length(b.loadOnBody(t, 1.0).impulse_Ns);
    const double covered = length(b.loadOnBody(t, los).impulse_Ns);
    CHECK(covered < exposed);
    CHECK_NEAR(covered, exposed * los, exposed * 1e-6);
}

PON_TEST(warhead_contact_fuze_detonates_on_impact) {
    struct Ground final : World {
        Material m = material_water();
        bool raycast(Vec3 a, Vec3 bb, HitResult& o) const override {
            if (a.y > 0.0 && bb.y <= 0.0) {
                o.t = a.y / (a.y - bb.y); o.point = a + (bb - a) * o.t;
                o.normal = {0, 1, 0}; o.surface = 0; return true;
            }
            return false;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } ground;

    Sim sim;
    ProjectileType t;
    t.id = "he"; t.klass = ProjectileClass::Shell; t.dragModel = DragModel::G1;
    t.ballisticCoefficient = 0.5; t.mass_kg = 20.0; t.refDiameter_m = 0.1;
    t.warhead = tnt(5.0);
    t.warhead.surfaceBurst = true;
    const TypeId id = sim.registerType(t);

    LaunchParams lp;
    lp.position = {0, 30, 0}; lp.direction = {1, -0.2, 0}; lp.speed = 200.0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    VectorEventSink sink;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 500.0, ground, sink);

    int detonations = 0; Event det{};
    for (const Event& e : sink.events)
        if (e.type == EventType::Detonated) { ++detonations; det = e; }
    CHECK(detonations == 1);
    CHECK_NEAR(det.point.y, 0.0, 0.2);
    CHECK(det.payload_kg > 8.0 && det.payload_kg < 10.0);   // 5 kg * 1.8 surface
    CHECK(det.energy_J > 3.0e7);
    CHECK((sim.state(h).flags & kFlagDetonated) != 0);
    CHECK(!sim.state(h).alive);
}

PON_TEST(warhead_timed_airburst_detonates_in_the_air) {
    Sim sim;
    ProjectileType t;
    t.id = "airburst"; t.klass = ProjectileClass::Shell; t.dragModel = DragModel::G1;
    t.ballisticCoefficient = 0.5; t.mass_kg = 15.0; t.refDiameter_m = 0.1;
    t.warhead = tnt(4.0, FuzeMode::TimedAirburst);
    t.warhead.fuzeDelay_s = 1.2;
    const TypeId id = sim.registerType(t);

    LaunchParams lp;
    lp.position = {0, 2, 0}; lp.direction = {1, 0.9, 0}; lp.speed = 180.0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    VectorEventSink sink;
    EmptyWorld air;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 500.0, air, sink);

    const Event* det = nullptr;
    for (const Event& e : sink.events)
        if (e.type == EventType::Detonated) det = &e;
    CHECK(det != nullptr);
    if (det) {
        CHECK_NEAR(det->time_s, 1.2, 0.05);
        CHECK(det->point.y > 20.0);
    }
}

PON_TEST(warhead_proximity_fuze_trips_near_geometry) {
    struct Target final : World {
        Material m = material_water();
        bool raycast(Vec3 a, Vec3 bb, HitResult& o) const override {
            const Vec3 c{50, 0, 0};
            const Vec3 d = bb - a;
            const double A = dot(d, d);
            if (A <= 0) return false;
            const double B = 2 * dot(a - c, d);
            const double C = dot(a - c, a - c) - 1.0;
            const double disc = B * B - 4 * A * C;
            if (disc < 0) return false;
            const double s = (-B - std::sqrt(disc)) / (2 * A);
            if (s < 0 || s > 1) return false;
            o.t = s; o.point = a + d * s; o.normal = normalized(o.point - c);
            o.surface = 0; return true;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } world;

    Sim sim;
    ProjectileType t;
    t.id = "prox"; t.klass = ProjectileClass::Shell; t.dragModel = DragModel::G1;
    t.ballisticCoefficient = 0.7; t.mass_kg = 10.0; t.refDiameter_m = 0.08;
    t.warhead = tnt(3.0, FuzeMode::Proximity);
    t.warhead.proximityRadius_m = 4.0;
    const TypeId id = sim.registerType(t);

    LaunchParams lp;
    lp.position = {0, 0, 0}; lp.direction = {1, 0, 0}; lp.speed = 300.0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);

    VectorEventSink sink;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 1000.0, world, sink);

    const Event* det = nullptr;
    for (const Event& e : sink.events)
        if (e.type == EventType::Detonated) det = &e;
    CHECK(det != nullptr);
    if (det) CHECK(det->point.x > 44.0 && det->point.x < 50.5);
}

PON_TEST(warhead_inert_round_emits_no_detonation) {
    struct Ground final : World {
        Material m = [] { Material mm; mm.name = "dirt";
            mm.behaviour = MaterialBehaviour::Granular; mm.density_kgm3 = 1600;
            mm.strength_Pa = 1e6; mm.thickness_m = 0.0; return mm; }();
        bool raycast(Vec3 a, Vec3 bb, HitResult& o) const override {
            if (a.y > 0.0 && bb.y <= 0.0) {
                o.t = a.y / (a.y - bb.y); o.point = a + (bb - a) * o.t;
                o.normal = {0, 1, 0}; o.surface = 0; return true;
            }
            return false;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } ground;

    Sim sim;
    const TypeId id = sim.registerType(catalog::get("762x51_175gr_smk"));
    LaunchParams lp;
    lp.position = {0, 2, 0}; lp.direction = {1, -0.05, 0};
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    VectorEventSink sink;
    for (int i = 0; i < 20000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 1000.0, ground, sink);
    for (const Event& e : sink.events)
        CHECK(e.type != EventType::Detonated);
    CHECK(!sink.events.empty());
}

PON_TEST(warhead_c_abi_burst) {
    pon_warhead_desc w{};
    w.charge_mass_kg = 5.0;
    w.tnt_equivalence = 1.0;
    pon_burst* b = pon_burst_create(pon_vec3{0, 0, 0}, &w, 0, 0, 0.0);
    CHECK(b != nullptr);

    pon_blast_sample s{};
    pon_burst_sample(b, pon_vec3{10, 0, 0}, &s);
    CHECK(s.peak_overpressure_pa > 1.0e4 && s.peak_overpressure_pa < 1.0e5);
    CHECK(s.reflected_overpressure_pa > s.peak_overpressure_pa);

    const double p_at_arrival =
        pon_burst_overpressure_at(b, pon_vec3{10, 0, 0}, s.arrival_time_s);
    CHECK_NEAR(p_at_arrival, s.peak_overpressure_pa, s.peak_overpressure_pa * 1e-6);

    pon_vec3 imp{}, dv{};
    pon_burst_load_on_body(b, pon_vec3{10, 0, 0}, 1.0, 40.0, 2.0, 1.0, 1.0, &imp, &dv);
    CHECK(imp.x > 0.0);
    CHECK_NEAR(dv.x, imp.x / 40.0, std::fabs(imp.x) * 1e-9);

    pon_burst_destroy(b);
}

// ===========================================================================
// Fragmentation (Phase 19 item 3, §8a) — Mott spectrum, Gurney speed, spray
// patterns, spawn-as-projectiles.
// ===========================================================================

namespace {
FragmentationDesc grenade_frag() {
    FragmentationDesc d;
    d.casingMass_kg         = 0.18;
    d.gurneyVelocity_mps    = 2700.0;   // Comp-B
    d.casingInnerDiameter_m = 0.050;
    d.casingWallThickness_m = 0.006;
    d.spray                 = FragmentSpray::Isotropic;
    d.maxFragments          = 400;
    return d;
}
} // namespace

PON_TEST(frag_mott_spectrum_is_ordered_and_conserves_the_casing) {
    const FragmentationDesc d = grenade_frag();
    const double mu = mott_mu_kg(d);
    CHECK(mu > 0.0 && mu < d.casingMass_kg);

    std::vector<FragmentSpec> f;
    const std::size_t n = generate_fragments(d, {0, 0, 0}, {0, 0, 0}, 0.06, f);
    CHECK(n == f.size());
    CHECK(n > 50 && n <= d.maxFragments);

    // Quantile sampling => strictly descending mass, heaviest first, none below
    // the dust cut, none heavier than the whole casing.
    for (std::size_t i = 1; i < f.size(); ++i) CHECK(f[i].mass_kg <= f[i - 1].mass_kg);
    CHECK(f.back().mass_kg >= d.minFragmentMass_kg);
    CHECK(f.front().mass_kg < d.casingMass_kg);

    // Each spec's represented count times its mass, summed, accounts for a good
    // fraction of the casing (the discarded light tail is the rest) and never
    // exceeds it.
    double m = 0.0;
    for (const auto& s : f) m += s.mass_kg * s.representsCount;
    CHECK(m > 0.35 * d.casingMass_kg);
    CHECK(m <= d.casingMass_kg);

    // Sphere-equivalent diameter tracks mass and density.
    for (const auto& s : f) {
        const double dia = std::cbrt(6.0 * s.mass_kg / (3.14159265358979 * d.fragmentDensity_kgm3));
        CHECK_NEAR(s.diameter_m, dia, dia * 1e-6);
    }
}

PON_TEST(frag_gurney_speed_matches_the_closed_form) {
    FragmentationDesc d = grenade_frag();
    const double M = d.casingMass_kg, C = 0.06;
    const double expect = d.gurneyVelocity_mps / std::sqrt(M / C + 0.5);
    CHECK_NEAR(gurney_fragment_velocity(d, C), expect, expect * 1e-9);

    std::vector<FragmentSpec> f;
    generate_fragments(d, {0, 0, 0}, {0, 0, 0}, C, f);
    for (const auto& s : f) {
        const double v = length(s.velocity);
        CHECK(v > expect * (1.0 - d.velocityScatter) - 1.0);
        CHECK(v < expect * (1.0 + d.velocityScatter) + 1.0);
    }
    // A thicker casing on the same charge is slower (more metal to push).
    FragmentationDesc heavy = d; heavy.casingMass_kg = 0.6;
    CHECK(gurney_fragment_velocity(heavy, C) < gurney_fragment_velocity(d, C));
}

PON_TEST(frag_spray_patterns_aim_where_asked) {
    FragmentationDesc iso = grenade_frag();
    std::vector<FragmentSpec> f;
    generate_fragments(iso, {0, 0, 0}, {0, 0, 0}, 0.06, f);
    int px = 0, nx = 0, py = 0, ny = 0, pz = 0, nz = 0;
    for (const auto& s : f) {
        const Vec3 u = normalized(s.velocity);
        (u.x > 0 ? px : nx)++; (u.y > 0 ? py : ny)++; (u.z > 0 ? pz : nz)++;
    }
    CHECK(px > 0 && nx > 0 && py > 0 && ny > 0 && pz > 0 && nz > 0); // all around

    FragmentationDesc cone = grenade_frag();
    cone.spray = FragmentSpray::Cone;
    cone.sprayAxis = {0, 0, 1};
    cone.coneHalfAngle_rad = 0.4;
    generate_fragments(cone, {0, 0, 0}, {0, 0, 0}, 0.06, f);
    for (const auto& s : f) {
        const Vec3 u = normalized(s.velocity);
        CHECK(dot(u, Vec3{0, 0, 1}) >= std::cos(0.4) - 1e-6);
    }

    FragmentationDesc beam = grenade_frag();
    beam.spray = FragmentSpray::CylinderBeam;
    beam.sprayAxis = {0, 1, 0};
    beam.beamHalfWidth_rad = 0.25;
    beam.beamForwardTilt_rad = 0.0;
    generate_fragments(beam, {0, 0, 0}, {0, 0, 0}, 0.06, f);
    for (const auto& s : f) {
        const Vec3 u = normalized(s.velocity);
        CHECK(std::fabs(dot(u, Vec3{0, 1, 0})) <= std::sin(0.25) + 1e-6); // near equator
    }
}

PON_TEST(frag_generation_is_deterministic_by_seed) {
    const FragmentationDesc d = grenade_frag();
    std::vector<FragmentSpec> a, b;
    generate_fragments(d, {1, 2, 3}, {10, 0, 0}, 0.06, a);
    generate_fragments(d, {1, 2, 3}, {10, 0, 0}, 0.06, b);
    CHECK(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].mass_kg == b[i].mass_kg);
        CHECK(a[i].velocity.x == b[i].velocity.x);
        CHECK(a[i].velocity.y == b[i].velocity.y);
        CHECK(a[i].velocity.z == b[i].velocity.z);
    }
    FragmentationDesc d2 = d; d2.seed = d.seed + 1;
    std::vector<FragmentSpec> c;
    generate_fragments(d2, {1, 2, 3}, {10, 0, 0}, 0.06, c);
    bool anyDiff = false;
    for (std::size_t i = 0; i < c.size() && i < a.size(); ++i)
        if (c[i].velocity.z != a[i].velocity.z) anyDiff = true;
    CHECK(anyDiff); // directions reseed
}

PON_TEST(frag_spawns_projectiles_that_fly_and_bite) {
    struct Plate final : World {
        Material m = [] {
            Material mm; mm.name = "sheet"; mm.behaviour = MaterialBehaviour::Ductile;
            mm.density_kgm3 = 7850; mm.strength_Pa = 4.0e7; mm.thickness_m = 0.002;
            return mm;
        }();
        bool raycast(Vec3 a, Vec3 b, HitResult& o) const override {
            if ((a.x - 12.0) * (b.x - 12.0) < 0.0) {
                o.t = (12.0 - a.x) / (b.x - a.x); o.point = a + (b - a) * o.t;
                o.normal = {-1, 0, 0}; o.surface = 1; return true;
            }
            return false;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } plate;

    Sim sim;
    FragmentationDesc d = grenade_frag();
    d.spray = FragmentSpray::Cone;
    d.sprayAxis = {1, 0, 0};
    d.coneHalfAngle_rad = 0.25;
    d.maxFragments = 120;

    std::vector<FragmentSpec> spray;
    const std::size_t nf = generate_fragments(d, {0, 0, 0}, {0, 0, 0}, 0.06, spray);

    std::vector<StateId> ids;
    const std::size_t spawned =
        spawn_fragments(sim, spray, FidelityTier::Integrated, "f", 8, &ids);
    CHECK(spawned == nf);
    CHECK(ids.size() == spawned);

    VectorEventSink sink;
    for (int i = 0; i < 2000; ++i) {
        std::size_t live = 0;
        for (StateId h : ids) if (sim.state(h).alive) ++live;
        if (!live) break;
        sim.step(1.0 / 1000.0, plate, sink);
    }
    int perforated = 0, hits = 0;
    for (const Event& e : sink.events) {
        if (e.type == EventType::Perforated) ++perforated, ++hits;
        else if (e.type == EventType::Embedded || e.type == EventType::Stopped) ++hits;
    }
    CHECK(hits > static_cast<int>(nf) / 2);   // a tight cone at a wall 12 m off
    CHECK(perforated > 0);                    // fragments punch a 2 mm sheet
}

PON_TEST(frag_c_abi_generate_fills_the_buffer) {
    pon_fragmentation_desc d{};
    d.casing_mass_kg = 0.2;
    d.gurney_velocity_mps = 2400.0;
    d.casing_inner_diameter_m = 0.05;
    d.casing_wall_thickness_m = 0.006;
    d.spray = PON_FRAG_CONE;
    d.spray_axis = pon_vec3{0, 0, 1};
    d.max_fragments = 64;

    pon_fragment_spec out[64];
    const size_t n = pon_generate_fragments(&d, pon_vec3{0, 0, 0}, pon_vec3{0, 0, 0},
                                            0.07, out, 64);
    CHECK(n > 10 && n <= 64);
    for (size_t i = 0; i < n; ++i) {
        CHECK(out[i].mass_kg > 0.0);
        const double v = std::sqrt(out[i].velocity.x * out[i].velocity.x +
                                   out[i].velocity.y * out[i].velocity.y +
                                   out[i].velocity.z * out[i].velocity.z);
        CHECK(v > 500.0 && v < 3000.0);
    }
}

PON_TEST(validation_frag_energy_stays_within_the_gurney_budget) {
    // Total fragment kinetic energy must not exceed the chemical energy that
    // drove it (Gurney puts ~2/3 of the charge energy into the metal; we only
    // check the hard ceiling — spawned KE < charge chemical energy).
    FragmentationDesc d = grenade_frag();
    const double C = 0.06;                       // 60 g Comp-B
    const double chem = C * 1.1 * 4.184e6;       // ~2.76e5 J

    std::vector<FragmentSpec> f;
    generate_fragments(d, {0, 0, 0}, {0, 0, 0}, C, f);
    double ke = 0.0;
    for (const auto& s : f) {
        const double v = length(s.velocity);
        ke += 0.5 * s.mass_kg * s.representsCount * v * v;
    }
    CHECK(ke > 0.05 * chem);   // it did real work
    CHECK(ke < chem);          // but not more than the charge carried
}

PON_TEST(validation_frag_fuzz_random_casings_stay_finite) {
    Rng rng(0xF7A6);
    for (int i = 0; i < 400; ++i) {
        FragmentationDesc d;
        d.casingMass_kg         = rng.uniform(0.02, 40.0);
        d.gurneyVelocity_mps    = rng.uniform(1500.0, 3000.0);
        d.casingInnerDiameter_m = rng.uniform(0.02, 0.15);
        d.casingWallThickness_m = rng.uniform(0.002, 0.03);
        d.spray                 = static_cast<FragmentSpray>(rng.range(3));
        d.sprayAxis             = {rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)};
        d.maxFragments          = 1u + static_cast<std::uint32_t>(rng.range(300));
        const double C          = rng.uniform(0.01, 10.0);

        std::vector<FragmentSpec> f;
        const std::size_t n = generate_fragments(d, {0, 0, 0}, {0, 0, 0}, C, f);
        CHECK(n <= d.maxFragments);
        for (const auto& s : f) {
            CHECK(std::isfinite(s.mass_kg) && s.mass_kg > 0.0);
            CHECK(std::isfinite(s.diameter_m) && s.diameter_m > 0.0);
            CHECK(std::isfinite(s.velocity.x) && std::isfinite(s.velocity.y) &&
                  std::isfinite(s.velocity.z));
            CHECK(length(s.velocity) < 6000.0);
        }
    }
}

// ===========================================================================
// Shaped charges / EFP (Phase 19 item 4, §8a) — Birkhoff/PER jet formation,
// standoff-dependent penetration, back-face spall.
// ===========================================================================

namespace {
ShapedChargeDesc heat100() {
    ShapedChargeDesc d;
    d.linerMass_kg       = 0.35;
    d.chargeDiameter_m   = 0.100;
    d.kind               = ShapedChargeType::ConicalJet;
    d.coneApexAngle_rad  = 60.0 * 3.14159265358979 / 180.0;
    d.gurneyVelocity_mps = 2930.0;
    return d;
}
Material rha_plate(double thickness_m = 0.06) {
    Material m;
    m.name = "RHA"; m.behaviour = MaterialBehaviour::Ductile;
    m.density_kgm3 = 7850.0; m.strength_Pa = 1.0e9; m.thickness_m = thickness_m;
    return m;
}
const double kFill = 0.6; // explosive fill behind the liner
} // namespace

PON_TEST(shaped_charge_jet_forms_at_hypervelocity) {
    const JetFormation f = shaped_charge_formation(heat100(), kFill);
    CHECK(f.tipVelocity_mps > 6000.0 && f.tipVelocity_mps < 11000.0);
    CHECK(f.tailVelocity_mps > 1000.0 && f.tailVelocity_mps < f.tipVelocity_mps);
    CHECK(f.jetMass_kg > 0.0 && f.jetMass_kg < 0.35);
    CHECK(f.slugMass_kg > f.jetMass_kg);                 // most of the liner slugs
    CHECK_NEAR(f.jetMass_kg + f.slugMass_kg, 0.35, 1e-9);
    CHECK(f.coherentLength_m > f.initialLength_m);       // it stretches
    CHECK(f.breakupStandoff_m > 2.0 * 0.100);            // at least a couple CD
    CHECK(!f.isEFP);
}

PON_TEST(shaped_charge_penetration_peaks_at_a_few_CD) {
    const ShapedChargeDesc d = heat100();
    const Material rha = rha_plate();
    const double CD = 0.100;

    double bestP = 0, bestS = 0;
    for (double s = 0.02; s <= 2.0; s += 0.02) {
        const auto p = shaped_charge_penetration(d, rha, s, -1.0, kFill);
        CHECK(p.depth_m >= 0.0 && std::isfinite(p.depth_m));
        if (p.depth_m > bestP) { bestP = p.depth_m; bestS = s; }
    }
    // A HEAT round drills several charge diameters of steel...
    CHECK(bestP > 4.0 * CD && bestP < 9.0 * CD);
    // ...best at a standoff of a couple to a few CD.
    CHECK(bestS > 1.5 * CD && bestS < 7.0 * CD);

    // Contact (near-zero standoff): the jet has not stretched — shallower.
    const double contact = shaped_charge_penetration(d, rha, 0.01, -1.0, kFill).depth_m;
    CHECK(contact < bestP * 0.85);
    // Far past jet breakup: dispersion has cut it down.
    const double far = shaped_charge_penetration(d, rha, 4.0, -1.0, kFill).depth_m;
    CHECK(far < bestP * 0.7);

    const auto opt = shaped_charge_penetration(d, rha, shaped_charge_optimal_standoff(d, kFill),
                                               -1.0, kFill);
    CHECK(opt.standoffEfficiency > 0.9);
}

PON_TEST(shaped_charge_perforates_plate_and_spalls) {
    const ShapedChargeDesc d = heat100();
    const double S = shaped_charge_optimal_standoff(d, kFill);

    const auto thin = shaped_charge_penetration(d, rha_plate(0.06), S, 0.06, kFill);
    CHECK(thin.perforated);
    CHECK(thin.spall);                       // a perforation scabs the exit
    CHECK(thin.residualVelocity_mps > 0.0);
    CHECK(thin.residualLength_m > 0.0);
    CHECK(thin.holeDiameter_m > 0.0);

    // Thick enough to stop the jet: no through hole. Pick a thickness above the
    // semi-infinite depth.
    const double deep = shaped_charge_penetration(d, rha_plate(1.0), S, -1.0, kFill).depth_m;
    const auto stopped = shaped_charge_penetration(d, rha_plate(deep + 0.2), S,
                                                   deep + 0.2, kFill);
    CHECK(!stopped.perforated);

    // A shade thicker than the jet reaches → back-face spall, no through hole.
    const double scabT = deep * 1.15;
    const auto scab = shaped_charge_penetration(d, rha_plate(scabT), S, scabT, kFill);
    CHECK(!scab.perforated);
    CHECK(scab.spall);
}

PON_TEST(shaped_charge_efp_is_shallow_but_long_ranged) {
    ShapedChargeDesc efp = heat100();
    efp.kind = ShapedChargeType::EFP;
    const Material rha = rha_plate();

    const JetFormation f = shaped_charge_formation(efp, kFill);
    CHECK(f.isEFP);
    CHECK(f.tipVelocity_mps > 1200.0 && f.tipVelocity_mps < 3000.0);
    CHECK(f.tailVelocity_mps == f.tipVelocity_mps);      // no velocity gradient

    const double heatP = shaped_charge_penetration(heat100(), rha,
                             shaped_charge_optimal_standoff(heat100(), kFill), -1.0, kFill).depth_m;
    const double efpNear = shaped_charge_penetration(efp, rha, 1.0, -1.0, kFill).depth_m;
    CHECK(efpNear > 0.02);
    CHECK(efpNear < heatP);                              // much shallower than a jet

    // Still effective at 50 CD, where a HEAT jet is long dead.
    const auto efpFar = shaped_charge_penetration(efp, rha, 50.0 * 0.100, -1.0, kFill);
    CHECK(efpFar.standoffEfficiency > 0.5);
    const double heatFar = shaped_charge_penetration(heat100(), rha, 5.0, -1.0, kFill).depth_m;
    CHECK(heatFar < heatP * 0.3);
}

PON_TEST(shaped_charge_softer_target_lets_the_jet_deeper) {
    const ShapedChargeDesc d = heat100();
    const double S = shaped_charge_optimal_standoff(d, kFill);

    Material concrete;
    concrete.behaviour = MaterialBehaviour::Brittle;
    concrete.density_kgm3 = 2400.0; concrete.strength_Pa = 4.0e7;

    const double inSteel    = shaped_charge_penetration(d, rha_plate(), S, -1.0, kFill).depth_m;
    const double inConcrete = shaped_charge_penetration(d, concrete, S, -1.0, kFill).depth_m;
    CHECK(inConcrete > inSteel * 1.5);   // lower density ⇒ the jet goes much deeper
}

PON_TEST(shaped_charge_behind_armour_spawns_flying_debris) {
    const ShapedChargeDesc d = heat100();
    const double S = shaped_charge_optimal_standoff(d, kFill);

    std::vector<FragmentSpec> debris;
    const std::size_t n = shaped_charge_behind_armour(
        d, rha_plate(0.06), {0, 0, 0}, {1, 0, 0}, S, 0.06, kFill, debris);
    CHECK(n > 5 && n == debris.size());
    for (const auto& f : debris) {
        CHECK(std::isfinite(f.mass_kg) && f.mass_kg > 0.0);
        CHECK(length(f.velocity) > 20.0 && length(f.velocity) < 12000.0);
        CHECK(normalized(f.velocity).x > 0.0);           // all heading downrange
    }

    Sim sim;
    std::vector<StateId> ids;
    const std::size_t spawned =
        spawn_fragments(sim, debris, FidelityTier::Integrated, "bah", 6, &ids);
    CHECK(spawned == n);

    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 200; ++i) sim.step(1.0 / 1000.0, world, sink);
    double reach = 0;
    for (StateId h : ids) reach = std::max(reach, sim.state(h).position.x);
    CHECK(reach > 1.0);                                  // debris crosses the compartment
}

PON_TEST(shaped_charge_detonation_event_carries_the_jet_axis) {
    struct Wall final : World {
        Material m = [] { Material mm; mm.name="w"; mm.behaviour=MaterialBehaviour::Ductile;
                          mm.density_kgm3=7850; mm.strength_Pa=5e8; mm.thickness_m=0.05; return mm; }();
        bool raycast(Vec3 a, Vec3 b, HitResult& o) const override {
            if ((a.x - 20.0) * (b.x - 20.0) < 0.0) {
                o.t = (20.0 - a.x) / (b.x - a.x); o.point = a + (b - a) * o.t;
                o.normal = {-1, 0, 0}; o.surface = 1; return true;
            }
            return false;
        }
        MediumId mediumAt(Vec3) const override { return kMediumAir; }
        const Material& material(SurfaceId) const override { return m; }
    } wall;

    Sim sim;
    ProjectileType heat;
    heat.id = "rpg"; heat.klass = ProjectileClass::Shell; heat.dragModel = DragModel::G1;
    heat.ballisticCoefficient = 0.2; heat.mass_kg = 2.5; heat.refDiameter_m = 0.1;
    heat.warhead.chargeMass_kg = 0.6; heat.warhead.fuze = FuzeMode::Contact;
    heat.warhead.shapedCharge = heat100();
    const TypeId ty = sim.registerType(heat);

    LaunchParams lp;
    lp.position = {0, 0, 0}; lp.direction = {1, 0, 0}; lp.speed = 300.0;
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(ty, lp);

    VectorEventSink sink;
    for (int i = 0; i < 5000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 1000.0, wall, sink);

    int det = 0;
    for (const Event& e : sink.events) {
        if (e.type != EventType::Detonated) continue;
        ++det;
        CHECK(length(e.channelAxis) > 0.9);              // unit-ish
        CHECK(e.channelAxis.x > 0.9);                    // aimed the way it flew
    }
    CHECK(det == 1);
}

PON_TEST(shaped_charge_c_abi) {
    pon_shaped_charge_desc d{};
    d.liner_mass_kg = 0.35;
    d.charge_diameter_m = 0.10;
    d.kind = PON_SHAPED_CONICAL_JET;
    d.gurney_velocity_mps = 2930.0;

    pon_jet_formation jf{};
    pon_shaped_charge_formation(&d, 0.6, &jf);
    CHECK(jf.tip_velocity_mps > 6000.0 && jf.tip_velocity_mps < 11000.0);
    CHECK(jf.is_efp == 0);

    const double S = pon_shaped_charge_optimal_standoff(&d, 0.6);
    CHECK(S > 0.15 && S < 0.7);

    pon_shaped_charge_penetration p{};
    // behaviour 1 = Ductile
    pon_shaped_charge_penetrate(&d, 7850.0, 1.0e9, 1, S, 0.06, 0.6, &p);
    CHECK(p.depth_m > 0.4 && p.depth_m < 0.9);
    CHECK(p.perforated == 1);
    CHECK(p.residual_velocity_mps > 0.0);
}

PON_TEST(validation_shaped_charge_hydrodynamic_limit) {
    // Against a semi-infinite target of the jet's own density and negligible
    // strength, the depth must approach the ideal hydrodynamic limit
    // P = L_eff·√(ρj/ρt) = L_eff.
    ShapedChargeDesc d = heat100();
    Material soft;
    soft.behaviour = MaterialBehaviour::Fluid;    // no strength cut
    soft.density_kgm3 = d.linerDensity_kgm3;

    const double S = 0.3;
    const auto p = shaped_charge_penetration(d, soft, S, -1.0, kFill);
    CHECK_NEAR(p.depth_m, p.effectiveJetLength_m, p.effectiveJetLength_m * 0.02);
    CHECK(p.effectiveJetLength_m > 0.1);
}

PON_TEST(validation_shaped_charge_fuzz_stays_finite) {
    Rng rng(0x5CFA);
    for (int i = 0; i < 400; ++i) {
        ShapedChargeDesc d;
        d.linerMass_kg      = rng.uniform(0.02, 5.0);
        d.chargeDiameter_m  = rng.uniform(0.02, 0.30);
        d.kind              = rng.range(2) ? ShapedChargeType::EFP : ShapedChargeType::ConicalJet;
        d.coneApexAngle_rad = rng.uniform(0.5, 2.4);
        d.gurneyVelocity_mps = rng.uniform(1800.0, 3200.0);
        const double C = rng.uniform(0.02, 8.0);

        Material t;
        t.behaviour = static_cast<MaterialBehaviour>(rng.range(6));
        t.density_kgm3 = rng.uniform(500.0, 9000.0);
        t.strength_Pa  = rng.uniform(1.0e6, 2.0e9);

        for (double S : {0.0, 0.05, 0.5, 3.0, 40.0}) {
            const auto p = shaped_charge_penetration(d, t, S, rng.uniform(0.01, 0.5), C);
            CHECK(std::isfinite(p.depth_m) && p.depth_m >= 0.0);
            CHECK(std::isfinite(p.residualVelocity_mps) && p.residualVelocity_mps >= 0.0);
            CHECK(std::isfinite(p.standoffEfficiency));
            CHECK(p.effectiveJetLength_m >= 0.0 && std::isfinite(p.effectiveJetLength_m));
        }
        const JetFormation f = shaped_charge_formation(d, C);
        CHECK(std::isfinite(f.tipVelocity_mps) && f.tipVelocity_mps > 0.0);
        CHECK(f.jetMass_kg >= 0.0 && f.jetMass_kg <= d.linerMass_kg + 1e-9);
    }
}

// ===========================================================================
// Guided munitions (Phase 19 item 5, §8a) — pursuit / proportional navigation
// over the per-step external-acceleration hook.
// ===========================================================================

namespace {

constexpr double kG0 = 9.80665;

ProjectileType missile(const char* id, GuidanceLaw law) {
    ProjectileType t;
    t.id            = id;
    t.klass         = ProjectileClass::Shell;
    t.dragModel     = DragModel::ConstantCd;
    t.dragCoefficient = 0.20;
    t.mass_kg       = 20.0;
    t.refDiameter_m = 0.13;
    t.maxLifetime_s = 30.0;
    t.guidance.law               = law;
    t.guidance.navConstant       = 4.0;
    t.guidance.maxLateralAccel_g = 40.0;
    t.guidance.thrustAccel_mps2  = 200.0;   // a sustainer so the airframe keeps energy
    t.guidance.burnTime_s        = 3.0;
    return t;
}

// Fly `m` to closest approach against a target moving at constant velocity.
double run_intercept(Sim& sim, StateId m, GuidanceLaw law, Vec3 tgt0, Vec3 tgtVel,
                     double dt = 1.0 / 200.0, int maxSteps = 6000) {
    EmptyWorld world; VectorEventSink sink;
    double best = 1e30;
    Vec3 tgt = tgt0;
    for (int i = 0; i < maxSteps && sim.state(m).alive; ++i) {
        tgt = tgt + tgtVel * dt;
        if (law != GuidanceLaw::None) sim.guide(m, tgt, tgtVel);
        sim.step(dt, world, sink);
        const double r = length(sim.state(m).position - tgt);
        if (r < best) best = r;
        if (r < 2.0 || (best < 1e29 && r > best + 100.0)) break;
    }
    return best;
}

} // namespace

PON_TEST(guidance_pursuit_reaches_a_static_target) {
    Sim sim;
    const TypeId ty = sim.registerType(missile("pur", GuidanceLaw::Pursuit));
    LaunchParams lp;
    lp.position = {0, 0, 0}; lp.direction = {1, 0.3, 0}; lp.speed = 400.0;
    lp.tier = FidelityTier::Integrated;
    const StateId m = sim.spawn(ty, lp);
    const double miss = run_intercept(sim, m, GuidanceLaw::Pursuit, {900, 120, 0}, {0, 0, 0});
    CHECK(miss < 5.0);
}

PON_TEST(guidance_pn_intercepts_a_crossing_target_pursuit_does_not) {
    const Vec3 tgt0{1400, 500, -350}, tgtVel{0, 0, 240};

    Sim a; const StateId ma = a.spawn(a.registerType(missile("pn", GuidanceLaw::ProportionalNav)),
        [] { LaunchParams l; l.position={0,0,0}; l.direction={0.6,0.8,0}; l.speed=380; l.tier=FidelityTier::Integrated; return l; }());
    const double missPN = run_intercept(a, ma, GuidanceLaw::ProportionalNav, tgt0, tgtVel);

    Sim b; const StateId mb = b.spawn(b.registerType(missile("pur", GuidanceLaw::Pursuit)),
        [] { LaunchParams l; l.position={0,0,0}; l.direction={0.6,0.8,0}; l.speed=380; l.tier=FidelityTier::Integrated; return l; }());
    const double missPur = run_intercept(b, mb, GuidanceLaw::Pursuit, tgt0, tgtVel);

    CHECK(missPN < 12.0);          // PN converges on a crosser
    CHECK(missPN < missPur * 0.5); // and clearly beats a tail chase
}

PON_TEST(guidance_command_respects_the_g_limit) {
    GuidanceDesc d; d.law = GuidanceLaw::ProportionalNav;
    d.navConstant = 5.0; d.maxLateralAccel_g = 25.0;
    GuidanceState gs; gs.hasTarget = true;
    // Target crossing hard and close ⇒ a huge raw PN command, must be clamped.
    gs.targetPos = {50, 0, 0}; gs.targetVel = {0, 0, 900};
    const GuidanceCommand c = compute_guidance(d, gs, {0, 0, 0}, {600, 0, 0}, 0.5, 1.0 / 200.0);
    CHECK(c.active);
    CHECK_NEAR(c.lateralAccel_mps2, 25.0 * kG0, 25.0 * kG0 * 1e-6);
    CHECK(std::fabs(dot(normalized(c.accel_mps2), normalized(Vec3{600, 0, 0}))) < 1e-6); // ⟂ v
}

PON_TEST(guidance_loses_lock_outside_the_seeker_fov) {
    GuidanceDesc d; d.law = GuidanceLaw::ProportionalNav; d.seekerHalfFov_rad = 0.5;
    GuidanceState gs; gs.hasTarget = true;
    gs.targetPos = {-100, 0, 0}; gs.targetVel = {0, 0, 0};   // dead astern
    const GuidanceCommand c = compute_guidance(d, gs, {0, 0, 0}, {300, 0, 0}, 1.0, 1.0 / 200.0);
    CHECK(gs.lockLost);
    CHECK(!c.active);
    CHECK(length(c.accel_mps2) < 1e-9);
    // Latched: even a target back in view stays lost.
    gs.targetPos = {500, 0, 0};
    const GuidanceCommand c2 = compute_guidance(d, gs, {0, 0, 0}, {300, 0, 0}, 1.02, 1.0 / 200.0);
    CHECK(!c2.active);
}

PON_TEST(guidance_activation_delay_coasts_first) {
    Sim sim;
    ProjectileType t = missile("delay", GuidanceLaw::ProportionalNav);
    t.guidance.activationDelay_s = 1.0;
    const TypeId ty = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0.1, 0}; lp.speed = 400.0;
    lp.tier = FidelityTier::Integrated;
    const StateId m = sim.spawn(ty, lp);

    EmptyWorld world; VectorEventSink sink;
    const Vec3 tgt{3000, 0, 900};
    Vec3 vCoast{}, vGuided{};
    for (int i = 0; i < 700; ++i) {
        sim.guide(m, tgt, {0, 0, 0});
        const double before = sim.state(m).timeAlive_s;
        sim.step(1.0 / 200.0, world, sink);
        const double ta = sim.state(m).timeAlive_s;
        if (before < 0.9 && ta >= 0.9) vCoast  = sim.state(m).velocity;
        if (before < 2.5 && ta >= 2.5) vGuided = sim.state(m).velocity;
    }
    // Before activation (t < 1 s) the z-velocity toward the offset target stays
    // ~0; well after, guidance has pulled the missile hard toward +z.
    CHECK(std::fabs(vCoast.z) < 5.0);
    CHECK(vGuided.z > 40.0);
}

PON_TEST(guidance_thrust_boosts_then_burns_out) {
    Sim sim;
    ProjectileType t = missile("boost", GuidanceLaw::Pursuit);
    t.guidance.thrustAccel_mps2 = 300.0;
    t.guidance.burnTime_s       = 1.5;
    const TypeId ty = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0.05, 0}; lp.speed = 200.0;
    lp.tier = FidelityTier::Integrated;
    const StateId m = sim.spawn(ty, lp);

    EmptyWorld world; VectorEventSink sink;
    double vAtBurnout = 0, vLater = 0;
    for (int i = 0; i < 1000; ++i) {
        sim.guide(m, {5000, 200, 0}, {0, 0, 0});
        sim.step(1.0 / 200.0, world, sink);
        const double ta = sim.state(m).timeAlive_s;
        if (vAtBurnout == 0.0 && ta >= 1.5) vAtBurnout = length(sim.state(m).velocity);
        if (ta >= 3.0) { vLater = length(sim.state(m).velocity); break; }
    }
    CHECK(vAtBurnout > 200.0 + 300.0);   // the sustainer added real speed
    CHECK(vLater < vAtBurnout);           // and after burnout drag bleeds it
}

PON_TEST(guidance_external_accel_hook_bends_an_unguided_shot) {
    Sim sim;
    const TypeId ty = sim.registerType(missile("hook", GuidanceLaw::None));
    auto fly = [&](Vec3 extAccel) {
        LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0}; lp.speed = 300.0;
        lp.tier = FidelityTier::Integrated;
        const StateId m = sim.spawn(ty, lp);
        if (length_sq(extAccel) > 0) sim.setExternalAccel(m, extAccel);
        EmptyWorld world; VectorEventSink sink;
        for (int i = 0; i < 200; ++i) sim.step(1.0 / 200.0, world, sink);
        return sim.state(m).position;
    };
    const Vec3 straight = fly({0, 0, 0});
    const Vec3 pushed   = fly({0, 0, 40.0});   // 40 m/s² sideways for 1 s
    CHECK(std::fabs(straight.z) < 1e-6);
    CHECK(pushed.z > 10.0);                    // ~½·40·1² ≈ 20 m
}

PON_TEST(guidance_promotes_a_hitscan_launch_to_integrated) {
    Sim sim;
    const TypeId ty = sim.registerType(missile("promote", GuidanceLaw::Pursuit));
    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {1, 0, 0}; lp.speed = 300.0;
    lp.tier = FidelityTier::Hitscan;
    const StateId m = sim.spawn(ty, lp);
    CHECK(sim.state(m).tier == FidelityTier::Integrated);
}

PON_TEST(guidance_c_abi) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    CHECK(sim != nullptr);

    pon_guidance_desc g{};
    g.law = PON_GUIDANCE_PROPORTIONAL_NAV;
    g.nav_constant = 4.0;
    g.max_lateral_accel_g = 40.0;
    g.thrust_accel_mps2 = 200.0;
    g.burn_time_s = 3.0;

    pon_projectile_desc d{};
    d.id = "sam_c";
    d.klass = 7; // Shell
    d.drag_model = PON_DRAG_CONSTANT_CD;
    d.drag_coefficient = 0.2;
    d.mass_kg = 20.0;
    d.ref_diameter_m = 0.13;
    d.guidance = &g;
    const uint32_t ty = pon_register_type(sim, &d);
    CHECK(ty != 0xFFFFFFFFu);

    const uint32_t m = pon_spawn_precise(sim, ty, pon_vec3{0, 0, 0}, pon_vec3{0.6, 0.8, 0},
                                         380.0, 2 /*Integrated*/, 0);
    CHECK(m != 0xFFFFFFFFu);

    pon_vec3 tgt{1400, 500, -350};
    const pon_vec3 tvel{0, 0, 240};
    double best = 1e30;
    for (int i = 0; i < 4000; ++i) {
        tgt.x += tvel.x / 200.0; tgt.y += tvel.y / 200.0; tgt.z += tvel.z / 200.0;
        pon_guide(sim, m, tgt, tvel);
        pon_step(sim, 1.0 / 200.0);
        pon_state st{};
        if (pon_get_state(sim, m, &st) != PON_OK || !st.alive) break;
        const double r = std::sqrt(std::pow(st.position.x - tgt.x, 2) +
                                   std::pow(st.position.y - tgt.y, 2) +
                                   std::pow(st.position.z - tgt.z, 2));
        if (r < best) best = r;
    }
    CHECK(best < 20.0);

    // Raw hook + clear.
    CHECK(pon_set_external_accel(sim, m, pon_vec3{0, 0, 0}) == PON_OK);
    CHECK(pon_clear_guidance(sim, m) == PON_OK);
    CHECK(pon_guide(sim, 999999u, tgt, tvel) == PON_INVALID_HANDLE);
    pon_sim_destroy(sim);
}

PON_TEST(validation_guidance_pn_beats_pursuit_across_geometries) {
    // Over a spread of crossing speeds and aspect angles, PN's miss distance is
    // always well under pure pursuit's, and PN always converges to a hit.
    const double speeds[] = {120, 200, 300};
    const double offs[]   = {-500, -200, 200, 500};
    for (double sp : speeds) {
        for (double off : offs) {
            const Vec3 tgt0{1300, 450, off};
            const Vec3 tv{0, 0, sp};
            Sim a; const StateId ma = a.spawn(
                a.registerType(missile("pn", GuidanceLaw::ProportionalNav)),
                [] { LaunchParams l; l.position={0,0,0}; l.direction={0.6,0.8,0}; l.speed=400; l.tier=FidelityTier::Integrated; return l; }());
            const double mPN = run_intercept(a, ma, GuidanceLaw::ProportionalNav, tgt0, tv);
            Sim b; const StateId mb = b.spawn(
                b.registerType(missile("pur", GuidanceLaw::Pursuit)),
                [] { LaunchParams l; l.position={0,0,0}; l.direction={0.6,0.8,0}; l.speed=400; l.tier=FidelityTier::Integrated; return l; }());
            const double mPur = run_intercept(b, mb, GuidanceLaw::Pursuit, tgt0, tv);
            CHECK(mPN < 15.0);
            CHECK(mPN < mPur);
        }
    }
}

PON_TEST(validation_guidance_fuzz_stays_finite) {
    Rng rng(0x6117D);
    for (int i = 0; i < 300; ++i) {
        Sim sim;
        ProjectileType t = missile("fz", static_cast<GuidanceLaw>(1 + rng.range(3)));
        t.guidance.navConstant       = rng.uniform(2.0, 6.0);
        t.guidance.maxLateralAccel_g = rng.uniform(5.0, 60.0);
        t.guidance.thrustAccel_mps2  = rng.range(2) ? rng.uniform(0.0, 400.0) : 0.0;
        t.guidance.burnTime_s        = rng.uniform(0.0, 3.0);
        t.guidance.activationDelay_s = rng.uniform(0.0, 1.0);
        t.guidance.inducedDragFactor = rng.range(2) ? rng.uniform(0.0, 2.0) : 0.0;
        const TypeId ty = sim.registerType(t);
        LaunchParams lp;
        lp.position = {0, 0, 0};
        lp.direction = {rng.uniform(-1, 1), rng.uniform(0.1, 1.0), rng.uniform(-1, 1)};
        lp.speed = rng.uniform(150.0, 600.0);
        lp.tier = FidelityTier::Integrated;
        const StateId m = sim.spawn(ty, lp);

        const Vec3 tgt0{rng.uniform(200, 2500), rng.uniform(-200, 1200), rng.uniform(-1500, 1500)};
        const Vec3 tv{rng.uniform(-300, 300), rng.uniform(-100, 100), rng.uniform(-300, 300)};
        EmptyWorld world; VectorEventSink sink;
        Vec3 tgt = tgt0;
        for (int k = 0; k < 1200 && sim.state(m).alive; ++k) {
            tgt = tgt + tv * (1.0 / 120.0);
            sim.guide(m, tgt, tv);
            sim.step(1.0 / 120.0, world, sink);
            const ProjectileState& s = sim.state(m);
            CHECK(std::isfinite(s.position.x) && std::isfinite(s.position.y) &&
                  std::isfinite(s.position.z));
            CHECK(std::isfinite(s.velocity.x));
            CHECK(length(s.velocity) < 6000.0);   // no runaway
        }
    }
}

// --- Part B2: Q32.32 fixed-point primitive ---------------------------------

PON_TEST(fx32_int_and_double_roundtrip) {
    using pon::detail::Fx32;
    for (std::int64_t v : {0LL, 1LL, -1LL, 7LL, -7LL, 1000LL, -1000LL, 2000000000LL}) {
        CHECK(Fx32::from_int(v).to_int() == v);
        CHECK(Fx32::from_int(v).to_double() == double(v));
    }
    // Double roundtrip is exact to the 2^-32 grid.
    for (double d : {0.0, 0.5, -0.5, 3.14159265, -2.71828, 1234.567, -9999.001}) {
        const double back = Fx32::from_double(d).to_double();
        CHECK(std::fabs(back - d) <= 2.4e-10);
    }
}

PON_TEST(fx32_add_sub_negate) {
    using pon::detail::Fx32;
    const Fx32 a = Fx32::from_double(123.25);
    const Fx32 b = Fx32::from_double(-40.5);
    CHECK((a + b).to_double() == 82.75);
    CHECK((a - b).to_double() == 163.75);
    CHECK((-a).to_double() == -123.25);
    CHECK((a + (-a)) == Fx32::from_int(0));
    CHECK((b - b).raw == 0);
}

PON_TEST(fx32_multiply_matches_double_within_resolution) {
    using pon::detail::Fx32;
    struct P { double x, y; };
    for (P p : {P{2.0, 3.0}, P{-2.0, 3.0}, P{2.0, -3.0}, P{-2.0, -3.0},
                P{0.25, 0.25}, P{1000.0, 1000.0}, P{-1234.5, 6.78},
                P{31000.0, 31000.0} /* ~9.6e8, near the top of the range */}) {
        const Fx32 r = Fx32::from_double(p.x) * Fx32::from_double(p.y);
        const double want = p.x * p.y;
        // Error bound: each operand rounds to 2^-32, so the product error is
        // ~|x|·2^-32 + |y|·2^-32 plus one 2^-32 truncation.
        const double tol = (std::fabs(p.x) + std::fabs(p.y) + 1.0) * 2.4e-10;
        CHECK(std::fabs(r.to_double() - want) <= tol);
    }
    // Exact on integers.
    CHECK((Fx32::from_int(6) * Fx32::from_int(7)) == Fx32::from_int(42));
    CHECK((Fx32::from_int(-6) * Fx32::from_int(7)) == Fx32::from_int(-42));
}

PON_TEST(fx32_divide_matches_double_and_inverts_multiply) {
    using pon::detail::Fx32;
    struct P { double x, y; };
    for (P p : {P{6.0, 3.0}, P{-6.0, 3.0}, P{1.0, 3.0}, P{355.0, 113.0},
                P{1234.5, -6.78}, P{1.0, 1024.0}, P{9.0e8, 3.0e4}}) {
        const Fx32 q = Fx32::from_double(p.x) / Fx32::from_double(p.y);
        const double want = p.x / p.y;
        const double tol = (std::fabs(want) + 1.0) * 3.0e-9;
        CHECK(std::fabs(q.to_double() - want) <= tol);
    }
    CHECK((Fx32::from_int(42) / Fx32::from_int(7)) == Fx32::from_int(6));
    // (a * b) / b ≈ a
    const Fx32 a = Fx32::from_double(17.3), b = Fx32::from_double(4.25);
    CHECK(std::fabs(((a * b) / b).to_double() - 17.3) <= 1e-8);
}

PON_TEST(fx32_ordering_and_abs) {
    using pon::detail::Fx32;
    CHECK(Fx32::from_double(-1.0) < Fx32::from_double(1.0));
    CHECK(Fx32::from_double(1.0)  > Fx32::from_double(-1.0));
    CHECK(Fx32::from_double(2.0) <= Fx32::from_double(2.0));
    CHECK(Fx32::from_double(2.0) >= Fx32::from_double(2.0));
    CHECK(fx32_abs(Fx32::from_double(-3.5)) == Fx32::from_double(3.5));
    CHECK(fx32_abs(Fx32::from_double(3.5))  == Fx32::from_double(3.5));
}

PON_TEST(fx32_operations_are_bit_deterministic) {
    using pon::detail::Fx32;
    // The whole point: same inputs, same raw bits, every run / platform.
    const Fx32 x = Fx32::from_double(287.6531);
    const Fx32 y = Fx32::from_double(-13.0009);
    const std::int64_t sum = (x + y).raw;
    const std::int64_t prod = (x * y).raw;
    const std::int64_t quot = (x / y).raw;
    for (int i = 0; i < 32; ++i) {
        CHECK((x + y).raw == sum);
        CHECK((x * y).raw == prod);
        CHECK((x / y).raw == quot);
    }
}

// --- Part B3: deterministic fixed-point sqrt -------------------------------

PON_TEST(fx32_sqrt_perfect_squares_are_exact) {
    using pon::detail::Fx32; using pon::detail::fx32_sqrt;
    // r*r must fit a Q32.32 (r*r << 32 in int64) ⇒ r < ~46340.
    for (std::int64_t r : {0LL, 1LL, 2LL, 3LL, 5LL, 12LL, 100LL, 4096LL, 40000LL}) {
        CHECK(fx32_sqrt(Fx32::from_int(r * r)) == Fx32::from_int(r));
    }
    CHECK(fx32_sqrt(Fx32::from_double(0.25)).to_double() == 0.5);
    CHECK(fx32_sqrt(Fx32::from_double(6.25)).to_double() == 2.5);
}

PON_TEST(fx32_sqrt_matches_std_sqrt_within_resolution) {
    using pon::detail::Fx32; using pon::detail::fx32_sqrt;
    for (double x : {1e-4, 0.01, 0.5, 1.0, 2.0, 3.0, 10.0, 123.456,
                     9999.0, 250000.0, 4.0e6, 1.9e9}) {
        const Fx32   xf  = Fx32::from_double(x);
        const double got = fx32_sqrt(xf).to_double();
        // Compare against sqrt of the *representable* input — isolates the
        // digit-by-digit sqrt from the 2^-32 input quantisation (which, near
        // x = 0, is amplified by 1/(2√x)). fx32_sqrt itself is exact-floor.
        const double want = std::sqrt(xf.to_double());
        CHECK(std::fabs(got - want) <= 3.0e-9 + want * 1.0e-9);
        // And it still tracks the true sqrt to a sane absolute bound.
        CHECK(std::fabs(got - std::sqrt(x)) <= 1.0e-7 + std::sqrt(x) * 1.0e-9);
    }
}

PON_TEST(fx32_sqrt_is_the_exact_integer_floor_sqrt) {
    using pon::detail::Fx32; using pon::detail::fx32_sqrt;
    // fx32_sqrt(x).raw must be floor(sqrt(x.raw << 32)) exactly. For x.raw <
    // 2^20 the value (x.raw << 32) is < 2^52, so a double holds it exactly and
    // std::sqrt is correctly rounded — the reference floor is then exact.
    for (std::int64_t raw = 1; raw < (std::int64_t(1) << 20); raw += 97) {
        const std::int64_t got = fx32_sqrt(Fx32::from_raw(raw)).raw;
        const double       n   = std::ldexp(double(raw), 32);   // raw * 2^32, exact
        const std::int64_t ref = std::int64_t(std::floor(std::sqrt(n)));
        CHECK(got == ref);
        // strict floor: got^2 <= n < (got+1)^2, both squares < 2^42, double-exact
        CHECK(double(got) * double(got) <= n);
        CHECK(double(got + 1) * double(got + 1) > n);
    }
}

PON_TEST(fx32_sqrt_is_monotone_non_decreasing) {
    using pon::detail::Fx32; using pon::detail::fx32_sqrt;
    Fx32 prev = Fx32::from_raw(0);
    for (std::int64_t raw = 1; raw < (std::int64_t(1) << 40); raw += 1'500'007 /*prime*/) {
        const Fx32 r = fx32_sqrt(Fx32::from_raw(raw));
        CHECK(r >= prev);
        prev = r;
    }
}

PON_TEST(fx32_sqrt_is_bit_deterministic) {
    using pon::detail::Fx32; using pon::detail::fx32_sqrt;
    const Fx32 x = Fx32::from_double(2.0);
    const std::int64_t bits = fx32_sqrt(x).raw;
    for (int i = 0; i < 64; ++i) CHECK(fx32_sqrt(x).raw == bits);
    // sqrt(2) to the grid
    CHECK(std::fabs(fx32_sqrt(x).to_double() - 1.4142135623730951) <= 3.0e-9);
}

// Chasing the macOS/ARM64-only cross-platform mismatch on the SixDOF and
// SpinDrift+Coriolis shots (both, and only, the shots that exercise cross()
// on negative Fx32 operands): `Fx32::operator*`'s `#else` branch does
// `static_cast<std::int64_t>(p >> 32)` on a possibly-negative `__int128`.
// Right-shift of a negative signed value is implementation-defined pre-C++20
// — in practice always arithmetic/sign-propagating, but that is an
// assumption, not something the standard (at whatever -std= this TU builds
// under) guarantees identical across every compiler/architecture. This test
// cross-checks the operator's actual runtime result against an INDEPENDENT
// reference that never performs a signed right-shift of a negative wide
// integer — it multiplies as unsigned, shifts logically (well-defined for
// unsigned), and manually sign-extends the top 32 bits — so if Apple
// Clang's AArch64 codegen for the native `>>` ever disagreed with the
// mathematical floor-shift, this would catch it independently of whatever
// the operator itself does.
// Portable 64x64->128 unsigned multiply via 32-bit limbs — no __int128, no
// compiler intrinsic, no shift of a signed value anywhere. `hi`/`lo` form the
// 128-bit product mag = hi*2^64 + lo.
inline void fx32_test_umul64(std::uint64_t a, std::uint64_t b,
                             std::uint64_t& hi, std::uint64_t& lo) {
    const std::uint64_t aLo = static_cast<std::uint32_t>(a);
    const std::uint64_t aHi = a >> 32;
    const std::uint64_t bLo = static_cast<std::uint32_t>(b);
    const std::uint64_t bHi = b >> 32;

    const std::uint64_t t0 = aLo * bLo;
    const std::uint64_t t1 = aHi * bLo + (t0 >> 32);
    const std::uint64_t t2 = aLo * bHi + (t1 & 0xFFFFFFFFull);
    hi = aHi * bHi + (t1 >> 32) + (t2 >> 32);
    lo = (t2 << 32) | (t0 & 0xFFFFFFFFull);
}

// (a*b) >> 32 with correct floor semantics for negative results, computed
// entirely from the unsigned magnitude product above — never a right-shift
// of a negative (or wide/`__int128`) value. Independent of whatever codegen
// `Fx32::operator*`'s native `p >> 32` on a signed `__int128` produces.
inline std::int64_t fx32_test_ref_mul(std::int64_t a, std::int64_t b) {
    const bool neg = (a < 0) != (b < 0);
    const std::uint64_t ua = a < 0 ? (~static_cast<std::uint64_t>(a) + 1)
                                   : static_cast<std::uint64_t>(a);
    const std::uint64_t ub = b < 0 ? (~static_cast<std::uint64_t>(b) + 1)
                                   : static_cast<std::uint64_t>(b);
    std::uint64_t hi, lo;
    fx32_test_umul64(ua, ub, hi, lo);
    // shift the 128-bit magnitude right by 32 (fits our test ranges in 64 bits)
    const std::uint64_t mag = (hi << 32) | (lo >> 32);
    const bool exact = (lo & 0xFFFFFFFFull) == 0;
    std::int64_t r = static_cast<std::int64_t>(mag);
    if (neg) r = exact ? -r : -r - 1;
    return r;
}

PON_TEST(fx32_multiply_matches_shift_free_reference_for_negative_operands) {
    using pon::detail::Fx32;

    const std::int64_t kValues[] = {
        0, 1, -1, 1000000, -1000000,
        313094,            // ~ earth-rate omega component, Q32.32 raw
        -313094,
        3435973836LL,      // ~0.8 in raw Q32.32
        -3435973836LL,
        3435973836000LL,   // ~800 m/s in raw Q32.32 (velocity-scale)
        -3435973836000LL,
        (std::int64_t(1) << 40) - 1,
        -((std::int64_t(1) << 40) - 1),
    };
    for (std::int64_t a : kValues) {
        for (std::int64_t b : kValues) {
            const Fx32 fa = Fx32::from_raw(a), fb = Fx32::from_raw(b);
            const std::int64_t got  = (fa * fb).raw;
            const std::int64_t want = fx32_test_ref_mul(a, b);
            CHECK(got == want);
        }
    }
}

// Same idea for cross(): the operation shot2 (SixDOF) and shot3
// (SpinDrift+Coriolis) exercise that shot0/1/4 never do. Builds two AVec3<Fx32>
// with a realistic mix of signs/magnitudes (velocity-scale and earth-rate-scale)
// and checks every component against the shift-free reference multiply/subtract.
PON_TEST(fx32_cross_matches_shift_free_reference) {
    using pon::detail::Fx32; using pon::detail::AVec3; using pon::detail::cross;

    auto refMul = fx32_test_ref_mul;
    auto refSub = [](std::int64_t x, std::int64_t y) -> std::int64_t {
        return static_cast<std::int64_t>(static_cast<std::uint64_t>(x) -
                                         static_cast<std::uint64_t>(y));
    };

    const Fx32 omega[3] = {Fx32::from_raw(313094), Fx32::from_raw(0),
                           Fx32::from_raw(-27)}; // earth-rate-scale, one exactly 0
    const Fx32 vel[3]   = {Fx32::from_raw(3435973836000LL),
                           Fx32::from_raw(-859993459000LL),
                           Fx32::from_raw(214998364750LL)}; // velocity-scale, mixed sign

    const AVec3<Fx32> a{omega[0], omega[1], omega[2]};
    const AVec3<Fx32> b{vel[0], vel[1], vel[2]};
    const AVec3<Fx32> got = cross(a, b);

    const std::int64_t wantX = refSub(refMul(omega[1].raw, vel[2].raw), refMul(omega[2].raw, vel[1].raw));
    const std::int64_t wantY = refSub(refMul(omega[2].raw, vel[0].raw), refMul(omega[0].raw, vel[2].raw));
    const std::int64_t wantZ = refSub(refMul(omega[0].raw, vel[1].raw), refMul(omega[1].raw, vel[0].raw));

    CHECK(got.x.raw == wantX);
    CHECK(got.y.raw == wantY);
    CHECK(got.z.raw == wantZ);
}

// --- Part B4: fixed-point transcendental LUTs ------------------------------

PON_TEST(fx_sin_matches_std_over_zero_to_pi) {
    using pon::detail::Fx32; using pon::detail::fx_sin;
    for (int k = 0; k <= 200; ++k) {
        const double a = k * (3.14159265358979323846 / 200.0);
        const double got = fx_sin(Fx32::from_double(a)).to_double();
        CHECK(std::fabs(got - std::sin(a)) <= 5.0e-6);
    }
    CHECK(std::fabs(fx_sin(Fx32::from_double(0.0)).to_double()) <= 1e-9);
    CHECK(std::fabs(fx_sin(Fx32::from_double(3.14159265358979)).to_double()) <= 5.0e-6);
}

PON_TEST(fx_sin_cos_full_match_std_over_many_periods) {
    // fx_sin's own table is [0,pi] only; fx_sin_full/fx_cos_full (Tier: real
    // cross-platform CI bug fix, sim.cpp's 6-DOF roll->Quat write-back) must
    // stay correct — not just self-consistent — over an UNBOUNDED angle:
    // several full turns positive and negative, not just one period.
    using pon::detail::Fx32; using pon::detail::fx_sin_full; using pon::detail::fx_cos_full;
    const double twoPi = 2.0 * 3.14159265358979323846;
    for (int k = -400; k <= 400; ++k) {
        const double a = k * (twoPi / 37.0); // steps that don't line up with pi/2
        const double gotSin = fx_sin_full(Fx32::from_double(a)).to_double();
        const double gotCos = fx_cos_full(Fx32::from_double(a)).to_double();
        CHECK(std::fabs(gotSin - std::sin(a)) <= 5.0e-6);
        CHECK(std::fabs(gotCos - std::cos(a)) <= 5.0e-6);
        // sin^2 + cos^2 == 1 to the same tolerance, from the SAME fixed-point
        // path an actual writeBack call takes (not compared against std here).
        CHECK(std::fabs(gotSin * gotSin + gotCos * gotCos - 1.0) <= 5.0e-5);
    }
    // Exact quarter-turns: sin/cos should be unambiguous, not near a table edge.
    CHECK(std::fabs(fx_sin_full(Fx32::from_double(0.0)).to_double() - 0.0) <= 1e-6);
    CHECK(std::fabs(fx_cos_full(Fx32::from_double(0.0)).to_double() - 1.0) <= 1e-6);
    CHECK(std::fabs(fx_sin_full(Fx32::from_double(3.14159265358979)).to_double() - 0.0) <= 1e-5);
    CHECK(std::fabs(fx_cos_full(Fx32::from_double(3.14159265358979)).to_double() - (-1.0)) <= 1e-5);
}

PON_TEST(fx_acos_matches_std) {
    using pon::detail::Fx32; using pon::detail::fx_acos;
    for (int k = -95; k <= 95; ++k) {           // |c| ≤ 0.95: gentle slope
        const double c = k / 100.0;
        CHECK(std::fabs(fx_acos(Fx32::from_double(c)).to_double() - std::acos(c)) <= 2.0e-4);
    }
    // near the endpoints acos has a vertical tangent — coarser bound, still sane
    CHECK(std::fabs(fx_acos(Fx32::from_double(0.999)).to_double() - std::acos(0.999)) <= 5.0e-2);
    CHECK(std::fabs(fx_acos(Fx32::from_double(-1.0)).to_double() - 3.14159265) <= 1.0e-2);
    CHECK(fx_acos(Fx32::from_double(2.0)).to_double() <= 1e-6);        // clamped
}

PON_TEST(fx_exp_matches_std_for_negative_args) {
    using pon::detail::Fx32; using pon::detail::fx_exp;
    for (int k = 0; k <= 200; ++k) {
        const double x = k * (20.0 / 200.0);    // arg = -x
        const double got  = fx_exp(Fx32::from_double(-x)).to_double();
        const double want = std::exp(-x);
        CHECK(std::fabs(got - want) <= 1.0e-4 * want + 3.0e-9);
    }
    CHECK(fx_exp(Fx32::from_double(0.0)).to_double() == 1.0);
    CHECK(fx_exp(Fx32::from_double(3.0)).to_double() == 1.0);          // arg > 0 ⇒ 1
}

PON_TEST(fx_pow_neg017_matches_std) {
    using pon::detail::Fx32; using pon::detail::fx_pow_neg017;
    for (double t : {1.0e-3, 5.0e-3, 0.02, 0.1, 0.5, 1.0, 3.0, 10.0, 40.0, 120.0, 300.0}) {
        const double got  = fx_pow_neg017(Fx32::from_double(t)).to_double();
        const double want = std::pow(t, -0.17);
        CHECK(std::fabs(got - want) <= 5.0e-5 * want + 1.0e-6);
    }
}

PON_TEST(fx_lut_sampling_is_bit_deterministic) {
    using pon::detail::Fx32;
    const Fx32 a = Fx32::from_double(1.2345);
    const std::int64_t s = pon::detail::fx_sin(a).raw;
    const std::int64_t e = pon::detail::fx_exp(Fx32::from_double(-2.5)).raw;
    const std::int64_t p = pon::detail::fx_pow_neg017(Fx32::from_double(7.0)).raw;
    for (int i = 0; i < 48; ++i) {
        CHECK(pon::detail::fx_sin(a).raw == s);
        CHECK(pon::detail::fx_exp(Fx32::from_double(-2.5)).raw == e);
        CHECK(pon::detail::fx_pow_neg017(Fx32::from_double(7.0)).raw == p);
    }
}

// --- Part B5: fixed-point drag/lift LUT sampling ---------------------------

PON_TEST(lut_sample_fx_tracks_the_double_path_synthetic) {
    using pon::detail::Fx32; using pon::detail::lut_sample_fx;
    Lut1D lut;
    lut.x0 = 0.0; lut.x1 = 5.0;
    for (int k = 0; k < 51; ++k) {
        const double m = k * 0.1;
        lut.y.push_back(0.10 + 0.5 * std::exp(-(m - 1.1) * (m - 1.1) * 6.0)); // a bump
    }
    for (int k = 0; k <= 500; ++k) {
        const double x   = k * (5.0 / 500.0);
        const double dbl = lut.sample(x);
        const double fx  = lut_sample_fx(lut, Fx32::from_double(x)).to_double();
        CHECK(std::fabs(fx - dbl) <= 5.0e-9);
    }
    // clamp outside the domain
    CHECK(std::fabs(lut_sample_fx(lut, Fx32::from_double(-3.0)).to_double() - lut.sample(-3.0)) <= 5e-9);
    CHECK(std::fabs(lut_sample_fx(lut, Fx32::from_double(99.0)).to_double() - lut.sample(99.0)) <= 5e-9);
}

PON_TEST(lut_sample_fx_tracks_the_double_path_on_a_real_g7_table) {
    using namespace pon::detail;
    ProjectileType t;
    t.id = "762x51_175smk"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    DragLuts L; compile_drag_lut(t, L);
    CHECK(L.dragRet.y.size() >= 2);

    double worst = 0.0;
    for (int k = 0; k <= 1000; ++k) {
        const double mach = k * (5.0 / 1000.0);
        const double dbl  = L.dragRet.sample(mach);
        const double fx   = lut_sample_fx(L.dragRet, Fx32::from_double(mach)).to_double();
        worst = std::max(worst, std::fabs(fx - dbl));
    }
    // dragRet values are ~1e-4..1e-3 m^2/kg; fixed-point sampling must match the
    // double lerp to a few ULP of Q32.32, not merely "close".
    CHECK(worst <= 1.0e-9);
}

PON_TEST(lut_sample_fx_is_bit_deterministic) {
    using pon::detail::Fx32; using pon::detail::lut_sample_fx;
    Lut1D lut; lut.x0 = 0.0; lut.x1 = 1.5e6;          // Reynolds-span table
    for (int k = 0; k < 257; ++k) lut.y.push_back(0.5 - 0.0000002 * (k * 5859.0));
    const Fx32 x = Fx32::from_double(732000.0);
    const std::int64_t bits = lut_sample_fx(lut, x).raw;
    for (int i = 0; i < 64; ++i) CHECK(lut_sample_fx(lut, x).raw == bits);
}

// --- Part B6a: the fixed-point numeric stack composes into a trajectory ----

PON_TEST(fx32_stands_in_as_a_scalar_accumulator) {
    using pon::detail::Fx32;
    // The literal forms the integrator uses on `Accum`.
    CHECK((Fx32(2) * Fx32(3)) == Fx32(6));
    CHECK(std::fabs((Fx32(-9) / Fx32(10)).to_double() - (-0.9)) <= 3e-9);
    CHECK(std::fabs((Fx32(1) / Fx32(6)).to_double() - (1.0 / 6.0)) <= 3e-9);
    CHECK(Fx32(0.5).to_double() == 0.5);          // 0.5 is exact on the grid
    Fx32 a = Fx32(0);
    a += Fx32(2) * Fx32(0.25);
    CHECK(a == Fx32(0.5));
    // acc_* dispatch resolves to the fixed-point ops for Fx32.
    CHECK(std::fabs(pon::detail::acc_sqrt(Fx32(2.0)).to_double() - std::sqrt(2.0)) <= 3e-9);
    CHECK(std::fabs(pon::detail::acc_sin(Fx32(1.0)).to_double()  - std::sin(1.0))  <= 5e-6);
}

namespace {
// One force model: gravity + quadratic constant-k drag, no wind. Templated on
// the accumulator so the exact same step order runs in double and in Fx32.
template <class Acc>
void semi_implicit_flight(pon::detail::AVec3<Acc>& p, pon::detail::AVec3<Acc>& v,
                          Acc k, Acc g, Acc h, int steps) {
    using pon::detail::acc_sqrt;
    for (int i = 0; i < steps; ++i) {
        const Acc sp = acc_sqrt(dot(v, v));
        pon::detail::AVec3<Acc> a{Acc(0), -g, Acc(0)};
        a = a - v * (k * sp);                 // -k|v|v
        v += a * h;
        p += v * h;
    }
}
} // namespace

PON_TEST(fixed_point_trajectory_tracks_the_double_trajectory) {
    using pon::detail::Fx32; using pon::detail::AVec3;

    const double k = 0.002, g = 9.80665, h = 1.0 / 2000.0;
    const int steps = 6000;                    // 3 s of flight

    AVec3<double> pd{0, 2, 0}, vd{380, 220, 0};
    semi_implicit_flight<double>(pd, vd, k, g, h, steps);

    AVec3<Fx32> pf{Fx32(0), Fx32(2), Fx32(0)}, vf{Fx32(380), Fx32(220), Fx32(0)};
    semi_implicit_flight<Fx32>(pf, vf, Fx32(k), Fx32(g), Fx32(h), steps);

    // Q32.32 rounding accumulates over 6000 steps but must stay sub-decimetre
    // over a ~1 km flight, and the speed within a fraction of a m/s.
    CHECK(std::fabs(pf.x.to_double() - pd.x) <= 0.10);
    CHECK(std::fabs(pf.y.to_double() - pd.y) <= 0.10);
    const double spd = std::sqrt(vd.x * vd.x + vd.y * vd.y);
    const double spf = std::sqrt(vf.x.to_double() * vf.x.to_double() +
                                 vf.y.to_double() * vf.y.to_double());
    CHECK(std::fabs(spf - spd) <= 0.20);

    // And it is bit-for-bit repeatable.
    AVec3<Fx32> pf2{Fx32(0), Fx32(2), Fx32(0)}, vf2{Fx32(380), Fx32(220), Fx32(0)};
    semi_implicit_flight<Fx32>(pf2, vf2, Fx32(k), Fx32(g), Fx32(h), steps);
    CHECK(pf2.x.raw == pf.x.raw && pf2.y.raw == pf.y.raw && vf2.x.raw == vf.x.raw);
}

// --- Part B6b: the integrator's BitExact path via the entry points --------

PON_TEST(step_rk4_bitexact_path_tracks_the_double_path) {
    using namespace pon::detail;
    // Compile a real G7 drag LUT and drive both paths through step_rk4 with the
    // same FlightModel, same step order.
    ProjectileType t;
    t.id = "b6b"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
    t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    DragLuts L; compile_drag_lut(t, L);

    FlightModel fm;
    fm.gravity = {0, -9.80665, 0};
    fm.dragRet = &L.dragRet;
    fm.rhoEff  = 1.225;
    fm.dragAbscissaScale = 1.0 / 340.294;

    auto fly = [&](bool bx) {
        Vec3 p{0, 0, 0}, v{792, 0, 0};
        for (int i = 0; i < 3000; ++i)          // 1.5 s at 0.5 ms
            step_rk4(fm, p, v, 0.0, 1.0 / 2000.0, bx);
        return std::pair<Vec3, Vec3>{p, v};
    };

    const auto d  = fly(false);
    const auto fx = fly(true);

    CHECK(std::fabs(d.first.x - fx.first.x) <= 0.05);      // < 5 cm over ~900 m
    CHECK(std::fabs(d.first.y - fx.first.y) <= 0.02);
    CHECK(std::fabs(length(d.second) - length(fx.second)) <= 0.20);

    // The fixed-point run is bit-for-bit repeatable.
    Vec3 p1{0, 0, 0}, v1{792, 0, 0}, p2{0, 0, 0}, v2{792, 0, 0};
    for (int i = 0; i < 800; ++i) {
        step_rk4(fm, p1, v1, 0.0, 1.0 / 2000.0, true);
        step_rk4(fm, p2, v2, 0.0, 1.0 / 2000.0, true);
    }
    CHECK(p1.x == p2.x && p1.y == p2.y && v1.x == v2.x && v1.y == v2.y);
}

PON_TEST(step_rk4_double_path_is_unchanged_by_the_dispatch) {
    using namespace pon::detail;
    // bitExact = false must be the exact pre-B6 arithmetic: a plain vacuum
    // parabola is closed-form, so any drift shows immediately.
    FlightModel fm; fm.gravity = {0, -10, 0}; fm.rhoEff = 0.0;
    Vec3 p{0, 0, 0}, v{100, 50, 0};
    const double h = 1.0 / 1000.0;
    for (int i = 0; i < 1000; ++i) step_rk4(fm, p, v, 0.0, h, false);
    CHECK(std::fabs(p.x - 100.0) < 1e-9);
    CHECK(std::fabs(p.y - (50.0 - 5.0)) < 1e-9);          // 50·1 - ½·10·1²
    CHECK(std::fabs(v.y - 40.0) < 1e-9);
}

// --- Tier 1.1: imperial <-> SI unit helpers -------------------------------

PON_TEST(units_imperial_to_si_conversions) {
    // Anchored to published values a shooter would recognise.
    CHECK_NEAR(pon::grains(168), 0.010886, 1e-6);       // 168 gr Match bullet
    CHECK_NEAR(pon::grains(7000), pon::pounds(1.0), 1e-9); // 7000 gr = 1 lb, exact
    CHECK_NEAR(pon::fps(2650), 807.72, 0.01);           // .308 Match MV
    CHECK_NEAR(pon::inches(0.308), 0.0078232, 1e-7);
    CHECK_NEAR(pon::yards(1000), 914.4, 1e-9);
    CHECK_NEAR(pon::ftlb(2600), 3525.13, 0.01);         // muzzle energy figure
    CHECK_NEAR(pon::inhg(29.9213), 101325.0, 5.0);      // std atmosphere
    CHECK_NEAR(pon::fahrenheit(59.0), 288.15, 1e-9);    // ICAO sea level
    CHECK_NEAR(pon::celsius(15.0), 288.15, 1e-9);
    CHECK_NEAR(pon::moa(1.0), 0.00029089, 1e-8);        // 1 MOA ~ 1.047" @ 100 yd
    CHECK_NEAR(pon::yards(100) * pon::moa(1.0), pon::inches(1.047), 3e-4);
    CHECK_NEAR(pon::mph(60), 26.8224, 1e-4);
}

PON_TEST(units_round_trip_through_si) {
    for (double v : {1.0, 42.5, 1234.0, -7.0}) {
        CHECK_NEAR(pon::to_fps(pon::fps(v)),         v, 1e-9);
        CHECK_NEAR(pon::to_grains(pon::grains(v)),   v, 1e-9);
        CHECK_NEAR(pon::to_yards(pon::yards(v)),     v, 1e-9);
        CHECK_NEAR(pon::to_moa(pon::moa(v)),         v, 1e-9);
        CHECK_NEAR(pon::to_ftlb(pon::ftlb(v)),       v, 1e-9);
    }
    CHECK_NEAR(pon::to_fahrenheit(pon::fahrenheit(72.0)), 72.0, 1e-9);
    CHECK_NEAR(pon::to_celsius(pon::celsius(21.0)),       21.0, 1e-9);
}

PON_TEST(units_are_usable_in_a_launch) {
    // The whole point: build a shot from imperial figures, fire it, sanity-check.
    Sim sim;
    ProjectileType t;
    t.id = "u"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.223;
    t.mass_kg = pon::grains(168);
    t.refDiameter_m = pon::inches(0.308);
    const TypeId id = sim.registerType(t);
    CHECK(id != kInvalidType);
    LaunchParams lp; lp.position = {0, pon::feet(6), 0};
    lp.direction = {1, 0, 0}; lp.speed = pon::fps(2650);
    lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 200; ++i) sim.step(1.0 / 600.0, w, sink);
    const auto& s = sim.state(h);
    CHECK(s.position.x > 200.0 && std::isfinite(s.position.x));
    CHECK(pon::to_fps(length(s.velocity)) > 1500.0);   // still supersonic-ish
}

// --- Tier 1.3: pon::validate() + Sim::lastError() -------------------------

PON_TEST(validate_accepts_a_good_type_and_a_catalog_round) {
    ProjectileType t;
    t.id = "ok"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.243; t.mass_kg = 0.01; t.refDiameter_m = 0.0078;
    CHECK(!pon::validate(t).has_value());
    CHECK(!pon::validate(catalog::get("9x19_124gr_fmj")).has_value());
    // A Bullet with no mass/diameter is fine — the class default supplies them.
    ProjectileType b; b.id = "b"; b.klass = ProjectileClass::Bullet;
    b.dragModel = DragModel::G1;
    CHECK(!pon::validate(b).has_value());
}

PON_TEST(validate_flags_the_common_mistakes) {
    ProjectileType t; t.id = "";
    CHECK(pon::validate(t).has_value());                 // empty id

    ProjectileType cc; cc.id = "cc"; cc.klass = ProjectileClass::Bullet;
    cc.dragModel = DragModel::CustomCurve;               // empty curve, no fallback
    CHECK(pon::validate(cc).has_value());
    cc.dragCoefficient = 0.30;                           // ...now has a fallback
    CHECK(!pon::validate(cc).has_value());

    ProjectileType bp; bp.id = "bp"; bp.klass = ProjectileClass::SportsBall;
    bp.dragModel = DragModel::BallProfile;               // no ballProfile id
    CHECK(pon::validate(bp).has_value());

    ProjectileType badbc; badbc.id = "x"; badbc.klass = ProjectileClass::Bullet;
    badbc.dragModel = DragModel::G7; badbc.ballisticCoefficient = -1.0;
    CHECK(pon::validate(badbc).has_value());

    ProjectileType nan; nan.id = "n"; nan.klass = ProjectileClass::Bullet;
    nan.mass_kg = std::nan("");
    CHECK(pon::validate(nan).has_value());
}

PON_TEST(sim_lastError_explains_a_failed_register_and_spawn) {
    Sim sim;
    ProjectileType bad; bad.id = "b"; bad.klass = ProjectileClass::Bullet;
    bad.dragModel = DragModel::CustomCurve;               // empty curve, no fallback
    const TypeId id = sim.registerType(bad);
    CHECK(id == kInvalidType);
    CHECK(std::string(sim.lastError()).find("CustomCurve") != std::string::npos);

    ProjectileType good; good.id = "g"; good.klass = ProjectileClass::Bullet;
    good.dragModel = DragModel::G7; good.ballisticCoefficient = 0.24;
    const TypeId gid = sim.registerType(good);
    CHECK(gid != kInvalidType);
    CHECK(std::string(sim.lastError()).empty());          // cleared on success

    LaunchParams lp; lp.position = {0, 0, 0}; lp.direction = {0, 0, 0}; // zero dir
    CHECK(sim.spawn(gid, lp) == kInvalidState);
    CHECK(std::string(sim.lastError()).find("direction") != std::string::npos);

    CHECK(sim.spawn(999u, lp) == kInvalidState);
    CHECK(std::string(sim.lastError()).find("TypeId") != std::string::npos);
}

// --- Tier 1.4: pon::preview_arc() ----------------------------------------

PON_TEST(preview_arc_matches_a_spawned_analytic_shot) {
    ProjectileType t;
    t.id = "pv"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.243; t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;

    LaunchParams lp;
    lp.position = {0, 2, 0};
    lp.direction = {std::cos(0.05), std::sin(0.05), 0};
    lp.speed = 800.0;

    Environment env;
    std::vector<Vec3> arc;
    pon::preview_arc(t, lp, env, 1.0 / 60.0, 3.0, arc, /*groundY*/ 0.0);
    CHECK(arc.size() > 10);
    CHECK(arc.front().y == 2.0 && arc.front().x == 0.0);
    CHECK(arc.back().x > 500.0);                      // travelled downrange
    for (const Vec3& p : arc) CHECK(std::isfinite(p.x) && std::isfinite(p.y));

    // The same shot spawned on the AnalyticDrag tier must land on the same
    // polyline (preview_arc IS that tier under the hood).
    Sim sim;
    const TypeId id = sim.registerType(t);
    LaunchParams sp = lp; sp.tier = FidelityTier::AnalyticDrag;
    const StateId h = sim.spawn(id, sp);
    EmptyWorld w; VectorEventSink sink;
    for (std::size_t i = 1; i < arc.size(); ++i) {
        sim.step(1.0 / 60.0, w, sink);
        const Vec3 p = sim.state(h).position;
        CHECK(std::fabs(p.x - arc[i].x) < 1e-6);
        CHECK(std::fabs(p.y - arc[i].y) < 1e-6);
    }
}

PON_TEST(preview_arc_handles_bad_input) {
    Environment env;
    std::vector<Vec3> arc{Vec3{9, 9, 9}};            // must be cleared
    ProjectileType bad; bad.id = "";                  // fails validate
    LaunchParams lp; lp.position = {0, 1, 0}; lp.direction = {1, 0, 0}; lp.speed = 100;
    pon::preview_arc(bad, lp, env, 0.01, 1.0, arc, -1e30);
    CHECK(arc.empty());

    ProjectileType ok; ok.id = "o"; ok.klass = ProjectileClass::Bullet;
    ok.dragModel = DragModel::G1;
    pon::preview_arc(ok, lp, env, 0.0, 1.0, arc, -1e30);   // dt = 0
    CHECK(arc.empty());
}

PON_TEST(preview_arc_ex_matches_the_plain_overload) {
    // Both preview_arc overloads share one driver (preview_arc_impl) — the
    // TrajectorySample positions must equal the Vec3 overload's, point for
    // point, with sane velocity/time alongside.
    ProjectileType t;
    t.id = "pvex"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.243; t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0.02, 0}; lp.speed = 800.0;
    Environment env;

    std::vector<Vec3> plain;
    pon::preview_arc(t, lp, env, 1.0 / 60.0, 2.0, plain, 0.0);
    std::vector<TrajectorySample> ex;
    pon::preview_arc(t, lp, env, 1.0 / 60.0, 2.0, ex, 0.0);

    CHECK(plain.size() == ex.size() && !plain.empty());
    for (std::size_t i = 0; i < plain.size(); ++i) {
        CHECK(ex[i].position.x == plain[i].x);
        CHECK(ex[i].position.y == plain[i].y);
    }
    CHECK(ex.front().time_s == 0.0);
    CHECK(ex.back().time_s > ex.front().time_s);
    CHECK(length(ex.front().velocity) > 700.0); // ~muzzle speed, minus a frame of drag
}

// --- Tier 1.5: pon::describe() ------------------------------------------

PON_TEST(mach_and_kinetic_energy_match_describe) {
    // pon::mach()/kinetic_energy_J() (B2) must agree with what describe()
    // folds into its "E=... M=..." tail — same source of truth.
    Sim sim;
    ProjectileType t;
    t.id = "mke"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.243; t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 120; ++i) sim.step(1.0 / 600.0, w, sink);

    const ProjectileState& s = sim.state(h);
    const Real m = pon::mach(s, sim.environment());
    const Real e = pon::kinetic_energy_J(s, t);
    CHECK(m > 0.5 && m < 3.0);
    CHECK(e > 0.0);

    char buf[16];
    std::snprintf(buf, sizeof buf, "M=%.2f", m);
    const std::string full = pon::describe(s, t, sim.environment());
    CHECK(full.find(buf) != std::string::npos);
    std::snprintf(buf, sizeof buf, "E=%.0f", e);
    CHECK(full.find(buf) != std::string::npos);

    // Unset mass -> 0 energy (describe() omits "E=" the same way).
    ProjectileType noMass = t; noMass.mass_kg = 0.0;
    CHECK(pon::kinetic_energy_J(s, noMass) == 0.0);
}

PON_TEST(describe_summarises_a_live_shot) {
    Sim sim;
    ProjectileType t;
    t.id = "d"; t.klass = ProjectileClass::Bullet; t.dragModel = DragModel::G7;
    t.ballisticCoefficient = 0.243; t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
    const TypeId id = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 2, 0}; lp.direction = {1, 0, 0};
    lp.speed = 800.0; lp.tier = FidelityTier::Integrated;
    const StateId h = sim.spawn(id, lp);
    EmptyWorld w; VectorEventSink sink;
    for (int i = 0; i < 120; ++i) sim.step(1.0 / 600.0, w, sink);

    const std::string bare = pon::describe(sim.state(h));
    CHECK(bare.find("v=") != std::string::npos);
    CHECK(bare.find("dist=") != std::string::npos);
    CHECK(bare.find("[integrated") != std::string::npos);
    CHECK(bare.find("E=") == std::string::npos);       // no mass in the bare form

    const std::string full = pon::describe(sim.state(h), t, sim.environment());
    CHECK(full.find("E=") != std::string::npos);       // ½mv²
    CHECK(full.find("M=") != std::string::npos);       // Mach
    std::printf("    %s\n", full.c_str());
}

PON_TEST(describe_notes_state_flags) {
    ProjectileState s;
    s.velocity = {600, 0, 0};
    s.tier = FidelityTier::AnalyticDrag;
    s.flags = kFlagInTransonic | kFlagExpanded;
    s.alive = false;
    const std::string d = pon::describe(s);
    CHECK(d.find("[dead]") != std::string::npos);
    CHECK(d.find("[analytic") != std::string::npos);
    CHECK(d.find("transonic") != std::string::npos);
    CHECK(d.find("expanded") != std::string::npos);
}

PON_TEST(describe_type_summarises_a_registered_round) {
    Sim sim;
    ProjectileType t;
    t.id = "9x19_124gr_fmj"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.00804; t.refDiameter_m = 0.00902; t.muzzleSpeed_mps = 360.0;
    const TypeId id = sim.registerType(t);
    CHECK(id != kInvalidType);

    const std::string d = pon::describe(sim.type(id));
    CHECK(d.find("9x19_124gr_fmj:") != std::string::npos);
    CHECK(d.find("8.0 g") != std::string::npos);
    CHECK(d.find("9.0 mm") != std::string::npos);
    CHECK(d.find("Cd 0.30") != std::string::npos);
    CHECK(d.find("360 m/s") != std::string::npos);
    CHECK(d.find("J muzzle") != std::string::npos); // 518 J -> not kJ/MJ scaled
    std::printf("    %s\n", d.c_str());

    ProjectileType g7;
    g7.id = "762x51"; g7.klass = ProjectileClass::Bullet;
    g7.dragModel = DragModel::G7; g7.ballisticCoefficient = 0.243;
    const std::string dg7 = pon::describe(g7); // unregistered -> raw fields (0 mass/diam)
    CHECK(dg7.find("G7 BC 0.243") != std::string::npos);
    CHECK(dg7.find("0.0 g") != std::string::npos); // honestly reports the unset field
}

PON_TEST(fire_registers_once_and_reuses_by_id) {
    Sim sim;
    ProjectileType t;
    t.id = "fire_9mm"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009; t.muzzleSpeed_mps = 360.0;

    const StateId a = sim.fire(t, {0, 1.7, 0}, {1, 0, 0});
    const StateId b = sim.fire(t, {0, 1.7, 0}, {1, 0, 0});
    CHECK(a != kInvalidState && b != kInvalidState && a != b);
    CHECK(std::string(sim.lastError()).empty());
    CHECK(sim.liveCount() == 2);
    // Same id the second time -> no second registration.
    CHECK(sim.state(a).typeId == sim.state(b).typeId);
    // Default speed came from the type.
    CHECK_NEAR(length(sim.state(a).velocity), 360.0, 1e-6);
    CHECK(sim.state(a).tier == FidelityTier::Integrated); // fire() default

    EmptyWorld world; VectorEventSink sink;
    for (int i = 0; i < 50; ++i) sim.step(1.0 / 500.0, world, sink);
    CHECK(sim.state(a).position.x > 0.0);
}

PON_TEST(live_ids_and_for_each_live_agree_with_live_count) {
    Sim sim;
    ProjectileType t;
    t.id = "liveids"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009; t.muzzleSpeed_mps = 360.0;
    const TypeId tid = sim.registerType(t);
    LaunchParams lp; lp.position = {0, 1.7, 0}; lp.direction = {1, 0, 0};

    const StateId a = sim.spawn(tid, lp);
    const StateId b = sim.spawn(tid, lp);
    const StateId c = sim.spawn(tid, lp);
    sim.despawn(b); // opens a hole in the middle of the slot range

    std::vector<StateId> ids;
    sim.liveIds(ids);
    CHECK(ids.size() == sim.liveCount());
    CHECK(ids.size() == 2);
    CHECK(std::find(ids.begin(), ids.end(), a) != ids.end());
    CHECK(std::find(ids.begin(), ids.end(), c) != ids.end());
    CHECK(std::find(ids.begin(), ids.end(), b) == ids.end());

    std::vector<StateId> visited;
    sim.forEachLive([&](StateId id, const ProjectileState&) { visited.push_back(id); });
    CHECK(visited == ids); // same set, same slot order

    // liveIds() clears `out` first — stale entries don't leak through.
    std::vector<StateId> stale{999, 998, 997};
    sim.liveIds(stale);
    CHECK(stale == ids);
}

PON_TEST(fire_reports_a_bad_type) {
    Sim sim;
    ProjectileType bad; // empty id, no mass, no drag data
    const StateId h = sim.fire(bad, {0, 0, 0}, {1, 0, 0});
    CHECK(h == kInvalidState);
    CHECK(!std::string(sim.lastError()).empty());
}

PON_TEST(fire_rejects_a_zero_aim_direction) {
    Sim sim;
    ProjectileType t;
    t.id = "fire_zero"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009; t.muzzleSpeed_mps = 300.0;
    const StateId h = sim.fire(t, {0, 1, 0}, {0, 0, 0});
    CHECK(h == kInvalidState);
    CHECK(!std::string(sim.lastError()).empty());
}

namespace {
TypeId register_disp_bullet(Sim& sim) {
    ProjectileType t;
    t.id = "disp_bullet"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.010; t.refDiameter_m = 0.0078;
    return sim.registerType(t);
}
StateId spawn_dispersed(Sim& sim, TypeId id, Vec3 aim, Real mrad) {
    LaunchParams lp;
    lp.position = {0, 0, 0};
    lp.direction = aim;
    lp.speed = 800.0;
    lp.precisionMrad = mrad;
    return sim.spawn(id, lp);
}
} // namespace

PON_TEST(dispersion_zero_is_exact_aim) {
    Sim sim;
    const TypeId id = register_disp_bullet(sim);
    const Vec3 v = sim.state(spawn_dispersed(sim, id, {1, 0, 0}, 0.0)).velocity;
    CHECK_NEAR(v.x, 800.0, 1e-9);   // untouched
    CHECK_NEAR(v.y, 0.0, 1e-12);
    CHECK_NEAR(v.z, 0.0, 1e-12);
}

PON_TEST(dispersion_is_seeded_and_repeatable) {
    auto run = [] {
        Sim sim; // default rngSeed
        const TypeId id = register_disp_bullet(sim);
        std::vector<Vec3> dirs;
        for (int i = 0; i < 8; ++i)
            dirs.push_back(normalized(
                sim.state(spawn_dispersed(sim, id, {1, 0, 0}, 3.0)).velocity));
        return dirs;
    };
    const auto a = run(), b = run();
    for (std::size_t i = 0; i < a.size(); ++i)
        CHECK(a[i].x == b[i].x && a[i].y == b[i].y && a[i].z == b[i].z);
    CHECK(!(a[0].y == a[1].y && a[0].z == a[1].z)); // shots actually spread
}

PON_TEST(dispersion_stays_inside_the_cone) {
    Sim sim;
    const TypeId id = register_disp_bullet(sim);
    const Vec3 aim{1, 0, 0};
    const double halfAngle = 5.0e-3; // 5 mrad
    double sy = 0, sz = 0, maxOff = 0;
    const int N = 4000;
    for (int i = 0; i < N; ++i) {
        const Vec3 d = normalized(sim.state(spawn_dispersed(sim, id, aim, 5.0)).velocity);
        const double off = std::acos(std::min(1.0, (double)dot(d, aim)));
        CHECK(off <= halfAngle + 1e-6);
        maxOff = std::max(maxOff, off);
        sy += d.y; sz += d.z;
    }
    CHECK(maxOff > halfAngle * 0.5);       // fills a good part of the cone
    CHECK(std::fabs(sy / N) < 3.0e-4);     // roughly centred on the aim
    CHECK(std::fabs(sz / N) < 3.0e-4);
}

PON_TEST(trace_sink_reports_frames_without_changing_the_shot) {
    auto fly = [](bool withTrace, std::vector<TraceEvent>* log) {
        SimConfig cfg;
        if (withTrace) cfg.traceSink = [log](const TraceEvent& e) { log->push_back(e); };
        Sim sim(Environment{}, cfg);
        ProjectileType t;
        t.id = "trace_308"; t.klass = ProjectileClass::Bullet;
        t.dragModel = DragModel::G7; t.ballisticCoefficient = 0.243;
        t.mass_kg = 0.01134; t.refDiameter_m = 0.00782;
        const TypeId id = sim.registerType(t);
        LaunchParams lp;
        lp.position = {0, 2, 0}; lp.direction = {1, 0, 0}; lp.speed = 800.0;
        lp.tier = FidelityTier::Integrated;
        lp.precision = PrecisionFlag::TransonicFlag;
        const StateId h = sim.spawn(id, lp);
        EmptyWorld world; VectorEventSink sink;
        for (int i = 0; i < 2000 && sim.state(h).alive; ++i)
            sim.step(1.0 / 200.0, world, sink);
        return sim.state(h).position;
    };

    std::vector<TraceEvent> log;
    const Vec3 traced = fly(true, &log);
    const Vec3 plain  = fly(false, nullptr);

    // The trace must not perturb the trajectory.
    CHECK(traced.x == plain.x && traced.y == plain.y && traced.z == plain.z);

    int frames = 0, substeps = 0, tEnter = 0, tExit = 0;
    for (const TraceEvent& e : log) {
        CHECK(e.shot == 0);
        if (e.kind == TraceKind::Frame) {
            ++frames;
            CHECK(e.i0 == static_cast<int>(FidelityTier::Integrated));
            CHECK(e.r0 > 0.0 && e.r0 <= 800.0);      // speed, monotone-ish decel
        } else if (e.kind == TraceKind::Substep) {
            ++substeps;
            CHECK(e.i0 >= 1);
        } else if (e.kind == TraceKind::TransonicEnter) {
            ++tEnter;
        } else if (e.kind == TraceKind::TransonicExit) {
            ++tExit;
        }
    }
    CHECK(frames > 1000);         // one per step while alive
    CHECK(substeps > 0);
    CHECK(tEnter == 1 && tExit == 1);   // an 800 m/s .308 decelerates through Mach 1 once
    std::printf("    trace: %d frames, %d substep reports\n", frames, substeps);
}

PON_TEST(trace_sink_null_is_the_default_and_silent) {
    SimConfig cfg;
    CHECK(!cfg.traceSink);   // nothing to do, nothing allocated
}

PON_TEST(set_trace_sink_arms_and_disarms_at_runtime) {
    Sim sim;                        // no traceSink in the config
    ProjectileType t;
    t.id = "rt_trace"; t.klass = ProjectileClass::Bullet;
    t.dragModel = DragModel::ConstantCd; t.dragCoefficient = 0.30;
    t.mass_kg = 0.008; t.refDiameter_m = 0.009; t.muzzleSpeed_mps = 400.0;
    const StateId h = sim.fire(t, {0, 2, 0}, {1, 0, 0});
    EmptyWorld world; VectorEventSink sink;

    int hits = 0;
    for (int i = 0; i < 20; ++i) sim.step(1.0 / 200.0, world, sink);   // silent
    CHECK(hits == 0);

    sim.setTraceSink([&](const TraceEvent&) { ++hits; });
    for (int i = 0; i < 20; ++i) sim.step(1.0 / 200.0, world, sink);   // armed
    const int armed = hits;
    CHECK(armed > 0);

    sim.setTraceSink({});                                             // disarmed
    for (int i = 0; i < 20; ++i) sim.step(1.0 / 200.0, world, sink);
    CHECK(hits == armed);
    CHECK(sim.state(h).position.x > 0.0);
}

namespace {
int g_c_trace_count = 0;
int g_c_trace_frames = 0;
void c_trace_cb(const pon_trace_event* ev, void* user) {
    ++g_c_trace_count;
    if (ev->kind == PON_TRACE_FRAME) ++g_c_trace_frames;
    *static_cast<uint32_t*>(user) = ev->shot;
}
} // namespace

PON_TEST(c_abi_trace_sink) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    CHECK(sim != nullptr);
    pon_projectile_desc d{};
    d.id = "c_trace"; d.klass = 0 /*Bullet*/; d.drag_model = PON_DRAG_CONSTANT_CD;
    d.drag_coefficient = 0.31; d.mass_kg = 0.008; d.ref_diameter_m = 0.009;
    d.muzzle_speed_mps = 400.0;
    const uint32_t tid = pon_register_type(sim, &d);
    CHECK(tid != 0xFFFFFFFFu);

    g_c_trace_count = g_c_trace_frames = 0;
    uint32_t seen_shot = 0xFFFFFFFFu;
    pon_sim_set_trace_sink(sim, c_trace_cb, &seen_shot);

    const uint32_t sid = pon_spawn_precise(sim, tid, {0, 2, 0}, {1, 0, 0}, 0.0,
                                           2 /*Integrated*/, 0);
    CHECK(sid != 0xFFFFFFFFu);
    for (int i = 0; i < 30; ++i) pon_step(sim, 1.0 / 200.0);
    CHECK(g_c_trace_frames >= 25);
    CHECK(seen_shot == sid);

    pon_sim_set_trace_sink(sim, nullptr, nullptr);   // clear
    const int after = g_c_trace_count;
    for (int i = 0; i < 30; ++i) pon_step(sim, 1.0 / 200.0);
    CHECK(g_c_trace_count == after);

    pon_sim_destroy(sim);
}

// --- Tier D1: <poncelet/worlds.hpp> stock Worlds ---------------------------

PON_TEST(plane_world_hits_and_reports_the_right_side_normal) {
    PlaneWorld w;
    w.position = {5, 0, 0};
    w.normal = {-1, 0, 0}; // outward toward -x
    HitResult h;
    CHECK(w.raycast({0, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 5.0, 1e-9);
    CHECK_NEAR(h.t, 0.5, 1e-9);
    CHECK_NEAR(h.normal.x, -1.0, 1e-9); // hit from the +normal side (a.x=0 is behind the plane)
    CHECK(!w.raycast({10, 0, 0}, {20, 0, 0}, h)); // segment never crosses x=5
}

PON_TEST(sphere_world_hits_the_near_intersection) {
    SphereWorld w;
    w.center = {5, 0, 0};
    w.radius_m = 1.0;
    HitResult h;
    CHECK(w.raycast({0, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 4.0, 1e-9); // near face, not far face
    CHECK_NEAR(length(h.point - w.center), 1.0, 1e-9);
    CHECK(!w.raycast({0, 5, 0}, {10, 5, 0}, h)); // passes well outside the sphere
}

PON_TEST(aabb_world_hits_the_near_face) {
    AabbWorld w;
    w.boundsMin = {4, -1, -1};
    w.boundsMax = {6, 1, 1};
    HitResult h;
    CHECK(w.raycast({0, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 4.0, 1e-9);
    CHECK_NEAR(h.normal.x, -1.0, 1e-9);
    CHECK(!w.raycast({0, 5, 0}, {10, 5, 0}, h)); // above the box
    // A segment starting inside the box: the near-face clip still resolves,
    // starting t stays within [0,1] (entry face behind the origin).
    CHECK(w.raycast({5, 0, 0}, {10, 0, 0}, h));
}

PON_TEST(slab_stack_world_hits_the_nearest_slab_unsorted) {
    SlabStackWorld w;
    w.axis = {1, 0, 0};
    w.slabs = {{6.0, material_air()}, {4.0, material_air()}, {8.0, material_air()}};
    HitResult h;
    CHECK(w.raycast({0, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 4.0, 1e-9); // nearest of the three, despite listed second
    CHECK(h.surface == 1);            // index into `slabs`, not distance order
}

PON_TEST(composite_world_routes_material_to_the_hit_sub_world) {
    PlaneWorld near_;
    near_.position = {4, 0, 0}; near_.normal = {-1, 0, 0};
    near_.mat.strength_Pa = 111.0;
    PlaneWorld far_;
    far_.position = {8, 0, 0}; far_.normal = {-1, 0, 0};
    far_.mat.strength_Pa = 222.0;

    CompositeWorld c;
    c.worlds = {&near_, &far_};

    HitResult h;
    CHECK(c.raycast({0, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 4.0, 1e-9); // nearer of the two wins
    CHECK_NEAR(c.material(h.surface).strength_Pa, 111.0, 1e-9); // routed to near_, not far_

    // A segment that only reaches the far plane.
    CHECK(c.raycast({5, 0, 0}, {10, 0, 0}, h));
    CHECK_NEAR(h.point.x, 8.0, 1e-9);
    CHECK_NEAR(c.material(h.surface).strength_Pa, 222.0, 1e-9);
}

int main(int argc, char** argv) {
    return ::pontest::run_all(argc > 1 ? argv[1] : nullptr);
}
