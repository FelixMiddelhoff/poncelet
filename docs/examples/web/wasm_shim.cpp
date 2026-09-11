// poncelet — WASM demo shim: flat-scalar wrappers over the C++ API so the
// browser-side JS glue can use Emscripten's plain `ccall` (no embind, no
// struct marshalling on the JS side). Uses pon::preview_arc directly (not
// the C ABI's pon_preview_arc, which has no per-call speed override) so the
// demo's speed slider actually reaches the simulation. Not part of the
// library; build.ps1 compiles this file plus the prebuilt poncelet static
// lib into poncelet_demo.js/.wasm.
// SPDX-License-Identifier: MIT
#include <poncelet/poncelet.hpp>

#include <cstddef>
#include <vector>

extern "C" {

// One Sim + one registered type for the whole page — plenty for a
// single-shot aim-preview demo.
static pon::Sim* g_sim = nullptr;

int wasm_init() {
    delete g_sim;
    g_sim = new pon::Sim();
    return g_sim != nullptr;
}

// Registers a ConstantCd bullet type. Returns a type id, or 0xFFFFFFFF.
unsigned int wasm_register_bullet(const char* id, double mass_kg, double ref_diameter_m,
                                  double drag_coefficient, double muzzle_speed_mps) {
    if (!g_sim) return 0xFFFFFFFFu;
    pon::ProjectileType t;
    t.id = id;
    t.klass = pon::ProjectileClass::Bullet;
    t.dragModel = pon::DragModel::ConstantCd;
    t.mass_kg = mass_kg;
    t.refDiameter_m = ref_diameter_m;
    t.dragCoefficient = drag_coefficient;
    t.muzzleSpeed_mps = muzzle_speed_mps;
    const pon::TypeId id2 = g_sim->registerType(t);
    return id2 == pon::kInvalidType ? 0xFFFFFFFFu : static_cast<unsigned int>(id2);
}

// Fills `out` (a caller-allocated double[max*3], x/y/z per point — allocate
// it with Module._malloc from JS) with the predicted arc; returns the point
// count actually written (<= max). `speed_mps <= 0` uses the type's own
// muzzle speed instead of overriding it.
size_t wasm_preview_arc(unsigned int type_id, double speed_mps,
                        double mx, double my, double mz,
                        double ax, double ay, double az,
                        double dt_s, double max_time_s, double ground_y,
                        double* out, size_t max) {
    if (!g_sim || !out) return 0;
    const pon::ProjectileType& type = g_sim->type(static_cast<pon::TypeId>(type_id));

    pon::LaunchParams lp;
    lp.position = {mx, my, mz};
    lp.direction = {ax, ay, az};
    if (speed_mps > 0.0) lp.speed = speed_mps;

    std::vector<pon::Vec3> arc;
    pon::preview_arc(type, lp, g_sim->environment(), dt_s, max_time_s, arc, ground_y);

    const size_t n = arc.size() < max ? arc.size() : max;
    for (size_t i = 0; i < n; ++i) {
        out[i * 3 + 0] = arc[i].x;
        out[i * 3 + 1] = arc[i].y;
        out[i * 3 + 2] = arc[i].z;
    }
    return n;
}

const char* wasm_last_error() {
    return g_sim ? g_sim->lastError() : "";
}

} // extern "C"
