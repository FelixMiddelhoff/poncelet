// poncelet — Godot 4 GDExtension sample: PonceletSim implementation.
// SPDX-License-Identifier: MIT
#include "poncelet_sim.hpp"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

namespace poncelet_godot {

namespace {
inline pon::Vec3 to_pon(Vector3 v) { return {v.x, v.y, v.z}; }
inline Vector3   to_godot(pon::Vec3 v) {
    return Vector3(static_cast<real_t>(v.x), static_cast<real_t>(v.y),
                   static_cast<real_t>(v.z));
}
inline std::string to_std(const String &s) { return s.utf8().get_data(); }
constexpr uint32_t kInvalid = 0xFFFFFFFFu;
} // namespace

PonceletSim::PonceletSim() : sim_(std::make_unique<pon::Sim>()) {}

int PonceletSim::register_bullet(const String &id, double mass_kg, double diameter_m,
                                 double g7_bc, double muzzle_mps) {
    pon::ProjectileType t;
    t.id                   = to_std(id);
    t.klass                = pon::ProjectileClass::Bullet;
    t.dragModel            = pon::DragModel::G7;
    t.ballisticCoefficient = g7_bc;
    t.mass_kg              = mass_kg;
    t.refDiameter_m        = diameter_m;
    if (muzzle_mps > 0.0) t.muzzleSpeed_mps = muzzle_mps;
    const uint32_t tid = sim_->registerType(std::move(t));
    return tid == pon::kInvalidType ? -1 : static_cast<int>(tid);
}

int PonceletSim::register_catalog(const String &id) {
    const std::string s = to_std(id);
    if (!pon::catalog::has(s)) return -1;
    const uint32_t tid = sim_->registerType(pon::catalog::get(s));
    return tid == pon::kInvalidType ? -1 : static_cast<int>(tid);
}

int PonceletSim::fire(int type_id, Vector3 muzzle, Vector3 aim) {
    if (type_id < 0) return -1;
    pon::LaunchParams lp;
    lp.position  = to_pon(muzzle);
    lp.direction = to_pon(aim);
    lp.tier      = pon::FidelityTier::Integrated;
    const uint32_t sid = sim_->spawn(static_cast<uint32_t>(type_id), lp);
    return sid == kInvalid ? -1 : static_cast<int>(sid);
}

void PonceletSim::step(double delta) {
    if (delta <= 0.0) return;
    sink_.events.clear();
    sim_->step(delta, world_, sink_);
}

Vector3 PonceletSim::get_position(int state_id) const {
    return state_id < 0 ? Vector3()
                        : to_godot(sim_->state(static_cast<uint32_t>(state_id)).position);
}

Vector3 PonceletSim::get_velocity(int state_id) const {
    return state_id < 0 ? Vector3()
                        : to_godot(sim_->state(static_cast<uint32_t>(state_id)).velocity);
}

bool PonceletSim::is_alive(int state_id) const {
    return state_id >= 0 && sim_->state(static_cast<uint32_t>(state_id)).alive;
}

int PonceletSim::live_count() const { return static_cast<int>(sim_->liveCount()); }

void PonceletSim::despawn(int state_id) {
    if (state_id >= 0) sim_->despawn(static_cast<uint32_t>(state_id));
}

PackedVector3Array PonceletSim::preview_arc(int type_id, Vector3 muzzle, Vector3 aim,
                                            double dt, double max_time,
                                            double ground_y) const {
    PackedVector3Array out;
    if (type_id < 0) return out;
    const pon::ProjectileType &t = sim_->type(static_cast<uint32_t>(type_id));
    if (t.id.empty()) return out;
    pon::LaunchParams lp;
    lp.position  = to_pon(muzzle);
    lp.direction = to_pon(aim);
    std::vector<pon::Vec3> arc;
    pon::preview_arc(t, lp, sim_->environment(), dt, max_time, arc, ground_y);
    out.resize(static_cast<int64_t>(arc.size()));
    for (int64_t i = 0; i < static_cast<int64_t>(arc.size()); ++i)
        out[i] = to_godot(arc[static_cast<size_t>(i)]);
    return out;
}

String PonceletSim::describe(int state_id) const {
    if (state_id < 0) return String();
    const uint32_t sid = static_cast<uint32_t>(state_id);
    const pon::ProjectileState &s = sim_->state(sid);
    const pon::ProjectileType  &t = sim_->type(s.typeId);
    return String(pon::describe(s, t, sim_->environment()).c_str());
}

void PonceletSim::set_atmosphere(double altitude_m) {
    sim_->environment().setAtmosphere({altitude_m});
}

void PonceletSim::set_gravity(Vector3 g) { sim_->environment().gravity = to_pon(g); }

void PonceletSim::_bind_methods() {
    ClassDB::bind_method(D_METHOD("register_bullet", "id", "mass_kg", "diameter_m",
                                  "g7_bc", "muzzle_mps"),
                         &PonceletSim::register_bullet);
    ClassDB::bind_method(D_METHOD("register_catalog", "id"),
                         &PonceletSim::register_catalog);
    ClassDB::bind_method(D_METHOD("fire", "type_id", "muzzle", "aim"),
                         &PonceletSim::fire);
    ClassDB::bind_method(D_METHOD("step", "delta"), &PonceletSim::step);
    ClassDB::bind_method(D_METHOD("get_position", "state_id"),
                         &PonceletSim::get_position);
    ClassDB::bind_method(D_METHOD("get_velocity", "state_id"),
                         &PonceletSim::get_velocity);
    ClassDB::bind_method(D_METHOD("is_alive", "state_id"), &PonceletSim::is_alive);
    ClassDB::bind_method(D_METHOD("live_count"), &PonceletSim::live_count);
    ClassDB::bind_method(D_METHOD("despawn", "state_id"), &PonceletSim::despawn);
    ClassDB::bind_method(D_METHOD("preview_arc", "type_id", "muzzle", "aim", "dt",
                                  "max_time", "ground_y"),
                         &PonceletSim::preview_arc);
    ClassDB::bind_method(D_METHOD("describe", "state_id"), &PonceletSim::describe);
    ClassDB::bind_method(D_METHOD("set_atmosphere", "altitude_m"),
                         &PonceletSim::set_atmosphere);
    ClassDB::bind_method(D_METHOD("set_gravity", "g"), &PonceletSim::set_gravity);
}

} // namespace poncelet_godot
