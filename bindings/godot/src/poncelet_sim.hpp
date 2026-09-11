// poncelet — Godot 4 GDExtension sample: a PonceletSim node.
// SPDX-License-Identifier: MIT
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <poncelet/poncelet.hpp>

#include <memory>

namespace poncelet_godot {

// A RefCounted wrapper around pon::Sim. Create one per weapon system (or one for
// the whole scene); register the rounds you fire once at load, then fire()/step()
// each frame and read positions back. Geometry / hit detection stays on the
// Godot side — this only flies the projectile.
//
// poncelet's world frame is SI metres, Y up — Godot's metre-scale default — so
// Vector3s pass straight through.
class PonceletSim : public godot::RefCounted {
    GDCLASS(PonceletSim, godot::RefCounted)

public:
    PonceletSim();
    ~PonceletSim() override = default;

    // Register a rifle-type round (G7 drag). Returns a type id for fire(), or -1.
    int register_bullet(const godot::String &id, double mass_kg, double diameter_m,
                        double g7_bc, double muzzle_mps);
    // Register a round from poncelet's shipped catalog by id
    // ("9x19_124gr_fmj", "762x51_175gr_smk", "50bmg_660gr_fmj", ...). -1 if unknown.
    int register_catalog(const godot::String &id);

    // Spawn one shot from `muzzle` toward `aim` (need not be unit) at the round's
    // muzzle speed. Returns a state id for the getters, or -1.
    int fire(int type_id, godot::Vector3 muzzle, godot::Vector3 aim);

    // Advance every live shot by `delta` seconds. Call once per frame.
    void step(double delta);

    godot::Vector3 get_position(int state_id) const;
    godot::Vector3 get_velocity(int state_id) const;
    bool           is_alive(int state_id) const;
    int            live_count() const;
    void           despawn(int state_id);

    // Predicted flight path for an aim indicator — no shot spawned. `dt` is the
    // sample spacing, `max_time` the horizon, `ground_y` a floor to stop at.
    godot::PackedVector3Array preview_arc(int type_id, godot::Vector3 muzzle,
                                          godot::Vector3 aim, double dt,
                                          double max_time, double ground_y) const;

    // One-line human summary of a shot, for an on-screen debug readout.
    godot::String describe(int state_id) const;

    void set_atmosphere(double altitude_m); // ISA; 0 = sea level
    void set_gravity(godot::Vector3 g);

protected:
    static void _bind_methods();

private:
    std::unique_ptr<pon::Sim> sim_;
    pon::EmptyWorld           world_;   // poncelet needs a World; geometry is Godot's job
    pon::VectorEventSink      sink_;    // drained each step()
};

} // namespace poncelet_godot
