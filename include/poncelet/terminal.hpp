// poncelet — terminal-ballistics event output.
// SPDX-License-Identifier: MIT
//
// Item 1: the event record + sink interface. The resolution pipeline
// (ricochet -> V_bl -> embed/perforate -> channel) is item 7.
#pragma once

#include "poncelet/types.hpp"

#include <vector>

namespace pon {

using TypeId = std::uint32_t;

struct Event {
    EventType    type;
    Vec3         point;
    Vec3         normal;
    Seconds      time_s        = 0.0;
    // Kinetic energy deposited into the surface by this interaction (J).
    Real         energy_J      = 0.0;
    // Set for Perforated / Ricochet / MediumChanged: the projectile's speed
    // after the interaction.
    MetersPerSec residualSpeed_mps = 0.0;
    // Penetration channel (Embedded, and the near-face channel of a Perforated):
    // entry point (`point`) + `channelAxis` (unit, direction of travel) + depth.
    Vec3         channelAxis;
    Meters       channelDepth_m    = 0.0;
    // Extra channel radius past the entry diameter once a long projectile starts
    // to yaw in soft media (L_yaw ~ 12 d, §3.8). 0 for a clean straight channel.
    Meters       channelWiden_m    = 0.0;
    // Linear impulse imparted to the struck body by a kinetic interaction —
    // the projectile's momentum change m·Δv, in N·s, pointing along the line of
    // travel (into the surface for an embed / perforation, partly back out for a
    // ricochet). Feeds an engine's rigid-body / destruction response. Zero for a
    // Detonated event (build a pon::Burst for the blast load) and pure-flight
    // events.
    Vec3         impulse_Ns;
    // Detonated only: the effective TNT-equivalent charge (kg), including the
    // surface-burst and thermobaric yield boosts. Build a pon::Burst from the
    // type's WarheadDesc for the full blast field.
    Kilograms    payload_kg        = 0.0;
    TypeId       projectile        = 0;
};

// The library writes events into a caller-owned buffer — no heap allocation in
// the step (§5). A std::vector sink is provided for tests / non-hot callers.
struct EventSink {
    virtual ~EventSink() = default;
    virtual void emit(const Event& e) = 0;
};

struct VectorEventSink final : EventSink {
    std::vector<Event> events;
    void emit(const Event& e) override { events.push_back(e); }
};

} // namespace pon
