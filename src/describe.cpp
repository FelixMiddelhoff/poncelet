// poncelet — one-line human summary of a live shot (logging / debug readout).
// SPDX-License-Identifier: MIT
#include "poncelet/sim.hpp"

#include <cmath>
#include <cstdio>

namespace pon {

namespace {

void append_flags(std::string& out, std::uint32_t flags, FidelityTier tier) {
    const char* tierName = tier == FidelityTier::Hitscan      ? "hitscan"
                         : tier == FidelityTier::AnalyticDrag ? "analytic"
                                                              : "integrated";
    char buf[16];
    std::snprintf(buf, sizeof buf, "  [%s", tierName);
    out += buf;
    if (flags & kFlagInTransonic)   out += " transonic";
    if (flags & kFlagPastTransonic) out += " post-transonic";
    if (flags & kFlagExpanded)      out += " expanded";
    if (flags & kFlagTumbling)      out += " tumbling";
    if (flags & kFlagDetonated)     out += " detonated";
    out += "]";
}

// "Cd 0.30" / "G7 BC 0.243" / "profile baseball" / "custom Cd(Mach) curve
// (12 pts)" — the field(s) that actually drive drag for this dragModel.
std::string drag_summary(const ProjectileType& t) {
    char buf[64];
    switch (t.dragModel) {
        case DragModel::ConstantCd:
            std::snprintf(buf, sizeof buf, "Cd %.2f", t.dragCoefficient.value_or(0.0));
            return buf;
        case DragModel::G1:
            std::snprintf(buf, sizeof buf, "G1 BC %.3f", t.ballisticCoefficient.value_or(0.0));
            return buf;
        case DragModel::G7:
            std::snprintf(buf, sizeof buf, "G7 BC %.3f", t.ballisticCoefficient.value_or(0.0));
            return buf;
        case DragModel::BallProfile:
            std::snprintf(buf, sizeof buf, "profile %s",
                          t.ballProfile.empty() ? "?" : t.ballProfile.c_str());
            return buf;
        case DragModel::CustomCurve:
            std::snprintf(buf, sizeof buf, "custom Cd(Mach) curve (%zu pts)",
                          t.customDragCurve.size());
            return buf;
    }
    return "?";
}

// "518 J" / "2.0 kJ" / "0.9 MJ" — auto-scaled so a rifle round and a tank
// round both read naturally.
std::string format_energy_J(double j) {
    char buf[32];
    if (j >= 1.0e6)      std::snprintf(buf, sizeof buf, "%.1f MJ", j / 1.0e6);
    else if (j >= 1.0e3) std::snprintf(buf, sizeof buf, "%.1f kJ", j / 1.0e3);
    else                 std::snprintf(buf, sizeof buf, "%.0f J", j);
    return buf;
}

} // namespace

std::string describe(const ProjectileState& s) {
    const double v = length(s.velocity);
    char buf[192];
    std::snprintf(buf, sizeof buf,
                  "v=%.0f m/s  t=%.2f s  dist=%.0f m  pos=(%.0f,%.0f,%.0f)  spin=%.0f rad/s",
                  v, s.timeAlive_s, s.distanceTravelled_m,
                  s.position.x, s.position.y, s.position.z, s.spin_radps);
    std::string out = buf;
    if (!s.alive) out += "  [dead]";
    append_flags(out, s.flags, s.tier);
    return out;
}

Real mach(const ProjectileState& s, const Environment& env) {
    const Real v = length(s.velocity);
    const Real a = env.speedOfSound_mps > Real(0) ? env.speedOfSound_mps : Real(340.294);
    return v / a;
}

Real kinetic_energy_J(const ProjectileState& s, const ProjectileType& type) {
    if (type.mass_kg <= Real(0)) return Real(0);
    const Real v = length(s.velocity);
    return Real(0.5) * type.mass_kg * v * v;
}

std::string describe(const ProjectileState& s, const ProjectileType& type,
                     const Environment& env) {
    std::string out = describe(s);
    char buf[96];
    const Real e = kinetic_energy_J(s, type);
    if (e > Real(0)) {
        std::snprintf(buf, sizeof buf, "  E=%.0f J", e);
        out += buf;
    }
    std::snprintf(buf, sizeof buf, "  M=%.2f", mach(s, env));
    out += buf;
    return out;
}

std::string describe(const ProjectileType& type) {
    const double v0 = type.muzzleSpeed_mps.value_or(0.0);
    const double e0 = 0.5 * type.mass_kg * v0 * v0;
    char buf[256];
    std::snprintf(buf, sizeof buf, "%s: %.1f g, %.1f mm, %s, ~%.0f m/s, ~%s muzzle",
                  type.id.empty() ? "?" : type.id.c_str(),
                  type.mass_kg * 1000.0, type.refDiameter_m * 1000.0,
                  drag_summary(type).c_str(), v0, format_energy_J(e0).c_str());
    return buf;
}

} // namespace pon
