// poncelet — ProjectileType pre-flight validation.
// SPDX-License-Identifier: MIT
//
// pon::validate() answers "will Sim::registerType() accept this, and if not,
// why?" before you call it. registerType() runs the same check and hands the
// reason to Sim::lastError(). The messages mirror the guide's "Gotchas" section.
#include "poncelet/projectile.hpp"

#include "defaults.hpp"

#include <cmath>
#include <string>

namespace pon {

namespace {
bool bad(Real v) { return !std::isfinite(v); }
}

std::optional<std::string> validate(const ProjectileType& type) {
    if (type.id.empty())
        return "ProjectileType::id is empty — the id is how the catalog and "
               "overlays key the round; give it a non-empty name.";

    if (bad(type.mass_kg) || type.mass_kg < 0.0)
        return "mass_kg is negative or not finite.";
    if (bad(type.refDiameter_m) || type.refDiameter_m < 0.0)
        return "refDiameter_m is negative or not finite.";

    if (type.ballisticCoefficient &&
        (bad(*type.ballisticCoefficient) || *type.ballisticCoefficient <= 0.0))
        return "ballisticCoefficient is set but is <= 0 or not finite — a BC is "
               "a positive lb/in^2 figure (e.g. G7 0.243).";
    if (type.dragCoefficient &&
        (bad(*type.dragCoefficient) || *type.dragCoefficient < 0.0))
        return "dragCoefficient is set but is negative or not finite.";
    if (type.muzzleSpeed_mps &&
        (bad(*type.muzzleSpeed_mps) || *type.muzzleSpeed_mps < 0.0))
        return "muzzleSpeed_mps is set but is negative or not finite.";

    if (type.dragModel == DragModel::CustomCurve &&
        type.customDragCurve.empty() && !type.dragCoefficient)
        return "dragModel is CustomCurve but customDragCurve is empty and there "
               "is no dragCoefficient fallback — supply Cd(Mach) samples or a "
               "constant dragCoefficient.";

    if (type.dragModel == DragModel::BallProfile && type.ballProfile.empty())
        return "dragModel is BallProfile but ballProfile is empty — name a "
               "profile (e.g. \"baseball\", \"golf_ball\") or use a different "
               "DragModel.";

    // The real gate: after class defaults, is there a usable mass + diameter?
    ProjectileType probe = type;
    if (!detail::apply_class_defaults(probe)) {
        const bool m = probe.mass_kg      > 0.0;
        const bool d = probe.refDiameter_m > 0.0;
        if (!m && !d)
            return "no usable mass_kg or refDiameter_m and the class default "
                   "supplied neither — set both explicitly, or pick a more "
                   "specific ProjectileClass.";
        if (!m)
            return "no usable mass_kg and the class default did not supply one — "
                   "set mass_kg (pon::grains() converts from an imperial figure).";
        return "no usable refDiameter_m and the class default did not supply one "
               "— set refDiameter_m (pon::inches() converts).";
    }

    return std::nullopt;
}

} // namespace pon
