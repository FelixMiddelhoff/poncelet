// poncelet — geometry-query callback interface. The library never owns
// geometry; the caller implements this against its own broad-phase (§3.4).
// SPDX-License-Identifier: MIT
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/material.hpp"

namespace pon {

struct HitResult {
    Vec3      point;
    Vec3      normal;           // unit, pointing back toward the ray origin side
    Real      t          = 1.0; // fraction along a->b where the hit occurred
    SurfaceId surface    = 0;
    // Target surface velocity at the time of impact, so the stepper can do a
    // swept-vs-swept test rather than sweeping against a stale position.
    Vec3      surfaceVelocity;
};

struct World {
    virtual ~World() = default;

    // Swept query: does the segment a->b hit anything? On a hit, fill `out` and
    // return true. Broad-phase is the caller's problem.
    virtual bool raycast(Vec3 a, Vec3 b, HitResult& out) const = 0;

    // Medium at a point (air / water / caller-defined) for surface-crossing
    // detection between raycast hits.
    virtual MediumId mediumAt(Vec3 p) const = 0;

    // Material of a surface a previous raycast() reported.
    virtual const Material& material(SurfaceId s) const = 0;
};

// A World that hits nothing — free-flight only. Handy for exterior-ballistics
// tests and as the default before the engine adapter is wired.
struct EmptyWorld final : World {
    Material air = material_air();
    bool raycast(Vec3, Vec3, HitResult&) const override { return false; }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override { return air; }
};

} // namespace pon
