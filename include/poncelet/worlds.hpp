// poncelet — a handful of trivial World implementations for content
// prototyping. SPDX-License-Identifier: MIT
//
// Only EmptyWorld (world.hpp) ships in the core: the library never owns
// geometry, the caller always implements World against its own broad-phase.
// But a dev prototyping terminal ballistics — "what does this round do to one
// plate / one sandbag / a stack of boards" — shouldn't have to write a whole
// World before seeing a single ricochet. These are that: one primitive each,
// header-only, no new .cpp. Reach for your engine's own World adapter for
// real geometry; these are for a test scene or the cookbook.
#pragma once

#include "poncelet/world.hpp"

#include <cmath>
#include <vector>

namespace pon {

// One infinite plane: a point on it + a unit outward normal. `mat` is
// returned for every hit — set it for a real surface (a bare PlaneWorld{}
// defaults to material_air(), i.e. detectable but physically inert).
struct PlaneWorld final : World {
    Vec3     position{0, 0, 0};
    Vec3     normal{0, 1, 0}; // unit; the outward / "up" side
    Material mat = material_air();

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        const Real da = dot(a - position, normal);
        const Real db = dot(b - position, normal);
        if ((da > Real(0)) == (db > Real(0))) return false; // no crossing
        const Real t = da / (da - db);
        out.t = t;
        out.point = a + (b - a) * t;
        out.normal = da > Real(0) ? normal : -normal;
        out.surface = 0;
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override { return mat; }
};

// One sphere. A raycast that enters it reports the near intersection.
struct SphereWorld final : World {
    Vec3     center{0, 0, 0};
    Real     radius_m = 1.0;
    Material mat = material_air();

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        const Vec3 d = b - a;
        const Vec3 m = a - center;
        const Real A = dot(d, d);
        if (A <= Real(0)) return false;
        const Real B = Real(2) * dot(m, d);
        const Real C = dot(m, m) - radius_m * radius_m;
        const Real disc = B * B - Real(4) * A * C;
        if (disc < Real(0)) return false;
        const Real sq = std::sqrt(disc);
        Real t = (-B - sq) / (Real(2) * A);
        if (t < Real(0) || t > Real(1)) {
            t = (-B + sq) / (Real(2) * A);
            if (t < Real(0) || t > Real(1)) return false;
        }
        out.t = t;
        out.point = a + d * t;
        out.normal = (out.point - center) * (Real(1) / radius_m);
        out.surface = 0;
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override { return mat; }
};

// One axis-aligned box (the slab method). A raycast that enters it reports
// the near face.
struct AabbWorld final : World {
    Vec3     boundsMin{-1, -1, -1};
    Vec3     boundsMax{1, 1, 1};
    Material mat = material_air();

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        const Vec3 d = b - a;
        Real tmin = Real(0), tmax = Real(1);
        Vec3 nrm{0, 0, 0};
        if (!clip(a.x, d.x, boundsMin.x, boundsMax.x, Vec3{1, 0, 0}, tmin, tmax, nrm)) return false;
        if (!clip(a.y, d.y, boundsMin.y, boundsMax.y, Vec3{0, 1, 0}, tmin, tmax, nrm)) return false;
        if (!clip(a.z, d.z, boundsMin.z, boundsMax.z, Vec3{0, 0, 1}, tmin, tmax, nrm)) return false;
        if (tmin < Real(0) || tmin > Real(1)) return false;
        out.t = tmin;
        out.point = a + d * tmin;
        out.normal = nrm;
        out.surface = 0;
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId) const override { return mat; }

private:
    // Clip [tmin, tmax] against one axis's slab [lo, hi]; updates `nrm` to
    // this axis's `axisNormal` (signed toward the ray origin) when this axis
    // tightens tmin. False ⇒ the segment misses the box entirely.
    static bool clip(Real a0, Real d0, Real lo, Real hi, Vec3 axisNormal,
                     Real& tmin, Real& tmax, Vec3& nrm) {
        if (std::fabs(d0) < Real(1e-12)) return a0 >= lo && a0 <= hi;
        Real t1 = (lo - a0) / d0, t2 = (hi - a0) / d0;
        Real sign = Real(-1);
        if (t1 > t2) { std::swap(t1, t2); sign = Real(1); }
        if (t1 > tmin) { tmin = t1; nrm = axisNormal * sign; }
        if (t2 < tmax) tmax = t2;
        return tmin <= tmax;
    }
};

// N parallel planes along one shared axis, each with its own Material — the
// "stack of boards" / "board then oak block" terminal-ballistics setup
// (cookbook recipe 4 / docs/examples/terminal_ballistics.cpp), promoted to a
// reusable primitive. Slabs need not be sorted; the nearest crossing wins.
struct SlabStackWorld final : World {
    struct Slab {
        Real     distance; // along `axis` from the origin
        Material mat;
    };
    Vec3              axis{1, 0, 0}; // unit; shared by every slab
    std::vector<Slab> slabs;

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        const Real da0 = dot(a, axis), db0 = dot(b, axis);
        Real best = Real(1);
        int idx = -1;
        for (std::size_t i = 0; i < slabs.size(); ++i) {
            const Real da = da0 - slabs[i].distance, db = db0 - slabs[i].distance;
            if ((da > Real(0)) == (db > Real(0))) continue;
            const Real t = da / (da - db);
            if (t >= Real(0) && t < best) { best = t; idx = static_cast<int>(i); }
        }
        if (idx < 0) return false;
        out.t = best;
        out.point = a + (b - a) * best;
        out.normal = da0 - slabs[idx].distance < Real(0) ? -axis : axis;
        out.surface = static_cast<SurfaceId>(idx);
        out.surfaceVelocity = {0, 0, 0};
        return true;
    }
    MediumId mediumAt(Vec3) const override { return kMediumAir; }
    const Material& material(SurfaceId s) const override { return slabs[s].mat; }
};

// Holds pointers to other Worlds (borrowed — they must outlive this) and
// returns whichever reports the nearest hit. Lets a scene compose primitives
// ("a plate behind a sandbag wall") without writing a bespoke World.
//
// material() must later route back to the SAME sub-world that produced a
// given hit's SurfaceId, not just guess — so raycast() packs the sub-world's
// index into the id's top byte (`worlds.size() <= 255`; the sub-world's own
// SurfaceId must fit the remaining 24 bits, which every world in this header
// does — they only ever report 0 or a small slab index).
struct CompositeWorld final : World {
    std::vector<const World*> worlds;

    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        HitResult best{};
        Real bestT = Real(1) + Real(1); // > any valid t
        int bestWorld = -1;
        for (std::size_t i = 0; i < worlds.size(); ++i) {
            HitResult h;
            if (worlds[i]->raycast(a, b, h) && h.t < bestT) {
                best = h; bestT = h.t; bestWorld = static_cast<int>(i);
            }
        }
        if (bestWorld < 0) return false;
        best.surface = (static_cast<SurfaceId>(bestWorld) << kWorldShift) |
                       (best.surface & kSurfaceMask);
        out = best;
        return true;
    }
    // First sub-world that reports a non-air medium wins; air if none do (or
    // the list is empty). Good enough for "a few solids in open air" scenes —
    // a caller with overlapping media should write a real World.
    MediumId mediumAt(Vec3 p) const override {
        for (const World* w : worlds) {
            const MediumId m = w->mediumAt(p);
            if (m != kMediumAir) return m;
        }
        return kMediumAir;
    }
    const Material& material(SurfaceId s) const override {
        const std::size_t wi = s >> kWorldShift;
        static const Material fallback = material_air();
        if (wi >= worlds.size()) return fallback;
        return worlds[wi]->material(s & kSurfaceMask);
    }

private:
    static constexpr SurfaceId kWorldShift  = 24;
    static constexpr SurfaceId kSurfaceMask = (SurfaceId(1) << kWorldShift) - 1;
};

} // namespace pon
