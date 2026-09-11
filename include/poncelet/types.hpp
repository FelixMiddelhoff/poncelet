// poncelet — core value types, SI unit aliases, fixed enums.
// SPDX-License-Identifier: MIT
//
// The whole public API is SI: metres, seconds, kilograms, kelvin, pascals,
// radians. No imperial anywhere at the boundary (owner decision 4).
#pragma once

#include <cmath>
#include <cstdint>

namespace pon {

// ---------------------------------------------------------------------------
// Scalar. `Real` is the accumulator the float/double core runs on. The
// integrator (src/integrate.cpp) is templated on the accumulator: `Core<Real>`
// is this path, `Core<Fx32>` is the Q32.32 fixed-point BitExact core selected
// by config::Determinism::BitExact. See docs/ballistics-phase-plan.md §3.6.
// ---------------------------------------------------------------------------
using Real = double;

// SI unit aliases — documentation only, all `Real`. They make signatures
// self-describing without a units library.
using Meters       = Real; // m
using Seconds      = Real; // s
using Kilograms    = Real; // kg
using MetersPerSec = Real; // m/s
using Radians      = Real; // rad
using RadPerSec    = Real; // rad/s
using Kelvin       = Real; // K
using Pascals      = Real; // Pa
using KgPerM3      = Real; // kg/m^3

// ---------------------------------------------------------------------------
// Vec3 — tiny POD. No DirectXMath / GLM in the public API; the engine-side
// adapter converts to/from XMFLOAT3.
// ---------------------------------------------------------------------------
struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    friend constexpr Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    friend constexpr Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    friend constexpr Vec3 operator-(Vec3 a)         { return {-a.x, -a.y, -a.z}; }
    friend constexpr Vec3 operator*(Vec3 a, Real s) { return {a.x * s, a.y * s, a.z * s}; }
    friend constexpr Vec3 operator*(Real s, Vec3 a) { return a * s; }
    friend constexpr Vec3 operator/(Vec3 a, Real s) { return {a.x / s, a.y / s, a.z / s}; }

    Vec3& operator+=(Vec3 b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vec3& operator-=(Vec3 b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
};

constexpr Real dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline Real length(Vec3 a)         { return std::sqrt(dot(a, a)); }
constexpr Real length_sq(Vec3 a)   { return dot(a, a); }

inline Vec3 normalized(Vec3 a) {
    const Real len = length(a);
    return len > Real(0) ? a / len : Vec3{};
}

// ---------------------------------------------------------------------------
// Quat — unit quaternion, body→world rotation. Used only by the 6-DOF rigid
// projectile path (PrecisionFlag::SixDOF); every other tier ignores it. `w` is
// the scalar part. Hamilton convention, right-handed.
// ---------------------------------------------------------------------------
struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;

    constexpr Quat() = default;
    constexpr Quat(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Hamilton product a·b (apply b first, then a).
    friend constexpr Quat operator*(Quat a, Quat b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }
    friend constexpr Quat operator*(Quat q, Real s) {
        return {q.w * s, q.x * s, q.y * s, q.z * s};
    }
    friend constexpr Quat operator+(Quat a, Quat b) {
        return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
    }

    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    // Rotate a vector from body frame into world frame.
    Vec3 rotate(Vec3 v) const {
        const Quat p = (*this) * Quat{0, v.x, v.y, v.z} * conjugate();
        return {p.x, p.y, p.z};
    }
    // Rotate a world-frame vector into the body frame.
    Vec3 inv_rotate(Vec3 v) const {
        const Quat p = conjugate() * Quat{0, v.x, v.y, v.z} * (*this);
        return {p.x, p.y, p.z};
    }
};

inline Quat normalized(Quat q) {
    const Real n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return n > Real(0) ? Quat{q.w / n, q.x / n, q.y / n, q.z / n} : Quat{};
}

// Shortest-arc rotation taking unit vector `from` to unit vector `to`.
inline Quat quat_from_to(Vec3 from, Vec3 to) {
    from = normalized(from);
    to   = normalized(to);
    const Real d = dot(from, to);
    if (d >= Real(1) - Real(1e-9)) return Quat{};
    if (d <= Real(-1) + Real(1e-9)) {
        // Anti-parallel: rotate pi about any axis square to `from`.
        Vec3 ax = cross(Vec3{1, 0, 0}, from);
        if (length_sq(ax) < Real(1e-12)) ax = cross(Vec3{0, 1, 0}, from);
        ax = normalized(ax);
        return Quat{0, ax.x, ax.y, ax.z};
    }
    const Vec3 c = cross(from, to);
    const Real s = std::sqrt((Real(1) + d) * Real(2));
    return normalized(Quat{s * Real(0.5), c.x / s, c.y / s, c.z / s});
}

struct Ray {
    Vec3 origin;
    Vec3 dir; // expected normalized
};

// Medium identity (index into Environment::media). Air and water are seeded by
// the registry; games register their own.
using MediumId = std::int32_t;
constexpr MediumId kMediumAir   = 0;
constexpr MediumId kMediumWater = 1;

// ---------------------------------------------------------------------------
// Fixed enums
// ---------------------------------------------------------------------------
namespace config {

// Determinism guarantee requested of a Sim.
//   Loose         — no ordering/RNG guarantees; fastest.
//   PlatformStable — v1 default: same platform + compiler + inputs -> identical
//                    trajectory (fixed timestep, fixed eval order, seeded RNG).
//                    Enough for local replay / demo recording.
//   BitExact      — cross-platform bit-identical: the integrator runs on the
//                   Q32.32 fixed-point core (deterministic sqrt + transcendental
//                   LUTs, including the AdaptiveRKF45 step-size controller's
//                   pow via a dedicated LUT), so the trajectory folds to the
//                   same bits on every OS / compiler / optimisation level. The
//                   per-type drag LUT is still compiled in double at
//                   registerType(), and the guidance law's external-
//                   acceleration term is not yet on the fixed-point path.
enum class Determinism { Loose, PlatformStable, BitExact };

} // namespace config

// Coarse projectile family. This is NOT how you pick "a 9 mm bullet" — it only
// (a) seeds defaults for blank fields and (b) selects the code path. The round
// identity is the rest of ProjectileType (normally from the named catalog).
enum class ProjectileClass {
    Bullet,
    Arrow,
    Bolt,
    Spear,       // javelin / atlatl dart
    ThrownBlade,
    SportsBall,
    Pellet,
    Shell,
    Rock,
    Custom,
};

enum class DragModel {
    ConstantCd,  // arrows, bolts, spears — steady subsonic Cd
    G1,          // flat-base / round-nose bullets, spheres
    G7,          // modern boat-tail bullets
    BallProfile, // per-ball-type Cd(Re) + Cl(spin) table (§3.2)
    CustomCurve, // caller-supplied Cd(Mach) samples
};

// Aerodynamic shape family for the BallProfile system (§3.2). Selects both the
// drag-curve abscissa and, for the non-spherical shapes, an orientation model.
//   Sphere          — Cd(Re) with a drag crisis; Magnus lift Cl(S).
//   ProlateSpheroid — American football / rugby: Cd interpolates between a
//                     clean spiral (long axis ∥ airflow, low Cd) and an
//                     end-over-end tumble (axis ⟂ airflow, high Cd) by the
//                     axis-to-airflow angle; small spiral lift bias.
//   Disc            — ice-hockey puck: a bluff body while airborne (rim/face
//                     drag). The surface-constrained ground slide is left to
//                     the caller / Phase 19.
enum class BallShape {
    Sphere,
    ProlateSpheroid,
    Disc,
};

// Which quantity indexes a compiled drag LUT. G1/G7/custom/constant curves are
// Cd(Mach); the BallProfile spheres are Cd(Reynolds) so the drag crisis lands
// at the right speed for the ball's size and the air's viscosity.
enum class DragAbscissa { Mach, Reynolds };

// How the spin axis behaves for the Magnus term.
enum class SpinAxisMode {
    AlongVelocity, // rifling — axis tracks the velocity vector
    Fixed,         // curveball / football spiral — axis fixed in world space
    Free,          // knuckleball — axis wanders (seeded RNG wobble)
};

// Hit-detection cost/accuracy tier, chosen per shot (§3.4).
enum class FidelityTier {
    Hitscan,      // one raycast, drag-free
    AnalyticDrag, // closed-form drag+gravity -> polyline -> raycast
    Integrated,   // full sub-stepped integration + per-step sweep
};

// Target-material response family (§3.5).
enum class MaterialBehaviour {
    Brittle,  // concrete, glass
    Ductile,  // metal
    Fibrous,  // wood, flesh, cloth
    Membrane, // balloon, drywall paper, foliage
    Granular, // sand
    Fluid,    // water
};

// Impact / flight events emitted into the caller's sink (§3.5 "Output").
enum class EventType {
    SurfaceCrossed,
    Ricochet,
    Embedded,
    Perforated,
    Stopped,
    MediumChanged,
    Popped,
    Shattered,
    TransonicWindow, // flight event: projectile is in the Mach ~0.8..1.2 band
    Expired,         // lifetime / range budget exhausted
    Detonated,       // a fuzed warhead went off (§8a); point/time = detonation,
                     // energy_J = chemical yield, payload_kg = TNT-equiv charge
};

// Opt-in second-order precision effects (§3.7). Default 0 = fast path.
enum class PrecisionFlag : std::uint32_t {
    None            = 0,
    LocalSpeedSound = 1u << 0, // Mach abscissa tracks ISA temperature with altitude
    SpinDrift       = 1u << 1, // Litz closed-form gyroscopic drift
    Coriolis        = 1u << 2, // -2 Omega x v  (needs latitude + northAzimuth)
    AeroJump        = 1u << 3, // crosswind -> one-time vertical muzzle impulse
    AdaptiveRKF45   = 1u << 4, // error-controlled step for the Integrated tier
    TransonicFlag   = 1u << 5, // emit a TransonicWindow event on entering Mach 0.8..1.2
    SixDOF          = 1u << 6, // full rigid-body angular flight (Phase 19 item 1)
};

// Per-instance status bits carried in ProjectileState::flags.
constexpr std::uint32_t kFlagInTransonic   = 1u << 0; // currently in Mach 0.8..1.2
constexpr std::uint32_t kFlagPastTransonic = 1u << 1; // has exited the band once
constexpr std::uint32_t kFlagExpanded      = 1u << 2; // deformable round has mushroomed on a hard hit
constexpr std::uint32_t kFlagTumbling      = 1u << 3; // 6-DOF: yaw exceeded ~60 deg — flight is no longer gyroscopically stable
constexpr std::uint32_t kFlagDetonated     = 1u << 4; // fuzed warhead has detonated (a Detonated event was emitted)

constexpr PrecisionFlag operator|(PrecisionFlag a, PrecisionFlag b) {
    return static_cast<PrecisionFlag>(static_cast<std::uint32_t>(a) |
                                      static_cast<std::uint32_t>(b));
}
constexpr bool has(PrecisionFlag set, PrecisionFlag f) {
    return (static_cast<std::uint32_t>(set) & static_cast<std::uint32_t>(f)) != 0;
}

// Result of a Sim call that can fail without throwing (C-ABI friendly).
enum class Status {
    Ok,
    Unsupported,   // reserved; no Sim path returns this today
    InvalidType,
    InvalidHandle,
    InvalidArg,
};

} // namespace pon
