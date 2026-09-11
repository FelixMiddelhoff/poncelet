// poncelet — projectile description (registered once) and per-instance state.
// SPDX-License-Identifier: MIT
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/drag.hpp"
#include "poncelet/warhead.hpp"
#include "poncelet/guidance.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pon {

using TypeId = std::uint32_t;
constexpr TypeId kInvalidType = 0xFFFFFFFFu;

// Terminal-ballistics parameters of a projectile (§3.5). Split out so the
// exterior-only fields stay readable.
struct TerminalParams {
    // Nose-shape resistance factor N*: sharp broadhead ~0.4 << round-nose ~0.9
    // << flat ~1.3. 0 ⇒ filled from the class default (§4).
    Real  noseShapeFactor = 0.0;
    Real  hardness        = 1.0;   // relative; drives deformation threshold
    bool  deformable      = false; // lead mushrooms, JHP expands -> loses SD
    bool  fragile         = false; // thrown pot shatters, deposits all energy
    // Front-of-centre balance for arrows: folded into a small Cd multiplier and
    // a penetration bonus (§3.2). 0 = neutral.
    Real  focBias         = 0.0;
};

// Rigid-body / angular aerodynamics of an axisymmetric projectile, read only by
// the 6-DOF flight path (PrecisionFlag::SixDOF, Phase 19 item 1). Every field
// left at 0 is derived at spawn from mass + diameter + class (a solid-body
// inertia estimate, a class length/diameter ratio, and the seed coefficient set
// below). Coefficients are dimensionless, per radian of yaw, on the reference
// area π/4·d² and reference length `length_m`; they are seeds — a game tunes
// them per round against range data (see docs/ballistics-phase-plan.md §8a).
struct AeroAngular {
    Real length_m   = 0.0;   // 0 ⇒ classLengthOverDiameter · refDiameter_m
    Real axialInertia_kgm2      = 0.0; // Ix about the spin axis; 0 ⇒ derived
    Real transverseInertia_kgm2 = 0.0; // It about a diameter;    0 ⇒ derived
    // Static (overturning) pitching-moment slope C_Mα. > 0 ⇒ the aero moment
    // rotates the nose away from the velocity vector (statically unstable on its
    // own — spin stabilises it). 0 ⇒ seed 2.5.
    Real overturningMomentSlope = 0.0;
    // Pitch-damping moment sum (C_Mq + C_Mα̇), non-dimensionalised by d/2V. < 0
    // damps the coning/nutation. 0 ⇒ seed -6.0.
    Real pitchDampingMoment     = 0.0;
    // Magnus-moment slope C_Mpα (per radian, × spin·d/2V). Drives the yaw of
    // repose that becomes spin drift. 0 ⇒ seed 0.6 (sign follows the spin).
    Real magnusMomentSlope      = 0.0;
    // Roll-damping coefficient C_lp (< 0). Bleeds spin over the flight. 0 ⇒
    // seed -0.02.
    Real rollDampingMoment      = 0.0;
    // Lift-force slope C_Lα (per radian). The yaw-induced side force that curves
    // the trajectory toward the nose. 0 ⇒ seed 2.0.
    Real liftForceSlope         = 0.0;
    // Yaw-drag factor: ΔC_D = yawDragFactor · α². 0 ⇒ seed 4.0.
    Real yawDragFactor          = 0.0;
    // Transonic instability: C_Mα is multiplied by this in Mach 0.9..1.2. 0 ⇒
    // seed 1.0 (no bump); 1.5–2.0 makes a marginally-stable round tumble through
    // the transonic window.
    Real transonicOverturnMul   = 0.0;
};

// Registered once at content-load, keyed by `id`, holds everything needed to
// precompute the per-type acceleration LUTs. Fields left as std::nullopt / 0
// are filled from the class defaults or the catalog entry (§3.1).
struct ProjectileType {
    std::string    id;                                  // "9x19_124gr_fmj", ...
    ProjectileClass klass          = ProjectileClass::Custom;
    DragModel       dragModel      = DragModel::ConstantCd;

    Real mass_kg                   = 0.0;  // 0 -> from class/catalog
    Real refDiameter_m             = 0.0;  // reference diameter for the drag area

    // For G1/G7: the scalar BC. Else derived from mass+diameter+form factor.
    std::optional<Real> ballisticCoefficient;
    // For ConstantCd.
    std::optional<Real> dragCoefficient;
    // For dragModel == CustomCurve: caller-supplied Cd(Mach) samples (e.g.
    // Doppler-radar data). Need not be uniformly spaced; used as-is (no BC
    // form-factor scaling). Empty falls back to dragCoefficient.
    std::vector<DragCurvePoint> customDragCurve;
    // For dragModel == BallProfile: id into the BallProfile table (§4).
    std::string ballProfile;

    // Catalog default launch speed; the caller may still override per shot.
    std::optional<MetersPerSec> muzzleSpeed_mps;

    // Magnus / spin.
    std::optional<RadPerSec> spinRate_radps;
    SpinAxisMode             spinAxisMode = SpinAxisMode::AlongVelocity;
    // Gyroscopic spin drift (PrecisionFlag::SpinDrift, §3.7). `twistRate_m` is
    // signed — >0 (or 0) = right-hand twist ⇒ drifts right, <0 = left twist ⇒
    // drifts left. Only its sign is read by the Litz closed form; the magnitude
    // is kept for the item 19 6-DOF upgrade. `millerStability` is the Miller Sg;
    // <= 0 ⇒ a typical 1.8 is assumed.
    Real twistRate_m     = 0.0;
    Real millerStability = 0.0;
    // Default spin axis for SpinAxisMode::Fixed / Free (world frame, need not be
    // normalized). Ignored for AlongVelocity. The caller overrides per shot via
    // LaunchParams::spinAxis. {0,0,0} ⇒ fall back to a horizontal axis square to
    // the launch direction (topspin/backspin plane).
    Vec3 spinAxis = {0, 0, 0};

    TerminalParams terminal;

    // Angular aerodynamics — used only under PrecisionFlag::SixDOF.
    AeroAngular aero;

    // Explosive payload (§8a). chargeMass_kg <= 0 (default) ⇒ inert round, the
    // warhead path is skipped entirely. Otherwise the fuze picks a detonation
    // point/time and step() emits an EventType::Detonated there; build a
    // pon::Burst (<poncelet/explosion.hpp>) from this for the blast field.
    WarheadDesc warhead;

    // Guided munition (Phase 19 item 5). law == None (default) ⇒ unguided, the
    // guidance path is skipped. Otherwise call Sim::guide(id, targetPos,
    // targetVel) each frame; the Sim runs the law and steers via the per-step
    // external-acceleration hook. See <poncelet/guidance.hpp>.
    GuidanceDesc guidance;

    // Optional hard caps so a stray projectile is reaped.
    Seconds maxLifetime_s = 60.0;
    Meters  maxRange_m    = 20000.0;
};

// Pre-flight check: returns a human-readable reason the type will NOT register
// (empty id, no usable mass/diameter and a class default that can't supply one,
// a CustomCurve model with no curve and no fallback Cd, a non-finite or
// negative field, …), or std::nullopt if Sim::registerType() will accept it.
// registerType() runs the same check internally and surfaces the string through
// Sim::lastError().
std::optional<std::string> validate(const ProjectileType& type);

// Per-instance hot data. Stored SoA in batches once the SIMD pass lands
// (WORKPLAN item 12); this struct is the AoS reference layout.
struct ProjectileState {
    Vec3         position;
    Vec3         velocity;
    Real         spin_radps        = 0.0;
    Vec3         spinAxis          = {0, 0, 1};
    std::int32_t mediumId          = 0;   // 0 = air (see MediumRegistry)
    Real         distanceTravelled_m = 0.0;
    Real         timeAlive_s       = 0.0;
    std::uint32_t flags            = 0;
    TypeId       typeId            = kInvalidType;
    FidelityTier tier              = FidelityTier::AnalyticDrag;
    PrecisionFlag precision        = PrecisionFlag::None;
    // Impact-yaw scalar (keyholing, §3.8): 0 for a clean shot, raised after a
    // graze or a perforation; scales presented area + nose factor on the next
    // hit.
    Real         impactYaw         = 0.0;
    // Deformable-round expansion (§3.8): 0 ⇒ use ProjectileType::refDiameter_m.
    // Set to the mushroomed diameter after the first hard hit (kFlagExpanded),
    // which lowers sectional density for every subsequent layer. Terminal-only
    // in v1 — the exterior drag LUT is not recompiled.
    Real         expandedDiameter_m = 0.0;
    bool         alive             = true;

    // --- 6-DOF angular state (PrecisionFlag::SixDOF only; identity/zero for
    // every other tier). `orientation` is body→world with the projectile's nose
    // along body +x; `angVel_radps` is the angular velocity in the BODY frame
    // (x component is the spin rate p). `angleOfAttack_rad` is the last
    // total-yaw angle between the nose and the airflow (informational). ---
    Quat         orientation;
    Vec3         angVel_radps       = {0, 0, 0};
    Real         angleOfAttack_rad  = 0.0;
    Real         spinPhase_rad      = 0.0;  // ∫p — visual roll phase (6-DOF)

    // --- Warhead fuze bookkeeping (ProjectileType::warhead only). ---
    // Time-alive at the first surface contact, or < 0 if the round has not hit
    // anything yet. Drives the Contact / Delayed fuzes.
    Real         fuzeArmTime_s      = -1.0;
    Vec3         fuzePoint;                 // where that first contact happened
    Vec3         fuzeDir;                   // unit flight direction at that contact

    // --- Guidance (ProjectileType::guidance only). ---
    // The seeker track + guidance bookkeeping, driven by Sim::guide(). Plus a
    // generic per-step external acceleration (m/s², world frame) the caller
    // pokes via Sim::setExternalAccel — thrust / scripted course correction /
    // a custom law. Both are added to the force model every sub-step and persist
    // until changed. Zero ⇒ no effect (an unguided round is byte-unchanged).
    GuidanceState guidance;
    Vec3          externalAccel_mps2 = {0, 0, 0};
                                            // (the shaped-charge jet aim / channel
                                            // axis carried on the Detonated event)
};

// --- 6-DOF orientation helpers (PrecisionFlag::SixDOF) ----------------------
//
// `orientation` is identity for every other tier, so these are always defined
// (nose_direction/up_direction return the unrotated body axes, spin_phase
// returns 0) — a renderer doesn't need to branch on the fidelity tier.

// World-frame direction the nose points: body +x rotated into world.
inline Vec3 nose_direction(const ProjectileState& s) {
    return s.orientation.rotate(Vec3{1, 0, 0});
}

// World-frame direction the round's "up" reference (body +y) points — at
// identity orientation this is world +y, so a renderer can use it directly as
// a tracer's up vector without special-casing an unrotated shot.
inline Vec3 up_direction(const ProjectileState& s) {
    return s.orientation.rotate(Vec3{0, 1, 0});
}

// The accumulated roll angle (∫p, radians) — ProjectileState::spinPhase_rad
// under a name that reads at the call site. Unwrapped (grows without bound
// over a long flight); wrap with std::fmod(spin_phase(s), 2*pi) if you only
// want the current roll orientation.
inline Real spin_phase(const ProjectileState& s) { return s.spinPhase_rad; }

// Launch parameters for a single shot.
struct LaunchParams {
    Vec3                        position;
    Vec3                        direction;             // need not be normalized
    std::optional<MetersPerSec> speed;                 // nullopt -> catalog default
    std::optional<RadPerSec>    spin;                  // nullopt -> type default
    // Spin axis for this shot (world frame, need not be normalized). nullopt ⇒
    // the type default / a mode-derived axis. Ignored for AlongVelocity.
    std::optional<Vec3>         spinAxis;
    // Medium this shot starts in (index into Environment::media). nullopt ⇒ air.
    // Set it when spawning already submerged; otherwise the medium is picked up
    // from World::mediumAt on the first step (a MediumChanged event fires).
    std::optional<std::int32_t> medium;
    FidelityTier                tier   = FidelityTier::AnalyticDrag;
    PrecisionFlag               precision = PrecisionFlag::None;
    // Opt-in muzzle dispersion: cone half-angle in milliradians (0 = exact aim,
    // the default). At spawn the aim `direction` is rotated by a random offset
    // inside this cone, uniform in solid angle, drawn from the Sim's seeded RNG
    // (SimConfig::rngSeed) and advanced deterministically per spawn — same seed
    // + same spawn order ⇒ same spread. Round-to-round dispersion is a weapon /
    // shooter property, not physics; this is a convenience, not a model (see the
    // guide). PlatformStable, NOT BitExact — the sample uses std::sin/cos/sqrt;
    // leave it 0 for BitExact server-side hit validation and apply your own
    // fixed-point spread upstream if you need one.
    Real                        precisionMrad = 0.0;
    // 6-DOF only: an initial total-yaw angle (rad) between the nose and the
    // launch velocity — a muzzle tip-off. nullopt ⇒ nose exactly on the
    // velocity vector. The coning that follows decays at the pitch-damping rate
    // (or diverges into a tumble if the round is not gyroscopically stable).
    std::optional<Radians>      initialYaw;
};

} // namespace pon
