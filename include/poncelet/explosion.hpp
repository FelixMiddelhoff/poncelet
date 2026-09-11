// poncelet — explosive blast field: Friedlander overpressure, impulse on rigid
// bodies, underwater & thermobaric variants (Phase 19 item 2, §8a).
// SPDX-License-Identifier: MIT
//
// `Burst` is a standalone value object — no Sim, no projectile. Build one at a
// detonation point (the Sim emits an EventType::Detonated telling you where and
// when, or you place it yourself) and query the blast at any field point or on
// any rigid body. Air bursts use the Kinney & Graham scaled-distance fits for a
// spherical TNT charge; `WarheadDesc::underwater` switches to Cole similitude.
#pragma once

#include "poncelet/types.hpp"
#include "poncelet/warhead.hpp"
#include "poncelet/environment.hpp"
#include "poncelet/world.hpp"

namespace pon {

// Friedlander positive-phase description of the shock at one field point. The
// negative (suction) phase is not modelled.
struct BlastSample {
    Meters  standoff_m               = 0.0;
    Real    scaledDistance           = 0.0; // Z = R / W^(1/3)  (m·kg^-1/3)
    Seconds arrivalTime_s            = 0.0; // after the detonation instant
    Pascals peakOverpressure_Pa      = 0.0; // side-on (incident), above ambient
    Pascals reflectedOverpressure_Pa = 0.0; // normal-reflected (face-on)
    Pascals dynamicPressure_Pa       = 0.0; // peak blast-wind stagnation pressure
    Seconds positiveDuration_s       = 0.0;
    Real    waveformDecay            = 0.0; // Friedlander b (dimensionless)
    Real    specificImpulse_Pa_s     = 0.0; // incident positive-phase impulse/area
};

// A rigid body exposed to the blast, for loadOnBody().
struct BlastTarget {
    Vec3      centroid;
    Real      area_m2          = 0.0; // area presented toward the burst
    Kilograms mass_kg          = 0.0; // 0 ⇒ impulse only, no deltaVelocity
    Real      dragCoefficient  = 2.0; // bluff-body Cd for the blast wind
    Real      reflectionFactor = 1.0; // 1 = flat wall; <1 rounded / porous
};

// Positive-phase load delivered to a BlastTarget.
struct BlastLoad {
    Vec3    impulse_Ns;                  // net linear impulse (points away from the burst)
    Vec3    torqueImpulse_Nms;           // about the centroid (0 unless comOffset given)
    Vec3    deltaVelocity_mps;           // impulse / mass (0 when mass_kg <= 0)
    Pascals peakReflected_Pa = 0.0;
    Real    lineOfSight      = 1.0;      // occlusion fraction applied (0..1)
};

class Burst {
public:
    // `env` supplies ambient air density + sound speed (ambient pressure is
    // derived, P0 = ρc²/γ). Ignored when the warhead is `underwater`.
    Burst(Vec3 origin, const WarheadDesc& warhead, const Environment& env,
          Seconds detonationTime_s = 0.0);

    Vec3      origin() const           { return origin_; }
    Seconds   detonationTime_s() const { return t0_; }
    Kilograms effectiveCharge_kg() const { return W_; } // TNT-equiv, incl. boosts
    bool      isUnderwater() const     { return underwater_; }

    // Free-field blast parameters at `point` (no occlusion).
    BlastSample sampleAt(Vec3 point) const;

    // Incident overpressure (Pa, above ambient) at `point` at absolute time `t`
    // (same clock as detonationTime_s). 0 before arrival and after the positive
    // phase; the Friedlander waveform in between.
    Pascals overpressureAt(Vec3 point, Seconds t) const;

    // Positive-phase load on a rigid body. `losFraction` scales the whole load
    // for cover (from blast_line_of_sight); `comOffset` is the centre of mass
    // relative to `target.centroid`, for the torque impulse.
    BlastLoad loadOnBody(const BlastTarget& target, Real losFraction = 1.0,
                         Vec3 comOffset = {}) const;

private:
    Vec3         origin_;
    Seconds      t0_             = 0.0;
    Kilograms    W_              = 0.0;   // effective TNT-equivalent charge (kg)
    Real         cubeRootW_      = 0.0;
    Pascals      ambientP_       = 101325.0;
    KgPerM3      ambientRho_     = 1.225;
    MetersPerSec soundSpeed_     = 340.294;
    bool         underwater_     = false;
    Real         durationStretch_ = 1.0;  // thermobaric positive-phase stretch
    Real         impulseBoost_    = 1.0;  // thermobaric afterburn impulse ×
};

// Fraction of the straight path origin->target left unobstructed by World
// geometry: 1.0 in the clear, or the hit fraction (0..1) when something is in
// the way. A one-ray cover gate — "behind a wall" vs "in the open" — not a
// diffraction model.
Real blast_line_of_sight(const World& world, Vec3 origin, Vec3 target);

} // namespace pon
