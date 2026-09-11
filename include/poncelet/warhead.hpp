// poncelet — explosive-warhead description and fuze modes (Phase 19 item 2).
// SPDX-License-Identifier: MIT
//
// A `WarheadDesc` on a ProjectileType turns the shot into a live round: the Sim
// picks the detonation point/time from the fuze and emits an `EventType::Detonated`
// there. The blast field itself (Friedlander overpressure, impulse on bodies,
// underwater / thermobaric variants) is `pon::Burst` in <poncelet/explosion.hpp>
// — a standalone object the caller builds from this same WarheadDesc.
#pragma once

#include "poncelet/types.hpp"

namespace pon {

// Spray geometry of a casing break-up (Phase 19 item 3, §8a).
enum class FragmentSpray {
    Isotropic,    // uniform over the full sphere (a symmetric grenade)
    Cone,         // within `coneHalfAngle_rad` of `sprayAxis` (a directional mine
                  // / a nose-fuzed shell throwing forward)
    CylinderBeam, // the equatorial side-spray of a cylindrical casing: a band
                  // `beamHalfWidth_rad` wide about the plane perpendicular to
                  // `sprayAxis`, tilted forward by `beamForwardTilt_rad` for the
                  // casing's own motion (the classic artillery fragment beam)
};

// Casing break-up description carried on a WarheadDesc. Left at
// casingMass_kg <= 0 (default) no fragmentation is produced. The fragment count,
// mass spectrum (Mott), launch speed (Gurney) and spray directions are all
// derived from these fields + the parent warhead's chargeMass_kg; build the
// actual fragment projectiles with pon::generate_fragments / pon::spawn_fragments
// (<poncelet/fragmentation.hpp>) off a Detonated event.
struct FragmentationDesc {
    Kilograms casingMass_kg = 0.0;             // metal that breaks up; <=0 ⇒ none
    // Gurney explosive velocity √(2E′): TNT ≈ 2440, Comp-B ≈ 2680, RDX ≈ 2930,
    // PETN ≈ 2930, ANFO ≈ 1600 m/s. Fragment speed = this ·(M/C + 1/2)^-1/2.
    MetersPerSec gurneyVelocity_mps = 2440.0;
    // Mott mean-fragment-mass parameter µ (kg); mean fragment mass = 2µ, total
    // count ≈ casingMass/(2µ). 0 ⇒ derived from the casing geometry below via
    // µ^½ = B · t^(5/6) · d_i^(1/3) · (1 + t/d_i), or a coarse fallback if that
    // is also unset.
    Kilograms mottMu_kg               = 0.0;
    Meters    casingInnerDiameter_m   = 0.0;
    Meters    casingWallThickness_m   = 0.0;
    Real      mottConstantB           = 1.2;   // SI-fitted seed (fragmentation
                                               // grenade); tune per casing/filler
    Kilograms minFragmentMass_kg      = 2.0e-4;// drop dust below this
    std::uint32_t maxFragments        = 256;   // hard cap; the lightest are cut
    KgPerM3   fragmentDensity_kgm3     = 7850.0;// mild steel
    Real      fragmentDragCd           = 1.10; // irregular tumbling chunk (bluff)
    FragmentSpray spray               = FragmentSpray::Isotropic;
    Vec3      sprayAxis               = {1, 0, 0}; // casing long axis, world frame
    Radians   coneHalfAngle_rad       = 0.5236;   // 30°  (Cone)
    Radians   beamHalfWidth_rad       = 0.3491;   // 20°  (CylinderBeam)
    Radians   beamForwardTilt_rad     = 0.1745;   // 10°  forward drift
    Real      velocityScatter         = 0.15;     // ± fraction, uniform per frag
    std::uint32_t massClasses         = 12;    // spawn_fragments() type buckets
    std::uint64_t seed                = 0xF00DCAFEull; // spray-direction RNG
};

// Shaped-charge / EFP liner type (Phase 19 item 4, §8a).
enum class ShapedChargeType {
    ConicalJet, // HEAT: a deep conical liner collapses into a stretching
                // hypervelocity jet (tip ~7-9 km/s) trailing a slow slug.
                // Penetration ~5-8 charge diameters into steel, but only over a
                // narrow band of standoff (a few CD) — past jet particulation
                // it disperses and the depth falls off.
    EFP,        // explosively formed penetrator: a shallow dish forms a single
                // coherent slug (~1.5-2.5 km/s, L/D ~1-3). ~0.5-1 CD of
                // penetration, but it stays effective to many hundreds of CD
                // of standoff (a self-forging fragment / "Miznay-Schardin").
};

// Shaped-charge payload carried on a WarheadDesc. Left at linerMass_kg <= 0
// (default) the round has no shaped-charge effect and the jet path is skipped.
// The jet / slug formation (Gurney collapse -> tip & tail velocity, jet vs slug
// mass) and the standoff-dependent penetration are derived from these fields +
// the parent warhead's chargeMass_kg; evaluate them with the functions in
// <poncelet/shapedcharge.hpp> off a Detonated event.
struct ShapedChargeDesc {
    Kilograms linerMass_kg      = 0.0;   // metal that forms the jet/slug; <=0 => none
    Meters    chargeDiameter_m  = 0.0;   // CD — the governing length scale; <=0 =>
                                         // coarse fallback from linerMass + density
    ShapedChargeType kind       = ShapedChargeType::ConicalJet;
    Radians   coneApexAngle_rad = 1.0472; // 60° included cone (ConicalJet); ignored for EFP
    KgPerM3   linerDensity_kgm3 = 8960.0; // OFHC copper
    // Gurney explosive velocity √(2E′) of the filler — same scale as
    // FragmentationDesc: TNT ≈ 2440, Comp-B ≈ 2680, RDX/Octol ≈ 2930 m/s.
    MetersPerSec gurneyVelocity_mps = 2680.0;
    // 0 ⇒ derived from the Gurney collapse + cone geometry. Tip velocity for a
    // ConicalJet; slug velocity for an EFP.
    MetersPerSec jetTipVelocity_mps  = 0.0;
    MetersPerSec jetTailVelocity_mps = 0.0;   // ConicalJet slowest jet element; 0 ⇒ derived
    // Jet particulation time for the liner metal (copper ≈ 120-200 µs). Once the
    // jet has stretched past this it is a particle stream — penetration keeps
    // going but efficiency drops with further standoff.
    Seconds   particulationTime_s = 1.6e-4;
    Real      jetMassFraction     = 0.0;  // 0 ⇒ derived from the cone angle
    Radians   spallConeHalfAngle_rad = 0.26; // ~15°  behind-armour spall cone
    std::uint64_t seed = 0x5CEDCA5EULL;   // behind-armour debris RNG
};

// When a fuzed round detonates (§8a).
enum class FuzeMode {
    Contact,       // on the first surface contact
    Delayed,       // `fuzeDelay_s` after the first contact — the emitted
                   // Detonated event carries the future time_s (the round is
                   // spent at the face; the sub-frame penetration depth of a
                   // real delay fuze is not modelled)
    TimedAirburst, // `fuzeDelay_s` after launch, wherever the shot then is
    Proximity,     // when geometry enters `proximityRadius_m` ahead of the shot
};

// Explosive payload of a projectile. Left at chargeMass_kg <= 0 the round is
// inert and nothing in the warhead path runs (zero behaviour change).
struct WarheadDesc {
    Kilograms chargeMass_kg     = 0.0;  // explosive fill mass
    Real      tntEquivalence    = 1.0;  // blast yield relative to TNT
                                        // (TNT 1.0, Comp-B ≈ 1.1, RDX ≈ 1.3,
                                        //  PETN ≈ 1.27, ANFO ≈ 0.82)
    FuzeMode  fuze              = FuzeMode::Contact;
    Seconds   fuzeDelay_s       = 0.0;  // Delayed / TimedAirburst
    Meters    proximityRadius_m = 0.0;  // Proximity
    // Ground / hard-surface burst: the hemispherical reflection nearly doubles
    // the effective free-air yield (standard 1.8× surface-burst factor). Set it
    // for a contact-fuzed shell hitting the ground; leave it off for an airburst.
    bool      surfaceBurst     = false;
    // Underwater detonation — Cole similitude (shock pressure ~50 MPa·(W^⅓/R)^1.13
    // near field, exponential decay) instead of the air Kinney–Graham model.
    bool      underwater       = false;
    // Thermobaric / enhanced-blast: sustained overpressure from afterburning
    // fuel. 0 = conventional; >0 stretches the positive phase (×(1+2·t)) and
    // boosts delivered impulse (×(1+t)), and raises the effective yield ×(1+t).
    Real      thermobaric      = 0.0;
    // Casing break-up (Phase 19 item 3). Inert (no fragments) unless
    // fragmentation.casingMass_kg > 0.
    FragmentationDesc fragmentation{};
    // Shaped charge / EFP (Phase 19 item 4). No jet unless
    // shapedCharge.linerMass_kg > 0.
    ShapedChargeDesc shapedCharge{};
};

// Chemical energy released, joules — TNT-equivalent mass × 4.184 MJ/kg, including
// the thermobaric yield boost. Carried on the Detonated event's `energy_J`.
inline Real warhead_energy_J(const WarheadDesc& w) {
    const Real tnt = w.chargeMass_kg * w.tntEquivalence *
                     (w.thermobaric > 0.0 ? (1.0 + w.thermobaric) : 1.0) *
                     (w.surfaceBurst ? 1.8 : 1.0);
    return tnt * 4.184e6;
}

} // namespace pon
