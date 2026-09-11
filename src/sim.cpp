// poncelet — Sim front end.
// SPDX-License-Identifier: MIT
//
// Item 2: the exterior-ballistics core. Each live projectile is advanced per
// frame through one of three fidelity tiers (§3.4):
//   * Hitscan       — one straight, drag-free raycast over the frame.
//   * AnalyticDrag   — the closed-form linear-drag trajectory (src/integrate.*)
//                      sampled to a polyline and swept against the World.
//   * Integrated     — fixed-step RK4 / semi-implicit Euler with a deterministic
//                      adaptive sub-step count and a swept query per sub-step.
// The FP-sensitive arithmetic all lives in src/integrate.cpp; this file only
// orchestrates, resolves events, and rebuilds the per-medium force model.
#include "poncelet/sim.hpp"

#include "ball_profiles.hpp"
#include "defaults.hpp"
#include "drag_tables.hpp"
#include "fixed_lut.hpp"
#include "integrate.hpp"
#include "terminal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace pon {

namespace {
constexpr Real kNuFallback = 1.4607e-5; // ISA sea-level kinematic viscosity, m^2/s
constexpr Real kEarthRate_radps = 7.2921159e-5;
constexpr Real kTropoLapse_KpM  = -0.0065; // ISA troposphere dT/dh

// Horizontal "right" of a heading for a viewer looking down the shot with world
// up = +y  (right = up × forward). For a +x shot this is -z.
Vec3 right_of(Vec3 dir) {
    Vec3 r = cross(Vec3{0, 1, 0}, normalized(dir));
    const Real l2 = length_sq(r);
    return l2 > Real(1e-12) ? r * (Real(1) / std::sqrt(l2)) : Vec3{0, 0, -1};
}

// Distance-travelled bookkeeping (BitExact-hash-visible, but never fed back
// into the physics) needs the same FMA immunity as the rest of this TU, and
// -ffp-contract=off turned out NOT to be enough on its own: a real
// cross-platform CI run caught macOS's *Debug* build (Apple Clang -O0)
// still fusing dot()'s `a.x*b.x + a.y*b.y + a.z*b.z` into hardware FMA
// despite the flag — Release (-O2) was NOT affected, only -O0's codegen
// path. Since AArch64 has no ISA-level way to disable FMA (it's baseline,
// unlike x86's optional FMA3), the only fix that can't be silently ignored
// by a compiler/opt-level quirk is a source-level one: routing each product
// through a `volatile` forces it to materialise before the add, which
// makes fusion impossible on any backend at any optimisation level.
Real dist_no_fma(Vec3 a, Vec3 b) {
    const volatile Real dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const volatile Real xx = dx * dx, yy = dy * dy, zz = dz * dz;
    const Real sum = xx + yy + zz;
    return std::sqrt(sum);
}

// Same story, same fix, for the 6-DOF writeBack's orientation Quat multiply
// (quat_from_to(...) * Quat{rollCos, rollSin, 0, 0}): a real cross-platform
// CI run caught orientation.y/orientation.z off by 1-2 ULP on isolated
// frames on macOS Debug — never persisting (orientation is rebuilt fresh
// each frame from the otherwise-exact nose/roll, so the fused rounding
// doesn't accumulate), but still visible in the hashed state. Quat's
// `a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z`-style components are the same
// a*b+c*d contraction bait as dot(), so the same volatile-materialise
// barrier applies, one term at a time.
Quat quat_mul_no_fma(Quat a, Quat b) {
    const volatile Real aw_bw = a.w * b.w, ax_bx = a.x * b.x,
                        ay_by = a.y * b.y, az_bz = a.z * b.z;
    const volatile Real aw_bx = a.w * b.x, ax_bw = a.x * b.w,
                        ay_bz = a.y * b.z, az_by = a.z * b.y;
    const volatile Real aw_by = a.w * b.y, ax_bz = a.x * b.z,
                        ay_bw = a.y * b.w, az_bx = a.z * b.x;
    const volatile Real aw_bz = a.w * b.z, ax_by = a.x * b.y,
                        ay_bx = a.y * b.x, az_bw = a.z * b.w;
    return {aw_bw - ax_bx - ay_by - az_bz,
            aw_bx + ax_bw + ay_bz - az_by,
            aw_by - ax_bz + ay_bw + az_bx,
            aw_bz + ax_by - ay_bx + az_bw};
}

// Rotate `v` by `ang` radians about unit axis `k` (Rodrigues).
Vec3 rotate_axis(Vec3 v, Vec3 k, Real ang) {
    const Real cs = std::cos(ang), sn = std::sin(ang);
    return v * cs + cross(k, v) * sn + k * (dot(k, v) * (Real(1) - cs));
}

// splitmix64 draw, advancing `s` in place — the same generator the
// fragmentation / shaped-charge sprays use, so every seeded draw in poncelet
// is one stream.
std::uint64_t splitmix64(std::uint64_t& s) {
    s += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// Rotate unit `dir` to a random heading inside a cone of half-angle
// `halfAngle_rad`, uniform in solid angle. Two draws from `rng`. PlatformStable,
// not BitExact (std::cos/sin/sqrt). `halfAngle_rad` is assumed small and > 0.
Vec3 disperse_cone(Vec3 dir, Real halfAngle_rad, std::uint64_t& rng) {
    const Real u = Real(splitmix64(rng) >> 11) * (Real(1) / Real(9007199254740992.0));
    const Real v = Real(splitmix64(rng) >> 11) * (Real(1) / Real(9007199254740992.0));
    const Real cosMax = std::cos(halfAngle_rad);
    const Real cosT   = Real(1) - u * (Real(1) - cosMax);
    const Real s2     = Real(1) - cosT * cosT;
    const Real sinT   = s2 > Real(0) ? std::sqrt(s2) : Real(0);
    const Real phi    = Real(6.28318530717958647692) * v;
    const Vec3 e0 = right_of(dir);
    const Vec3 e1 = cross(normalized(dir), e0);
    return normalized(normalized(dir) * cosT +
                      (e0 * std::cos(phi) + e1 * std::sin(phi)) * sinT);
}

// Seed length/diameter ratio per class, used to derive the 6-DOF reference
// length and inertia tensor when ProjectileType::aero leaves them at 0.
Real class_length_over_diameter(ProjectileClass k) {
    switch (k) {
        case ProjectileClass::Bullet:      return 4.2;
        case ProjectileClass::Arrow:       return 150.0;
        case ProjectileClass::Bolt:        return 55.0;
        case ProjectileClass::Spear:       return 130.0;
        case ProjectileClass::ThrownBlade: return 8.0;
        case ProjectileClass::Pellet:      return 1.2;
        case ProjectileClass::Shell:       return 4.5;
        case ProjectileClass::Rock:        return 1.5;
        case ProjectileClass::SportsBall:  return 1.0;
        case ProjectileClass::Custom:
        default:                           return 4.0;
    }
}

// Earth angular-velocity vector in the sim world frame (§3.7). Convention: the
// shot's downrange direction is world +x horizontal, world up is +y, and
// `northAzimuth` is that direction's compass bearing (0 = north, clockwise).
Vec3 earth_omega_world(const Environment& env) {
    const Real Az = env.northAzimuth;
    const Vec3 north{std::cos(Az), 0.0, -std::sin(Az)};
    const Vec3 up{0.0, 1.0, 0.0};
    return north * (kEarthRate_radps * std::cos(env.latitude)) +
           up    * (kEarthRate_radps * std::sin(env.latitude));
}

// Force model for `s` in whichever medium it currently occupies. The drag LUT
// (`luts.dragRet`) already bakes in Cd(x), the form factor / profile curve and
// A/2m; here we supply the environmental factors (density × medium scale, the
// drag-abscissa scale) and resolve the Magnus term for a spinning ball.
detail::FlightModel make_flight_model(const Environment& env,
                                     const ProjectileType& t,
                                     const ProjectileState& s,
                                     const DragLuts& luts) {
    const MediumDesc& med = env.media.get(s.mediumId);
    const Real rho = (s.mediumId == kMediumAir) ? env.airDensity_kgm3
                                                : med.density_kgm3;
    detail::FlightModel fm;
    const Real buoy = med.buoyancy > Real(0) ? med.buoyancy : Real(0);
    fm.gravity  = env.gravity - env.gravity * buoy; // net of buoyancy
    fm.env      = env.wind ? &env : nullptr;
    fm.windTime = s.timeAlive_s;
    fm.dragRet  = luts.dragRet.y.size() >= 2 ? &luts.dragRet : nullptr;
    fm.rhoEff   = rho * med.dragScale;

    // Supercavitation (§3.5 special media). A purpose-built round in a medium
    // flagged for it rides inside a gas cavity once it is fast enough to sustain
    // one — only the nose and a short wetted length see the liquid, so the
    // effective drag collapses to a small fraction and underwater reach extends
    // from ~1 m to tens of metres. Below the cavity-formation speed the bubble
    // closes and full wetted drag returns. Speed-only ⇒ deterministic.
    if (med.supercavitation) {
        constexpr Real kCavityFormSpeed_mps = 60.0;
        constexpr Real kCavityDragFraction  = 0.06; // skin friction on the cavity
        if (length(s.velocity) > kCavityFormSpeed_mps)
            fm.rhoEff *= kCavityDragFraction;
    }

    if (luts.abscissa == DragAbscissa::Reynolds) {
        const Real nu = env.airKinematicViscosity_m2s > Real(0)
                            ? env.airKinematicViscosity_m2s : kNuFallback;
        fm.dragAbscissaScale = luts.reLength_m / nu; // ×|v| ⇒ Reynolds number
    } else {
        const Real a = env.speedOfSound_mps > Real(0) ? env.speedOfSound_mps
                                                      : Real(340.294);
        fm.dragAbscissaScale = Real(1) / a;          // ×|v| ⇒ Mach number
    }

    // ProlateSpheroid drag. AlongVelocity ⇒ a clean spiral: the nose tracks the
    // trajectory, so the compiled spiral Cd holds the whole flight. Fixed ⇒ an
    // end-over-end tumble about a world axis: Cd rises toward the tumble value
    // as that axis leaves the airflow direction.
    if (luts.shape == BallShape::ProlateSpheroid && luts.cdSpiral > Real(0) &&
        t.spinAxisMode != SpinAxisMode::AlongVelocity &&
        length_sq(s.spinAxis) > Real(0)) {
        const Vec3 vdir = normalized(s.velocity);
        const Vec3 ax   = normalized(s.spinAxis);
        const Real c    = std::fabs(dot(ax, vdir)); // 1 = nose-on, 0 = broadside
        const Real cd   = luts.cdTumble + (luts.cdSpiral - luts.cdTumble) * (c * c);
        fm.dragScaleExtra = cd / luts.cdSpiral;
    }

    // Magnus / lift. Off for rifle spin (AlongVelocity) — the axis tracks the
    // velocity so ŝ × v vanishes; gyroscopic spin drift is WORKPLAN item 6.
    if (luts.liftCoeffVsSpin.y.size() >= 2 && s.spin_radps != Real(0) &&
        t.spinAxisMode != SpinAxisMode::AlongVelocity &&
        length_sq(s.spinAxis) > Real(0)) {
        fm.liftVsSpin     = &luts.liftCoeffVsSpin;
        fm.liftAreaOver2m = luts.liftAreaOver2m;
        fm.spin_radps     = s.spin_radps;
        fm.spinRadius_m   = luts.liftRadius_m;
        fm.spinAxis       = normalized(s.spinAxis);
    }

    // --- Precision effects (§3.7). Filled only for the flags set on the shot;
    // every field defaults to zero so the fast path is unchanged. ---
    fm.flightTime0 = s.timeAlive_s;
    if (s.precision != PrecisionFlag::None) {
        if (has(s.precision, PrecisionFlag::Coriolis))
            fm.coriolisOmega = earth_omega_world(env);

        if (has(s.precision, PrecisionFlag::SpinDrift)) {
            const Real Sg = t.millerStability > Real(0) ? t.millerStability : Real(1.8);
            // Litz: drift(inches) ≈ 1.25·(Sg + 1.2)·t^1.83 ⇒ C(metres).
            fm.spinDriftCoeff = Real(0.0254) * Real(1.25) * (Sg + Real(1.2));
            const Real twistSign = t.twistRate_m < Real(0) ? Real(-1) : Real(1);
            fm.spinDriftDir = right_of(s.velocity) * twistSign;
        }

        if (has(s.precision, PrecisionFlag::LocalSpeedSound) &&
            luts.abscissa == DragAbscissa::Mach) {
            fm.soundLapse_KpM = kTropoLapse_KpM;
            fm.siteTemp_K     = env.siteTemperature_K > Real(0) ? env.siteTemperature_K
                                                               : Real(288.15);
        }
    }
    return fm;
}

// Rigid-body model for the 6-DOF path (PrecisionFlag::SixDOF). Reuses the
// translational force model (gravity + C_D0 drag + wind + §3.7 precision) and
// adds the angular aerodynamics from ProjectileType::aero, deriving the
// reference length + inertia tensor from mass/diameter/class where they are
// left at 0. The ball-Magnus force term is dropped — 6-DOF resolves side forces
// itself from the yaw angle.
detail::RigidModel make_rigid_model(const Environment& env,
                                    const ProjectileType& t,
                                    const ProjectileState& s,
                                    const DragLuts& luts) {
    detail::RigidModel rm;
    rm.flight = make_flight_model(env, t, s, luts);
    rm.flight.liftVsSpin = nullptr;
    rm.rhoEff = rm.flight.rhoEff;

    const Real d = t.refDiameter_m;
    rm.refArea_m2 = luts.refArea_m2 > Real(0)
                        ? luts.refArea_m2
                        : Real(0.25) * Real(3.14159265358979323846) * d * d;
    // Moment reference length is the calibre (ballistics C_M* are per-diameter).
    rm.refLen_m = d;
    const Real bodyLen = t.aero.length_m > Real(0)
                             ? t.aero.length_m
                             : class_length_over_diameter(t.klass) * d;

    const Real r = d * Real(0.5);
    rm.mass_kg = t.mass_kg;
    // Ix: a spun projectile is not a solid rod — mass sits off-axis less than a
    // uniform cylinder, ~0.5·m·r² for a bullet is close enough as a seed.
    rm.Ix = t.aero.axialInertia_kgm2 > Real(0)
                ? t.aero.axialInertia_kgm2
                : Real(0.5) * t.mass_kg * r * r;
    rm.It = t.aero.transverseInertia_kgm2 > Real(0)
                ? t.aero.transverseInertia_kgm2
                : t.mass_kg * (Real(3) * r * r + bodyLen * bodyLen) / Real(12);

    // Finned projectiles (arrow / bolt / spear) are statically STABLE — the
    // pressure centre sits behind the mass centre, so the aero moment restores
    // rather than overturns. Seed a negative C_Mα and strong pitch damping for
    // them; spin-stabilised classes get the overturning (positive) seed.
    const bool finned = t.klass == ProjectileClass::Arrow ||
                        t.klass == ProjectileClass::Bolt ||
                        t.klass == ProjectileClass::Spear;
    auto seed = [](Real v, Real def) { return v != Real(0) ? v : def; };
    rm.CMa  = seed(t.aero.overturningMomentSlope, finned ? Real(-9.0) : Real(2.5));
    rm.CMq  = seed(t.aero.pitchDampingMoment,     finned ? Real(-40.0) : Real(-8.0));
    rm.CMpa = seed(t.aero.magnusMomentSlope,      Real(0.35));
    rm.Clp  = seed(t.aero.rollDampingMoment,      Real(-0.02));
    rm.CLa  = seed(t.aero.liftForceSlope,         Real(2.0));
    rm.CDa2 = seed(t.aero.yawDragFactor,          Real(4.0));
    rm.transonicMul = t.aero.transonicOverturnMul > Real(0)
                          ? t.aero.transonicOverturnMul : Real(1.0);
    if (luts.abscissa == DragAbscissa::Mach) {
        const Real a = env.speedOfSound_mps > Real(0) ? env.speedOfSound_mps
                                                      : Real(340.294);
        rm.invSpeedOfSound = Real(1) / a;
    }
    return rm;
}

// --- Hit detection (§3.4) ---------------------------------------------------
//
// The caller's World::raycast tests the projectile segment against geometry at
// the pose it held when `step()` was entered ("frozen pose"). When the hit
// surface reports a velocity we correct for its motion: the surface is taken
// locally as the plane through the frozen hit point / normal, translating at
// surfaceVelocity, and the exact contact fraction along the sub-step is solved
// analytically (see below). This accounts for surface travel both *before* the
// sub-step (`queryAge_s`, frozen pose → sub-step start) and *across* it
// (`segSpan_s`, the sub-step's flight-time width). Exact for planar geometry and
// a constant surface velocity; locally linear otherwise.
//
// Limits (documented, tightened by the engine adapter — WORKPLAN item 11): the
// frozen-pose raycast is still the gate, so a target that is clear of the frozen
// segment but sweeps into the corridor is missed, and a target *closing* on the
// shot registers a sub-step or two late (bounded by surfaceVelocity·subStep).
// The common cases — a crossing, closing or receding target's impact point,
// time and closing energy — are corrected.
//
// Cold path: correct a moving-surface hit already written into `io` by the
// frozen-pose raycast. Treat the surface locally as the plane through io.point
// with normal io.normal, translating at io.surfaceVelocity. The projectile
// sweeps a→b over the sub-step; its flight-relative time at fraction u is
// queryAge + u·segSpan. Contact solves
//   n · (proj(u) − p0 − v·(queryAge + u·segSpan)) = 0
//   u = −[ n·(a − p0 − v·queryAge) ] / [ n·((b − a) − v·segSpan) ]
// u < 0 ⇒ the true contact was an earlier sub-step (the frozen-pose query only
// fires once the projectile reaches the *frozen* plane — for a closing target
// that is a sub-step or two late); clamp and report it now.
void refine_moving_hit(Vec3 a, Vec3 b, Seconds queryAge_s, Seconds segSpan_s,
                       HitResult& io) {
    const Vec3 v  = io.surfaceVelocity;
    const Vec3 n  = io.normal;
    const Vec3 p0 = io.point;
    const Real denom = dot(n, (b - a) - v * segSpan_s);
    if (std::fabs(denom) < Real(1e-12)) return; // grazing — keep the frozen hit
    Real u = -dot(n, a - p0 - v * queryAge_s) / denom;
    u = std::min(std::max(u, Real(0)), Real(1));
    io.t     = u;
    io.point = a + (b - a) * u; // projectile & surface coincide here
}

// Fast path: one raycast against the frozen pose; refine only on a moving hit.
// Force-inlined so the miss path (the per-sub-step hot loop) is exactly a
// World::raycast with no call-boundary cost.
#if defined(_MSC_VER)
#define PON_ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define PON_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define PON_ALWAYS_INLINE inline
#endif
PON_ALWAYS_INLINE bool sweep_segment(const World& world, Vec3 a, Vec3 b,
                                     Seconds queryAge_s, Seconds segSpan_s,
                                     HitResult& out) {
    if (!world.raycast(a, b, out)) return false;
    if (length_sq(out.surfaceVelocity) > Real(0) && segSpan_s > Real(0))
        refine_moving_hit(a, b, queryAge_s, segSpan_s, out);
    return true;
}

// Transonic-window flag (§3.7). When PrecisionFlag::TransonicFlag is set, emit
// a TransonicWindow event the first time the projectile's Mach enters the
// 0.8..1.2 band; latch kFlagPastTransonic on the way out. Cheap (one divide +
// compare); no effect on the trajectory.
void check_transonic(const Environment& env, EventSink& sink, ProjectileState& s,
                     const ProjectileType& t) {
    if (!has(s.precision, PrecisionFlag::TransonicFlag)) return;
    const Real a = env.speedOfSound_mps > Real(0) ? env.speedOfSound_mps
                                                  : Real(340.294);
    const Real mach = length(s.velocity) / a;
    const bool inBand = mach >= Real(0.8) && mach <= Real(1.2);
    const bool wasIn  = (s.flags & kFlagInTransonic) != 0;
    if (inBand && !wasIn) {
        s.flags |= kFlagInTransonic;
        Event e;
        e.type              = EventType::TransonicWindow;
        e.point             = s.position;
        e.time_s            = s.timeAlive_s;
        e.residualSpeed_mps = length(s.velocity);
        e.projectile        = s.typeId;
        sink.emit(e);
    } else if (!inBand && wasIn) {
        s.flags = (s.flags & ~kFlagInTransonic) | kFlagPastTransonic;
    }
    (void)t;
}

// Density the force model uses for a medium (air reads the live ISA field).
Real medium_density(const Environment& env, MediumId id) {
    return id == kMediumAir ? env.airDensity_kgm3
                            : env.media.get(id).density_kgm3;
}

// Fraction of speed a projectile keeps when it crosses from a medium of density
// `rhoFrom` into a much denser one (air → water): a one-time surface-slap /
// cavity-formation loss right at the interface, before the drag-down of the new
// medium takes over. Returns 1 (no impulse) unless the density jump is large
// (≥ 4×). Empirical: high-speed video of rifle rounds entering water shows a
// single-digit-% step at the surface; the metre-scale deceleration that follows
// is the medium drag, not this. (§3.5 special media.)
Real surface_entry_keep(Real rhoFrom, Real rhoTo) {
    if (!(rhoTo > rhoFrom * Real(4))) return Real(1);
    const Real jump = Real(1) - rhoFrom / std::max(rhoTo, Real(1e-6)); // ~1 air→water
    return Real(1) - Real(0.10) * jump;
}

// Returns true if the medium under `s` changed (and updates s.mediumId). On a
// step up in density (entering water) a surface impulse bleeds a little speed
// at the interface first, so the emitted MediumChanged carries the post-impulse
// speed.
bool check_medium_change(const Environment& env, const World& world,
                         EventSink& sink, ProjectileState& s) {
    const MediumId m = world.mediumAt(s.position);
    if (m == s.mediumId) return false;
    const Real keep = surface_entry_keep(medium_density(env, s.mediumId),
                                         medium_density(env, m));
    if (keep < Real(1)) s.velocity *= keep;
    Event e;
    e.type              = EventType::MediumChanged;
    e.point             = s.position;
    e.time_s            = s.timeAlive_s;
    e.residualSpeed_mps = length(s.velocity);
    e.projectile        = s.typeId;
    sink.emit(e);
    s.mediumId = m;
    return true;
}

// Returns true if the projectile hit a lifetime / range cap (and was reaped).
bool check_expiry(EventSink& sink, ProjectileState& s, const ProjectileType& t) {
    if (s.timeAlive_s < t.maxLifetime_s && s.distanceTravelled_m < t.maxRange_m)
        return false;
    s.alive = false;
    Event e;
    e.type       = EventType::Expired;
    e.point      = s.position;
    e.time_s     = s.timeAlive_s;
    e.projectile = s.typeId;
    sink.emit(e);
    return true;
}
} // namespace

Sim::Sim(Environment env, SimConfig cfg)
    : env_(std::move(env)), cfg_(cfg), rng_(cfg.rngSeed) {
    // BitExact routes the integrator through its Q32.32 fixed-point core
    // (Part B): deterministic sqrt + transcendental LUTs, bit-identical across
    // platforms. The per-type drag LUT is still compiled in double at
    // registerType() — see docs/ballistics-phase-plan.md §3.6.
    bitExact_ = cfg_.determinism == config::Determinism::BitExact;
}

TypeId Sim::registerType(ProjectileType type) {
    if (auto why = validate(type)) {
        // Point lastError_ at storage that outlives the call. The validate()
        // strings are string literals wrapped in std::string; stash the last
        // one so the char* stays valid.
        static std::string held;
        held = std::move(*why);
        lastError_ = held.c_str();
        return kInvalidType;
    }

    // Resolve BallProfile dimensions from the named profile before the coarse
    // class defaults fill any remaining gaps.
    if (type.dragModel == DragModel::BallProfile) {
        if (const detail::BallProfile* p = detail::find_ball_profile(type.ballProfile)) {
            if (type.refDiameter_m <= 0.0) type.refDiameter_m = p->diameter_m;
            if (type.mass_kg       <= 0.0) type.mass_kg       = p->mass_kg;
            if (!type.muzzleSpeed_mps && p->typicalSpeed_mps > 0.0)
                type.muzzleSpeed_mps = p->typicalSpeed_mps;
            // Balls curve; they are not rifling-stabilised. A sphere or a puck
            // defaults to a fixed world-frame spin axis (so the Magnus term is
            // live). A prolate spheroid keeps AlongVelocity — that is a clean
            // gyroscopically stable spiral (low drag); the caller selects Fixed
            // with a square axis for an end-over-end tumble.
            if ((p->shape == BallShape::Sphere || p->shape == BallShape::Disc) &&
                type.spinAxisMode == SpinAxisMode::AlongVelocity)
                type.spinAxisMode = SpinAxisMode::Fixed;
        }
    }

    if (!detail::apply_class_defaults(type))
        return kInvalidType;

    DragLuts luts;
    detail::compile_drag_lut(type, luts);

    types_.push_back(std::move(type));
    luts_.push_back(std::move(luts));
    lastError_ = "";
    const TypeId id = static_cast<TypeId>(types_.size() - 1);
    if (!types_[id].id.empty()) typeIds_[types_[id].id] = id; // for fire()
    return id;
}

StateId Sim::fire(const ProjectileType& type, Vec3 muzzle, Vec3 aimDir,
                  FidelityTier tier) {
    TypeId id = kInvalidType;
    if (!type.id.empty()) {
        const auto it = typeIds_.find(type.id);
        if (it != typeIds_.end()) id = it->second;
    }
    if (id == kInvalidType) {
        id = registerType(type);
        if (id == kInvalidType) return kInvalidState; // lastError_ set above
    }
    LaunchParams lp;
    lp.position  = muzzle;
    lp.direction = aimDir;
    lp.tier      = tier;
    return spawn(id, lp);
}

const ProjectileType& Sim::type(TypeId id) const {
    static const ProjectileType kNull{};
    return id < types_.size() ? types_[id] : kNull;
}

StateId Sim::spawn(TypeId type, const LaunchParams& launch) {
    if (type >= types_.size()) {
        lastError_ = "spawn: TypeId is out of range — use the value returned by "
                     "registerType() (it may have returned kInvalidType).";
        return kInvalidState;
    }
    const ProjectileType& t = types_[type];

    Real speed = launch.speed ? *launch.speed
                              : (t.muzzleSpeed_mps ? *t.muzzleSpeed_mps : Real(0));
    Vec3 dir = normalized(launch.direction);
    if (length_sq(dir) <= Real(0)) {
        lastError_ = "spawn: LaunchParams::direction is a zero vector — give it "
                     "a heading (it need not be normalised).";
        return kInvalidState;
    }
    lastError_ = "";

    // Opt-in muzzle dispersion (LaunchParams::precisionMrad > 0): rotate the aim
    // by a random offset inside that cone half-angle, drawn from the seeded RNG
    // and advanced per spawn. See the field doc — PlatformStable, not BitExact.
    if (launch.precisionMrad > Real(0))
        dir = disperse_cone(dir, launch.precisionMrad * Real(1e-3), rng_);

    ProjectileState s;
    s.position  = launch.position;
    s.velocity  = dir * speed;
    s.typeId    = type;
    s.mediumId  = launch.medium ? *launch.medium : kMediumAir;
    s.spin_radps = launch.spin ? *launch.spin
                               : (t.spinRate_radps ? *t.spinRate_radps : Real(0));

    // Resolve the spin axis (world frame). AlongVelocity tracks the launch
    // direction (rifling); everything else takes the shot override, then the
    // type default, then a shape-appropriate fallback.
    Vec3 axis{0, 0, 0};
    if (t.spinAxisMode == SpinAxisMode::AlongVelocity) {
        axis = dir;
    } else if (launch.spinAxis && length_sq(*launch.spinAxis) > Real(0)) {
        axis = *launch.spinAxis;
    } else if (length_sq(t.spinAxis) > Real(0)) {
        axis = t.spinAxis;
    } else if (luts_[type].shape != BallShape::Sphere) {
        axis = dir; // spiral / puck default: spin about the flight axis
    } else {
        // Sphere with no axis given: a horizontal axis square to the shot and
        // world up — the topspin / backspin / sidespin plane.
        axis = cross(dir, Vec3{0, 1, 0});
        if (length_sq(axis) < Real(1e-12)) axis = Vec3{0, 0, 1};
    }
    s.spinAxis  = normalized(axis);
    s.tier      = launch.tier;
    s.precision = launch.precision;
    s.alive     = true;

    // A guided round must integrate (the steering command is a per-step force);
    // silently promote a Hitscan / AnalyticDrag launch.
    if (t.guidance.law != GuidanceLaw::None) s.tier = FidelityTier::Integrated;

    // 6-DOF angular state (PrecisionFlag::SixDOF). Nose starts on the velocity
    // vector unless the caller asked for a muzzle tip-off; body-frame spin is
    // about the nose, signed by the barrel twist (right-hand ⇒ +).
    if (has(s.precision, PrecisionFlag::SixDOF)) {
        Vec3 nose = dir;
        if (launch.initialYaw && *launch.initialYaw != Real(0))
            nose = normalized(rotate_axis(dir, right_of(dir), *launch.initialYaw));
        s.orientation  = quat_from_to(Vec3{1, 0, 0}, nose);
        const Real twistSign = t.twistRate_m < Real(0) ? Real(-1) : Real(1);
        s.spin_radps   = s.spin_radps * twistSign;
        s.angVel_radps = Vec3{s.spin_radps, 0, 0}; // body frame; no initial coning
        s.spinPhase_rad = Real(0);
        s.spinAxis     = nose;
    }

    // Aerodynamic jump (§3.7): a crosswind at the muzzle gives a one-time
    // vertical velocity kick, proportional to the crosswind component and the
    // bullet's gyroscopic stability. Sign per Litz — for a right-hand twist a
    // wind from the right (blowing toward the shooter's left) throws the shot
    // high. `kAeroJump` is an approximate lumped coefficient (rad per m/s of
    // crosswind, per unit Sg), tuned so a 4.5 m/s crosswind at Sg≈1.8 gives a
    // jump angle around 0.1 mrad.
    if (has(s.precision, PrecisionFlag::AeroJump) && env_.wind) {
        constexpr Real kAeroJump = 1.2e-5;
        const Vec3 wind  = env_.windAt(s.position, Real(0));
        const Vec3 right = right_of(dir);
        const Real wCross = dot(wind, right); // >0 ⇒ wind blows toward the right
        const Real Sg = t.millerStability > Real(0) ? t.millerStability : Real(1.8);
        const Real twistSign = t.twistRate_m < Real(0) ? Real(-1) : Real(1);
        s.velocity.y += -kAeroJump * Sg * wCross * twistSign * speed;
    }

    // Reuse a free slot if one exists, else append.
    StateId slot = kInvalidState;
    for (std::size_t i = 0; i < states_.size(); ++i) {
        if (!used_[i]) { states_[i] = s; used_[i] = 1; slot = static_cast<StateId>(i); break; }
    }
    if (slot == kInvalidState) {
        states_.push_back(s);
        used_.push_back(1);
        slot = static_cast<StateId>(states_.size() - 1);
    }

    // Trajectory cache: reset this slot's ring and stamp the launch point.
    if (cfg_.trajectoryCacheFrames > 0) {
        if (traj_.size() < states_.size()) traj_.resize(states_.size());
        TrajRing& r = traj_[slot];
        r.ring.assign(cfg_.trajectoryCacheFrames, TrajectorySample{});
        r.head = 0;
        r.count = 0;
        recordTrajectory(slot);
    }
    return slot;
}

// Append one per-frame flight-path sample for state slot `i` into its ring.
void Sim::recordTrajectory(std::size_t i) {
    if (cfg_.trajectoryCacheFrames == 0 || i >= traj_.size()) return;
    const std::size_t cap = cfg_.trajectoryCacheFrames;
    TrajRing& r = traj_[i];
    if (r.ring.size() != cap) { r.ring.assign(cap, TrajectorySample{}); r.head = 0; r.count = 0; }
    const ProjectileState& s = states_[i];
    r.ring[r.head] = TrajectorySample{s.position, s.velocity, s.timeAlive_s};
    r.head = (r.head + 1) % cap;
    if (r.count < cap) ++r.count;
}

// Plain fast path only: Integrated tier, still air, no wind, no Magnus /
// orientation-shaped drag, no precision effects. Such shots share a force model
// and can be advanced as a lockstep group.
bool Sim::batchEligible(const ProjectileState& s) const {
    if (s.tier != FidelityTier::Integrated) return false;
    if (s.mediumId != kMediumAir) return false;
    if (s.precision != PrecisionFlag::None) return false;
    if (env_.wind) return false;
    if (s.typeId >= types_.size()) return false;
    const DragLuts& L = luts_[s.typeId];
    if (L.shape == BallShape::ProlateSpheroid || L.shape == BallShape::Disc) return false;
    const ProjectileType& t = types_[s.typeId];
    if (t.warhead.chargeMass_kg > Real(0)) return false; // fuze runs per-shot
    if (t.guidance.law != GuidanceLaw::None) return false;        // guidance runs per-shot
    if (s.externalAccel_mps2.x != Real(0) || s.externalAccel_mps2.y != Real(0) ||
        s.externalAccel_mps2.z != Real(0))
        return false;                                             // caller course-correction
    const bool magnus = L.liftCoeffVsSpin.y.size() >= 2 && s.spin_radps != Real(0) &&
                        t.spinAxisMode != SpinAxisMode::AlongVelocity &&
                        length_sq(s.spinAxis) > Real(0);
    return !magnus;
}

// SoA batch advance of a group of batch-eligible Integrated-tier shots that
// share a type. The force model is built once for the group; the sub-step
// count is the group maximum (a slow lane is only ever carried at a *finer*
// step than it would choose alone — strictly more accurate, still deterministic
// and inside posTolerance_m). A lane that impacts, expires or crosses a medium
// boundary is pulled out and finished on the per-shot path for its remainder.
void Sim::advanceIntegratedBatch(const std::uint32_t* idx, std::size_t n,
                                 const ProjectileType& t, Seconds dt,
                                 const World& world, EventSink& sink) {
    if (n == 0) return;
    const DragLuts& L = luts_[states_[idx[0]].typeId];
    const detail::FlightModel fm = make_flight_model(env_, t, states_[idx[0]], L);

    const Seconds hNom = cfg_.fixedStep_s > Real(0) ? cfg_.fixedStep_s : dt;
    const int maxSub = static_cast<int>(cfg_.maxSubsteps < 1 ? 1 : cfg_.maxSubsteps);

    std::vector<Seconds> t0(n);
    std::vector<char>    active(n, 1);
    for (std::size_t li = 0; li < n; ++li) {
        ProjectileState& s = states_[idx[li]];
        t0[li] = s.timeAlive_s;
        if (!s.alive) active[li] = 0;
    }

    Seconds remaining = dt;
    int guard = 0;
    while (remaining > Real(0) && guard++ < 200000) {
        bool any = false;
        for (std::size_t li = 0; li < n; ++li) if (active[li]) { any = true; break; }
        if (!any) return;

        const Seconds hBlock = remaining < hNom ? remaining : hNom;

        int nsub = 1;
        for (std::size_t li = 0; li < n; ++li) {
            if (!active[li]) continue;
            const ProjectileState& s = states_[idx[li]];
            const Vec3 a0 = detail::flight_accel(fm, s.position, s.velocity, Real(0), bitExact_);
            const int ns = detail::adaptive_substeps(s.velocity, a0, hBlock,
                                                     cfg_.posTolerance_m,
                                                     cfg_.maxSubstepDist_m, maxSub, bitExact_);
            if (ns > nsub) nsub = ns;
        }
        const Seconds h = hBlock / Real(nsub);
        const Seconds frameElapsed0 = dt - remaining;

        Seconds tRel = Real(0);
        for (int k = 0; k < nsub; ++k) {
            for (std::size_t li = 0; li < n; ++li) {
                if (!active[li]) continue;
                ++stats_.subSteps;
                ProjectileState& s = states_[idx[li]];
                const Vec3 prev  = s.position;
                const Vec3 vPrev = s.velocity;
                if (cfg_.integrator == Integrator::RK4)
                    detail::step_rk4(fm, s.position, s.velocity, tRel, h, bitExact_);
                else
                    detail::step_semi_implicit(fm, s.position, s.velocity, tRel, h, bitExact_);

                HitResult hit;
                if (sweep_segment(world, prev, s.position, frameElapsed0 + tRel, h, hit)) {
                    const Vec3 vImp = vPrev + (s.velocity - vPrev) * hit.t;
                    s.distanceTravelled_m += dist_no_fma(hit.point, prev);
                    const Seconds impact = s.timeAlive_s + h * hit.t;
                    active[li] = 0;
                    if (handleImpact(s, t, world, sink, hit, vImp, impact))
                        advanceRest(s, t, dt - (impact - t0[li]), world, sink, 1);
                    continue;
                }

                s.distanceTravelled_m += dist_no_fma(s.position, prev);
                s.timeAlive_s += h;

                if (check_medium_change(env_, world, sink, s)) {
                    // No longer on the plain fast path — hand the rest of the
                    // frame to the per-shot integrator.
                    active[li] = 0;
                    const Seconds rest = remaining - tRel - h;
                    if (rest > Real(0) && s.alive)
                        advanceIntegrated(s, t, rest, world, sink, 0);
                    continue;
                }
                if (check_expiry(sink, s, t)) { active[li] = 0; continue; }
            }
            tRel += h;
        }
        remaining -= hBlock;
    }
}

// Terminal-ballistics resolution of one confirmed impact (§3.5 / §3.8). Emits
// the outcome event(s), stamps the exact time-of-impact state onto `s`, and
// returns true if the shot survives — a ricochet, a perforation with residual
// speed, or fluid entry — so the caller flies it on for the rest of the frame.
bool Sim::handleImpact(ProjectileState& s, const ProjectileType& t,
                       const World& world, EventSink& sink, const HitResult& hit,
                       Vec3 impactVel, Seconds impactTime_s) {
    // A live warhead ends its flight at the first contact — the kinetic terminal
    // outcome (embed / perforate / ricochet) is not resolved; resolveWarhead()
    // emits the Detonated event from the recorded fuze arm point in step().
    if (t.warhead.chargeMass_kg > Real(0) && !(s.flags & kFlagDetonated)) {
        if (s.fuzeArmTime_s < Real(0)) {
            s.fuzeArmTime_s = impactTime_s;
            s.fuzePoint     = hit.point;
            const Real sp   = length(impactVel);
            s.fuzeDir       = sp > Real(1e-6) ? impactVel * (Real(1) / sp) : Vec3{};
        }
        s.timeAlive_s = impactTime_s;
        s.position    = hit.point;
        s.velocity    = Vec3{};
        s.alive       = false;
        return false;
    }

    const Vec3 vRel    = impactVel - hit.surfaceVelocity;
    const Real closing = length(vRel);
    const Vec3 vDir    = closing > Real(1e-6) ? vRel * (Real(1) / closing)
                                              : normalized(impactVel);

    detail::TerminalProjectile tp;
    tp.mass_kg         = t.mass_kg;
    tp.diameter_m      = s.expandedDiameter_m > Real(0) ? s.expandedDiameter_m
                                                       : t.refDiameter_m;
    tp.speed_mps       = closing;
    tp.noseShapeFactor = t.terminal.noseShapeFactor;
    tp.hardness        = t.terminal.hardness;
    tp.impactYaw       = s.impactYaw;
    tp.deformable      = t.terminal.deformable;
    tp.fragile         = t.terminal.fragile;
    tp.alreadyExpanded = (s.flags & kFlagExpanded) != 0;

    const Material& mat = world.material(hit.surface);
    const detail::TerminalResult tr =
        detail::resolve_terminal(tp, mat, vDir, hit.normal, hit.point);

    s.timeAlive_s = impactTime_s;

    // Momentum delivered to the surface: m·(v_in − v_out), both relative to the
    // struck body. The per-outcome branches below fill in v_out.
    const Real impactMass = std::max(t.mass_kg, Real(0));

    Event e;
    e.point      = hit.point;
    e.normal     = hit.normal;
    e.time_s     = impactTime_s;
    e.projectile = s.typeId;
    e.energy_J   = tr.energyDeposited_J;

    using detail::TerminalOutcome;
    switch (tr.outcome) {
        case TerminalOutcome::Stopped:
        case TerminalOutcome::Shattered:
            e.type = tr.outcome == TerminalOutcome::Shattered ? EventType::Shattered
                                                              : EventType::Stopped;
            e.impulse_Ns = vDir * (closing * impactMass);   // all momentum absorbed
            sink.emit(e);
            s.position = hit.point;
            s.alive    = false;
            return false;

        case TerminalOutcome::Embedded:
            e.type           = EventType::Embedded;
            e.channelAxis    = tr.exitDir;
            e.channelDepth_m = tr.channelDepth_m;
            e.channelWiden_m = tr.channelWiden_m;
            e.impulse_Ns     = vDir * (closing * impactMass); // rider stops in the body
            sink.emit(e);
            s.position = hit.point + tr.exitDir * tr.channelDepth_m;
            s.velocity = hit.surfaceVelocity; // rides with the target
            s.alive    = false;
            return false;

        case TerminalOutcome::Ricochet:
            e.type              = EventType::Ricochet;
            e.residualSpeed_mps = tr.residualSpeed_mps;
            e.impulse_Ns        = (vRel - tr.exitDir * tr.residualSpeed_mps) * impactMass;
            sink.emit(e);
            s.position    = hit.point + hit.normal * Real(1e-3);
            s.velocity    = tr.exitDir * tr.residualSpeed_mps + hit.surfaceVelocity;
            s.spin_radps *= tr.spinScale;
            s.impactYaw   = tr.newImpactYaw;
            return true;

        case TerminalOutcome::Perforated: {
            Event crossed = e;
            crossed.type = EventType::SurfaceCrossed;
            sink.emit(crossed);
            e.type              = EventType::Perforated;
            e.point             = tr.exitPoint;
            e.residualSpeed_mps = tr.residualSpeed_mps;
            e.channelAxis       = vDir;
            e.channelDepth_m    = tr.channelDepth_m;
            e.channelWiden_m    = tr.channelWiden_m;
            e.impulse_Ns        = (vRel - tr.exitDir * tr.residualSpeed_mps) * impactMass;
            sink.emit(e);
            s.position    = tr.exitPoint + tr.exitDir * Real(1e-3);
            s.velocity    = tr.exitDir * tr.residualSpeed_mps + hit.surfaceVelocity;
            s.spin_radps *= tr.spinScale;
            s.impactYaw   = tr.newImpactYaw;
            if (tr.newDiameter_m > Real(0)) {
                s.expandedDiameter_m = tr.newDiameter_m;
                s.flags |= kFlagExpanded;
            }
            if (tr.residualSpeed_mps < Real(1)) { s.alive = false; return false; }
            return true;
        }

        case TerminalOutcome::EnteredFluid: {
            // Surface impulse at the interface (§3.5): a one-time step loss on
            // crossing into the much denser fluid, then the medium switches and
            // its drag-down takes over for the metre-scale deceleration.
            const MediumId into = mat.entersMedium >= 0 ? mat.entersMedium
                                                        : s.mediumId;
            const Real keep = surface_entry_keep(
                medium_density(env_, s.mediumId), medium_density(env_, into));
            if (keep < Real(1)) s.velocity *= keep;
            e.type              = EventType::MediumChanged;
            e.residualSpeed_mps = length(s.velocity);
            e.impulse_Ns        = vDir * (closing * (Real(1) - keep) * impactMass);
            sink.emit(e);
            s.position = hit.point + vDir * Real(1e-3);
            s.mediumId = into;
            if (length(s.velocity) < Real(1)) { s.alive = false; return false; }
            return true;
        }
    }
    return s.alive;
}

// Fly the shot on for `rest` seconds after it survived an impact, in its own
// fidelity tier. `layer` bounds the perforation-chaining recursion.
void Sim::advanceRest(ProjectileState& s, const ProjectileType& t, Seconds rest,
                      const World& world, EventSink& sink, int layer) {
    if (rest <= Real(1e-9) || !s.alive || layer >= 6) return;
    switch (s.tier) {
        case FidelityTier::Hitscan:      advanceHitscan(s, t, rest, world, sink, layer); break;
        case FidelityTier::AnalyticDrag: advanceAnalytic(s, t, rest, world, sink, layer); break;
        case FidelityTier::Integrated:   advanceIntegrated(s, t, rest, world, sink, layer); break;
    }
}

void Sim::advanceHitscan(ProjectileState& s, const ProjectileType& t, Seconds dt,
                         const World& world, EventSink& sink, int layer) {
    const Seconds t0   = s.timeAlive_s;
    const Vec3 prev = s.position;
    const Vec3 next = s.position + s.velocity * dt; // straight, drag-free, no drop

    HitResult hit;
    if (sweep_segment(world, prev, next, Real(0), dt, hit)) {
        s.distanceTravelled_m += dist_no_fma(hit.point, prev);
        const Seconds impact = t0 + dt * hit.t;
        if (handleImpact(s, t, world, sink, hit, s.velocity, impact))
            advanceRest(s, t, dt - (impact - t0), world, sink, layer + 1);
        return;
    }
    s.position = next;
    s.distanceTravelled_m += dist_no_fma(next, prev);
    s.timeAlive_s += dt;
    check_medium_change(env_, world, sink, s);
    check_expiry(sink, s, t);
}

void Sim::advanceAnalytic(ProjectileState& s, const ProjectileType& t, Seconds dt,
                          const World& world, EventSink& sink, int layer) {
    const Seconds t0 = s.timeAlive_s;
    const DragLuts& L = luts_[s.typeId];
    const detail::FlightModel fm = make_flight_model(env_, t, s, L);

    // The closed form assumes constant gravity, no wind and a purely
    // along-velocity drag force. Drag (constant-Cd or a G1/G7 / custom / Cd(Re)
    // curve) is handled by re-linearising the retardation each frame. A wind
    // field, an active Magnus term or an orientation-dependent shape all add a
    // transverse force the closed form cannot represent — hand those to the
    // integrator.
    // Precision effects that add a transverse acceleration or bend the drag
    // abscissa down the trajectory are outside the closed form's assumptions.
    constexpr PrecisionFlag kNeedsIntegrator =
        PrecisionFlag::SpinDrift | PrecisionFlag::Coriolis |
        PrecisionFlag::LocalSpeedSound | PrecisionFlag::AdaptiveRKF45 |
        PrecisionFlag::SixDOF;
    if (env_.wind || fm.liftVsSpin || L.shape == BallShape::ProlateSpheroid ||
        L.shape == BallShape::Disc ||
        has(s.precision, kNeedsIntegrator)) {
        advanceIntegrated(s, t, dt, world, sink, layer);
        return;
    }

    const Real speedRef = length(s.velocity);
    detail::AnalyticTrajectory tr;
    tr.x0      = s.position;
    tr.v0      = s.velocity;
    tr.gravity  = fm.gravity;
    tr.c        = detail::linear_drag_rate(fm, speedRef, bitExact_); // re-linearised each frame
    tr.bitExact = bitExact_;

    const Real sampleDist = std::max(cfg_.analyticSampleDist_m, Real(1e-3));
    int n = static_cast<int>(std::ceil(speedRef * dt / sampleDist));
    n = std::min(std::max(n, 1), 4096);

    Vec3 prev = s.position;
    Seconds tPrev = Real(0);
    for (int i = 1; i <= n && s.alive; ++i) {
        const Seconds ti = dt * (Real(i) / Real(n));
        Vec3 pos, vel;
        tr.at(ti, pos, vel);

        HitResult hit;
        if (sweep_segment(world, prev, pos, tPrev, ti - tPrev, hit)) {
            // Exact TOI within the chord's time span, then the closed-form
            // state at that instant (better than the chord-linear point).
            const Seconds tHit = tPrev + (ti - tPrev) * hit.t;
            Vec3 hp, hv;
            tr.at(tHit, hp, hv);
            s.distanceTravelled_m += dist_no_fma(hit.point, prev);
            if (handleImpact(s, t, world, sink, hit, hv, t0 + tHit))
                advanceRest(s, t, dt - tHit, world, sink, layer + 1);
            return;
        }

        s.distanceTravelled_m += dist_no_fma(pos, prev);
        s.position = pos;
        s.velocity = vel;
        prev = pos;
        tPrev = ti;
        check_transonic(env_, sink, s, t);

        if (check_medium_change(env_, world, sink, s)) {
            // Medium boundary crossed mid-frame — the closed form is no longer
            // valid; hand the remainder of the frame to the integrator.
            s.timeAlive_s += ti;
            const Seconds rest = dt - ti;
            if (rest > Real(0) && s.alive)
                advanceIntegrated(s, t, rest, world, sink, layer);
            return;
        }
    }

    s.timeAlive_s += dt;
    check_expiry(sink, s, t);
}

// Embedded Cash-Karp RK4(5) with a deterministic step-size controller
// (PrecisionFlag::AdaptiveRKF45). Targets cfg_.posTolerance_m of position error
// per step instead of the geometry-distance-only substep heuristic. The swept
// World query runs between accepted steps, same as the fixed-step path.
void Sim::advanceIntegratedAdaptive(ProjectileState& s, const ProjectileType& t,
                                    Seconds dt, const World& world, EventSink& sink,
                                    int layer) {
    const Vec3 extAccel = layer == 0 ? evalGuidance(s, t, dt) : s.externalAccel_mps2;
    const Seconds t0 = s.timeAlive_s;
    const Real tol = std::max(cfg_.posTolerance_m, Real(1e-6));
    const Seconds hCap = cfg_.fixedStep_s > Real(0) ? cfg_.fixedStep_s * Real(8) : dt;
    Seconds h = std::min(hCap, dt);

    Seconds remaining = dt;
    int guard = 0;
    while (remaining > Real(0) && s.alive && guard++ < 100000) {
        detail::FlightModel fm =
            make_flight_model(env_, t, s, luts_[s.typeId]);
        fm.externalAccel = extAccel;

        Seconds hTry = std::min(h, remaining);
        const Real sp = length(s.velocity);
        if (sp > Real(0) && cfg_.maxSubstepDist_m > Real(0))
            hTry = std::min(hTry, cfg_.maxSubstepDist_m / sp);

        Vec3 p5, v5;
        Real pErr, vErr;
        detail::step_rkck(fm, s.position, s.velocity, Real(0), hTry, p5, v5, pErr, vErr,
                          bitExact_);

        // Step-size controller: standard 0.9·(tol/err)^(1/5), clamped.
        const Real ratio = pErr > Real(0) ? tol / pErr : Real(8);
        const Real fac = std::min(std::max(Real(0.9) * std::pow(ratio, Real(0.2)),
                                           Real(0.2)), Real(4.0));

        if (pErr > tol && hTry > Real(2e-7)) {
            h = hTry * std::min(fac, Real(0.9)); // reject, shrink, retry
            continue;
        }

        ++stats_.subSteps; // accepted (not a rejected/retried trial above)
        const Vec3 prev = s.position;
        HitResult hit;
        if (sweep_segment(world, prev, p5, dt - remaining, hTry, hit)) {
            const Vec3 vImp = s.velocity + (v5 - s.velocity) * hit.t;
            s.distanceTravelled_m += dist_no_fma(hit.point, prev);
            const Seconds impact = s.timeAlive_s + hTry * hit.t;
            if (handleImpact(s, t, world, sink, hit, vImp, impact))
                advanceRest(s, t, dt - (impact - t0), world, sink, layer + 1);
            return;
        }

        s.position = p5;
        s.velocity = v5;
        s.distanceTravelled_m += dist_no_fma(p5, prev);
        s.timeAlive_s += hTry;
        remaining     -= hTry;
        h = hTry * fac; // grow (or shrink) for the next step

        const bool mediumChanged = check_medium_change(env_, world, sink, s);
        check_transonic(env_, sink, s, t);
        if (check_expiry(sink, s, t)) return;
        (void)mediumChanged; // model is rebuilt next iteration regardless
    }
}

// Full 6-DOF rigid-body flight (PrecisionFlag::SixDOF). Integrates orientation +
// body angular velocity alongside the point mass with step_rigid_rk4; yaw of
// repose, spin drift, epicyclic coning and tumbling all emerge. Same block /
// adaptive-substep / swept-query / medium / expiry bookkeeping as the plain
// Integrated path.
void Sim::advanceSixDOF(ProjectileState& s, const ProjectileType& t, Seconds dt,
                        const World& world, EventSink& sink, int layer) {
    const Vec3 extAccel = layer == 0 ? evalGuidance(s, t, dt) : s.externalAccel_mps2;
    const Seconds t0   = s.timeAlive_s;
    const Seconds hNom = cfg_.fixedStep_s > Real(0) ? cfg_.fixedStep_s : dt;
    const int maxSub = static_cast<int>(cfg_.maxSubsteps < 1 ? 1 : cfg_.maxSubsteps);

    Seconds remaining = dt;
    while (remaining > Real(0) && s.alive) {
        detail::RigidModel rm = make_rigid_model(env_, t, s, luts_[s.typeId]);
        rm.flight.externalAccel = extAccel;

        const Seconds hBlock = remaining < hNom ? remaining : hNom;
        const Vec3 a0 =
            detail::flight_accel(rm.flight, s.position, s.velocity, Real(0), bitExact_);
        const int n = detail::adaptive_substeps(s.velocity, a0, hBlock,
                                                cfg_.posTolerance_m,
                                                cfg_.maxSubstepDist_m, maxSub, bitExact_);
        if (cfg_.traceSink && static_cast<std::uint32_t>(n) > traceSubsteps_)
            traceSubsteps_ = static_cast<std::uint32_t>(n);
        const Seconds h = hBlock / Real(n);

        detail::RigidState rs;
        rs.pos    = s.position;
        rs.vel    = s.velocity;
        rs.nose   = normalized(s.orientation.rotate(Vec3{1, 0, 0}));
        // Transverse rate to world; the stored body angVel is (spin, y, z).
        rs.omegaT = s.orientation.rotate(Vec3{0, s.angVel_radps.y, s.angVel_radps.z});
        rs.omegaT = rs.omegaT - rs.nose * dot(rs.omegaT, rs.nose);
        rs.spin   = s.angVel_radps.x;
        rs.roll   = s.spinPhase_rad;

        auto writeBack = [&](Real alpha) {
            // rs.roll is exact (integrated in Fx32 the whole way when
            // bitExact_ — see integrate.cpp) but std::cos/std::sin are not
            // guaranteed bit-identical across platforms for a general input
            // (unlike +,-,*,/,sqrt, which IEEE-754 mandates exact rounding
            // for). Route through the fixed-point LUT in that mode so this
            // Quat — part of the hashed state — stays exact too; a real
            // cross-platform CI run caught this (see fixed_lut.hpp's
            // fx_sin_full/fx_cos_full comment).
            const Real half = rs.roll * Real(0.5);
            Real rollCos, rollSin;
            if (bitExact_) {
                const detail::Fx32 halfFx = detail::Fx32::from_double(half);
                rollCos = detail::fx_cos_full(halfFx).to_double();
                rollSin = detail::fx_sin_full(halfFx).to_double();
            } else {
                rollCos = std::cos(half);
                rollSin = std::sin(half);
            }
            s.orientation = quat_mul_no_fma(quat_from_to(Vec3{1, 0, 0}, rs.nose),
                                           Quat{rollCos, rollSin, 0, 0});
            const Vec3 wBody = s.orientation.inv_rotate(rs.omegaT);
            s.angVel_radps      = Vec3{rs.spin, wBody.y, wBody.z};
            s.spin_radps        = rs.spin;
            s.spinPhase_rad     = rs.roll;
            s.spinAxis          = rs.nose;
            s.angleOfAttack_rad = alpha;
        };

        Seconds tRel = Real(0);
        bool mediumChanged = false;
        for (int k = 0; k < n && s.alive; ++k) {
            ++stats_.subSteps;
            const Vec3 prev  = rs.pos;
            const Vec3 vPrev = rs.vel;
            Real alpha = Real(0);
            detail::step_rigid_rk4(rm, rs, tRel, h, alpha, bitExact_);

            HitResult hit;
            if (sweep_segment(world, prev, rs.pos, dt - remaining, h, hit)) {
                const Vec3 vImp = vPrev + (rs.vel - vPrev) * hit.t;
                writeBack(alpha);
                s.distanceTravelled_m += dist_no_fma(hit.point, prev);
                const Seconds impact = s.timeAlive_s + h * hit.t;
                if (handleImpact(s, t, world, sink, hit, vImp, impact))
                    advanceRest(s, t, dt - (impact - t0), world, sink, layer + 1);
                return;
            }

            s.position = rs.pos;
            s.velocity = rs.vel;
            writeBack(alpha);
            if (alpha > Real(1.0471975512) && (s.flags & kFlagTumbling) == 0)
                s.flags |= kFlagTumbling;   // > 60 deg — no longer stable
            s.distanceTravelled_m += dist_no_fma(rs.pos, prev);
            s.timeAlive_s += h;
            tRel          += h;
            remaining     -= h;

            mediumChanged = check_medium_change(env_, world, sink, s);
            check_transonic(env_, sink, s, t);
            if (check_expiry(sink, s, t)) return;
            if (mediumChanged) break; // rebuild rm/rs for the new medium
        }
        if (!s.alive) return;
    }
}

Vec3 Sim::evalGuidance(ProjectileState& s, const ProjectileType& t, Seconds dt) {
    Vec3 ext = s.externalAccel_mps2;
    if (t.guidance.law != GuidanceLaw::None && s.guidance.hasTarget) {
        const GuidanceCommand gc = compute_guidance(
            t.guidance, s.guidance, s.position, s.velocity, s.timeAlive_s, dt);
        ext += gc.accel_mps2;
    }
    return ext;
}

void Sim::advanceIntegrated(ProjectileState& s, const ProjectileType& t, Seconds dt,
                            const World& world, EventSink& sink, int layer) {
    if (has(s.precision, PrecisionFlag::SixDOF)) {
        advanceSixDOF(s, t, dt, world, sink, layer);
        return;
    }
    if (has(s.precision, PrecisionFlag::AdaptiveRKF45)) {
        advanceIntegratedAdaptive(s, t, dt, world, sink, layer);
        return;
    }
    const Vec3 extAccel = layer == 0 ? evalGuidance(s, t, dt) : s.externalAccel_mps2;
    const Seconds t0 = s.timeAlive_s;
    const Seconds hNom = cfg_.fixedStep_s > Real(0) ? cfg_.fixedStep_s : dt;
    const int maxSub = static_cast<int>(cfg_.maxSubsteps < 1 ? 1 : cfg_.maxSubsteps);

    Seconds remaining = dt;
    while (remaining > Real(0) && s.alive) {
        // Rebuild the force model each block: cheap, and it picks up a medium
        // change from the previous block plus a fresh wind time origin.
        detail::FlightModel fm = make_flight_model(env_, t, s, luts_[s.typeId]);
        fm.externalAccel = extAccel;

        const Seconds hBlock = remaining < hNom ? remaining : hNom;
        const Vec3 a0 = detail::flight_accel(fm, s.position, s.velocity, Real(0), bitExact_);
        const int n = detail::adaptive_substeps(s.velocity, a0, hBlock,
                                                cfg_.posTolerance_m,
                                                cfg_.maxSubstepDist_m, maxSub, bitExact_);
        if (cfg_.traceSink && static_cast<std::uint32_t>(n) > traceSubsteps_)
            traceSubsteps_ = static_cast<std::uint32_t>(n);
        const Seconds h = hBlock / Real(n);

        Seconds tRel = Real(0);
        for (int k = 0; k < n && s.alive; ++k) {
            ++stats_.subSteps;
            const Vec3 prev  = s.position;
            const Vec3 vPrev = s.velocity;
            if (cfg_.integrator == Integrator::RK4)
                detail::step_rk4(fm, s.position, s.velocity, tRel, h, bitExact_);
            else
                detail::step_semi_implicit(fm, s.position, s.velocity, tRel, h, bitExact_);

            HitResult hit;
            if (sweep_segment(world, prev, s.position, dt - remaining, h, hit)) {
                const Vec3 vImp = vPrev + (s.velocity - vPrev) * hit.t;
                s.distanceTravelled_m += dist_no_fma(hit.point, prev);
                const Seconds impact = s.timeAlive_s + h * hit.t;
                if (handleImpact(s, t, world, sink, hit, vImp, impact))
                    advanceRest(s, t, dt - (impact - t0), world, sink, layer + 1);
                return;
            }

            s.distanceTravelled_m += dist_no_fma(s.position, prev);
            s.timeAlive_s += h;
            tRel          += h;
            remaining     -= h;

            const bool mediumChanged = check_medium_change(env_, world, sink, s);
            check_transonic(env_, sink, s, t);
            if (check_expiry(sink, s, t)) return;
            if (mediumChanged) break; // rebuild the model for the new medium
        }
    }
}

// Warhead fuze resolution. Called once per live warhead shot per step, after the
// advance. Picks the detonation point + time from the fuze, emits one Detonated
// event and marks the round spent (kFlagDetonated, alive = false).
void Sim::resolveWarhead(ProjectileState& s, const ProjectileType& t,
                         const World& world, EventSink& sink) {
    const WarheadDesc& w = t.warhead;
    const bool contacted = s.fuzeArmTime_s >= Real(0);

    bool     go = false;
    Vec3     point = s.position;
    Seconds  when  = s.timeAlive_s;

    switch (w.fuze) {
        case FuzeMode::Contact:
            if (contacted) { go = true; point = s.fuzePoint; when = s.fuzeArmTime_s; }
            break;
        case FuzeMode::Delayed:
            if (contacted) {
                go = true;
                point = s.fuzePoint;
                when  = s.fuzeArmTime_s + std::max(w.fuzeDelay_s, Real(0));
            }
            break;
        case FuzeMode::TimedAirburst:
            if (s.timeAlive_s >= w.fuzeDelay_s) {
                go = true; point = s.position; when = std::max(s.timeAlive_s, w.fuzeDelay_s);
            } else if (contacted) {           // hit the ground before the timer
                go = true; point = s.fuzePoint; when = s.fuzeArmTime_s;
            }
            break;
        case FuzeMode::Proximity: {
            if (contacted) {                  // real contact always trips it
                go = true; point = s.fuzePoint; when = s.fuzeArmTime_s;
            } else if (w.proximityRadius_m > Real(0) && s.alive &&
                       length_sq(s.velocity) > Real(0)) {
                const Vec3 a = s.position;
                const Vec3 b = s.position +
                               normalized(s.velocity) * w.proximityRadius_m;
                HitResult hr;
                if (world.raycast(a, b, hr)) {
                    go = true; point = s.position; when = s.timeAlive_s;
                }
            }
            break;
        }
    }

    if (!go) return;

    // Jet aim / channel axis for a shaped-charge payload (Phase 19 item 4): the
    // flight direction at detonation — the recorded contact direction for a
    // contact/delayed fuze, else the current velocity.
    Vec3 axis = contacted ? s.fuzeDir : Vec3{};
    if (length_sq(axis) <= Real(0) && length_sq(s.velocity) > Real(0))
        axis = normalized(s.velocity);

    Event e;
    e.type       = EventType::Detonated;
    e.point      = point;
    e.normal     = Vec3{0, 1, 0};
    e.channelAxis = axis;
    e.time_s     = when;
    e.energy_J   = warhead_energy_J(w);
    e.payload_kg = w.chargeMass_kg * std::max(w.tntEquivalence, Real(0)) *
                   (w.thermobaric > Real(0) ? (Real(1) + w.thermobaric) : Real(1)) *
                   (w.surfaceBurst ? Real(1.8) : Real(1));
    e.projectile = s.typeId;
    sink.emit(e);

    s.flags   |= kFlagDetonated;
    s.position = point;
    s.velocity = Vec3{};
    s.alive    = false;
}

namespace {
// Thin forwarding proxies so step() can count "how many swept queries / how
// many events" for SimStats without touching every raycast()/emit() call
// site scattered across advanceHitscan/advanceAnalytic/.../handleImpact —
// step() wraps `world`/`sink` once and passes the proxy down; every one of
// those functions already takes a `const World&`/`EventSink&` it forwards
// transparently, so the wrapper is invisible to them.
struct CountingWorld final : World {
    const World&   inner;
    std::uint64_t& counter;
    CountingWorld(const World& w, std::uint64_t& c) : inner(w), counter(c) {}
    bool raycast(Vec3 a, Vec3 b, HitResult& out) const override {
        ++counter;
        return inner.raycast(a, b, out);
    }
    MediumId mediumAt(Vec3 p) const override { return inner.mediumAt(p); }
    const Material& material(SurfaceId s) const override { return inner.material(s); }
};

struct CountingEventSink final : EventSink {
    EventSink&     inner;
    std::uint32_t& counter;
    CountingEventSink(EventSink& s, std::uint32_t& c) : inner(s), counter(c) {}
    void emit(const Event& e) override { ++counter; inner.emit(e); }
};
} // namespace

void Sim::step(Seconds dt, const World& world, EventSink& sink) {
    if (dt <= Real(0)) return;
    const bool cache = cfg_.trajectoryCacheFrames > 0;

    stats_ = SimStats{};
    CountingWorld     countingWorld(world, stats_.sweptQueries);
    CountingEventSink countingSink(sink, stats_.eventsEmitted);

    // --- Batch pass (opt-in). Group batch-eligible Integrated-tier shots by
    // type; a group of at least batchMinGroup rides the SoA lockstep path. All
    // other shots fall through to the per-shot loop below. ---
    batchHandled_.assign(states_.size(), 0);
    if (cfg_.batchIntegrator) {
        batchScratch_.clear();
        for (std::size_t i = 0; i < states_.size(); ++i)
            if (used_[i] && states_[i].alive && batchEligible(states_[i]))
                batchScratch_.push_back(static_cast<std::uint32_t>(i));
        std::sort(batchScratch_.begin(), batchScratch_.end(),
                  [&](std::uint32_t a, std::uint32_t b) {
                      const TypeId ta = states_[a].typeId, tb = states_[b].typeId;
                      return ta != tb ? ta < tb : a < b;
                  });
        std::size_t g = 0;
        while (g < batchScratch_.size()) {
            const TypeId tid = states_[batchScratch_[g]].typeId;
            std::size_t e = g;
            while (e < batchScratch_.size() && states_[batchScratch_[e]].typeId == tid) ++e;
            const std::size_t cnt = e - g;
            if (cnt >= cfg_.batchMinGroup) {
                ++stats_.batchGroups;
                advanceIntegratedBatch(&batchScratch_[g], cnt, types_[tid], dt,
                                       countingWorld, countingSink);
                for (std::size_t k = g; k < e; ++k) batchHandled_[batchScratch_[k]] = 1;
            }
            g = e;
        }
    }

    for (std::size_t i = 0; i < states_.size(); ++i) {
        if (!used_[i]) continue;
        ProjectileState& s = states_[i];
        if (!s.alive) { used_[i] = 0; continue; }

        const ProjectileType& t = types_[s.typeId];

        switch (s.tier) {
            case FidelityTier::Hitscan:      ++stats_.liveHitscan;    break;
            case FidelityTier::AnalyticDrag: ++stats_.liveAnalytic;   break;
            case FidelityTier::Integrated:   ++stats_.liveIntegrated; break;
        }

        // Pre-advance snapshot for the diagnostic trace (see SimConfig::traceSink).
        const bool     trace     = static_cast<bool>(cfg_.traceSink);
        const bool     wasTrans  = trace && (s.flags & kFlagInTransonic) != 0;
        const std::int32_t wasMedium = s.mediumId;
        const bool     wasLock   = trace && s.guidance.lockLost;
        traceSubsteps_ = 0;

        if (!batchHandled_[i]) {
            switch (s.tier) {
                case FidelityTier::Hitscan:
                    advanceHitscan(s, t, dt, countingWorld, countingSink);
                    break;
                case FidelityTier::AnalyticDrag:
                    advanceAnalytic(s, t, dt, countingWorld, countingSink);
                    break;
                case FidelityTier::Integrated:
                    advanceIntegrated(s, t, dt, countingWorld, countingSink);
                    break;
            }
        }

        if (t.warhead.chargeMass_kg > Real(0) && !(s.flags & kFlagDetonated))
            resolveWarhead(s, t, countingWorld, countingSink);

        if (trace) {
            emitTrace(TraceKind::Frame, s, static_cast<std::int32_t>(s.tier),
                      length(s.velocity));
            if (traceSubsteps_ > 0)
                emitTrace(TraceKind::Substep, s,
                          static_cast<std::int32_t>(traceSubsteps_));
            const bool isTrans = (s.flags & kFlagInTransonic) != 0;
            if (isTrans && !wasTrans) emitTrace(TraceKind::TransonicEnter, s);
            else if (!isTrans && wasTrans) emitTrace(TraceKind::TransonicExit, s);
            if (s.mediumId != wasMedium)
                emitTrace(TraceKind::MediumChanged, s, s.mediumId);
            if (s.guidance.lockLost && !wasLock)
                emitTrace(TraceKind::GuidanceLost, s);
        }

        if (cache) recordTrajectory(i);
        if (!s.alive) used_[i] = 0;
    }
}

void Sim::emitTrace(TraceKind kind, const ProjectileState& s,
                    std::int32_t i0, Real r0) const {
    if (!cfg_.traceSink) return;
    TraceEvent e;
    e.kind     = kind;
    e.shot     = static_cast<StateId>(&s - states_.data());
    e.time_s   = s.timeAlive_s;
    e.position = s.position;
    e.velocity = s.velocity;
    e.i0       = i0;
    e.r0       = r0;
    cfg_.traceSink(e);
}

const ProjectileState& Sim::state(StateId id) const {
    static const ProjectileState kNull{};
    return id < states_.size() ? states_[id] : kNull;
}

std::size_t Sim::liveCount() const {
    std::size_t n = 0;
    for (std::size_t i = 0; i < states_.size(); ++i)
        if (used_[i] && states_[i].alive) ++n;
    return n;
}

void Sim::liveIds(std::vector<StateId>& out) const {
    out.clear();
    for (std::size_t i = 0; i < states_.size(); ++i)
        if (used_[i] && states_[i].alive) out.push_back(static_cast<StateId>(i));
}

void Sim::despawn(StateId id) {
    if (id < states_.size()) {
        states_[id].alive = false;
        used_[id] = 0;
    }
}

Status Sim::guide(StateId id, Vec3 targetPos, Vec3 targetVel) {
    if (id >= states_.size() || !used_[id]) return Status::InvalidHandle;
    ProjectileState& s = states_[id];
    s.guidance.targetPos = targetPos;
    s.guidance.targetVel = targetVel;
    s.guidance.hasTarget = true;
    return Status::Ok;
}

Status Sim::clearGuidanceTarget(StateId id) {
    if (id >= states_.size() || !used_[id]) return Status::InvalidHandle;
    states_[id].guidance.hasTarget = false;
    return Status::Ok;
}

Status Sim::setExternalAccel(StateId id, Vec3 accel_mps2) {
    if (id >= states_.size() || !used_[id]) return Status::InvalidHandle;
    states_[id].externalAccel_mps2 = accel_mps2;
    return Status::Ok;
}

namespace {
// FNV-1a 64. `bits()` reinterprets a scalar's object representation so the
// digest is over exact IEEE-754 / integer bits, not a decimal rounding of them.
struct Fnv {
    std::uint64_t h = 0xCBF29CE484222325ull;
    void byte(std::uint8_t b) { h = (h ^ b) * 0x100000001B3ull; }
    void raw(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::uint8_t*>(p);
        for (std::size_t i = 0; i < n; ++i) byte(b[i]);
    }
    template <class T> void bits(const T& v) { raw(&v, sizeof(T)); }
    void vec(const Vec3& v) { bits(v.x); bits(v.y); bits(v.z); }
    void quat(const Quat& q) { bits(q.w); bits(q.x); bits(q.y); bits(q.z); }
};

// The single field walk every determinism-relevant piece of ProjectileState
// goes through, in a fixed order. stateHash() (hash, via Fnv) and snapshot()
// (serialize, via ByteWriter below) both call this so the two can never drift
// — anything added to ProjectileState that matters to the next step() is added
// here once. `W` need only expose byte()/bits()/vec()/quat() like Fnv.
template <class W>
void visit_state(W& w, const ProjectileState& s) {
    w.vec(s.position);
    w.vec(s.velocity);
    w.bits(s.spin_radps);
    w.vec(s.spinAxis);
    w.bits(s.mediumId);
    w.bits(s.distanceTravelled_m);
    w.bits(s.timeAlive_s);
    w.bits(s.flags);
    w.bits(static_cast<std::uint32_t>(s.typeId));
    w.bits(static_cast<std::int32_t>(s.tier));
    w.bits(static_cast<std::int32_t>(s.precision));
    w.bits(s.impactYaw);
    w.bits(s.expandedDiameter_m);
    w.byte(s.alive ? 1u : 0u);
    w.quat(s.orientation);
    w.vec(s.angVel_radps);
    w.bits(s.angleOfAttack_rad);
    w.bits(s.spinPhase_rad);
    w.bits(s.fuzeArmTime_s);
    w.vec(s.fuzePoint);
    w.vec(s.fuzeDir);
    const GuidanceState& g = s.guidance;
    w.vec(g.targetPos);
    w.vec(g.targetVel);
    w.byte(g.hasTarget ? 1u : 0u);
    w.byte(g.lockLost ? 1u : 0u);
    w.bits(g.timeAlive_s);
    w.vec(g.lastLos);
    w.bits(g.lastLosTime);
    w.vec(g.lastTargetVel);
    w.byte(g.haveLastTargetVel ? 1u : 0u);
    w.vec(s.externalAccel_mps2);
}
} // namespace

std::uint64_t Sim::stateHash() const {
    Fnv f;
    f.bits(rng_);
    f.bits(static_cast<std::uint64_t>(states_.size()));
    for (std::size_t i = 0; i < states_.size(); ++i) {
        f.bits(static_cast<std::uint8_t>(used_[i]));
        if (!used_[i]) continue;
        visit_state(f, states_[i]);
    }
    return f.h;
}

std::uint64_t Sim::stateHash(StateId id) const {
    if (id >= states_.size() || !used_[id]) return 0;
    Fnv f;
    visit_state(f, states_[id]);
    return f.h;
}

namespace {
// Fixed-size little-endian-on-this-platform encoding for snapshot() —
// exact scalar object representation, same discipline as Fnv (never a struct
// memcpy: padding is not initialised).
struct ByteWriter {
    std::vector<std::byte> buf;
    void byte(std::uint8_t b) { buf.push_back(static_cast<std::byte>(b)); }
    void raw(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::byte*>(p);
        buf.insert(buf.end(), b, b + n);
    }
    template <class T> void bits(const T& v) { raw(&v, sizeof(T)); }
    void vec(const Vec3& v) { bits(v.x); bits(v.y); bits(v.z); }
    void quat(const Quat& q) { bits(q.w); bits(q.x); bits(q.y); bits(q.z); }
};

// Reads back what ByteWriter wrote. `ok` latches false on a short buffer;
// callers check it once after a batch of reads instead of after every field.
struct ByteReader {
    const std::byte* p;
    const std::byte* end;
    bool ok = true;
    void raw(void* dst, std::size_t n) {
        if (!ok || static_cast<std::size_t>(end - p) < n) { ok = false; return; }
        std::memcpy(dst, p, n);
        p += n;
    }
    std::uint8_t byteVal() { std::uint8_t v = 0; raw(&v, 1); return v; }
    template <class T> T bitsVal() { T v{}; raw(&v, sizeof(T)); return v; }
    Vec3 vecVal() { Vec3 v{}; v.x = bitsVal<Real>(); v.y = bitsVal<Real>(); v.z = bitsVal<Real>(); return v; }
    Quat quatVal() { Quat q{}; q.w = bitsVal<Real>(); q.x = bitsVal<Real>(); q.y = bitsVal<Real>(); q.z = bitsVal<Real>(); return q; }
};

// Mirrors visit_state()'s field order exactly, reading instead of writing —
// the two must be changed together.
void read_state(ByteReader& r, ProjectileState& s) {
    s.position = r.vecVal();
    s.velocity = r.vecVal();
    s.spin_radps = r.bitsVal<Real>();
    s.spinAxis = r.vecVal();
    s.mediumId = r.bitsVal<std::int32_t>();
    s.distanceTravelled_m = r.bitsVal<Real>();
    s.timeAlive_s = r.bitsVal<Real>();
    s.flags = r.bitsVal<std::uint32_t>();
    s.typeId = r.bitsVal<std::uint32_t>();
    s.tier = static_cast<FidelityTier>(r.bitsVal<std::int32_t>());
    s.precision = static_cast<PrecisionFlag>(static_cast<std::uint32_t>(r.bitsVal<std::int32_t>()));
    s.impactYaw = r.bitsVal<Real>();
    s.expandedDiameter_m = r.bitsVal<Real>();
    s.alive = r.byteVal() != 0;
    s.orientation = r.quatVal();
    s.angVel_radps = r.vecVal();
    s.angleOfAttack_rad = r.bitsVal<Real>();
    s.spinPhase_rad = r.bitsVal<Real>();
    s.fuzeArmTime_s = r.bitsVal<Real>();
    s.fuzePoint = r.vecVal();
    s.fuzeDir = r.vecVal();
    GuidanceState& g = s.guidance;
    g.targetPos = r.vecVal();
    g.targetVel = r.vecVal();
    g.hasTarget = r.byteVal() != 0;
    g.lockLost = r.byteVal() != 0;
    g.timeAlive_s = r.bitsVal<Real>();
    g.lastLos = r.vecVal();
    g.lastLosTime = r.bitsVal<Real>();
    g.lastTargetVel = r.vecVal();
    g.haveLastTargetVel = r.byteVal() != 0;
    s.externalAccel_mps2 = r.vecVal();
}

constexpr std::uint32_t kSnapshotMagic   = 0x314E5350u; // "PSN1"
constexpr std::uint32_t kSnapshotVersion = 1;

// Identifies the registered-type set a snapshot was taken against: every
// type's `id` string, in registration order. Content (mass, drag model, ...)
// is not hashed — restore() only needs "the same types in the same order",
// not "byte-identical registrations".
std::uint64_t type_registry_digest(const std::vector<ProjectileType>& types) {
    Fnv f;
    for (const auto& t : types) {
        f.raw(t.id.data(), t.id.size());
        f.byte(0);
    }
    return f.h;
}
} // namespace

std::vector<std::byte> Sim::snapshot() const {
    ByteWriter w;
    w.bits(kSnapshotMagic);
    w.bits(kSnapshotVersion);
    w.bits(static_cast<std::uint64_t>(types_.size()));
    w.bits(type_registry_digest(types_));
    w.bits(static_cast<std::uint32_t>(cfg_.trajectoryCacheFrames));
    w.bits(rng_);
    w.bits(static_cast<std::uint64_t>(states_.size()));
    for (std::size_t i = 0; i < states_.size(); ++i) {
        w.byte(used_[i] ? 1u : 0u);
        if (!used_[i]) continue;
        visit_state(w, states_[i]);
    }
    if (cfg_.trajectoryCacheFrames > 0) {
        const std::size_t cap = cfg_.trajectoryCacheFrames;
        const TrajRing emptyRing;
        for (std::size_t i = 0; i < states_.size(); ++i) {
            const TrajRing& r = i < traj_.size() ? traj_[i] : emptyRing;
            w.bits(static_cast<std::uint64_t>(r.head));
            w.bits(static_cast<std::uint64_t>(r.count));
            for (std::size_t k = 0; k < cap; ++k) {
                const TrajectorySample smp = k < r.ring.size() ? r.ring[k] : TrajectorySample{};
                w.vec(smp.position);
                w.vec(smp.velocity);
                w.bits(smp.time_s);
            }
        }
    }
    return w.buf;
}

bool Sim::restore(const std::byte* data, std::size_t size) {
    if (!data) return false;
    ByteReader r{data, data + size};
    const auto magic       = r.bitsVal<std::uint32_t>();
    const auto version     = r.bitsVal<std::uint32_t>();
    const auto typeCount   = r.bitsVal<std::uint64_t>();
    const auto typeDigest  = r.bitsVal<std::uint64_t>();
    const auto trajFrames  = r.bitsVal<std::uint32_t>();
    const auto rng         = r.bitsVal<std::uint64_t>();
    const auto stateCount  = r.bitsVal<std::uint64_t>();
    if (!r.ok) return false;
    if (magic != kSnapshotMagic || version != kSnapshotVersion) return false;
    if (typeCount != types_.size()) return false;
    if (typeDigest != type_registry_digest(types_)) return false;
    if (trajFrames != cfg_.trajectoryCacheFrames) return false;

    std::vector<ProjectileState> newStates(static_cast<std::size_t>(stateCount));
    std::vector<std::uint8_t>    newUsed(static_cast<std::size_t>(stateCount), 0);
    for (std::uint64_t i = 0; i < stateCount; ++i) {
        newUsed[i] = r.byteVal();
        if (!r.ok) return false;
        if (newUsed[i]) read_state(r, newStates[i]);
    }
    if (!r.ok) return false;

    std::vector<TrajRing> newTraj;
    if (trajFrames > 0) {
        newTraj.resize(static_cast<std::size_t>(stateCount));
        for (std::uint64_t i = 0; i < stateCount; ++i) {
            TrajRing& ring = newTraj[i];
            const auto head  = r.bitsVal<std::uint64_t>();
            const auto count = r.bitsVal<std::uint64_t>();
            ring.ring.resize(trajFrames);
            for (std::uint32_t k = 0; k < trajFrames; ++k) {
                TrajectorySample smp;
                smp.position = r.vecVal();
                smp.velocity = r.vecVal();
                smp.time_s   = r.bitsVal<Real>();
                ring.ring[k] = smp;
            }
            ring.head  = static_cast<std::size_t>(head);
            ring.count = static_cast<std::size_t>(count);
        }
        if (!r.ok) return false;
    }

    // Everything parsed and validated — commit atomically.
    rng_    = rng;
    states_ = std::move(newStates);
    used_   = std::move(newUsed);
    traj_   = std::move(newTraj);
    return true;
}

std::size_t Sim::trajectorySize(StateId id) const {
    return id < traj_.size() ? traj_[id].count : 0;
}

std::size_t Sim::trajectory(StateId id, TrajectorySample* out, std::size_t max) const {
    if (id >= traj_.size() || !out) return 0;
    const TrajRing& r = traj_[id];
    const std::size_t cap = r.ring.size();
    if (cap == 0 || r.count == 0) return 0;
    const std::size_t start = (r.head + cap - r.count) % cap;
    const std::size_t nw = r.count < max ? r.count : max;
    for (std::size_t k = 0; k < nw; ++k) out[k] = r.ring[(start + k) % cap];
    return nw;
}

bool Sim::sampleTrajectory(StateId id, Seconds t, Vec3& pos, Vec3& vel) const {
    if (id >= traj_.size()) return false;
    const TrajRing& r = traj_[id];
    const std::size_t cap = r.ring.size();
    if (r.count < 2) return false;
    const std::size_t start = (r.head + cap - r.count) % cap;
    auto at = [&](std::size_t k) -> const TrajectorySample& {
        return r.ring[(start + k) % cap];
    };
    if (t <= at(0).time_s) { pos = at(0).position; vel = at(0).velocity; return true; }
    const TrajectorySample& last = at(r.count - 1);
    if (t >= last.time_s) { pos = last.position; vel = last.velocity; return true; }
    std::size_t lo = 0, hi = r.count - 1;
    while (hi - lo > 1) {
        const std::size_t mid = (lo + hi) / 2;
        if (at(mid).time_s <= t) lo = mid; else hi = mid;
    }
    const TrajectorySample& a = at(lo);
    const TrajectorySample& b = at(hi);
    const Real span = b.time_s - a.time_s;
    const Real f = span > Real(0) ? (t - a.time_s) / span : Real(0);
    pos = a.position + (b.position - a.position) * f;
    vel = a.velocity + (b.velocity - a.velocity) * f;
    return true;
}

} // namespace pon
