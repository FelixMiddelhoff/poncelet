/* poncelet — stable C ABI.
 * SPDX-License-Identifier: MIT
 *
 * A flat C surface over the C++ Sim so the library can be bound from other
 * engines, Rust, C#, Python, etc. SI units throughout. Item 1 exposes the
 * minimal create / register / spawn / step / query loop; it grows alongside
 * the C++ API.
 */
#ifndef PONCELET_H
#define PONCELET_H

#include <stdint.h>
#include <stddef.h>

#if defined(_WIN32) && defined(PONCELET_SHARED)
  #if defined(PONCELET_BUILD)
    #define PON_API __declspec(dllexport)
  #else
    #define PON_API __declspec(dllimport)
  #endif
#else
  #define PON_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pon_vec3 { double x, y, z; } pon_vec3;

typedef enum pon_status {
    PON_OK = 0,
    PON_UNSUPPORTED,
    PON_INVALID_TYPE,
    PON_INVALID_HANDLE,
    PON_INVALID_ARG
} pon_status;

typedef enum pon_determinism {
    PON_DET_LOOSE = 0,
    PON_DET_PLATFORM_STABLE,
    PON_DET_BIT_EXACT
} pon_determinism;

typedef enum pon_drag_model {
    PON_DRAG_CONSTANT_CD = 0,
    PON_DRAG_G1,
    PON_DRAG_G7,
    PON_DRAG_BALL_PROFILE,
    PON_DRAG_CUSTOM_CURVE
} pon_drag_model;

/* A subset of ProjectileType flat enough for C. `id` is borrowed for the
 * duration of the call. Zeroed fields fall back to class defaults. */
/* Opt-in second-order precision effects (§3.7), OR together on a precise spawn. */
enum {
    PON_PRECISION_NONE             = 0,
    PON_PRECISION_LOCAL_SPEED_SOUND = 1u << 0,
    PON_PRECISION_SPIN_DRIFT       = 1u << 1,
    PON_PRECISION_CORIOLIS         = 1u << 2,
    PON_PRECISION_AERO_JUMP        = 1u << 3,
    PON_PRECISION_ADAPTIVE_RKF45   = 1u << 4,
    PON_PRECISION_TRANSONIC_FLAG   = 1u << 5,
    PON_PRECISION_SIX_DOF          = 1u << 6  /* full rigid-body angular flight */
};

/* ProjectileState::flags bits reported in pon_state.flags. */
enum {
    PON_FLAG_IN_TRANSONIC   = 1u << 0,
    PON_FLAG_PAST_TRANSONIC = 1u << 1,
    PON_FLAG_EXPANDED       = 1u << 2,
    PON_FLAG_TUMBLING       = 1u << 3   /* 6-DOF: yaw exceeded ~60 deg */
};

/* Angular aerodynamics for the 6-DOF path (PON_PRECISION_SIX_DOF). Point
 * `pon_projectile_desc::aero` at one of these, or leave it NULL for all seeds.
 * Every 0 field is derived from mass/diameter/class. Coefficients are per
 * radian of yaw on the reference area and `length_m`; they are seeds. */
typedef struct pon_aero_angular {
    double length_m;                  /* 0 => class ratio * ref_diameter_m */
    double axial_inertia_kgm2;        /* Ix; 0 => solid-body estimate */
    double transverse_inertia_kgm2;   /* It; 0 => solid-body estimate */
    double overturning_moment_slope;  /* C_Ma, >0 statically unstable; 0 => 2.5 */
    double pitch_damping_moment;      /* C_Mq sum, <0 damps; 0 => -6.0 */
    double magnus_moment_slope;       /* C_Mpa; 0 => 0.6 */
    double roll_damping_moment;       /* C_lp, <0; 0 => -0.02 */
    double lift_force_slope;          /* C_La; 0 => 2.0 */
    double yaw_drag_factor;           /* delta C_D = factor*alpha^2; 0 => 4.0 */
    double transonic_overturn_mul;    /* C_Ma *this in Mach 0.9..1.2; 0 => 1.0 */
} pon_aero_angular;

typedef struct pon_projectile_desc {
    const char*    id;
    int            klass;              /* pon::ProjectileClass */
    pon_drag_model drag_model;
    double         mass_kg;
    double         ref_diameter_m;
    double         ballistic_coefficient; /* 0 => derived / unused */
    double         drag_coefficient;      /* 0 => unused */
    double         muzzle_speed_mps;      /* 0 => none */
    double         spin_rate_radps;       /* 0 => none */
    double         twist_rate_m;          /* signed; <0 left twist. 0 => right */
    double         miller_stability;      /* Miller Sg; 0 => 1.8 assumed */
    /* CustomCurve only: flattened [mach0,cd0, mach1,cd1, ...], borrowed for the
     * call. NULL / 0 => fall back to drag_coefficient. */
    const double*  custom_drag_curve;
    size_t         custom_drag_curve_count; /* number of (mach,cd) PAIRS */
    /* 6-DOF angular aerodynamics (PON_PRECISION_SIX_DOF); NULL => all seeds.
     * Borrowed for the call. */
    const pon_aero_angular* aero;
    /* Explosive payload (§8a); NULL => inert round. Borrowed for the call. */
    const struct pon_warhead_desc* warhead;
    /* Guided munition (§8a); NULL => unguided. Borrowed for the call. */
    const struct pon_guidance_desc* guidance;
} pon_projectile_desc;

typedef enum pon_guidance_law {
    PON_GUIDANCE_NONE = 0,
    PON_GUIDANCE_PURSUIT,          /* steer straight at the target */
    PON_GUIDANCE_PROPORTIONAL_NAV, /* a = N * Vc * LOS-rate, leads a crosser */
    PON_GUIDANCE_AUGMENTED_PN      /* + N/2 * target-accel */
} pon_guidance_law;

/* Guided-munition config on a projectile type. law == PON_GUIDANCE_NONE => the
 * guidance path is skipped. Update the seeker track each frame with pon_guide();
 * pon_set_external_accel() is the raw per-step force hook (thrust / scripted
 * course correction), independent of the law. */
typedef struct pon_guidance_desc {
    int    law;                    /* pon_guidance_law */
    double nav_constant;           /* N (PN/APN); 0 => 4.0 */
    double max_lateral_accel_g;    /* airframe g-limit; 0 => 30 */
    double seeker_half_fov_rad;    /* lock lost past this off-boresight; 0 => 0.70 */
    double activation_delay_s;     /* guidance inert for the first t of flight */
    double thrust_accel_mps2;      /* > 0 => axial boost while burning */
    double burn_time_s;            /* thrust duration from launch */
    double induced_drag_factor;    /* > 0 => speed loss proportional to |a_lat| */
} pon_guidance_desc;

typedef enum pon_fuze_mode {
    PON_FUZE_CONTACT = 0,   /* on the first surface contact */
    PON_FUZE_DELAYED,       /* fuze_delay_s after the first contact */
    PON_FUZE_TIMED_AIRBURST,/* fuze_delay_s after launch */
    PON_FUZE_PROXIMITY      /* geometry within proximity_radius_m ahead */
} pon_fuze_mode;

/* Explosive warhead on a projectile type. charge_mass_kg <= 0 => inert. */
typedef struct pon_warhead_desc {
    double charge_mass_kg;
    double tnt_equivalence;    /* blast yield vs TNT; 0 => 1.0 */
    int    fuze;               /* pon_fuze_mode */
    double fuze_delay_s;
    double proximity_radius_m;
    int    surface_burst;      /* 0/1 — ~1.8x yield for a ground/hard burst */
    int    underwater;         /* 0/1 — Cole similitude instead of Kinney-Graham */
    double thermobaric;        /* 0 = conventional; >0 = enhanced blast */
} pon_warhead_desc;

typedef struct pon_state {
    pon_vec3 position;
    pon_vec3 velocity;
    double   time_alive_s;
    double   distance_travelled_m;
    int      medium_id;
    int      alive;
    uint32_t flags;              /* PON_FLAG_* */
    /* 6-DOF only (0 / identity otherwise). Quaternion is (w,x,y,z), body->world,
     * nose along body +x. `ang_vel_radps` is body-frame (x = spin rate). */
    double   orientation_quat[4];
    pon_vec3 ang_vel_radps;
    double   angle_of_attack_rad;
} pon_state;

typedef struct pon_sim pon_sim;

PON_API const char* pon_version_string(void);

PON_API pon_sim* pon_sim_create(pon_determinism determinism);
PON_API void     pon_sim_destroy(pon_sim* sim);

typedef enum pon_integrator {
    PON_INTEGRATOR_RK4 = 0,
    PON_INTEGRATOR_SEMI_IMPLICIT = 1
} pon_integrator;

/* A flat subset of SimConfig for pon_sim_create_ex. Zero-initialize (`= {0}`)
 * to get the plain pon_sim_create(det) defaults for every field: 0 falls back
 * to fixedStep_s = 1/1000, posTolerance_m = 0.02; integrator 0 IS
 * PON_INTEGRATOR_RK4 (the default); batch_integrator 0 and
 * trajectory_cache_frames 0 ARE their SimConfig defaults (off). */
typedef struct pon_sim_config {
    double   fixed_step_s;            /* 0 => 1.0/1000.0 */
    int      integrator;              /* pon_integrator */
    double   pos_tolerance_m;         /* 0 => 0.02 */
    int      batch_integrator;        /* 0/1 */
    uint32_t trajectory_cache_frames; /* 0 => off */
} pon_sim_config;

/* As pon_sim_create, plus the SimConfig fields pon_sim_config exposes.
 * `cfg = NULL` is exactly pon_sim_create(determinism). */
PON_API pon_sim* pon_sim_create_ex(pon_determinism determinism,
                                   const pon_sim_config* cfg);

PON_API void pon_sim_set_gravity(pon_sim* sim, pon_vec3 g);
PON_API void pon_sim_set_air_density(pon_sim* sim, double rho_kgm3);

/* Resolve firing-site conditions through the ISA model and store the resulting
 * air density + local speed of sound on the sim's environment. Pass 0 for
 * temperature_K / pressure_Pa to use the ISA value at that altitude;
 * rel_humidity is 0..1. */
PON_API void pon_sim_set_atmosphere(pon_sim* sim, double altitude_m,
                                    double temperature_K, double pressure_Pa,
                                    double rel_humidity);

/* Wind: world-frame wind velocity (m/s) at `pos` and absolute sim time `t`
 * (seconds since a projectile was spawned — poncelet passes each projectile's
 * own time-alive). `user` is the pointer handed to pon_sim_set_wind and must
 * stay valid for as long as the wind function is installed. Pass fn = NULL to
 * clear the wind field (back to still air). */
typedef pon_vec3 (*pon_wind_fn)(pon_vec3 pos, double t, void* user);
PON_API void pon_sim_set_wind(pon_sim* sim, pon_wind_fn fn, void* user);

/* Optional per-shot diagnostic trace — the C mirror of SimConfig::traceSink.
 * Off until installed and zero-cost when off; read-only, so a traced run behaves
 * (and hashes) identically to an untraced one. Answers "why did that shot do
 * that" without a debugger. */
typedef enum pon_trace_kind {
    PON_TRACE_FRAME           = 0, /* once per live shot per pon_step: i0 = fidelity tier (0/1/2), r0 = speed m/s */
    PON_TRACE_SUBSTEP         = 1, /* adaptive sub-step budget that frame: i0 = peak sub-step count */
    PON_TRACE_TRANSONIC_ENTER = 2, /* Mach entered 0.8..1.2 (needs PON_PRECISION_TRANSONIC_FLAG) */
    PON_TRACE_TRANSONIC_EXIT  = 3, /* ...and left it */
    PON_TRACE_MEDIUM_CHANGED  = 4, /* crossed into a new medium: i0 = new medium id */
    PON_TRACE_GUIDANCE_LOST   = 5  /* seeker track dropped — the shot now coasts */
} pon_trace_kind;

typedef struct pon_trace_event {
    int      kind;          /* pon_trace_kind */
    uint32_t shot;          /* state id */
    double   time_s;        /* shot's time-alive */
    pon_vec3 position;
    pon_vec3 velocity;
    int32_t  i0;            /* kind-specific integer (see pon_trace_kind) */
    double   r0;            /* kind-specific real    (see pon_trace_kind) */
} pon_trace_event;

/* `user` is the pointer handed here and must outlive the callback. Pass
 * fn = NULL to remove the trace. */
typedef void (*pon_trace_fn)(const pon_trace_event* ev, void* user);
PON_API void pon_sim_set_trace_sink(pon_sim* sim, pon_trace_fn fn, void* user);

/* A caller-defined medium (§3.3). `name` is copied. Returns the new medium id
 * (>= 2; 0 = air, 1 = water are built in), or -1 on failure. */
typedef struct pon_medium_desc {
    const char* name;
    double      density_kgm3;
    double      drag_scale;      /* multiplies the drag deceleration */
    double      buoyancy;        /* upward accel as a fraction of |gravity| */
    int         supercavitation; /* 0 / 1 */
} pon_medium_desc;
PON_API int32_t pon_sim_add_medium(pon_sim* sim, const pon_medium_desc* desc);

/* Coriolis / Eötvös inputs (PON_PRECISION_CORIOLIS). `latitude_rad` is signed
 * (north positive); `firing_azimuth_from_north_rad` is the compass bearing of
 * the shot's downrange direction (0 = north, clockwise). */
PON_API void pon_sim_set_coriolis(pon_sim* sim, double latitude_rad,
                                  double firing_azimuth_from_north_rad);

/* Muzzle velocity shifted for powder temperature (§3.7): a linear sensitivity
 * (m/s per K) about a reference temperature. An input helper — nothing applies
 * it automatically. */
PON_API double pon_mv_from_powder_temp(double base_mv_mps, double base_T_K,
                                       double sensitivity_mps_per_K, double T_K);

/* Returns a type id, or 0xFFFFFFFF on failure. */
PON_API uint32_t pon_register_type(pon_sim* sim, const pon_projectile_desc* desc);

/* 1 if `id` names an entry in the shipped projectile catalog (data/
 * projectiles.csv) or a runtime overlay, else 0. */
PON_API int pon_catalog_has(const char* id);

/* Register a named catalog round by id. Returns a type id, or 0xFFFFFFFF if
 * `id` is unknown. Equivalent to pon::catalog::get(id) + registerType. */
PON_API uint32_t pon_register_catalog_type(pon_sim* sim, const char* id);

/* Returns a state id, or 0xFFFFFFFF on failure. `speed_mps <= 0` uses the
 * type's muzzle speed. */
PON_API uint32_t pon_spawn(pon_sim* sim, uint32_t type_id,
                           pon_vec3 position, pon_vec3 direction,
                           double speed_mps);

/* As pon_spawn, but the shot starts in `medium_id` (index from
 * pon_sim_add_medium, or the built-in 0 = air / 1 = water). Use it when spawning
 * already submerged. */
PON_API uint32_t pon_spawn_in_medium(pon_sim* sim, uint32_t type_id,
                                     pon_vec3 position, pon_vec3 direction,
                                     double speed_mps, int32_t medium_id);

/* As pon_spawn, plus a FidelityTier (0 Hitscan / 1 AnalyticDrag / 2 Integrated)
 * and a PON_PRECISION_* bitmask. */
PON_API uint32_t pon_spawn_precise(pon_sim* sim, uint32_t type_id,
                                   pon_vec3 position, pon_vec3 direction,
                                   double speed_mps, int fidelity_tier,
                                   uint32_t precision_flags);

/* Advance all live projectiles by dt against an empty (hit-nothing) world.
 * The World callback surface comes to the C ABI in item 6. */
PON_API void pon_step(pon_sim* sim, double dt_s);

PON_API pon_status pon_get_state(const pon_sim* sim, uint32_t state_id,
                                 pon_state* out);

/* --- Aim / debug helpers ---------------------------------------------------
 * Finish the bindings story for a pure-C caller: draw an aim arc, get the
 * one-line debug string, find out why a register/spawn call failed. */

/* Predicted flight path without spawning, on `type_id`'s registration (must
 * be a value pon_register_type()/pon_register_catalog_type() returned on this
 * sim; an out-of-range or never-registered id writes/returns nothing). Runs
 * the AnalyticDrag closed form (drag + gravity + wind) — mirrors
 * pon::preview_arc. Fills up to `max` points of `out`, oldest (muzzle) first;
 * returns the number of points the arc actually has (call with out = NULL, or
 * max = 0, to measure before allocating; a return greater than `max` means
 * the arc was truncated). */
PON_API size_t pon_preview_arc(const pon_sim* sim, uint32_t type_id,
                               pon_vec3 muzzle, pon_vec3 aim,
                               double dt_s, double max_time_s, double ground_y,
                               pon_vec3* out, size_t max);

/* One point on a predicted flight path, with velocity + flight time alongside
 * position (mirrors pon::TrajectorySample). */
typedef struct pon_trajectory_sample {
    pon_vec3 position;
    pon_vec3 velocity;
    double   time_s;
} pon_trajectory_sample;

/* As pon_preview_arc, but each point also carries velocity + flight time —
 * an aim UI can read impact speed / time-to-target at the reticle instead of
 * just drawing a line. Mirrors the pon::preview_arc TrajectorySample overload. */
PON_API size_t pon_preview_arc_ex(const pon_sim* sim, uint32_t type_id,
                                  pon_vec3 muzzle, pon_vec3 aim,
                                  double dt_s, double max_time_s, double ground_y,
                                  pon_trajectory_sample* out, size_t max);

/* --- Trajectory cache (SimConfig::trajectoryCacheFrames > 0) ---------------
 * Read back the flight-path polyline pon_step() has been recording for a live
 * shot — bullet-cam / late-join replay from a binding, without re-integrating.
 * Mirrors Sim::trajectory() / trajectorySize() / sampleTrajectory(). */

/* Copies the cached samples for `state_id`, oldest first, into `out` (up to
 * `max`); returns the number written (call with out = NULL, or max = 0, to
 * measure). Empty when the cache is off, the id never lived, or no frame has
 * stepped yet. */
PON_API size_t pon_trajectory(const pon_sim* sim, uint32_t state_id,
                              pon_trajectory_sample* out, size_t max);

/* Number of samples currently cached for `state_id`. */
PON_API size_t pon_trajectory_size(const pon_sim* sim, uint32_t state_id);

/* Position + velocity at flight time `t_s` (ProjectileState::timeAlive_s
 * units), linearly interpolated between the two bracketing cached samples and
 * clamped at the ends. Returns 0 (out_pos/out_vel untouched) with fewer than
 * two samples cached, 1 on success. */
PON_API int pon_sample_trajectory(const pon_sim* sim, uint32_t state_id, double t_s,
                                  pon_vec3* out_pos, pon_vec3* out_vel);

/* One-line human summary of a live shot into `buf` (NUL-terminated, truncated
 * to `cap` bytes including the NUL; pass cap = 0 / buf = NULL to just measure).
 * Returns the untruncated length (excluding the NUL) that was or would have
 * been written — compare against `cap` to detect truncation. Mirrors
 * pon::describe(state, type, env); an invalid `state_id` describes the
 * (dead, zeroed) default state rather than failing. */
PON_API size_t pon_describe(const pon_sim* sim, uint32_t state_id,
                            char* buf, size_t cap);

/* One-line human summary of a registered TYPE, for content authoring / a
 * catalog browser: "9x19_124gr_fmj: 8.0 g, 9.0 mm, Cd 0.30, ~360 m/s, ~518 J
 * muzzle". Same truncation convention as pon_describe. `type_id` out of range
 * describes an all-zero type ("?: 0.0 g, ..."), not an error. Mirrors
 * pon::describe(const ProjectileType&). */
PON_API size_t pon_describe_type(const pon_sim* sim, uint32_t type_id,
                                 char* buf, size_t cap);

/* Human-readable reason the most recent pon_register_type / pon_spawn* on
 * this sim returned an invalid handle. "" when that call succeeded, or when
 * none has been made yet. Pointer into sim-owned storage, valid until the
 * next such call on this sim — copy it if you need to keep it. Mirrors
 * Sim::lastError(). */
PON_API const char* pon_last_error(const pon_sim* sim);

/* Local Mach number / kinetic energy (J) of a live shot — the same figures
 * pon_describe folds into its "E=... M=..." tail, for a HUD that wants the
 * numbers without string-parsing. `energy_j` is 0 for an invalid `state_id`
 * or a type with no mass_kg set (see pon::kinetic_energy_J). */
PON_API double pon_shot_mach(const pon_sim* sim, uint32_t state_id);
PON_API double pon_shot_energy_j(const pon_sim* sim, uint32_t state_id);

/* 6-DOF orientation helpers (PON_PRECISION_SIX_DOF) — world-frame nose / "up"
 * direction and the accumulated roll angle (radians), the same figures
 * pon_state's orientation_quat carries, without a caller doing the quaternion
 * rotation itself. Identity / 0 for every other fidelity tier — no need to
 * branch on it before calling. Mirrors pon::nose_direction / up_direction /
 * spin_phase. An out-of-range `state_id` reads the same all-zero default
 * state pon_get_state would — nose/up come back as the identity-orientation
 * {1,0,0} / {0,1,0}, spin_phase as 0 — not a {0,0,0} sentinel. */
PON_API pon_vec3 pon_shot_nose(const pon_sim* sim, uint32_t state_id);
PON_API pon_vec3 pon_shot_up(const pon_sim* sim, uint32_t state_id);
PON_API double   pon_shot_spin_phase(const pon_sim* sim, uint32_t state_id);

/* --- Guided munitions (§8a) --------------------------------------------------
 * Update the seeker track for a guided shot each frame before pon_step (pass
 * a zero target_vel for a static aim point); pon_clear_guidance drops the track
 * (the shot coasts). pon_set_external_accel adds a constant world-frame
 * acceleration (m/s²) to the shot every sub-step until changed — thrust, a
 * scripted correction, or the caller's own guidance law; pass {0,0,0} to clear.
 */
PON_API pon_status pon_guide(pon_sim* sim, uint32_t state_id,
                             pon_vec3 target_pos, pon_vec3 target_vel);
PON_API pon_status pon_clear_guidance(pon_sim* sim, uint32_t state_id);
PON_API pon_status pon_set_external_accel(pon_sim* sim, uint32_t state_id,
                                          pon_vec3 accel_mps2);
PON_API size_t     pon_live_count(const pon_sim* sim);
PON_API void       pon_despawn(pon_sim* sim, uint32_t state_id);

/* Fills up to `max` entries of `out` with the id of every live projectile;
 * returns the live count (same as pon_live_count — call with out = NULL, or
 * max = 0, to just get the count). */
PON_API size_t     pon_live_ids(const pon_sim* sim, uint32_t* out, size_t max);

/* Aggregate per-frame counters from the last pon_step() — the profiling
 * counterpart to pon_sim_set_trace_sink's per-shot view. Mirrors SimStats. */
typedef struct pon_sim_stats {
    uint32_t live_hitscan;
    uint32_t live_analytic;
    uint32_t live_integrated;
    uint64_t sub_steps;      /* Integrated/SixDOF sub-step integrations run */
    uint64_t swept_queries;  /* World::raycast() calls */
    uint32_t batch_groups;   /* batch_integrator groups formed (0 if off) */
    uint32_t events_emitted;
} pon_sim_stats;
PON_API void pon_sim_get_stats(const pon_sim* sim, pon_sim_stats* out);

/* --- Snapshot / restore (rollback netcode) --------------------------------
 * The C mirror of Sim::snapshot()/restore(). `pon_sim_snapshot` fills up to
 * `cap` bytes of `buf` and always returns the size actually needed — call
 * once with cap = 0 (buf may be NULL) to measure, allocate, call again.
 * `pon_sim_restore` requires a sim with the same registered types (same
 * count, same ids, same order) and the same trajectory-cache setting as the
 * sim the snapshot was taken from; it returns 0 and leaves the sim untouched
 * on a truncated buffer or a version/registry/cache mismatch, 1 on success. */
PON_API size_t pon_sim_snapshot(const pon_sim* sim, void* buf, size_t cap);
PON_API int    pon_sim_restore(pon_sim* sim, const void* buf, size_t size);

/* --- Explosive blast field (§8a) ------------------------------------------
 *
 * A standalone object — no sim, no projectile. Build one at a detonation point
 * (from a Detonated event, or placed by hand) and query the blast. Air bursts
 * use the Kinney & Graham fits; warhead.underwater switches to Cole similitude.
 */
typedef struct pon_burst pon_burst;

/* `ambient_*` describe the air at the burst; pass 0 for ISA sea-level defaults.
 * Ignored when warhead->underwater is set. */
PON_API pon_burst* pon_burst_create(pon_vec3 origin, const pon_warhead_desc* warhead,
                                    double ambient_density_kgm3,
                                    double sound_speed_mps,
                                    double detonation_time_s);
PON_API void       pon_burst_destroy(pon_burst* burst);
PON_API double     pon_burst_effective_charge_kg(const pon_burst* burst);

typedef struct pon_blast_sample {
    double standoff_m;
    double scaled_distance;
    double arrival_time_s;
    double peak_overpressure_pa;
    double reflected_overpressure_pa;
    double dynamic_pressure_pa;
    double positive_duration_s;
    double waveform_decay;
    double specific_impulse_pa_s;
} pon_blast_sample;

PON_API void   pon_burst_sample(const pon_burst* burst, pon_vec3 point,
                                pon_blast_sample* out);
/* Incident overpressure (Pa) at `point` at absolute time `t` (Friedlander). */
PON_API double pon_burst_overpressure_at(const pon_burst* burst, pon_vec3 point,
                                         double t);
/* Positive-phase load on a rigid body. `los_fraction` (0..1) scales for cover.
 * `out_impulse_ns` / `out_delta_v` may be NULL. */
PON_API void   pon_burst_load_on_body(const pon_burst* burst, pon_vec3 centroid,
                                      double area_m2, double mass_kg,
                                      double drag_cd, double reflection_factor,
                                      double los_fraction,
                                      pon_vec3* out_impulse_ns,
                                      pon_vec3* out_delta_v);

/* --- Casing fragmentation (§8a) -----------------------------------------------
 *
 * Turn a detonation into a fragment spray: Mott mass spectrum + Gurney launch
 * speed. Each fragment is one projectile — spawn them into a sim with the C++
 * pon::spawn_fragments, or drive your own spawn loop off `pon_fragment_spec`.
 */
typedef enum pon_fragment_spray {
    PON_FRAG_ISOTROPIC = 0,
    PON_FRAG_CONE,
    PON_FRAG_CYLINDER_BEAM
} pon_fragment_spray;

typedef struct pon_fragmentation_desc {
    double   casing_mass_kg;        /* <= 0 => no fragmentation */
    double   gurney_velocity_mps;   /* sqrt(2E'); 0 => 2440 (TNT) */
    double   mott_mu_kg;            /* 0 => from casing geometry / fallback */
    double   casing_inner_diameter_m;
    double   casing_wall_thickness_m;
    double   mott_constant_b;       /* 0 => 1.2 (SI-fitted seed) */
    double   min_fragment_mass_kg;  /* 0 => 2e-4 */
    uint32_t max_fragments;         /* 0 => 256 */
    double   fragment_density_kgm3; /* 0 => 7850 (steel) */
    double   fragment_drag_cd;      /* 0 => 1.10 */
    int      spray;                 /* pon_fragment_spray */
    pon_vec3 spray_axis;            /* {0,0,0} => {1,0,0} */
    double   cone_half_angle_rad;   /* 0 => 0.5236 */
    double   beam_half_width_rad;   /* 0 => 0.3491 */
    double   beam_forward_tilt_rad; /* 0 => 0.1745 */
    double   velocity_scatter;      /* 0 => 0.15 */
    uint64_t seed;                  /* 0 => 0xF00DCAFE */
} pon_fragmentation_desc;

typedef struct pon_fragment_spec {
    pon_vec3 position;
    pon_vec3 velocity;              /* world, includes the source velocity */
    double   mass_kg;
    double   diameter_m;
    double   drag_cd;
    double   represents_count;
} pon_fragment_spec;

/* Fill up to `max` entries of `out` with the fragment spray; returns the count
 * written. Deterministic given the desc + seed. `charge_mass_kg` is the parent
 * warhead's explosive fill (for the Gurney ratio). */
PON_API size_t pon_generate_fragments(const pon_fragmentation_desc* desc,
                                      pon_vec3 origin, pon_vec3 source_velocity,
                                      double charge_mass_kg,
                                      pon_fragment_spec* out, size_t max);

/* --- Shaped charges / EFP (§8a) ---------------------------------------------
 *
 * Simplified Birkhoff/PER jet formation + standoff-dependent hydrodynamic
 * penetration + back-face spall. Stateless — evaluate off a Detonated event
 * (its channel_axis is the jet aim). `charge_mass_kg` is the parent warhead's
 * explosive fill. Behind-armour debris is C++ only (pon::shaped_charge_behind_armour).
 */
typedef enum pon_shaped_charge_type {
    PON_SHAPED_CONICAL_JET = 0, /* HEAT — stretching hypervelocity jet */
    PON_SHAPED_EFP              /* explosively formed penetrator — single slug */
} pon_shaped_charge_type;

typedef struct pon_shaped_charge_desc {
    double   liner_mass_kg;        /* <= 0 => no shaped-charge effect */
    double   charge_diameter_m;    /* CD; 0 => coarse fallback from liner mass */
    int      kind;                 /* pon_shaped_charge_type */
    double   cone_apex_angle_rad;  /* 0 => 1.0472 (60°); ConicalJet only */
    double   liner_density_kgm3;   /* 0 => 8960 (copper) */
    double   gurney_velocity_mps;  /* 0 => 2680 (Comp-B) */
    double   jet_tip_velocity_mps; /* 0 => derived */
    double   jet_tail_velocity_mps;/* 0 => derived */
    double   particulation_time_s; /* 0 => 1.6e-4 */
    double   jet_mass_fraction;    /* 0 => derived from the cone angle */
    double   spall_cone_half_angle_rad; /* 0 => 0.26 */
    uint64_t seed;                 /* 0 => 0x5CEDCA5E */
} pon_shaped_charge_desc;

typedef struct pon_jet_formation {
    double jet_mass_kg;
    double slug_mass_kg;
    double tip_velocity_mps;
    double tail_velocity_mps;
    double avg_velocity_mps;
    double initial_length_m;
    double coherent_length_m;
    double breakup_standoff_m;
    double jet_diameter_m;
    int    is_efp;
} pon_jet_formation;

typedef struct pon_shaped_charge_penetration {
    double depth_m;
    double effective_jet_length_m;
    double standoff_efficiency;
    double hole_diameter_m;
    int    perforated;
    int    spall;
    double residual_length_m;
    double residual_velocity_mps;
} pon_shaped_charge_penetration;

PON_API void pon_shaped_charge_formation(const pon_shaped_charge_desc* desc,
                                         double charge_mass_kg,
                                         pon_jet_formation* out);

PON_API double pon_shaped_charge_optimal_standoff(const pon_shaped_charge_desc* desc,
                                                  double charge_mass_kg);

/* Penetrate a target defined by density / strength / behaviour
 * (0..5 = Brittle/Ductile/Fibrous/Membrane/Granular/Fluid, matching
 * pon MaterialBehaviour) after `standoff_m` of flight. `target_thickness_m <= 0`
 * treats the target as semi-infinite (no perforation / spall verdict). */
PON_API void pon_shaped_charge_penetrate(const pon_shaped_charge_desc* desc,
                                         double target_density_kgm3,
                                         double target_strength_pa,
                                         int target_behaviour,
                                         double standoff_m,
                                         double target_thickness_m,
                                         double charge_mass_kg,
                                         pon_shaped_charge_penetration* out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PONCELET_H */
