// poncelet — C ABI implementation over the C++ Sim.
// SPDX-License-Identifier: MIT
#ifndef PONCELET_BUILD
#define PONCELET_BUILD
#endif
#include "poncelet/poncelet.h"
#include "poncelet/version.hpp"
#include "poncelet/sim.hpp"
#include "poncelet/catalog.hpp"
#include "poncelet/explosion.hpp"
#include "poncelet/fragmentation.hpp"
#include "poncelet/shapedcharge.hpp"

#include <cstring>
#include <new>
#include <utility>
#include <vector>

using namespace pon;

namespace {
Vec3 to_vec3(pon_vec3 v) { return {v.x, v.y, v.z}; }
pon_vec3 from_vec3(Vec3 v) { return {v.x, v.y, v.z}; }

config::Determinism to_det(pon_determinism d) {
    switch (d) {
        case PON_DET_LOOSE:           return config::Determinism::Loose;
        case PON_DET_BIT_EXACT:       return config::Determinism::BitExact;
        case PON_DET_PLATFORM_STABLE:
        default:                      return config::Determinism::PlatformStable;
    }
}
} // namespace

struct pon_sim {
    Sim       sim;
    EmptyWorld world;
    VectorEventSink sink;
    explicit pon_sim(pon_determinism d)
        : sim(Environment{}, SimConfig{to_det(d)}) {}
    explicit pon_sim(SimConfig cfg)
        : sim(Environment{}, std::move(cfg)) {}
};

extern "C" {

const char* pon_version_string(void) { return PONCELET_VERSION_STRING; }

pon_sim* pon_sim_create(pon_determinism determinism) {
    return new (std::nothrow) pon_sim(determinism);
}

pon_sim* pon_sim_create_ex(pon_determinism determinism, const pon_sim_config* cfg) {
    SimConfig sc;
    sc.determinism = to_det(determinism);
    if (cfg) {
        if (cfg->fixed_step_s > 0.0)    sc.fixedStep_s    = cfg->fixed_step_s;
        if (cfg->pos_tolerance_m > 0.0) sc.posTolerance_m = cfg->pos_tolerance_m;
        sc.integrator = cfg->integrator == PON_INTEGRATOR_SEMI_IMPLICIT
                          ? Integrator::SemiImplicit : Integrator::RK4;
        sc.batchIntegrator       = cfg->batch_integrator != 0;
        sc.trajectoryCacheFrames = cfg->trajectory_cache_frames;
    }
    return new (std::nothrow) pon_sim(std::move(sc));
}

void pon_sim_destroy(pon_sim* sim) { delete sim; }

void pon_sim_set_gravity(pon_sim* sim, pon_vec3 g) {
    if (sim) sim->sim.environment().gravity = to_vec3(g);
}

void pon_sim_set_air_density(pon_sim* sim, double rho_kgm3) {
    if (sim) sim->sim.environment().airDensity_kgm3 = rho_kgm3;
}

void pon_sim_set_atmosphere(pon_sim* sim, double altitude_m, double temperature_K,
                            double pressure_Pa, double rel_humidity) {
    if (!sim) return;
    IsaConditions c;
    c.altitude_m = altitude_m;
    if (temperature_K > 0.0) c.temperature_K = temperature_K;
    if (pressure_Pa   > 0.0) c.pressure_Pa   = pressure_Pa;
    c.relativeHumidity = rel_humidity;
    sim->sim.environment().setAtmosphere(c);
}

void pon_sim_set_wind(pon_sim* sim, pon_wind_fn fn, void* user) {
    if (!sim) return;
    if (!fn) {
        sim->sim.environment().wind = nullptr;
        return;
    }
    sim->sim.environment().wind = [fn, user](Vec3 p, Seconds t) -> Vec3 {
        const pon_vec3 w = fn(pon_vec3{p.x, p.y, p.z}, t, user);
        return Vec3{w.x, w.y, w.z};
    };
}

void pon_sim_set_trace_sink(pon_sim* sim, pon_trace_fn fn, void* user) {
    if (!sim) return;
    if (!fn) { sim->sim.setTraceSink({}); return; }
    sim->sim.setTraceSink([fn, user](const TraceEvent& e) {
        pon_trace_event ev;
        ev.kind     = static_cast<int>(e.kind);
        ev.shot     = e.shot;
        ev.time_s   = e.time_s;
        ev.position = pon_vec3{e.position.x, e.position.y, e.position.z};
        ev.velocity = pon_vec3{e.velocity.x, e.velocity.y, e.velocity.z};
        ev.i0       = e.i0;
        ev.r0       = e.r0;
        fn(&ev, user);
    });
}

int32_t pon_sim_add_medium(pon_sim* sim, const pon_medium_desc* desc) {
    if (!sim || !desc) return -1;
    MediumDesc m;
    m.name            = desc->name ? desc->name : "";
    m.density_kgm3    = desc->density_kgm3;
    m.dragScale       = desc->drag_scale;
    m.buoyancy        = desc->buoyancy;
    m.supercavitation = desc->supercavitation != 0;
    return sim->sim.environment().media.add(m);
}

uint32_t pon_register_type(pon_sim* sim, const pon_projectile_desc* desc) {
    if (!sim || !desc) return kInvalidType;
    ProjectileType t;
    t.id = desc->id ? desc->id : "";
    t.klass = static_cast<ProjectileClass>(desc->klass);
    t.dragModel = static_cast<DragModel>(desc->drag_model);
    t.mass_kg = desc->mass_kg;
    t.refDiameter_m = desc->ref_diameter_m;
    if (desc->ballistic_coefficient > 0.0)
        t.ballisticCoefficient = desc->ballistic_coefficient;
    if (desc->drag_coefficient > 0.0)
        t.dragCoefficient = desc->drag_coefficient;
    if (desc->muzzle_speed_mps > 0.0)
        t.muzzleSpeed_mps = desc->muzzle_speed_mps;
    if (desc->spin_rate_radps != 0.0)
        t.spinRate_radps = desc->spin_rate_radps;
    t.twistRate_m     = desc->twist_rate_m;
    t.millerStability = desc->miller_stability;
    if (desc->aero) {
        const pon_aero_angular& a = *desc->aero;
        t.aero.length_m                 = a.length_m;
        t.aero.axialInertia_kgm2        = a.axial_inertia_kgm2;
        t.aero.transverseInertia_kgm2   = a.transverse_inertia_kgm2;
        t.aero.overturningMomentSlope   = a.overturning_moment_slope;
        t.aero.pitchDampingMoment       = a.pitch_damping_moment;
        t.aero.magnusMomentSlope        = a.magnus_moment_slope;
        t.aero.rollDampingMoment        = a.roll_damping_moment;
        t.aero.liftForceSlope           = a.lift_force_slope;
        t.aero.yawDragFactor            = a.yaw_drag_factor;
        t.aero.transonicOverturnMul     = a.transonic_overturn_mul;
    }
    if (desc->warhead) {
        const pon_warhead_desc& w = *desc->warhead;
        t.warhead.chargeMass_kg     = w.charge_mass_kg;
        t.warhead.tntEquivalence    = w.tnt_equivalence > 0.0 ? w.tnt_equivalence : 1.0;
        t.warhead.fuze              = static_cast<FuzeMode>(w.fuze);
        t.warhead.fuzeDelay_s       = w.fuze_delay_s;
        t.warhead.proximityRadius_m = w.proximity_radius_m;
        t.warhead.surfaceBurst      = w.surface_burst != 0;
        t.warhead.underwater        = w.underwater != 0;
        t.warhead.thermobaric       = w.thermobaric;
    }
    if (desc->guidance) {
        const pon_guidance_desc& g = *desc->guidance;
        t.guidance.law               = static_cast<GuidanceLaw>(g.law);
        t.guidance.navConstant       = g.nav_constant > 0.0 ? g.nav_constant : 4.0;
        t.guidance.maxLateralAccel_g = g.max_lateral_accel_g > 0.0 ? g.max_lateral_accel_g : 30.0;
        t.guidance.seekerHalfFov_rad = g.seeker_half_fov_rad > 0.0 ? g.seeker_half_fov_rad : 0.70;
        t.guidance.activationDelay_s = g.activation_delay_s;
        t.guidance.thrustAccel_mps2  = g.thrust_accel_mps2;
        t.guidance.burnTime_s        = g.burn_time_s;
        t.guidance.inducedDragFactor = g.induced_drag_factor;
    }
    if (desc->custom_drag_curve && desc->custom_drag_curve_count > 0) {
        t.customDragCurve.reserve(desc->custom_drag_curve_count);
        for (size_t i = 0; i < desc->custom_drag_curve_count; ++i)
            t.customDragCurve.push_back({desc->custom_drag_curve[2 * i],
                                        desc->custom_drag_curve[2 * i + 1]});
    }
    return sim->sim.registerType(std::move(t));
}

int pon_catalog_has(const char* id) {
    return (id && catalog::has(id)) ? 1 : 0;
}

uint32_t pon_register_catalog_type(pon_sim* sim, const char* id) {
    if (!sim || !id) return kInvalidType;
    auto t = catalog::find(id);
    if (!t) return kInvalidType;
    return sim->sim.registerType(std::move(*t));
}

uint32_t pon_spawn(pon_sim* sim, uint32_t type_id,
                   pon_vec3 position, pon_vec3 direction, double speed_mps) {
    if (!sim) return kInvalidState;
    LaunchParams lp;
    lp.position = to_vec3(position);
    lp.direction = to_vec3(direction);
    if (speed_mps > 0.0) lp.speed = speed_mps;
    return sim->sim.spawn(type_id, lp);
}

uint32_t pon_spawn_in_medium(pon_sim* sim, uint32_t type_id,
                             pon_vec3 position, pon_vec3 direction,
                             double speed_mps, int32_t medium_id) {
    if (!sim) return kInvalidState;
    LaunchParams lp;
    lp.position = to_vec3(position);
    lp.direction = to_vec3(direction);
    if (speed_mps > 0.0) lp.speed = speed_mps;
    lp.medium = medium_id;
    return sim->sim.spawn(type_id, lp);
}

uint32_t pon_spawn_precise(pon_sim* sim, uint32_t type_id,
                           pon_vec3 position, pon_vec3 direction,
                           double speed_mps, int fidelity_tier,
                           uint32_t precision_flags) {
    if (!sim) return kInvalidState;
    LaunchParams lp;
    lp.position = to_vec3(position);
    lp.direction = to_vec3(direction);
    if (speed_mps > 0.0) lp.speed = speed_mps;
    if (fidelity_tier >= 0 && fidelity_tier <= 2)
        lp.tier = static_cast<FidelityTier>(fidelity_tier);
    lp.precision = static_cast<PrecisionFlag>(precision_flags);
    return sim->sim.spawn(type_id, lp);
}

void pon_sim_set_coriolis(pon_sim* sim, double latitude_rad,
                          double firing_azimuth_from_north_rad) {
    if (sim) sim->sim.environment().setCoriolis(latitude_rad,
                                                firing_azimuth_from_north_rad);
}

double pon_mv_from_powder_temp(double base_mv_mps, double base_T_K,
                               double sensitivity_mps_per_K, double T_K) {
    return mv_from_powder_temp(base_mv_mps, base_T_K, sensitivity_mps_per_K, T_K);
}

void pon_step(pon_sim* sim, double dt_s) {
    if (sim) sim->sim.step(dt_s, sim->world, sim->sink);
}

pon_status pon_get_state(const pon_sim* sim, uint32_t state_id, pon_state* out) {
    if (!sim || !out) return PON_INVALID_ARG;
    const ProjectileState& s = sim->sim.state(state_id);
    out->position = from_vec3(s.position);
    out->velocity = from_vec3(s.velocity);
    out->time_alive_s = s.timeAlive_s;
    out->distance_travelled_m = s.distanceTravelled_m;
    out->medium_id = s.mediumId;
    out->alive = s.alive ? 1 : 0;
    out->flags = s.flags;
    out->orientation_quat[0] = s.orientation.w;
    out->orientation_quat[1] = s.orientation.x;
    out->orientation_quat[2] = s.orientation.y;
    out->orientation_quat[3] = s.orientation.z;
    out->ang_vel_radps       = from_vec3(s.angVel_radps);
    out->angle_of_attack_rad = s.angleOfAttack_rad;
    return PON_OK;
}

// --- Aim / debug helpers -----------------------------------------------------

size_t pon_preview_arc(const pon_sim* sim, uint32_t type_id,
                       pon_vec3 muzzle, pon_vec3 aim,
                       double dt_s, double max_time_s, double ground_y,
                       pon_vec3* out, size_t max) {
    if (!sim) return 0;
    const ProjectileType& type = sim->sim.type(type_id);
    if (type.id.empty()) return 0; // out-of-range / never-registered type_id

    LaunchParams lp;
    lp.position  = to_vec3(muzzle);
    lp.direction = to_vec3(aim);

    std::vector<Vec3> pts;
    preview_arc(type, lp, sim->sim.environment(), dt_s, max_time_s, pts, ground_y);

    if (!out) return pts.size();
    const size_t nw = pts.size() < max ? pts.size() : max;
    for (size_t i = 0; i < nw; ++i) out[i] = from_vec3(pts[i]);
    return nw;
}

size_t pon_preview_arc_ex(const pon_sim* sim, uint32_t type_id,
                          pon_vec3 muzzle, pon_vec3 aim,
                          double dt_s, double max_time_s, double ground_y,
                          pon_trajectory_sample* out, size_t max) {
    if (!sim) return 0;
    const ProjectileType& type = sim->sim.type(type_id);
    if (type.id.empty()) return 0;

    LaunchParams lp;
    lp.position  = to_vec3(muzzle);
    lp.direction = to_vec3(aim);

    std::vector<TrajectorySample> pts;
    preview_arc(type, lp, sim->sim.environment(), dt_s, max_time_s, pts, ground_y);

    if (!out) return pts.size();
    const size_t nw = pts.size() < max ? pts.size() : max;
    for (size_t i = 0; i < nw; ++i) {
        out[i].position = from_vec3(pts[i].position);
        out[i].velocity = from_vec3(pts[i].velocity);
        out[i].time_s   = pts[i].time_s;
    }
    return nw;
}

// --- Trajectory cache --------------------------------------------------------

size_t pon_trajectory(const pon_sim* sim, uint32_t state_id,
                      pon_trajectory_sample* out, size_t max) {
    if (!sim) return 0;
    if (!out || max == 0) return sim->sim.trajectorySize(state_id);
    std::vector<TrajectorySample> buf(max);
    const size_t n = sim->sim.trajectory(state_id, buf.data(), max);
    for (size_t i = 0; i < n; ++i) {
        out[i].position = from_vec3(buf[i].position);
        out[i].velocity = from_vec3(buf[i].velocity);
        out[i].time_s   = buf[i].time_s;
    }
    return n;
}

size_t pon_trajectory_size(const pon_sim* sim, uint32_t state_id) {
    return sim ? sim->sim.trajectorySize(state_id) : 0;
}

int pon_sample_trajectory(const pon_sim* sim, uint32_t state_id, double t_s,
                          pon_vec3* out_pos, pon_vec3* out_vel) {
    if (!sim) return 0;
    Vec3 pos{}, vel{};
    if (!sim->sim.sampleTrajectory(state_id, t_s, pos, vel)) return 0;
    if (out_pos) *out_pos = from_vec3(pos);
    if (out_vel) *out_vel = from_vec3(vel);
    return 1;
}

size_t pon_describe(const pon_sim* sim, uint32_t state_id, char* buf, size_t cap) {
    if (!sim) { if (buf && cap) buf[0] = '\0'; return 0; }
    const ProjectileState& s = sim->sim.state(state_id);
    const ProjectileType&  t = sim->sim.type(s.typeId);
    const std::string text = describe(s, t, sim->sim.environment());
    if (buf && cap > 0) {
        const size_t n = text.size() < cap - 1 ? text.size() : cap - 1;
        std::memcpy(buf, text.data(), n);
        buf[n] = '\0';
    }
    return text.size();
}

size_t pon_describe_type(const pon_sim* sim, uint32_t type_id, char* buf, size_t cap) {
    if (!sim) { if (buf && cap) buf[0] = '\0'; return 0; }
    const std::string text = describe(sim->sim.type(type_id));
    if (buf && cap > 0) {
        const size_t n = text.size() < cap - 1 ? text.size() : cap - 1;
        std::memcpy(buf, text.data(), n);
        buf[n] = '\0';
    }
    return text.size();
}

const char* pon_last_error(const pon_sim* sim) {
    return sim ? sim->sim.lastError() : "";
}

double pon_shot_mach(const pon_sim* sim, uint32_t state_id) {
    if (!sim) return 0.0;
    return mach(sim->sim.state(state_id), sim->sim.environment());
}

double pon_shot_energy_j(const pon_sim* sim, uint32_t state_id) {
    if (!sim) return 0.0;
    const ProjectileState& s = sim->sim.state(state_id);
    return kinetic_energy_J(s, sim->sim.type(s.typeId));
}

pon_vec3 pon_shot_nose(const pon_sim* sim, uint32_t state_id) {
    if (!sim) return {0, 0, 0};
    return from_vec3(nose_direction(sim->sim.state(state_id)));
}

pon_vec3 pon_shot_up(const pon_sim* sim, uint32_t state_id) {
    if (!sim) return {0, 0, 0};
    return from_vec3(up_direction(sim->sim.state(state_id)));
}

double pon_shot_spin_phase(const pon_sim* sim, uint32_t state_id) {
    if (!sim) return 0.0;
    return spin_phase(sim->sim.state(state_id));
}

static pon_status to_c_status(Status s) {
    switch (s) {
        case Status::Ok:            return PON_OK;
        case Status::Unsupported:   return PON_UNSUPPORTED;
        case Status::InvalidType:   return PON_INVALID_TYPE;
        case Status::InvalidHandle: return PON_INVALID_HANDLE;
        case Status::InvalidArg:    return PON_INVALID_ARG;
    }
    return PON_INVALID_ARG;
}

pon_status pon_guide(pon_sim* sim, uint32_t state_id, pon_vec3 target_pos,
                     pon_vec3 target_vel) {
    if (!sim) return PON_INVALID_ARG;
    return to_c_status(sim->sim.guide(state_id, to_vec3(target_pos),
                                      to_vec3(target_vel)));
}

pon_status pon_clear_guidance(pon_sim* sim, uint32_t state_id) {
    if (!sim) return PON_INVALID_ARG;
    return to_c_status(sim->sim.clearGuidanceTarget(state_id));
}

pon_status pon_set_external_accel(pon_sim* sim, uint32_t state_id,
                                  pon_vec3 accel_mps2) {
    if (!sim) return PON_INVALID_ARG;
    return to_c_status(sim->sim.setExternalAccel(state_id, to_vec3(accel_mps2)));
}

size_t pon_live_count(const pon_sim* sim) {
    return sim ? sim->sim.liveCount() : 0;
}

void pon_despawn(pon_sim* sim, uint32_t state_id) {
    if (sim) sim->sim.despawn(state_id);
}

size_t pon_live_ids(const pon_sim* sim, uint32_t* out, size_t max) {
    if (!sim) return 0;
    std::vector<StateId> ids;
    sim->sim.liveIds(ids);
    if (out) {
        const size_t nw = ids.size() < max ? ids.size() : max;
        for (size_t i = 0; i < nw; ++i) out[i] = ids[i];
    }
    return ids.size();
}

void pon_sim_get_stats(const pon_sim* sim, pon_sim_stats* out) {
    if (!out) return;
    if (!sim) { *out = pon_sim_stats{}; return; }
    const SimStats& s = sim->sim.stats();
    out->live_hitscan    = s.liveHitscan;
    out->live_analytic   = s.liveAnalytic;
    out->live_integrated = s.liveIntegrated;
    out->sub_steps        = s.subSteps;
    out->swept_queries    = s.sweptQueries;
    out->batch_groups     = s.batchGroups;
    out->events_emitted   = s.eventsEmitted;
}

// --- Snapshot / restore ------------------------------------------------------

size_t pon_sim_snapshot(const pon_sim* sim, void* buf, size_t cap) {
    if (!sim) return 0;
    const std::vector<std::byte> snap = sim->sim.snapshot();
    if (buf && cap >= snap.size()) {
        std::memcpy(buf, snap.data(), snap.size());
    }
    return snap.size();
}

int pon_sim_restore(pon_sim* sim, const void* buf, size_t size) {
    if (!sim || !buf) return 0;
    return sim->sim.restore(static_cast<const std::byte*>(buf), size) ? 1 : 0;
}

// --- Blast field -----------------------------------------------------------

} // extern "C"

struct pon_burst {
    Burst burst;
    pon_burst(Vec3 o, const WarheadDesc& w, const Environment& e, Seconds t)
        : burst(o, w, e, t) {}
};

extern "C" {

pon_burst* pon_burst_create(pon_vec3 origin, const pon_warhead_desc* w,
                            double ambient_density_kgm3, double sound_speed_mps,
                            double detonation_time_s) {
    if (!w) return nullptr;
    WarheadDesc wd;
    wd.chargeMass_kg     = w->charge_mass_kg;
    wd.tntEquivalence    = w->tnt_equivalence > 0.0 ? w->tnt_equivalence : 1.0;
    wd.fuze              = static_cast<FuzeMode>(w->fuze);
    wd.fuzeDelay_s       = w->fuze_delay_s;
    wd.proximityRadius_m = w->proximity_radius_m;
    wd.surfaceBurst      = w->surface_burst != 0;
    wd.underwater        = w->underwater != 0;
    wd.thermobaric       = w->thermobaric;
    Environment env;
    if (ambient_density_kgm3 > 0.0) env.airDensity_kgm3  = ambient_density_kgm3;
    if (sound_speed_mps      > 0.0) env.speedOfSound_mps = sound_speed_mps;
    return new (std::nothrow) pon_burst(to_vec3(origin), wd, env, detonation_time_s);
}

void pon_burst_destroy(pon_burst* b) { delete b; }

double pon_burst_effective_charge_kg(const pon_burst* b) {
    return b ? b->burst.effectiveCharge_kg() : 0.0;
}

void pon_burst_sample(const pon_burst* b, pon_vec3 point, pon_blast_sample* out) {
    if (!b || !out) return;
    const BlastSample s = b->burst.sampleAt(to_vec3(point));
    out->standoff_m                 = s.standoff_m;
    out->scaled_distance            = s.scaledDistance;
    out->arrival_time_s             = s.arrivalTime_s;
    out->peak_overpressure_pa       = s.peakOverpressure_Pa;
    out->reflected_overpressure_pa  = s.reflectedOverpressure_Pa;
    out->dynamic_pressure_pa        = s.dynamicPressure_Pa;
    out->positive_duration_s        = s.positiveDuration_s;
    out->waveform_decay             = s.waveformDecay;
    out->specific_impulse_pa_s      = s.specificImpulse_Pa_s;
}

double pon_burst_overpressure_at(const pon_burst* b, pon_vec3 point, double t) {
    return b ? b->burst.overpressureAt(to_vec3(point), t) : 0.0;
}

void pon_burst_load_on_body(const pon_burst* b, pon_vec3 centroid, double area_m2,
                            double mass_kg, double drag_cd,
                            double reflection_factor, double los_fraction,
                            pon_vec3* out_impulse_ns, pon_vec3* out_delta_v) {
    if (!b) return;
    BlastTarget tgt;
    tgt.centroid         = to_vec3(centroid);
    tgt.area_m2          = area_m2;
    tgt.mass_kg          = mass_kg;
    tgt.dragCoefficient  = drag_cd > 0.0 ? drag_cd : 2.0;
    tgt.reflectionFactor = reflection_factor > 0.0 ? reflection_factor : 1.0;
    const BlastLoad load = b->burst.loadOnBody(tgt, los_fraction);
    if (out_impulse_ns) *out_impulse_ns = from_vec3(load.impulse_Ns);
    if (out_delta_v)    *out_delta_v    = from_vec3(load.deltaVelocity_mps);
}

// --- Fragmentation -------------------------------------------------------------

size_t pon_generate_fragments(const pon_fragmentation_desc* d, pon_vec3 origin,
                              pon_vec3 source_velocity, double charge_mass_kg,
                              pon_fragment_spec* out, size_t max) {
    if (!d || !out || max == 0) return 0;
    FragmentationDesc fd;
    fd.casingMass_kg          = d->casing_mass_kg;
    fd.gurneyVelocity_mps     = d->gurney_velocity_mps > 0.0 ? d->gurney_velocity_mps : 2440.0;
    fd.mottMu_kg              = d->mott_mu_kg;
    fd.casingInnerDiameter_m  = d->casing_inner_diameter_m;
    fd.casingWallThickness_m  = d->casing_wall_thickness_m;
    fd.mottConstantB          = d->mott_constant_b > 0.0 ? d->mott_constant_b : 1.2;
    fd.minFragmentMass_kg     = d->min_fragment_mass_kg > 0.0 ? d->min_fragment_mass_kg : 2.0e-4;
    fd.maxFragments           = d->max_fragments ? d->max_fragments : 256u;
    fd.fragmentDensity_kgm3   = d->fragment_density_kgm3 > 0.0 ? d->fragment_density_kgm3 : 7850.0;
    fd.fragmentDragCd         = d->fragment_drag_cd > 0.0 ? d->fragment_drag_cd : 1.10;
    fd.spray                  = static_cast<FragmentSpray>(d->spray);
    fd.sprayAxis              = to_vec3(d->spray_axis);
    fd.coneHalfAngle_rad      = d->cone_half_angle_rad > 0.0 ? d->cone_half_angle_rad : 0.5236;
    fd.beamHalfWidth_rad      = d->beam_half_width_rad > 0.0 ? d->beam_half_width_rad : 0.3491;
    fd.beamForwardTilt_rad    = d->beam_forward_tilt_rad > 0.0 ? d->beam_forward_tilt_rad : 0.1745;
    fd.velocityScatter        = d->velocity_scatter > 0.0 ? d->velocity_scatter : 0.15;
    fd.seed                   = d->seed ? d->seed : 0xF00DCAFEull;

    std::vector<FragmentSpec> frags;
    generate_fragments(fd, to_vec3(origin), to_vec3(source_velocity),
                       charge_mass_kg, frags);
    const size_t n = frags.size() < max ? frags.size() : max;
    for (size_t i = 0; i < n; ++i) {
        out[i].position         = from_vec3(frags[i].position);
        out[i].velocity         = from_vec3(frags[i].velocity);
        out[i].mass_kg          = frags[i].mass_kg;
        out[i].diameter_m       = frags[i].diameter_m;
        out[i].drag_cd          = frags[i].dragCoefficient;
        out[i].represents_count = frags[i].representsCount;
    }
    return n;
}

// --- Shaped charges / EFP ------------------------------------------------------

namespace {
ShapedChargeDesc to_shaped_charge(const pon_shaped_charge_desc* d) {
    ShapedChargeDesc sc;
    sc.linerMass_kg         = d->liner_mass_kg;
    sc.chargeDiameter_m     = d->charge_diameter_m;
    sc.kind                 = static_cast<ShapedChargeType>(d->kind);
    sc.coneApexAngle_rad    = d->cone_apex_angle_rad > 0.0 ? d->cone_apex_angle_rad : 1.0472;
    sc.linerDensity_kgm3    = d->liner_density_kgm3 > 0.0 ? d->liner_density_kgm3 : 8960.0;
    sc.gurneyVelocity_mps   = d->gurney_velocity_mps > 0.0 ? d->gurney_velocity_mps : 2680.0;
    sc.jetTipVelocity_mps   = d->jet_tip_velocity_mps;
    sc.jetTailVelocity_mps  = d->jet_tail_velocity_mps;
    sc.particulationTime_s  = d->particulation_time_s > 0.0 ? d->particulation_time_s : 1.6e-4;
    sc.jetMassFraction      = d->jet_mass_fraction;
    sc.spallConeHalfAngle_rad = d->spall_cone_half_angle_rad > 0.0 ? d->spall_cone_half_angle_rad : 0.26;
    sc.seed                 = d->seed ? d->seed : 0x5CEDCA5EULL;
    return sc;
}
Material to_target(double density, double strength, int behaviour) {
    Material m;
    m.density_kgm3 = density > 0.0 ? density : 1000.0;
    m.strength_Pa  = strength >= 0.0 ? strength : 0.0;
    m.behaviour    = static_cast<MaterialBehaviour>(
        behaviour >= 0 && behaviour <= 5 ? behaviour
                                         : static_cast<int>(MaterialBehaviour::Ductile));
    return m;
}
} // namespace

void pon_shaped_charge_formation(const pon_shaped_charge_desc* d, double charge_mass_kg,
                                 pon_jet_formation* out) {
    if (!d || !out) return;
    const JetFormation f = shaped_charge_formation(to_shaped_charge(d), charge_mass_kg);
    out->jet_mass_kg       = f.jetMass_kg;
    out->slug_mass_kg      = f.slugMass_kg;
    out->tip_velocity_mps  = f.tipVelocity_mps;
    out->tail_velocity_mps = f.tailVelocity_mps;
    out->avg_velocity_mps  = f.avgVelocity_mps;
    out->initial_length_m  = f.initialLength_m;
    out->coherent_length_m = f.coherentLength_m;
    out->breakup_standoff_m = f.breakupStandoff_m;
    out->jet_diameter_m    = f.jetDiameter_m;
    out->is_efp            = f.isEFP ? 1 : 0;
}

double pon_shaped_charge_optimal_standoff(const pon_shaped_charge_desc* d,
                                          double charge_mass_kg) {
    if (!d) return 0.0;
    return shaped_charge_optimal_standoff(to_shaped_charge(d), charge_mass_kg);
}

void pon_shaped_charge_penetrate(const pon_shaped_charge_desc* d,
                                 double target_density_kgm3, double target_strength_pa,
                                 int target_behaviour, double standoff_m,
                                 double target_thickness_m, double charge_mass_kg,
                                 pon_shaped_charge_penetration* out) {
    if (!d || !out) return;
    const ShapedChargePenetration p = shaped_charge_penetration(
        to_shaped_charge(d),
        to_target(target_density_kgm3, target_strength_pa, target_behaviour),
        standoff_m, target_thickness_m, charge_mass_kg);
    out->depth_m                = p.depth_m;
    out->effective_jet_length_m = p.effectiveJetLength_m;
    out->standoff_efficiency    = p.standoffEfficiency;
    out->hole_diameter_m        = p.holeDiameter_m;
    out->perforated             = p.perforated ? 1 : 0;
    out->spall                  = p.spall ? 1 : 0;
    out->residual_length_m      = p.residualLength_m;
    out->residual_velocity_mps  = p.residualVelocity_mps;
}

} // extern "C"
