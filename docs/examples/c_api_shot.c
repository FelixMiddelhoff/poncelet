/* poncelet example — the same basic shot through the stable C ABI.
 * SPDX-License-Identifier: MIT
 *
 * Compiles as C++ here for the example build, but the header and every call
 * are plain C — bind the same way from Rust/C#/Python/etc.
 */
#include <poncelet/poncelet.h>

#include <stdio.h>

/* Optional diagnostic trace — the C mirror of SimConfig::traceSink. `user`
 * points at a counter we bump per Frame event. */
static void on_trace(const pon_trace_event* ev, void* user) {
    if (ev->kind == PON_TRACE_FRAME) ++*(int*)user;
    else if (ev->kind == PON_TRACE_TRANSONIC_ENTER)
        printf("  [trace] shot %u went transonic at t=%.2fs\n", ev->shot, ev->time_s);
}

int main(void) {
    pon_sim* sim = pon_sim_create(PON_DET_PLATFORM_STABLE);
    if (!sim) return 1;

    pon_sim_set_gravity(sim, (pon_vec3){0.0, -9.80665, 0.0});
    pon_sim_set_air_density(sim, 1.225);

    int frame_traces = 0;
    pon_sim_set_trace_sink(sim, on_trace, &frame_traces);   /* pass NULL to clear */

    pon_projectile_desc d = {0};
    d.id               = "9x19_124gr_fmj";
    d.klass            = 0;                     /* pon::ProjectileClass::Bullet */
    d.drag_model       = PON_DRAG_CONSTANT_CD;
    d.mass_kg          = 0.00804;
    d.ref_diameter_m   = 0.00902;
    d.drag_coefficient = 0.30;
    d.muzzle_speed_mps = 360.0;

    uint32_t type = pon_register_type(sim, &d);
    if (type == 0xFFFFFFFFu) { pon_sim_destroy(sim); return 1; }

    const pon_vec3 muzzle = {0.0, 1.6, 0.0};
    const pon_vec3 aim    = {1.0, 0.0, 0.0};

    /* Aim arc before spawning — draw it on a HUD, or just eyeball the drop. */
    pon_vec3 arc[64];
    size_t arc_n = pon_preview_arc(sim, type, muzzle, aim,
                                   1.0 / 60.0, 3.0, -1e30, arc, 64);
    printf("preview arc: %zu points, muzzle=(%.1f,%.1f,%.1f) last=(%.1f,%.1f,%.1f)\n",
           arc_n, arc[0].x, arc[0].y, arc[0].z,
           arc[arc_n - 1].x, arc[arc_n - 1].y, arc[arc_n - 1].z);

    uint32_t shot = pon_spawn(sim, type, muzzle, aim, 0.0); /* 0 -> muzzle speed */

    for (int i = 0; i < 180; ++i) {
        pon_step(sim, 1.0 / 60.0);
        if (i % 30 == 0) {
            char line[192];
            pon_describe(sim, shot, line, sizeof line);
            printf("%s\n", line);
        }
    }

    printf("%zu projectile(s) still live, %d frame traces\n",
           pon_live_count(sim), frame_traces);
    pon_sim_destroy(sim);
    return frame_traces > 0 ? 0 : 1;
}
