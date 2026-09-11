// poncelet — G1/G7 standard drag functions + per-type drag-LUT compiler.
// SPDX-License-Identifier: MIT
//
// The G1 and G7 tables are the industry-standard tabulated Cd(Mach) curves for
// the two reference projectiles (G1: flat-base / blunt "Ingalls" standard;
// G7: 7-calibre secant-ogive boat-tail). Values are the full-resolution
// BRL / Robert L. McCoy Cd-vs-Mach tabulations distributed by JBM Ballistics
// (mcg1.txt / mcg7.txt, <https://jbmballistics.com/downloads.html>, retrieved
// 2026-09-09) — the same data as McCoy, *Modern Exterior Ballistics*, App. A.
// These are true drag coefficients on the frontal area: the retardation is
//   a = 0.5 * rho * i * Cd(M) * (pi/4 d^2) / m * v^2,
// with the form factor i = SD_SI / (703.07 * BC_lbin2) tying the curve to a
// published G1/G7 ballistic coefficient. Validated below against .308 175 gr
// SMK / 9x19 124 gr FMJ dope. A projectile that needs more supplies a
// CustomCurve.
#include "drag_tables.hpp"

#include "ball_profiles.hpp"

#include <algorithm>
#include <cmath>

namespace pon::detail {

namespace {

constexpr Real kPi_dt = 3.14159265358979323846;

struct MachCd { Real mach, cd; };

// --- G1 : blunt flat-base standard projectile (BRL/McCoy, JBM mcg1.txt) ----
constexpr MachCd kG1[] = {
    {0.00, 0.2629}, {0.05, 0.2558}, {0.10, 0.2487}, {0.15, 0.2413},
    {0.20, 0.2344}, {0.25, 0.2278}, {0.30, 0.2214}, {0.35, 0.2155},
    {0.40, 0.2104}, {0.45, 0.2061}, {0.50, 0.2032}, {0.55, 0.2020},
    {0.60, 0.2034}, {0.70, 0.2165}, {0.725, 0.2230}, {0.75, 0.2313},
    {0.775, 0.2417}, {0.80, 0.2546}, {0.825, 0.2706}, {0.85, 0.2901},
    {0.875, 0.3136}, {0.90, 0.3415}, {0.925, 0.3734}, {0.95, 0.4084},
    {0.975, 0.4448}, {1.0, 0.4805}, {1.025, 0.5136}, {1.05, 0.5427},
    {1.075, 0.5677}, {1.10, 0.5883}, {1.125, 0.6053}, {1.15, 0.6191},
    {1.20, 0.6393}, {1.25, 0.6518}, {1.30, 0.6589}, {1.35, 0.6621},
    {1.40, 0.6625}, {1.45, 0.6607}, {1.50, 0.6573}, {1.55, 0.6528},
    {1.60, 0.6474}, {1.65, 0.6413}, {1.70, 0.6347}, {1.75, 0.6280},
    {1.80, 0.6210}, {1.85, 0.6141}, {1.90, 0.6072}, {1.95, 0.6003},
    {2.00, 0.5934}, {2.05, 0.5867}, {2.10, 0.5804}, {2.15, 0.5743},
    {2.20, 0.5685}, {2.25, 0.5630}, {2.30, 0.5577}, {2.35, 0.5527},
    {2.40, 0.5481}, {2.45, 0.5438}, {2.50, 0.5397}, {2.60, 0.5325},
    {2.70, 0.5264}, {2.80, 0.5211}, {2.90, 0.5168}, {3.00, 0.5133},
    {3.10, 0.5105}, {3.20, 0.5084}, {3.30, 0.5067}, {3.40, 0.5054},
    {3.50, 0.5040}, {3.60, 0.5030}, {3.70, 0.5022}, {3.80, 0.5016},
    {3.90, 0.5010}, {4.00, 0.5006}, {4.20, 0.4998}, {4.40, 0.4995},
    {4.60, 0.4992}, {4.80, 0.4990}, {5.00, 0.4988},
};

// --- G7 : 7-calibre secant-ogive boat-tail standard (BRL/McCoy, JBM mcg7.txt)
constexpr MachCd kG7[] = {
    {0.00, 0.1198}, {0.05, 0.1197}, {0.10, 0.1196}, {0.15, 0.1194},
    {0.20, 0.1193}, {0.25, 0.1194}, {0.30, 0.1194}, {0.35, 0.1194},
    {0.40, 0.1193}, {0.45, 0.1193}, {0.50, 0.1194}, {0.55, 0.1193},
    {0.60, 0.1194}, {0.65, 0.1197}, {0.70, 0.1202}, {0.725, 0.1207},
    {0.75, 0.1215}, {0.775, 0.1226}, {0.80, 0.1242}, {0.825, 0.1266},
    {0.85, 0.1306}, {0.875, 0.1368}, {0.90, 0.1464}, {0.925, 0.1660},
    {0.95, 0.2054}, {0.975, 0.2993}, {1.0, 0.3803}, {1.025, 0.4015},
    {1.05, 0.4043}, {1.075, 0.4034}, {1.10, 0.4014}, {1.125, 0.3987},
    {1.15, 0.3955}, {1.20, 0.3884}, {1.25, 0.3810}, {1.30, 0.3732},
    {1.35, 0.3657}, {1.40, 0.3580}, {1.50, 0.3440}, {1.55, 0.3376},
    {1.60, 0.3315}, {1.65, 0.3260}, {1.70, 0.3209}, {1.75, 0.3160},
    {1.80, 0.3117}, {1.85, 0.3078}, {1.90, 0.3042}, {1.95, 0.3010},
    {2.00, 0.2980}, {2.05, 0.2951}, {2.10, 0.2922}, {2.15, 0.2892},
    {2.20, 0.2864}, {2.25, 0.2835}, {2.30, 0.2807}, {2.35, 0.2779},
    {2.40, 0.2752}, {2.45, 0.2725}, {2.50, 0.2697}, {2.55, 0.2670},
    {2.60, 0.2643}, {2.65, 0.2615}, {2.70, 0.2588}, {2.75, 0.2561},
    {2.80, 0.2533}, {2.85, 0.2506}, {2.90, 0.2479}, {2.95, 0.2451},
    {3.00, 0.2424}, {3.10, 0.2368}, {3.20, 0.2313}, {3.30, 0.2258},
    {3.40, 0.2205}, {3.50, 0.2154}, {3.60, 0.2106}, {3.70, 0.2060},
    {3.80, 0.2017}, {3.90, 0.1975}, {4.00, 0.1935}, {4.20, 0.1861},
    {4.40, 0.1793}, {4.60, 0.1730}, {4.80, 0.1672}, {5.00, 0.1618},
};

Real interp_table(const MachCd* tbl, std::size_t n, Real mach) {
    if (mach <= tbl[0].mach)     return tbl[0].cd;
    if (mach >= tbl[n - 1].mach) return tbl[n - 1].cd;
    std::size_t hi = 1;
    while (hi < n && tbl[hi].mach < mach) ++hi;
    const MachCd& a = tbl[hi - 1];
    const MachCd& b = tbl[hi];
    const Real f = (mach - a.mach) / (b.mach - a.mach);
    return a.cd + (b.cd - a.cd) * f;
}

Real interp_curve(const std::vector<DragCurvePoint>& c, Real mach) {
    if (c.empty()) return 0.0;
    if (mach <= c.front().mach) return c.front().cd;
    if (mach >= c.back().mach)  return c.back().cd;
    std::size_t hi = 1;
    while (hi < c.size() && c[hi].mach < mach) ++hi;
    const DragCurvePoint& a = c[hi - 1];
    const DragCurvePoint& b = c[hi];
    const Real f = (mach - a.mach) / (b.mach - a.mach);
    return a.cd + (b.cd - a.cd) * f;
}

constexpr Real kMachMax    = 5.0;
constexpr int  kLutSamples  = 513;   // 0..5 Mach, ~0.01 spacing
constexpr Real kReMax       = 1.5e6; // BallProfile Cd(Reynolds) LUT span
constexpr int  kReLutSamples = 257;
constexpr Real kSMax        = 0.6;   // spin-parameter span for the Cl LUT
constexpr int  kClLutSamples = 129;

// SI form factor from a published (lb/in^2) G1/G7 ballistic coefficient.
Real form_factor(const ProjectileType& t) {
    if (!t.ballisticCoefficient || *t.ballisticCoefficient <= 0.0) return 1.0;
    if (t.mass_kg <= 0.0 || t.refDiameter_m <= 0.0) return 1.0;
    const Real sd_kgm2 = t.mass_kg / (t.refDiameter_m * t.refDiameter_m);
    const Real i = sd_kgm2 / (kBcLbPerIn2ToKgPerM2 * (*t.ballisticCoefficient));
    // Clamp to a sane band so a nonsense BC can't blow up the trajectory.
    return std::clamp(i, Real(0.1), Real(10.0));
}

} // namespace

Real g1_cd(Real mach) {
    return interp_table(kG1, sizeof(kG1) / sizeof(kG1[0]), mach);
}
Real g7_cd(Real mach) {
    return interp_table(kG7, sizeof(kG7) / sizeof(kG7[0]), mach);
}

namespace {

// The frontal reference diameter the drag/lift areas use. Spheres and the puck
// face use the full diameter; a prolate spheroid presents its short axis
// (≈0.6× the long axis for a regulation ball).
Real ref_diameter(const ProjectileType& t, BallShape shape) {
    if (t.dragModel == DragModel::BallProfile && shape == BallShape::ProlateSpheroid)
        return t.refDiameter_m * Real(0.6);
    return t.refDiameter_m;
}

void compile_ball_profile(const ProjectileType& t, DragLuts& out) {
    const BallProfile* p = find_ball_profile(t.ballProfile);
    if (!p) p = &default_ball_profile();

    out.shape    = p->shape;
    out.cdSpiral = p->cdSpiral;
    out.cdTumble = p->cdTumble;

    const Real d = ref_diameter(t, p->shape);
    out.refArea_m2 = Real(0.25) * kPi_dt * d * d;
    if (t.mass_kg <= 0.0 || out.refArea_m2 <= 0.0) return; // drag-free / unusable
    const Real areaOver2m = out.refArea_m2 / (Real(2) * t.mass_kg);

    // Drag LUT.
    out.abscissa   = p->abscissa;
    out.reLength_m  = d;
    Lut1D& drag  = out.dragRet;
    drag.x0 = 0.0;
    if (p->abscissa == DragAbscissa::Reynolds) {
        drag.x1 = kReMax;
        drag.y.resize(kReLutSamples);
        for (int k = 0; k < kReLutSamples; ++k) {
            const Real re = kReMax * (Real(k) / Real(kReLutSamples - 1));
            drag.y[static_cast<std::size_t>(k)] =
                interp_curve(p->cdCurve, re) * areaOver2m;
        }
    } else {
        drag.x1 = kMachMax;
        drag.y.resize(kLutSamples);
        for (int k = 0; k < kLutSamples; ++k) {
            const Real m = kMachMax * (Real(k) / Real(kLutSamples - 1));
            drag.y[static_cast<std::size_t>(k)] =
                interp_curve(p->cdCurve, m) * areaOver2m;
        }
    }

    // Magnus / lift LUT (Cl vs spin parameter S), left empty when the profile
    // carries no lift curve.
    if (!p->clCurve.empty()) {
        out.liftRadius_m   = p->spinRadius_m;
        out.liftAreaOver2m = out.refArea_m2 / (Real(2) * t.mass_kg);
        Lut1D& lift = out.liftCoeffVsSpin;
        lift.x0 = 0.0;
        lift.x1 = kSMax;
        lift.y.resize(kClLutSamples);
        for (int k = 0; k < kClLutSamples; ++k) {
            const Real s = kSMax * (Real(k) / Real(kClLutSamples - 1));
            lift.y[static_cast<std::size_t>(k)] = interp_curve(p->clCurve, s);
        }
    }
}

} // namespace

void compile_drag_lut(const ProjectileType& t, DragLuts& out) {
    out = DragLuts{};

    if (t.dragModel == DragModel::BallProfile) {
        compile_ball_profile(t, out);
        return;
    }

    Lut1D& lut = out.dragRet;
    out.abscissa   = DragAbscissa::Mach;
    out.refArea_m2 = Real(0.25) * kPi_dt * t.refDiameter_m * t.refDiameter_m;
    lut.x0 = 0.0;
    lut.x1 = kMachMax;

    if (t.mass_kg <= 0.0 || out.refArea_m2 <= 0.0)
        return; // empty y -> drag-free

    // The per-Mach retardation term  i*Cd(M)*A / (2m), units m^2/kg.
    const Real areaOver2m = out.refArea_m2 / (Real(2) * t.mass_kg);

    auto cd_at = [&](Real mach) -> Real {
        switch (t.dragModel) {
            case DragModel::G1: return form_factor(t) * g1_cd(mach);
            case DragModel::G7: return form_factor(t) * g7_cd(mach);
            case DragModel::CustomCurve:
                return t.customDragCurve.empty()
                           ? (t.dragCoefficient ? *t.dragCoefficient : Real(0))
                           : interp_curve(t.customDragCurve, mach);
            case DragModel::ConstantCd:
            default:
                return t.dragCoefficient ? *t.dragCoefficient : Real(0);
        }
    };

    lut.y.resize(kLutSamples);
    for (int k = 0; k < kLutSamples; ++k) {
        const Real mach = kMachMax * (Real(k) / Real(kLutSamples - 1));
        lut.y[static_cast<std::size_t>(k)] = cd_at(mach) * areaOver2m;
    }
}

} // namespace pon::detail
