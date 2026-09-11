// poncelet — drag / lift models.
// SPDX-License-Identifier: MIT
//
// Item 3 adds the G1/G7 standard drag functions, BC scaling and caller-supplied
// custom Cd(Mach) curves; the BallProfile system is item 4. Every model
// resolves, per registered type, to a branch-free lookup table (a Mach-indexed
// retardation term, + a lift LUT vs spin parameter) so the hot loop is table
// reads + one cross product — no pow/exp/sqrt (§3.2, and the determinism prep
// in §3.6).
#pragma once

#include "poncelet/types.hpp"

#include <vector>

namespace pon {

// A 1-D linearly-interpolated, branch-free lookup table over [x0, x1].
struct Lut1D {
    Real              x0 = 0;
    Real              x1 = 1;
    std::vector<Real> y;   // >= 2 samples, uniformly spaced

    Real sample(Real x) const {
        if (y.size() < 2) return y.empty() ? Real(0) : y.front();
        const Real u  = (x - x0) / (x1 - x0);
        const Real fi = u * Real(y.size() - 1);
        const Real cl = fi < Real(0) ? Real(0)
                       : (fi > Real(y.size() - 1) ? Real(y.size() - 1) : fi);
        const std::size_t i = static_cast<std::size_t>(cl);
        const std::size_t j = i + 1 < y.size() ? i + 1 : i;
        const Real f = cl - Real(i);
        return y[i] + (y[j] - y[i]) * f;
    }
};

// One sample of a caller-supplied custom drag curve (Doppler-radar "custom
// drag model"). `mach` samples need not be uniformly spaced; the model
// compiler resamples them onto the uniform LUT. `cd` is the true drag
// coefficient of *this* projectile on its reference area — no BC form-factor
// scaling is applied on top.
struct DragCurvePoint {
    Real mach = 0;
    Real cd   = 0;
};

// lb/in^2 -> kg/m^2, for turning a published (imperial) ballistic coefficient
// into an SI form factor. BC[lb/in^2] = (m/d^2)[kg/m^2] / (i * 703.0696...).
constexpr Real kBcLbPerIn2ToKgPerM2 = 703.069579;

// Per-registered-type precomputed aerodynamic tables. Compiled at
// registerType() from the projectile's drag model; an empty `dragRet` means
// "no drag". `liftCoeffVsSpin` is populated for spinning BallProfile / Magnus
// types (item 4); empty ⇒ no Magnus term.
struct DragLuts {
    // Drag retardation term  i*Cd(x)*A / (2*m)   in m^2/kg, indexed by the
    // abscissa `x` (Mach or Reynolds — see `abscissa`) over [x0, x1]. The hot
    // loop forms  |a_drag| = dragRet(x) * rho * v^2.
    Lut1D        dragRet;
    DragAbscissa abscissa = DragAbscissa::Mach;
    // Reynolds abscissa only: characteristic length d, so the sim forms the
    // per-frame abscissa scale d/ν from the environment's air viscosity.
    Real         reLength_m = 0;

    // Cl as a function of the spin parameter S = |ω| r / |v_rel|, indexed over
    // [0, Smax]. Signed: a soccer ball's low-S negative-Magnus regime and a
    // backspun golf ball's strong positive lift both live on this one curve.
    Lut1D liftCoeffVsSpin;
    Real  liftRadius_m    = 0;   // r in the spin parameter (sphere radius)
    Real  liftAreaOver2m  = 0;   // 0.5 * A / m, the Magnus magnitude factor

    Real  refArea_m2      = 0;   // pi/4 * refDiameter^2, cached
    BallShape shape       = BallShape::Sphere;
    // ProlateSpheroid only: the Cd endpoints the axis-angle model blends. The
    // compiled `dragRet` holds the spiral (low-drag) end; the sim scales it up
    // toward `cdTumble/cdSpiral` as the long axis leaves the airflow direction.
    Real  cdSpiral        = 0;
    Real  cdTumble        = 0;
};

} // namespace pon
