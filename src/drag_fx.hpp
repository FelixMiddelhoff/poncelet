// poncelet — fixed-point drag/lift LUT sampling for the BitExact core (Part B5).
// SPDX-License-Identifier: MIT
//
// The integrator's force model reads two compiled tables per step: the drag
// retardation term dragRet(x) and (spin types only) the Magnus lift Cl(S). Both
// are `pon::Lut1D` — uniformly-spaced samples with a linear interpolation. The
// double `Lut1D::sample()` stays the public/float path; `lut_sample_fx` is the
// bit-identical fixed-point equivalent used once `Accum = Fx32` (B6).
//
// The table itself (its `y[]`, x0, x1) is built once at registerType() in
// double — that one-time compile is out of scope here (see
// docs/ballistics-phase-plan.md §3.6); B5 only guarantees that, given the
// compiled table, the per-step sample folds to the same bits on every target.
// Each call converts just the two bracketing samples via Fx32::from_double
// (deterministic for a fixed input double), so no cached mirror is needed.
#pragma once

#include "poncelet/drag.hpp"
#include "fixed_point.hpp"

namespace pon::detail {

// Fixed-point port of Lut1D::sample(). Same clamp, same index, same lerp,
// every step on Fx32.
inline Fx32 lut_sample_fx(const Lut1D& lut, Fx32 x) {
    const std::size_t n = lut.y.size();
    if (n < 2) return Fx32::from_double(n == 0 ? Real(0) : lut.y.front());

    const Fx32 x0   = Fx32::from_double(lut.x0);
    const Fx32 x1   = Fx32::from_double(lut.x1);
    const Fx32 span = x1 - x0;                       // > 0 for every compiled table
    const Fx32 top  = Fx32::from_int(static_cast<std::int64_t>(n - 1));

    Fx32 fi = ((x - x0) / span) * top;               // fractional index
    const Fx32 zero = Fx32::from_int(0);
    if (fi < zero) fi = zero;
    if (fi > top)  fi = top;

    const std::int64_t i  = fi.to_int();
    const std::size_t  si = static_cast<std::size_t>(i);
    const std::size_t  sj = si + 1 < n ? si + 1 : si;
    const Fx32 f  = fi - Fx32::from_int(i);          // [0, 1)
    const Fx32 yi = Fx32::from_double(lut.y[si]);
    const Fx32 yj = Fx32::from_double(lut.y[sj]);
    return yi + (yj - yi) * f;
}

} // namespace pon::detail
