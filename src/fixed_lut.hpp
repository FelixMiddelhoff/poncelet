// poncelet — fixed-point transcendental LUTs for the BitExact core (Part B4).
// SPDX-License-Identifier: MIT
//
// The integrator core (src/integrate.cpp) uses four transcendentals:
//   sin, acos   — 6-DOF angular flight
//   exp         — the analytic-drag closed-form fast path
//   pow(t,-0.17)— the Litz gyroscopic spin-drift term
// A platform libm gives slightly different bits for each, which breaks
// cross-platform BitExact. Here they resolve to a uniform sample table
// (generated offline by tools/gen_fixed_lut.py into generated/fixed_lut_tables.inc
// as raw Q32.32 values) plus a fixed-point linear interpolation — integer-only,
// identical on every target.
//
// Accuracy: 1024–2048 intervals over each domain; linear-interp error is well
// under the millimetre-scale tolerances the validation suite checks. Not the
// hot path — sin/acos run once per 6-DOF stage, exp once per analytic sample.
#pragma once

#include "fixed_point.hpp"

namespace pon::detail {

// Uniform-table descriptor. `invStep` is 1/((x1-x0)/n) so the fractional index
// is one multiply. Emitted with matching field order by gen_fixed_lut.py.
struct FxLutDesc {
    std::int64_t x0_raw;
    std::int64_t x1_raw;
    std::int64_t invStep_raw;
    int          n;         // interval count; the data array has n+1 samples
};

// Linear interpolation between the two bracketing samples, everything on Fx32.
// `x` is clamped to [x0, x1]; `v` has `d.n + 1` entries.
inline Fx32 fxlut_sample(const FxLutDesc& d, const long long* v, Fx32 x) {
    const Fx32 x0 = Fx32::from_raw(d.x0_raw);
    const Fx32 x1 = Fx32::from_raw(d.x1_raw);
    if (x < x0) x = x0;
    if (x > x1) x = x1;

    const Fx32 f = (x - x0) * Fx32::from_raw(d.invStep_raw); // fractional index ≥ 0
    std::int64_t i = f.to_int();
    if (i < 0)          i = 0;
    if (i > d.n - 1)    i = d.n - 1;
    const Fx32 frac = f - Fx32::from_int(i);                 // [0, 1)

    const Fx32 a = Fx32::from_raw(v[i]);
    const Fx32 b = Fx32::from_raw(v[i + 1]);
    return a + (b - a) * frac;
}

} // namespace pon::detail

#include "generated/fixed_lut_tables.inc"

namespace pon::detail {

// sin(a), a ∈ [0, π] (the only range the 6-DOF total angle of attack takes).
inline Fx32 fx_sin(Fx32 a) { return fxlut_sample(kFxLut_sin, kFxLutData_sin, a); }

// acos(c), c ∈ [-1, 1] (sampler clamps out-of-range).
inline Fx32 fx_acos(Fx32 c) { return fxlut_sample(kFxLut_acos, kFxLutData_acos, c); }

// exp(arg) for arg ≤ 0 (the integrator only ever evaluates exp of a negative
// number: -c·t on the analytic path). arg ≥ 0 clamps to 1; arg below the table
// floor (≈ -32) clamps to ≈ e^-32, effectively 0.
inline Fx32 fx_exp(Fx32 arg) {
    if (arg >= Fx32::from_int(0)) return Fx32::from_int(1);
    return fxlut_sample(kFxLut_exp_neg, kFxLutData_exp_neg, -arg);
}

// t^(-0.17) for t > 0. Sampled on w = t^0.25 (two nested exact fixed-point
// sqrts, B3): value(w) = w^(-0.68) = t^(-0.17). The quarter-power abscissa
// spreads the t→0 curvature across the table so even t ≈ 1e-3 stays accurate.
// Caller guards t ≥ 1e-3.
inline Fx32 fx_pow_neg017(Fx32 t) {
    return fxlut_sample(kFxLut_pow_neg017, kFxLutData_pow_neg017,
                        fx32_sqrt(fx32_sqrt(t)));
}

// --- Full-range periodic sin/cos, built on the [0, π] table above ---------
//
// fx_sin's domain is [0, π] — enough for a total angle of attack, but the
// 6-DOF *roll phase* (∫p, ProjectileState::spinPhase_rad) is an unbounded,
// continuously accumulating angle: thousands of radians over a real flight.
// Found by a real cross-platform CI run: the roll phase itself integrates
// exactly (Fx32 the whole way, B6), but src/sim.cpp's Quat-from-roll
// conversion used plain std::cos/std::sin unconditionally — and different
// platforms' libm give slightly different last-bit results for the same
// input, which lands directly in ProjectileState::orientation (hashed
// state) and cascades into a different digest despite the physics being
// bit-identical. fx_sin_full/fx_cos_full close that gap.
//
// kFx32PiRaw / kFx32TwoPiRaw / kFx32PiHalfRaw are embedded raw Q32.32
// integers (like every other LUT constant here) — computed once offline at
// higher precision than a double can hold, not derived from a runtime
// double-to-Fx32 conversion, so they carry no platform-dependent rounding.
// kFx32TwoPiRaw is exactly 2×kFx32PiRaw (not independently rounded) so the
// sin(r) = -sin(r-π) reflection below has no seam at the [0,π]/[π,2π) wrap.
inline constexpr std::int64_t kFx32PiRaw      = 13493037705;
inline constexpr std::int64_t kFx32TwoPiRaw   = 2 * kFx32PiRaw;
inline constexpr std::int64_t kFx32PiHalfRaw  = 6746518852;
inline constexpr Fx32 kFx32Pi     = Fx32::from_raw(kFx32PiRaw);
inline constexpr Fx32 kFx32TwoPi  = Fx32::from_raw(kFx32TwoPiRaw);
inline constexpr Fx32 kFx32PiHalf = Fx32::from_raw(kFx32PiHalfRaw);

// Reduce an arbitrary (possibly negative, possibly huge) angle into [0, 2π)
// via exact floor-division on the raw Q32.32 integers — integer-only, no FP,
// identical on every target regardless of magnitude.
inline Fx32 fx_mod_2pi(Fx32 x) {
    std::int64_t r = x.raw % kFx32TwoPiRaw; // C++ %: truncated toward zero
    if (r < 0) r += kFx32TwoPiRaw;          // floor-mod adjustment
    return Fx32::from_raw(r);
}

// sin(x) for any x, via periodic reduction + the [0, π] table's reflection
// identity sin(r) = -sin(r - π) for r in [π, 2π).
inline Fx32 fx_sin_full(Fx32 x) {
    const Fx32 r = fx_mod_2pi(x);
    return r <= kFx32Pi ? fx_sin(r) : -fx_sin(r - kFx32Pi);
}

// cos(x) = sin(x + π/2), full range.
inline Fx32 fx_cos_full(Fx32 x) { return fx_sin_full(x + kFx32PiHalf); }

} // namespace pon::detail
