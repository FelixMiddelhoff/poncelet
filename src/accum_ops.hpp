// poncelet — accumulator-generic math for the integrator core (Part B6).
// SPDX-License-Identifier: MIT
//
// The integrator body (src/integrate.cpp) is written on a scalar accumulator
// type `Acc`. For the float/double core `Acc = Real` and these forward to the
// platform `<cmath>`; for the BitExact core `Acc = Fx32` and they forward to
// the deterministic fixed-point sqrt (B3) and transcendental LUTs (B4). Same
// call site, one overload set — that is what lets the integrator template on
// `Acc` without a single `std::` in a hot path.
#pragma once

#include "poncelet/types.hpp"
#include "fixed_point.hpp"
#include "fixed_lut.hpp"
#include "avec3.hpp"

#include <cmath>

namespace pon::detail {

// --- Real (double) accumulator ------------------------------------------
inline Real acc_sqrt(Real v)       { return std::sqrt(v); }
inline Real acc_sin(Real a)        { return std::sin(a); }
inline Real acc_acos(Real c)       { return std::acos(c); }
inline Real acc_exp(Real arg)      { return std::exp(arg); }          // arg ≤ 0
inline Real acc_pow_neg017(Real t) { return std::pow(t, Real(-0.17)); }
inline Real acc_pow_ratio02(Real t) { return std::pow(t, Real(0.2)); }

inline Real acc_abs(Real v)        { return std::fabs(v); }
inline Real to_real(Real v)        { return v; }

// --- Fx32 accumulator --------------------------------------------------
inline Fx32 acc_sqrt(Fx32 v)       { return fx32_sqrt(v); }
inline Fx32 acc_sin(Fx32 a)        { return fx_sin(a); }
inline Fx32 acc_acos(Fx32 c)       { return fx_acos(c); }
inline Fx32 acc_exp(Fx32 arg)      { return fx_exp(arg); }
inline Fx32 acc_pow_neg017(Fx32 t) { return fx_pow_neg017(t); }
inline Fx32 acc_pow_ratio02(Fx32 t) { return fx_pow_ratio02(t); }
inline Fx32 acc_abs(Fx32 v)        { return fx32_abs(v); }
inline Real to_real(Fx32 v)        { return v.to_double(); }

// --- AVec3<Acc> length, shared by every Core<Acc>/GuidanceCore<Acc> ------
// (mirrors Vec3::length()/length_sq() in types.hpp; AVec3 itself only carries
// the ops the templated cores actually use, so this lives here rather than
// in avec3.hpp, alongside the acc_sqrt each instantiation forwards to.)
template <class T>
inline T acc_length(AVec3<T> v) { return acc_sqrt(dot(v, v)); }

} // namespace pon::detail
