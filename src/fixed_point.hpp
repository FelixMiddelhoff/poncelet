// poncelet — Q32.32 fixed-point scalar for the BitExact deterministic core.
// SPDX-License-Identifier: MIT
//
// `Fx32` is a signed 32.32 fixed-point number held in an `int64_t`:
//   * range     ±2^31           ≈ ±2.147e9
//   * resolution 2^-32          ≈ 2.328e-10
// That comfortably covers every quantity the exterior-ballistics integrator
// carries — positions (≤ 1e6 m), velocities (≤ ~2 km/s), accelerations, and the
// speed² · area · ρ products in the force model all sit well under 1e9.
//
// This file is the numeric primitive only. Part B2 delivers the type, its
// arithmetic, and unit tests; B3 adds a bitwise-exact sqrt; B4 the transcendental
// LUTs; B6 wires it in as `detail::Accum`. Nothing here reads physical data.
//
// Determinism contract: every operation is integer-only and fully specified.
//   * add / sub / negate go through `uint64_t` so signed overflow is never UB
//     (two's-complement wrap is defined and matches every target).
//   * multiply forms the full 128-bit product and arithmetic-shifts it right 32
//     (`__int128` on GCC/Clang, `_mul128` on MSVC).
//   * divide forms the 128-bit dividend `a << 32` and does a 128/64 divide
//     (`__int128` on GCC/Clang, `_div128` on MSVC).
//   * the double conversions use `llround` (round half away from zero) and are
//     meant for the Vec3/Real boundary only, never the hot loop.
#pragma once

#include <cmath>
#include <cstdint>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace pon::detail {

struct Fx32 {
    std::int64_t raw = 0;

    static constexpr int          kFracBits = 32;
    static constexpr std::int64_t kOne      = std::int64_t(1) << kFracBits;

    constexpr Fx32() = default;
    constexpr explicit Fx32(std::int64_t raw_, int /*tag*/) : raw(raw_) {}

    // Scalar-like constructors so Fx32 can stand in for `Real` as the
    // integrator's `Accum` (Part B6): integer literals are exact and constexpr;
    // the double ctor (compile-time-constant literals in practice) rounds via
    // llround, matching from_double. Not constexpr — llround is not until C++23.
    constexpr Fx32(int v)  : raw(std::int64_t(v) * kOne) {} // * not <<: defined for v < 0
    Fx32(double v)         : raw(std::llround(v * 4294967296.0 /* 2^32 */)) {}

    // --- constructors from the usual scalar types -------------------------
    static constexpr Fx32 from_raw(std::int64_t r) { return Fx32(r, 0); }
    static constexpr Fx32 from_int(std::int64_t v) { return Fx32(v * kOne, 0); } // defined for v < 0
    static Fx32           from_double(double v) {
        return Fx32(std::llround(v * 4294967296.0 /* 2^32 */), 0);
    }

    constexpr double   to_double() const { return double(raw) / 4294967296.0; }
    constexpr std::int64_t to_int() const { return raw >> kFracBits; } // floor

    // --- unary ------------------------------------------------------------
    friend constexpr Fx32 operator-(Fx32 a) {
        return Fx32(std::int64_t(0ULL - std::uint64_t(a.raw)), 0);
    }

    // --- add / sub (defined two's-complement wrap) -----------------------
    friend constexpr Fx32 operator+(Fx32 a, Fx32 b) {
        return Fx32(std::int64_t(std::uint64_t(a.raw) + std::uint64_t(b.raw)), 0);
    }
    friend constexpr Fx32 operator-(Fx32 a, Fx32 b) {
        return Fx32(std::int64_t(std::uint64_t(a.raw) - std::uint64_t(b.raw)), 0);
    }

    // --- multiply: (a.raw * b.raw) >> 32, full 128-bit intermediate ------
    friend Fx32 operator*(Fx32 a, Fx32 b) {
#if defined(_MSC_VER) && defined(_M_X64)
        std::int64_t hi;
        std::uint64_t lo = static_cast<std::uint64_t>(_mul128(a.raw, b.raw, &hi));
        // arithmetic (a.raw*b.raw) >> 32 across the 128-bit product
        std::int64_t r = (hi << 32) | static_cast<std::int64_t>(lo >> 32);
        return Fx32(r, 0);
#else
        __int128 p = static_cast<__int128>(a.raw) * static_cast<__int128>(b.raw);
        return Fx32(static_cast<std::int64_t>(p >> 32), 0);
#endif
    }

    // --- divide: (a.raw << 32) / b.raw, 128-bit dividend -----------------
    friend Fx32 operator/(Fx32 a, Fx32 b) {
#if defined(_MSC_VER) && defined(_M_X64)
        std::int64_t  hi = a.raw >> 32;                       // arithmetic
        std::uint64_t lo = static_cast<std::uint64_t>(a.raw) << 32;
        std::int64_t  rem;
        std::int64_t  q = _div128(hi, static_cast<std::int64_t>(lo), b.raw, &rem);
        return Fx32(q, 0);
#else
        __int128 num = static_cast<__int128>(a.raw) << 32;
        return Fx32(static_cast<std::int64_t>(num / b.raw), 0);
#endif
    }

    // --- compound -------------------------------------------------------
    Fx32& operator+=(Fx32 b) { *this = *this + b; return *this; }
    Fx32& operator-=(Fx32 b) { *this = *this - b; return *this; }
    Fx32& operator*=(Fx32 b) { *this = *this * b; return *this; }
    Fx32& operator/=(Fx32 b) { *this = *this / b; return *this; }

    // --- comparison ---------------------------------------------------
    friend constexpr bool operator==(Fx32 a, Fx32 b) { return a.raw == b.raw; }
    friend constexpr bool operator!=(Fx32 a, Fx32 b) { return a.raw != b.raw; }
    friend constexpr bool operator<(Fx32 a, Fx32 b)  { return a.raw <  b.raw; }
    friend constexpr bool operator<=(Fx32 a, Fx32 b) { return a.raw <= b.raw; }
    friend constexpr bool operator>(Fx32 a, Fx32 b)  { return a.raw >  b.raw; }
    friend constexpr bool operator>=(Fx32 a, Fx32 b) { return a.raw >= b.raw; }
};

constexpr Fx32 fx32_abs(Fx32 a) { return a.raw < 0 ? -a : a; }

// ---------------------------------------------------------------------------
// Deterministic fixed-point square root (Part B3).
//
// sqrt of a Q32.32 value v = raw/2^32 is  sqrt(raw)/2^16 = sqrt(raw << 32),
// so the result's raw is the integer floor-sqrt of the 96-bit value
// `raw << 32`. Computed by the classic bit-by-bit ("abacus") integer sqrt:
// shifts, adds and compares only — no multiply, no divide, no std::sqrt, no
// floating point. Same bits on every platform. Result is the largest Fx32
// whose square does not exceed v (exact floor).
// ---------------------------------------------------------------------------
namespace fx_u128 {
struct U128 { std::uint64_t hi, lo; };
constexpr bool     is_zero(U128 v)       { return (v.hi | v.lo) == 0; }
constexpr bool     ge(U128 a, U128 b)    { return a.hi != b.hi ? a.hi > b.hi : a.lo >= b.lo; }
constexpr bool     gt(U128 a, U128 b)    { return a.hi != b.hi ? a.hi > b.hi : a.lo >  b.lo; }
constexpr U128     shr1(U128 v)          { return {v.hi >> 1, (v.lo >> 1) | (v.hi << 63)}; }
constexpr U128     shr2(U128 v)          { return {v.hi >> 2, (v.lo >> 2) | (v.hi << 62)}; }
constexpr U128     add(U128 a, U128 b) {
    const std::uint64_t lo = a.lo + b.lo;
    return {a.hi + b.hi + (lo < a.lo ? 1u : 0u), lo};
}
constexpr U128     sub(U128 a, U128 b) {
    const std::uint64_t lo = a.lo - b.lo;
    return {a.hi - b.hi - (a.lo < b.lo ? 1u : 0u), lo};
}
} // namespace fx_u128

constexpr Fx32 fx32_sqrt(Fx32 v) {
    using namespace fx_u128;
    if (v.raw <= 0) return Fx32::from_raw(0);

    U128 n{static_cast<std::uint64_t>(v.raw) >> 32,
           static_cast<std::uint64_t>(v.raw) << 32};   // raw << 32, ≤ 2^95
    U128 res{0, 0};
    U128 bit{std::uint64_t(1) << 62, 0};               // 2^126 (largest even power)
    while (gt(bit, n)) bit = shr2(bit);

    while (!is_zero(bit)) {
        const U128 rpb = add(res, bit);
        if (ge(n, rpb)) {
            n   = sub(n, rpb);
            res = add(shr1(res), bit);
        } else {
            res = shr1(res);
        }
        bit = shr2(bit);
    }
    return Fx32::from_raw(static_cast<std::int64_t>(res.lo));  // res < 2^48
}

} // namespace pon::detail
