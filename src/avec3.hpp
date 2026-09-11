// poncelet — accumulator-templated 3-vector for the integrator core.
// SPDX-License-Identifier: MIT
//
// `AVec3<T>` is the integrator's internal vector. The float/double core runs it
// as `AVec3<Real>` — field-identical to `pon::Vec3`, with every operator written
// in the SAME component order, so `AVec3<Real>` is bit-for-bit the old path. The
// BitExact core (Part B) instantiates it as `AVec3<Fx32>` for the Q32.32
// fixed-point path — no integrator body or call site changes.
//
// Kept deliberately tiny: only the ops the integrators in integrate.cpp use.
#pragma once

#include "poncelet/types.hpp"

namespace pon::detail {

template <class T>
struct AVec3 {
    T x{}, y{}, z{};

    constexpr AVec3() = default;
    constexpr AVec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    friend constexpr AVec3 operator+(AVec3 a, AVec3 b) {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }
    friend constexpr AVec3 operator-(AVec3 a, AVec3 b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    friend constexpr AVec3 operator-(AVec3 a) { return {-a.x, -a.y, -a.z}; }
    friend constexpr AVec3 operator*(AVec3 a, T s) { return {a.x * s, a.y * s, a.z * s}; }
    friend constexpr AVec3 operator*(T s, AVec3 a) { return {a.x * s, a.y * s, a.z * s}; }
    friend constexpr AVec3 operator/(AVec3 a, T s) { return {a.x / s, a.y / s, a.z / s}; }

    AVec3& operator+=(AVec3 b) { x = x + b.x; y = y + b.y; z = z + b.z; return *this; }
    AVec3& operator-=(AVec3 b) { x = x - b.x; y = y - b.y; z = z - b.z; return *this; }
    AVec3& operator*=(T s)     { x = x * s;   y = y * s;   z = z * s;   return *this; }
};

template <class T>
constexpr T dot(AVec3<T> a, AVec3<T> b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <class T>
constexpr AVec3<T> cross(AVec3<T> a, AVec3<T> b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

} // namespace pon::detail
