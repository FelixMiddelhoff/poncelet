// poncelet — UE <-> poncelet coordinate/unit conversion.
// SPDX-License-Identifier: MIT
//
// Unreal: centimetres, Z-up, X-forward/Y-right/Z-up (left-handed).
// poncelet: metres, Y-up, right-handed (docs/guide.md "SI units, everywhere";
// bindings/README.md — the frame Godot's and Unity's defaults pass straight
// through, since both are also Y-up).
//
// This mapping keeps "up is up" (poncelet Y <-> UE Z) and "poncelet's other
// horizontal axis is UE's right" (poncelet Z <-> UE Y), scaling cm <-> m. It
// is a coordinate PERMUTATION, which is handedness-changing (swapping two
// axes of a 3D frame flips chirality) — round-tripping a POSITION or
// VELOCITY through ToPoncelet()/FromPoncelet() is exact (see
// PonceletAutomationTest.cpp's round-trip + Sim-vs-component cross-check),
// but a chirality-DEPENDENT effect computed inside poncelet's own frame —
// Magnus lift direction (curveball curve), gyroscopic spin-drift sign — may
// come out mirrored relative to what feels intuitive from UE's side. That is
// NOT something a headless test can catch (it takes a human eye watching a
// spinning shot curve one way or the other in the editor); if a curveball
// bends the wrong way, negate the spin axis's Z (poncelet-space) component
// before spawning — see UPonceletSimComponent::Fire's `SpinAxisOverride`.
#pragma once

#include "CoreMinimal.h"
#include "poncelet/types.hpp"

namespace PonceletUE
{
	constexpr double kCmPerM = 100.0;

	FORCEINLINE pon::Vec3 ToPoncelet(const FVector& V)
	{
		return pon::Vec3(V.X / kCmPerM, V.Z / kCmPerM, V.Y / kCmPerM);
	}

	FORCEINLINE FVector FromPoncelet(const pon::Vec3& V)
	{
		return FVector(V.x * kCmPerM, V.z * kCmPerM, V.y * kCmPerM);
	}

	// Directions carry no translation component, but the axis permutation
	// still applies — kept distinct from the position helpers above so a call
	// site reads its intent (ToPonceletDir for aim/spin axes, ToPoncelet for
	// world positions), not because the math differs.
	FORCEINLINE pon::Vec3 ToPonceletDir(const FVector& V) { return ToPoncelet(V); }
	FORCEINLINE FVector FromPonceletDir(const pon::Vec3& V) { return FromPoncelet(V); }
}
