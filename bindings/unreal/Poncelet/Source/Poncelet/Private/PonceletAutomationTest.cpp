// poncelet — Unreal automation tests for the sample binding.
// SPDX-License-Identifier: MIT
//
// Headless (no editor GUI, no rendering) — runs under
// `UnrealEditor-Cmd.exe <project> -ExecCmds="Automation RunTests Poncelet;Quit" -unattended -nullrhi`.
// Covers what's mechanically verifiable without a human eye: the coordinate
// conversion round-trips exactly, and firing through the component actually
// drives the real poncelet static lib end to end (registers a catalog round,
// steps it, reads back a physically sane trajectory). It does NOT — cannot —
// verify that a chirality-dependent effect (Magnus curve direction, spin-drift
// sign) reads as "correct" to a human watching it in the editor; see
// PonceletConversion.h's header comment.
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "PonceletSimComponent.h"
#include "PonceletConversion.h"

using namespace PonceletUE;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPonceletConversionRoundTripTest, "Poncelet.Conversion.RoundTrip",
                                 EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPonceletConversionRoundTripTest::RunTest(const FString& Parameters)
{
	const FVector Samples[] = {
		FVector(0, 0, 0), FVector(100, 0, 0), FVector(0, 250, 0), FVector(0, 0, -170),
		FVector(1234.5, -678.9, 42.0), FVector(-500, 500, 1000),
	};
	for (const FVector& V : Samples)
	{
		const FVector RoundTripped = FromPoncelet(ToPoncelet(V));
		TestTrue(FString::Printf(TEXT("UE->poncelet->UE round-trips %s"), *V.ToString()),
		         V.Equals(RoundTripped, 1.0e-6));
	}

	// The mapping keeps "up is up": UE +Z should be poncelet +Y, and nothing else.
	const pon::Vec3 Up = ToPoncelet(FVector(0, 0, 100)); // 1 m up in UE
	TestTrue("UE +Z maps to poncelet +Y", Up.y > 0.99 && FMath::Abs(Up.x) < 1e-9 && FMath::Abs(Up.z) < 1e-9);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPonceletComponentFiresAndFallsTest, "Poncelet.Component.FiresAndFalls",
                                 EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPonceletComponentFiresAndFallsTest::RunTest(const FString& Parameters)
{
	UPonceletSimComponent* Comp = NewObject<UPonceletSimComponent>();
	if (!TestNotNull("component constructed", Comp)) return false;
	Comp->AddToRoot(); // keep it alive for this function's duration; no Actor/World needed

	const int32 TypeId = Comp->RegisterCatalogRound(TEXT("9x19_124gr_fmj"));
	TestTrue("known catalog round registers", TypeId >= 0);

	const int32 Unknown = Comp->RegisterCatalogRound(TEXT("not_a_real_round_id"));
	TestEqual("unknown catalog id -> -1", Unknown, -1);

	const FVector Muzzle(0, 0, 170); // 1.7 m up, UE cm/Z-up
	const FVector AimDir(1, 0, 0);   // level, +X (UE "forward")
	const int32 ShotId = Comp->Fire(TypeId, Muzzle, AimDir);
	TestTrue("fire returns a valid state id", ShotId >= 0);

	// Step ~1 second at 120 Hz.
	for (int32 i = 0; i < 120; ++i) Comp->StepSim(1.0f / 120.0f);

	FVector Pos, Vel;
	bool bAlive = false;
	const bool bGot = Comp->GetShotState(ShotId, Pos, Vel, bAlive);
	TestTrue("GetShotState succeeds for a live id", bGot);
	TestTrue("shot moved downrange (+X)", Pos.X > 1000.0); // > 10 m in ~1 s at ~360 m/s muzzle speed
	TestTrue("shot dropped below the muzzle (gravity pulls -Z)", Pos.Z < Muzzle.Z);
	TestTrue("shot is still moving in +X", Vel.X > 0.0);

	const FString Desc = Comp->DescribeShot(ShotId);
	TestTrue("DescribeShot returns a non-empty summary", !Desc.IsEmpty());
	TestTrue("GetLastError is empty after a successful run", Comp->GetLastError().IsEmpty());

	// A preview arc from the same muzzle/aim should start at the muzzle and
	// travel the same direction the live shot did.
	const TArray<FVector> Arc = Comp->PreviewArc(TypeId, Muzzle, AimDir, 0.02f, 2.0f, -1.0e6f);
	TestTrue("preview arc has multiple points", Arc.Num() > 5);
	if (Arc.Num() > 0)
	{
		TestTrue("preview arc starts at the muzzle", Arc[0].Equals(Muzzle, 1.0));
		TestTrue("preview arc travels downrange", Arc.Last().X > Arc[0].X);
	}

	Comp->RemoveFromRoot();
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
