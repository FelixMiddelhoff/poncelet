// poncelet — Unreal Engine sample binding: an ActorComponent over the C++ API.
// SPDX-License-Identifier: MIT
//
// Sample, not a finished plugin: exterior flight, the catalog, preview_arc and
// describe(), same scope as bindings/unity and bindings/godot. Terminal
// ballistics, warheads, guidance and the World geometry callback are all in
// <poncelet/poncelet.hpp> — extend this component the same way to reach them.
// See PonceletConversion.h for the coordinate/unit mapping and its one real
// gotcha (chirality-dependent effects may need a sign flip — verify visually).
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PonceletSimComponent.generated.h"

namespace pon { class Sim; struct World; struct EventSink; }

USTRUCT(BlueprintType)
struct FPonceletTraceEvent
{
	GENERATED_BODY()

	// pon::TraceKind, as an int (Frame=0, Substep=1, TransonicEnter=2,
	// TransonicExit=3, MediumChanged=4, GuidanceLost=5) — see <poncelet/sim.hpp>.
	UPROPERTY(BlueprintReadOnly, Category = "Poncelet")
	int32 Kind = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Poncelet")
	int32 ShotId = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Poncelet")
	float TimeAliveSeconds = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Poncelet")
	FVector Position = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Poncelet")
	FVector Velocity = FVector::ZeroVector;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPonceletTrace, const FPonceletTraceEvent&, Event);

// Owns one pon::Sim. Add to any Actor; call RegisterCatalogRound/RegisterRound
// once, Fire per shot, Step every tick (or drive it yourself and skip
// bAutoStep). Reads/writes UE's FVector (cm, Z-up) — this component does the
// conversion, callers never touch pon::Vec3.
UCLASS(ClassGroup = (Poncelet), meta = (BlueprintSpawnableComponent))
class PONCELET_API UPonceletSimComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPonceletSimComponent();

	// Sim/WorldImpl/SinkImpl are TUniquePtr<incomplete type> (pon::Sim/World/
	// EventSink are only forward-declared above, to keep
	// <poncelet/poncelet.hpp> out of a widely-included Public header) — UE's
	// own documented pattern for that (see e.g. UPrimitiveComponent) is a
	// destructor AND an `FVTableHelper&` constructor, both declared here and
	// defined out-of-line in the .cpp where the poncelet types are complete.
	// UHT's generated vtable-helper glue (DEFINE_VTABLE_PTR_HELPER_CTOR)
	// needs both; without them it instantiates TUniquePtr's deleter inline in
	// its own generated .cpp, which only sees the forward declarations (MSVC
	// C4150 "deleting pointer to incomplete type").
	UPonceletSimComponent(FVTableHelper& Helper);
	virtual ~UPonceletSimComponent() override;

	// If true (default), TickComponent steps the Sim by DeltaTime every tick.
	// Set false to call StepSim(dt) yourself (e.g. on a fixed-rate timer for
	// deterministic replay).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Poncelet")
	bool bAutoStep = true;

	// Register a named round from poncelet's shipped catalog (data/projectiles.csv
	// — "9x19_124gr_fmj", "762x51_175gr_smk", ...; see docs/guide.md
	// "The named catalog"). Returns a TypeId, or -1 if the id is unknown.
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	int32 RegisterCatalogRound(const FString& CatalogId);

	// Fire a shot. `MuzzleWorld`/`AimDirWorld` are UE-space (cm, Z-up) —
	// converted internally. `SpeedMps <= 0` uses the type's muzzle speed.
	// Returns a StateId, or -1 on failure (check GetLastError).
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	int32 Fire(int32 TypeId, FVector MuzzleWorld, FVector AimDirWorld, float SpeedMps = 0.f);

	// Advance every live shot by DeltaSeconds. Called automatically from
	// TickComponent when bAutoStep is set.
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	void StepSim(float DeltaSeconds);

	// Fills OutPosition/OutVelocity (UE-space) and OutAlive for a live shot.
	// Returns false for a StateId that never existed (an id valid-but-dead
	// still fills the fields, with OutAlive = false — same "state() is safe
	// on a stale id" contract pon::Sim::state() has).
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	bool GetShotState(int32 StateId, FVector& OutPosition, FVector& OutVelocity, bool& OutAlive) const;

	// Predicted flight path (no Sim state changed) — draw it as a line while
	// aiming. `GroundZWorld` is UE-space Z (cm); pass a very negative number
	// for "no ground clip."
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	TArray<FVector> PreviewArc(int32 TypeId, FVector MuzzleWorld, FVector AimDirWorld,
	                            float DtSeconds = 0.05f, float MaxTimeSeconds = 6.f,
	                            float GroundZWorld = -1.0e6f) const;

	// One-line human-readable summary of a live shot (pon::describe) — for an
	// on-screen debug readout or a log line.
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	FString DescribeShot(int32 StateId) const;

	// Reason the last RegisterCatalogRound/Fire failed, or "" — pon::Sim::lastError().
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	FString GetLastError() const;

	// Fired for each pon::TraceEvent once a trace is armed (SetTraceEnabled).
	UPROPERTY(BlueprintAssignable, Category = "Poncelet")
	FOnPonceletTrace OnTrace;

	// Arm/disarm the diagnostic trace (SimConfig::traceSink) at runtime — off
	// by default, zero-cost when off. See docs/guide.md "Diagnostic trace".
	UFUNCTION(BlueprintCallable, Category = "Poncelet")
	void SetTraceEnabled(bool bEnabled);

	// Direct access for C++ callers who want the full API (terminal ballistics,
	// warheads, guidance, 6-DOF, ...) beyond what this component exposes to
	// Blueprint. Lazily constructs the Sim on first call (same as every other
	// method here), so this is never null.
	pon::Sim* GetSim() const { EnsureSim(); return Sim.Get(); }

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                            FActorComponentTickFunction* ThisTickFunction) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// Constructs Sim/WorldImpl/SinkImpl on first call; a no-op after. Lazy
	// rather than strictly BeginPlay-only so the component also works when
	// driven directly in an editor-utility / automation-test context that
	// never runs a full Actor BeginPlay.
	void EnsureSim() const;

	// mutable: several BlueprintCallable query methods (GetShotState,
	// PreviewArc, DescribeShot, GetLastError) are logically const — they
	// don't change simulation state — but still need to lazily construct the
	// Sim on first call.
	mutable TUniquePtr<pon::Sim> Sim;
	mutable TUniquePtr<pon::World> WorldImpl;   // EmptyWorld — poncelet owns no
	                                            // geometry; swept-query your
	                                            // own World for real hits.
	mutable TUniquePtr<pon::EventSink> SinkImpl;
};
