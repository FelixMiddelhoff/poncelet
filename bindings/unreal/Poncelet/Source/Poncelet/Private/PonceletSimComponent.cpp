// poncelet — Unreal Engine sample binding implementation.
// SPDX-License-Identifier: MIT
#include "PonceletSimComponent.h"
#include "PonceletConversion.h"

#include "poncelet/poncelet.hpp"

using namespace PonceletUE;

UPonceletSimComponent::UPonceletSimComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

// Out-of-line so TUniquePtr's deleter instantiates here, where pon::Sim /
// pon::World / pon::EventSink are complete (see the declaration's comment).
UPonceletSimComponent::UPonceletSimComponent(FVTableHelper& Helper) : Super(Helper) {}
UPonceletSimComponent::~UPonceletSimComponent() = default;

void UPonceletSimComponent::EnsureSim() const
{
	if (Sim) return;
	Sim = MakeUnique<pon::Sim>();
	WorldImpl = MakeUnique<pon::EmptyWorld>();
	SinkImpl = MakeUnique<pon::VectorEventSink>();
}

void UPonceletSimComponent::BeginPlay()
{
	Super::BeginPlay();
	EnsureSim();
}

void UPonceletSimComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Sim.Reset();
	WorldImpl.Reset();
	SinkImpl.Reset();
	Super::EndPlay(EndPlayReason);
}

void UPonceletSimComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bAutoStep)
	{
		StepSim(DeltaTime);
	}
}

int32 UPonceletSimComponent::RegisterCatalogRound(const FString& CatalogId)
{
	EnsureSim();
	const std::string Id(TCHAR_TO_UTF8(*CatalogId));
	if (!pon::catalog::has(Id)) return -1;
	const pon::ProjectileType Type = pon::catalog::get(Id);
	const pon::TypeId Result = Sim->registerType(Type);
	return Result == pon::kInvalidType ? -1 : static_cast<int32>(Result);
}

int32 UPonceletSimComponent::Fire(int32 TypeId, FVector MuzzleWorld, FVector AimDirWorld, float SpeedMps)
{
	EnsureSim();
	if (TypeId < 0) return -1;

	pon::LaunchParams Launch;
	Launch.position = ToPoncelet(MuzzleWorld);
	Launch.direction = ToPonceletDir(AimDirWorld);
	if (SpeedMps > 0.f) Launch.speed = static_cast<pon::Real>(SpeedMps);

	const pon::StateId Result = Sim->spawn(static_cast<pon::TypeId>(TypeId), Launch);
	return Result == pon::kInvalidState ? -1 : static_cast<int32>(Result);
}

void UPonceletSimComponent::StepSim(float DeltaSeconds)
{
	EnsureSim();
	if (DeltaSeconds <= 0.f) return;
	Sim->step(static_cast<pon::Seconds>(DeltaSeconds), *WorldImpl, *SinkImpl);
	// v1 scope: shots are stepped and queryable via GetShotState; per-frame
	// Event (impact/expiry/...) consumption is left to a caller who wants it
	// via GetSim() -> a real World + reading the VectorEventSink directly, the
	// same "extend it" story as the terminal-ballistics / warhead API surface.
	static_cast<pon::VectorEventSink*>(SinkImpl.Get())->events.clear();
}

bool UPonceletSimComponent::GetShotState(int32 StateId, FVector& OutPosition, FVector& OutVelocity,
                                         bool& OutAlive) const
{
	EnsureSim();
	if (StateId < 0) { OutPosition = OutVelocity = FVector::ZeroVector; OutAlive = false; return false; }
	const pon::ProjectileState& State = Sim->state(static_cast<pon::StateId>(StateId));
	OutPosition = FromPoncelet(State.position);
	OutVelocity = FromPonceletDir(State.velocity);
	OutAlive = State.alive;
	return true;
}

TArray<FVector> UPonceletSimComponent::PreviewArc(int32 TypeId, FVector MuzzleWorld, FVector AimDirWorld,
                                                   float DtSeconds, float MaxTimeSeconds,
                                                   float GroundZWorld) const
{
	TArray<FVector> Out;
	EnsureSim();
	if (TypeId < 0 || DtSeconds <= 0.f || MaxTimeSeconds <= 0.f) return Out;

	const pon::ProjectileType& Type = Sim->type(static_cast<pon::TypeId>(TypeId));
	pon::LaunchParams Launch;
	Launch.position = ToPoncelet(MuzzleWorld);
	Launch.direction = ToPonceletDir(AimDirWorld);

	std::vector<pon::Vec3> Arc;
	pon::preview_arc(Type, Launch, Sim->environment(),
	                 static_cast<pon::Seconds>(DtSeconds), static_cast<pon::Seconds>(MaxTimeSeconds),
	                 Arc, static_cast<pon::Real>(GroundZWorld) / kCmPerM);

	Out.Reserve(static_cast<int32>(Arc.size()));
	for (const pon::Vec3& P : Arc) Out.Add(FromPoncelet(P));
	return Out;
}

FString UPonceletSimComponent::DescribeShot(int32 StateId) const
{
	EnsureSim();
	if (StateId < 0) return FString();
	const pon::ProjectileState& State = Sim->state(static_cast<pon::StateId>(StateId));
	const pon::ProjectileType& Type = Sim->type(State.typeId);
	return FString(UTF8_TO_TCHAR(pon::describe(State, Type, Sim->environment()).c_str()));
}

FString UPonceletSimComponent::GetLastError() const
{
	EnsureSim();
	return FString(UTF8_TO_TCHAR(Sim->lastError()));
}

void UPonceletSimComponent::SetTraceEnabled(bool bEnabled)
{
	EnsureSim();
	if (!bEnabled) { Sim->setTraceSink({}); return; }

	// Captured by value (a UObject pointer) — the trace callback runs
	// synchronously inside StepSim/step(), never after this component could
	// have been destroyed mid-call, so this is safe without a weak pointer.
	UPonceletSimComponent* Self = this;
	Sim->setTraceSink([Self](const pon::TraceEvent& Ev)
	{
		FPonceletTraceEvent Out;
		Out.Kind = static_cast<int32>(Ev.kind);
		Out.ShotId = static_cast<int32>(Ev.shot);
		Out.TimeAliveSeconds = static_cast<float>(Ev.time_s);
		Out.Position = FromPoncelet(Ev.position);
		Out.Velocity = FromPonceletDir(Ev.velocity);
		Self->OnTrace.Broadcast(Out);
	});
}
