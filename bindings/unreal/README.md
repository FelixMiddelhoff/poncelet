# poncelet — Unreal Engine plugin sample

`Poncelet/` is a UE plugin: `UPonceletSimComponent` (an `ActorComponent`)
wrapping poncelet's **C++ API** directly (`<poncelet/poncelet.hpp>`) — richer
types than the C ABI `bindings/unity` P/Invokes over, the same choice
`bindings/godot`'s GDExtension made.

> Sample, not a finished plugin: exterior flight, the shipped catalog,
> `preview_arc` and `describe()`. Terminal ballistics, warheads, guidance and
> the `World` geometry callback are all in `<poncelet/poncelet.hpp>` — extend
> `PonceletSimComponent` the same way to reach them. Built and compile-tested
> against a real **UE 5.8** install (see "What's actually verified" below).

## 1. Build poncelet and vendor it into the plugin

Unlike `bindings/unity`'s prebuilt-DLL step, this one copies poncelet's
headers + a static lib **into the plugin folder itself**
(`Poncelet/ThirdParty/poncelet/`) rather than referencing the poncelet repo
checkout by a relative path. That's not a style choice — `RunUAT BuildPlugin`
(and any real usage: copying this plugin into your own project's `Plugins/`)
physically moves the plugin directory, so anything the module needs must
travel *inside* it.

```powershell
pwsh bindings/unreal/setup_thirdparty.ps1
```

This builds poncelet **twice** (Debug and Release — MSVC's `std::` container
ABI is keyed by `_ITERATOR_DEBUG_LEVEL`, which differs between the two; the
plugin's headers are included directly into UE's own translation units, so
the prebuilt `.lib` must match whichever config UE is building) and copies
`include/poncelet/` + both `poncelet.lib`s into `ThirdParty/poncelet/`.
Re-run it after pulling a poncelet update. (No `pwsh`? Run the four `cmake`
lines the script prints, then copy `include/poncelet/` and
`build/{Debug,Release}/poncelet.lib` into
`ThirdParty/poncelet/{include,lib/Debug,lib/Release}/` by hand.)

## 2. Drop it into a project

Copy (or git-submodule) `bindings/unreal/Poncelet/` — **with its
`ThirdParty/poncelet/`** — into `<YourProject>/Plugins/Poncelet/`. Enable it
in your `.uproject` (or via Edit > Plugins in the editor).

## 3. Use it

```cpp
UPonceletSimComponent* Sim = ...; // add via the editor, or NewObject<>()

const int32 Type = Sim->RegisterCatalogRound(TEXT("762x51_175gr_smk"));
const int32 Shot = Sim->Fire(Type, MuzzleLocation, AimDirection); // UE-space: cm, Z-up

// bAutoStep (default true) steps it every tick. Or drive it yourself:
Sim->StepSim(DeltaSeconds);

FVector Position, Velocity; bool bAlive;
Sim->GetShotState(Shot, Position, Velocity, bAlive);

TArray<FVector> Arc = Sim->PreviewArc(Type, MuzzleLocation, AimDirection);
// draw Arc as a line while aiming, before firing

Sim->SetTraceEnabled(true);
Sim->OnTrace.AddDynamic(this, &AMyActor::OnPonceletTrace); // Blueprint-assignable too
```

Every method is `BlueprintCallable` — this all works from Blueprint with no
C++ of your own. For the rest of the API (terminal ballistics, warheads,
guidance, 6-DOF, ...), `Sim->GetSim()` returns the raw `pon::Sim*`.

## Coordinate / unit conversion — the real gotcha

Unreal is **centimetres, Z-up**; poncelet is **SI metres, Y-up, right-handed**
(the frame Godot's and Unity's defaults pass straight through — see
`../README.md` — because both are *also* Y-up; UE is the odd one out).
`PonceletConversion.h`'s `ToPoncelet`/`FromPoncelet` do the cm↔m and axis
mapping so callers never touch `pon::Vec3`.

The mapping is a coordinate axis **swap** (UE Z ↔ poncelet Y), which changes
chirality. Position/velocity round-trip exactly (verified —
`Poncelet.Conversion.RoundTrip`, below) since those are ordinary vectors, but
a chirality-*dependent* effect computed inside poncelet's own frame — Magnus
lift direction (curveball curve), gyroscopic spin-drift sign — may come out
mirrored relative to what looks intuitive from UE's side. **That cannot be
caught by an automated test** — it takes a human eye watching a spinning shot
curve one way or the other in the editor. If it looks backwards, negate the
converted spin axis's Y (poncelet-space) component before spawning. See the
full explanation in `PonceletConversion.h`.

## What's actually verified

`RunUAT BuildPlugin` against a real **UE 5.8** install: builds clean across
the Editor, Game (Development), and Game (Shipping) targets, linked against
poncelet's vendored static lib. `PonceletAutomationTest.cpp`
(`Poncelet.Conversion.RoundTrip`, `Poncelet.Component.FiresAndFalls`) then
run to completion under `UnrealEditor-Cmd.exe -ExecCmds="Automation RunTests
Poncelet;Quit" -unattended -nullrhi` — **both pass**, confirming the
coordinate round-trip is exact and a full register→fire→step→read cycle
against the real linked library behaves correctly (shot moves downrange,
drops under gravity, `DescribeShot`/`GetLastError` behave as documented).

Getting to a working Editor build required installing the **.NET Framework
4.6+ Targeting Pack (NetFxSDK)** — `UnrealEd`'s dependency chain
(`SwarmInterface`) needs it and it wasn't present alongside this fresh UE 5.8
install; added via `vs_installer.exe modify --add
Microsoft.Net.Component.4.8.SDK` (an existing Visual Studio Build Tools
install already has the right servicing channel). If your own machine hits
the same `SwarmInterface`/`NetFxSDK` error building anything UE-Editor-target
with this engine version, that's the fix.

This caught two real UE-5.8-specific bugs while writing the plugin (both
fixed in the code as committed): `EAutomationTestFlags::ApplicationContextMask`
was renamed to a free `EAutomationTestFlags_ApplicationContextMask` constant,
and a `TUniquePtr` of a forward-declared poncelet type needs UE's documented
`FVTableHelper&` constructor pattern (see `UPrimitiveComponent`), not just an
out-of-line destructor.

**Still not verified by any of this**: the one thing no headless test can
confirm — fire a spinning/curving shot in a real level with rendering and
eyeball the curve direction per the coordinate-conversion gotcha above.

## Frame / determinism notes (same as the other bindings)

poncelet owns no geometry — `UPonceletSimComponent` steps against
`EmptyWorld` (free-flight only); swept-query your own collision for real hits
the same way `bindings/unity`/`bindings/godot` do. `PlatformStable`
determinism (the default) is unaffected by this plugin; `BitExact` works the
same way it does linked into any other C++ consumer.
