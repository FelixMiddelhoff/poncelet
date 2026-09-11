# poncelet — Unity native-plugin sample

`Poncelet.cs` is a P/Invoke layer over poncelet's **C ABI** (`<poncelet/poncelet.h>`)
plus a small managed `PonceletSim : IDisposable`. `PonceletDemo.cs` is a
MonoBehaviour that fires a catalog round every ~1.2 s and draws it.

> Sample, not a finished package. It covers exterior flight, the diagnostic
> trace and the shipped catalog. Custom `pon_projectile_desc` types, warheads,
> guidance and the blast / fragmentation / shaped-charge helpers are all in the C
> header — add `[DllImport]`s the same way to reach them.

## 1. Build the native library

```bash
cd <poncelet repo>
cmake -S . -B build -DPONCELET_SHARED=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Output: `poncelet_shared.dll` (Windows) / `libponcelet_shared.so` (Linux) /
`libponcelet_shared.dylib` (macOS).

## 2. Drop it into Unity

```
Assets/
  Plugins/
    poncelet_shared.dll          # or .so / .dylib, per platform
  Poncelet/
    Poncelet.cs
    PonceletDemo.cs
```

Set the plugin's platform/CPU in the Unity inspector to match the build.

## 3. Use it

Add `PonceletDemo` to an empty GameObject (add a `LineRenderer` component to see
the tracer). Or from your own code:

```csharp
using Poncelet;

var sim = new PonceletSim();               // Dispose it on teardown
sim.SetAtmosphere(0.0);
uint t  = sim.RegisterCatalog("762x51_175gr_smk");
uint id = sim.Fire(t, new Vector3(0, 1.7f, 0), Vector3.right);

void FixedUpdate() => sim.Step(Time.fixedDeltaTime);
Vector3 p = sim.GetPosition(id);

sim.SetTraceSink(ev => Debug.Log($"{(PonTraceKind)ev.kind} shot {ev.shot}"));
```

Frame convention: poncelet is SI metres, Y up — Unity's default — so `Vector3`s
pass straight through (`PonVec3` narrows `double`→`float`). poncelet owns no
geometry; raycast Unity physics against `GetPosition()` for hits.

The trace delegate is held alive by `PonceletSim` for as long as it is
installed; it is marshalled per call, so keep the callback cheap.
