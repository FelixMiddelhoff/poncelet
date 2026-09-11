// poncelet — Unity native-plugin sample. P/Invoke over the stable C ABI.
// SPDX-License-Identifier: MIT
//
// Drop this + PonceletDemo.cs into a Unity project and put the native library
// (poncelet_shared.dll / libponcelet_shared.so / .dylib — built with
// -DPONCELET_SHARED=ON) in Assets/Plugins/. See README.md.
using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Poncelet
{
    [StructLayout(LayoutKind.Sequential)]
    public struct PonVec3
    {
        public double x, y, z;
        public PonVec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public static implicit operator PonVec3(Vector3 v) => new PonVec3(v.x, v.y, v.z);
        public Vector3 ToVector3() => new Vector3((float)x, (float)y, (float)z);
    }

    // Mirrors C `pon_state`. orientation is (w,x,y,z); zero/identity off the 6-DOF path.
    [StructLayout(LayoutKind.Sequential)]
    public struct PonState
    {
        public PonVec3 position;
        public PonVec3 velocity;
        public double timeAliveS;
        public double distanceTravelledM;
        public int mediumId;
        public int alive;
        public uint flags;
        public double qw, qx, qy, qz;
        public PonVec3 angVelRadps;
        public double angleOfAttackRad;
    }

    public enum PonTraceKind
    {
        Frame = 0, Substep = 1, TransonicEnter = 2, TransonicExit = 3,
        MediumChanged = 4, GuidanceLost = 5,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PonTraceEvent
    {
        public int kind;          // PonTraceKind
        public uint shot;
        public double timeS;
        public PonVec3 position;
        public PonVec3 velocity;
        public int i0;
        public double r0;
    }

    internal static class Native
    {
        // Windows: poncelet_shared.dll  •  Linux: libponcelet_shared.so  •  macOS: libponcelet_shared.dylib
        const string LIB = "poncelet_shared";
        const CallingConvention CC = CallingConvention.Cdecl;

        [DllImport(LIB, CallingConvention = CC)] public static extern IntPtr pon_version_string();
        [DllImport(LIB, CallingConvention = CC)] public static extern IntPtr pon_sim_create(int determinism);
        [DllImport(LIB, CallingConvention = CC)] public static extern void   pon_sim_destroy(IntPtr sim);
        [DllImport(LIB, CallingConvention = CC)] public static extern void   pon_sim_set_gravity(IntPtr sim, PonVec3 g);
        [DllImport(LIB, CallingConvention = CC)] public static extern void   pon_sim_set_atmosphere(IntPtr sim, double altitudeM, double temperatureK, double pressurePa, double relHumidity);
        [DllImport(LIB, CallingConvention = CC)] public static extern int    pon_catalog_has([MarshalAs(UnmanagedType.LPUTF8Str)] string id);
        [DllImport(LIB, CallingConvention = CC)] public static extern uint   pon_register_catalog_type(IntPtr sim, [MarshalAs(UnmanagedType.LPUTF8Str)] string id);
        [DllImport(LIB, CallingConvention = CC)] public static extern uint   pon_spawn(IntPtr sim, uint typeId, PonVec3 position, PonVec3 direction, double speedMps);
        [DllImport(LIB, CallingConvention = CC)] public static extern uint   pon_spawn_precise(IntPtr sim, uint typeId, PonVec3 position, PonVec3 direction, double speedMps, int fidelityTier, uint precisionFlags);
        [DllImport(LIB, CallingConvention = CC)] public static extern void   pon_step(IntPtr sim, double dtS);
        [DllImport(LIB, CallingConvention = CC)] public static extern int    pon_get_state(IntPtr sim, uint stateId, out PonState outState);
        [DllImport(LIB, CallingConvention = CC)] public static extern UIntPtr pon_live_count(IntPtr sim);
        [DllImport(LIB, CallingConvention = CC)] public static extern void   pon_despawn(IntPtr sim, uint stateId);

        [UnmanagedFunctionPointer(CC)]
        public delegate void TraceFn(ref PonTraceEvent ev, IntPtr user);
        [DllImport(LIB, CallingConvention = CC)] public static extern void pon_sim_set_trace_sink(IntPtr sim, TraceFn fn, IntPtr user);
    }

    /// <summary>Managed wrapper around one pon::Sim. Dispose it to free the native sim.</summary>
    public sealed class PonceletSim : IDisposable
    {
        public const uint Invalid = 0xFFFFFFFFu;
        public enum Tier { Hitscan = 0, AnalyticDrag = 1, Integrated = 2 }

        IntPtr _sim;
        Native.TraceFn _traceKeepAlive;   // must outlive the native callback

        public static string Version =>
            Marshal.PtrToStringAnsi(Native.pon_version_string());

        public PonceletSim(int determinism = 1 /* PlatformStable */)
        {
            _sim = Native.pon_sim_create(determinism);
            if (_sim == IntPtr.Zero) throw new InvalidOperationException("pon_sim_create failed");
        }

        public void SetGravity(Vector3 g) => Native.pon_sim_set_gravity(_sim, g);
        public void SetAtmosphere(double altitudeM) =>
            Native.pon_sim_set_atmosphere(_sim, altitudeM, 0, 0, 0);

        public bool CatalogHas(string id) => Native.pon_catalog_has(id) != 0;

        /// <summary>Register a shipped catalog round ("762x51_175gr_smk", …). Invalid if unknown.</summary>
        public uint RegisterCatalog(string id) => Native.pon_register_catalog_type(_sim, id);

        public uint Fire(uint typeId, Vector3 muzzle, Vector3 aim, Tier tier = Tier.Integrated) =>
            Native.pon_spawn_precise(_sim, typeId, muzzle, aim, 0.0, (int)tier, 0);

        public void Step(double dt) { if (dt > 0) Native.pon_step(_sim, dt); }

        public bool TryGetState(uint id, out PonState s) =>
            Native.pon_get_state(_sim, id, out s) == 0 /* PON_OK */;

        public Vector3 GetPosition(uint id) =>
            TryGetState(id, out var s) ? s.position.ToVector3() : Vector3.zero;

        public bool IsAlive(uint id) => TryGetState(id, out var s) && s.alive != 0;
        public int  LiveCount => (int)Native.pon_live_count(_sim).ToUInt64();
        public void Despawn(uint id) => Native.pon_despawn(_sim, id);

        /// <summary>Install a diagnostic trace, or null to clear. See PonTraceKind.</summary>
        public void SetTraceSink(Action<PonTraceEvent> sink)
        {
            if (sink == null) { _traceKeepAlive = null; Native.pon_sim_set_trace_sink(_sim, null, IntPtr.Zero); return; }
            _traceKeepAlive = (ref PonTraceEvent ev, IntPtr _) => sink(ev);
            Native.pon_sim_set_trace_sink(_sim, _traceKeepAlive, IntPtr.Zero);
        }

        public void Dispose()
        {
            if (_sim != IntPtr.Zero) { Native.pon_sim_destroy(_sim); _sim = IntPtr.Zero; }
            _traceKeepAlive = null;
            GC.SuppressFinalize(this);
        }
        ~PonceletSim() => Dispose();
    }
}
