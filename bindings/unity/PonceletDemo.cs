// poncelet — Unity demo. Fire a catalog round, draw its path, log its state.
// SPDX-License-Identifier: MIT
//
// Attach to an empty GameObject. Needs Poncelet.cs and the native library in
// Assets/Plugins/. A LineRenderer child (optional) draws the live trajectory.
using System.Collections.Generic;
using UnityEngine;
using Poncelet;

public class PonceletDemo : MonoBehaviour
{
    [SerializeField] string round = "762x51_175gr_smk";
    [SerializeField] Vector3 muzzle = new Vector3(0f, 1.7f, 0f);
    [SerializeField] Vector3 aim = new Vector3(1f, 0.02f, 0f);
    [SerializeField] float fireEvery = 1.2f;

    PonceletSim _sim;
    uint _type = PonceletSim.Invalid;
    readonly List<uint> _shots = new();
    readonly List<Vector3> _pts = new();
    LineRenderer _line;
    float _t;

    void Start()
    {
        Debug.Log($"poncelet {PonceletSim.Version}");
        _sim = new PonceletSim();
        _sim.SetAtmosphere(0.0);

        if (!_sim.CatalogHas(round)) { Debug.LogError($"unknown round '{round}'"); enabled = false; return; }
        _type = _sim.RegisterCatalog(round);

        _sim.SetTraceSink(ev =>
        {
            if (ev.kind == (int)PonTraceKind.TransonicEnter)
                Debug.Log($"shot {ev.shot} transonic at t={ev.timeS:F2}s, {ev.velocity.ToVector3().magnitude:F0} m/s");
        });

        _line = GetComponent<LineRenderer>();
    }

    void FixedUpdate()
    {
        if (_sim == null) return;
        _sim.Step(Time.fixedDeltaTime);

        _t += Time.fixedDeltaTime;
        if (_t >= fireEvery)
        {
            _t = 0f;
            _shots.Add(_sim.Fire(_type, muzzle, aim));
        }

        _shots.RemoveAll(id => !_sim.IsAlive(id));
    }

    void Update()
    {
        if (_sim == null || _line == null) return;
        _pts.Clear();
        foreach (var id in _shots) _pts.Add(_sim.GetPosition(id));
        _line.positionCount = _pts.Count;
        _line.SetPositions(_pts.ToArray());
    }

    void OnDestroy() => _sim?.Dispose();
}
