// poncelet — Unreal Engine plugin module entry point.
// SPDX-License-Identifier: MIT
#include "Modules/ModuleManager.h"

// No custom startup/shutdown needed — pon::Sim instances are owned per
// UPonceletSimComponent (BeginPlay/EndPlay), not by the module itself.
IMPLEMENT_MODULE(FDefaultModuleImpl, Poncelet)
