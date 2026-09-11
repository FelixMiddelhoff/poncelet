// poncelet — Unreal Engine plugin build rules.
// SPDX-License-Identifier: MIT
using System;
using System.IO;
using UnrealBuildTool;

public class Poncelet : ModuleRules
{
	public Poncelet(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		// UE 5.8 disallows C++17 outright (CppStandardVersion.Cpp17 is a
		// build-time error, not just deprecated) — poncelet's headers are
		// C++17-clean and compile fine under 20, so target the engine's floor.
		CppStandard = CppStandardVersion.Cpp20;
		// <poncelet/catalog.hpp>'s CSV loader uses std::stod (throws on a bad
		// cell, caught internally) and <poncelet/projectile.hpp> pulls in
		// std::optional/std::string; build with exceptions on like any other
		// std-library-using third-party module.
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });

		// This module binds poncelet's C++ API directly (PublicIncludePaths ->
		// <poncelet/poncelet.hpp>), the same choice bindings/godot's GDExtension
		// made — richer types than the C ABI bindings/unity P/Invokes over.
		//
		// The prebuilt STATIC `poncelet` lib + its headers are vendored under
		// ThirdParty/poncelet/ *inside this plugin folder* (not referenced by a
		// path back into the poncelet repo checkout) — `RunUAT BuildPlugin`
		// copies the whole plugin directory to a staging area before compiling
		// it (to produce a redistributable package), so a Build.cs that reached
		// "up and out" to a sibling repo checkout would silently break under
		// that packaging step; it also wouldn't work at all once a consumer
		// copies this plugin folder into their own project's Plugins/, which is
		// the actual normal way to use it. See README.md "Build poncelet" for
		// the one-time step that populates ThirdParty/poncelet/.
		string ThirdPartyDir = Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "poncelet");
		string PonceletInclude = Path.Combine(ThirdPartyDir, "include");
		PublicIncludePaths.Add(PonceletInclude);

		// MSVC's std:: container ABI is keyed by _ITERATOR_DEBUG_LEVEL, which
		// differs between a Debug and a Release/DebugGame build. poncelet's
		// headers are included directly by this module's own TUs (e.g. a
		// std::vector<pon::Vec3> gets instantiated here), so the prebuilt
		// poncelet.lib MUST have been built in the config that matches this UE
		// target's, or the two disagree about container layout — README.md's
		// setup step builds + vendors both configs so this picks the right one.
		bool bWantsDebugCRT = Target.Configuration == UnrealTargetConfiguration.Debug;
		string PonceletConfig = bWantsDebugCRT ? "Debug" : "Release";
		string PonceletLib = Path.Combine(ThirdPartyDir, "lib", PonceletConfig, "poncelet.lib");

		if (File.Exists(PonceletLib))
		{
			PublicAdditionalLibraries.Add(PonceletLib);
		}
		else
		{
			// Fail loudly at generation time rather than at a mysterious link
			// error — same "explain what's wrong" spirit as pon_last_error().
			throw new BuildException(string.Format(
				"Poncelet: {0} not found. Run bindings/unreal/setup_thirdparty.ps1 " +
				"(or see its steps in README.md) to build poncelet and vendor it " +
				"into ThirdParty/poncelet/ first.",
				PonceletLib));
		}
	}
}
