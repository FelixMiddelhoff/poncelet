# poncelet — Conan recipe.
# SPDX-License-Identifier: MIT
#
# Local/overlay use today: `conan create .` from the repo root packages this
# checkout directly (exports_sources below) — no registry, no remote needed.
# Publishing to conan-center-index (the actual central registry) is a
# separate step requiring a real GitHub remote + their own review process.
#
# Packages the STATIC `poncelet::poncelet` CMake target only — the one
# namespaced, find_package()-consumable target poncelet's own CMakeLists
# exports. PONCELET_SHARED adds a second, un-aliased `poncelet_shared` target
# meant for direct linking in-tree (see bindings/unity's README) rather than
# package consumption, so it isn't exposed as a Conan `shared` option here.
from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeToolchain, CMakeDeps, cmake_layout
from conan.tools.files import copy
import os


class PonceletConan(ConanFile):
    name = "poncelet"
    version = "1.0.0"
    license = "MIT"
    description = "Projectile & terminal-ballistics simulation library"
    topics = ("ballistics", "physics", "simulation", "game-development")
    settings = "os", "compiler", "build_type", "arch"
    options = {"fPIC": [True, False]}
    default_options = {"fPIC": True}

    exports_sources = (
        "CMakeLists.txt",
        "cmake/*",
        "include/*",
        "src/*",
        "data/*",
        "tools/bake_data.py",
        "LICENSE",
    )

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def validate(self):
        # poncelet's public headers use std::optional/std::string_view/
        # std::byte (guide.md, "the public headers use <optional>/
        # <string_view> — a consumer inherits C++17"). Fail loudly here
        # rather than let a consumer with an older default profile
        # (Conan's own default is cppstd=14) hit confusing errors deep in
        # <poncelet/...> headers.
        if self.settings.get_safe("compiler.cppstd"):
            check_min_cppstd(self, "17")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        # A pure library package — none of poncelet's own tests/bench/example
        # programs need to build (or their extra dependencies, e.g. enabling
        # C for the C-ABI example) just to produce+install the library.
        tc.variables["PONCELET_SHARED"] = False
        tc.variables["PONCELET_BUILD_TESTS"] = False
        tc.variables["PONCELET_BUILD_BENCH"] = False
        tc.variables["PONCELET_BUILD_EXAMPLES"] = False
        tc.variables["PONCELET_BUILD_DOCS"] = False
        tc.generate()
        CMakeDeps(self).generate()  # poncelet has no dependencies of its own; harmless no-op

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build(target="poncelet")

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "LICENSE", src=self.source_folder,
             dst=os.path.join(self.package_folder, "licenses"))

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "poncelet")
        self.cpp_info.set_property("cmake_target_name", "poncelet::poncelet")
        self.cpp_info.set_property("pkg_config_name", "poncelet")
        self.cpp_info.libs = ["poncelet"]
