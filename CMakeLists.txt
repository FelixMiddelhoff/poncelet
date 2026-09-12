# poncelet — projectile & terminal-ballistics library.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.20)
project(poncelet LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Standalone build vs. consumed as a subdirectory.
set(PONCELET_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(PONCELET_STANDALONE ON)
endif()

option(PONCELET_SHARED        "Also build a shared library"      OFF)
option(PONCELET_BUILD_TESTS   "Build the test suite"            ${PONCELET_STANDALONE})
option(PONCELET_BUILD_BENCH   "Build the perf bench"            ${PONCELET_STANDALONE})
option(PONCELET_BUILD_EXAMPLES "Build the docs/examples programs" ${PONCELET_STANDALONE})

set(PONCELET_SOURCES
  src/version.cpp
  src/defaults.cpp
  src/validate.cpp
  src/preview.cpp
  src/describe.cpp
  src/catalog.cpp
  src/atmosphere.cpp
  src/ball_profiles.cpp
  src/drag_tables.cpp
  src/integrate.cpp
  src/terminal.cpp
  src/explosion.cpp
  src/fragmentation.cpp
  src/shapedcharge.cpp
  src/guidance.cpp
  src/sim.cpp
  src/c_api.cpp
)

# --- Data tables: data/*.csv baked to src/generated/data_tables.inc.
# The generated file is committed so a Python-less build still works; when a
# Python 3 interpreter is available it is regenerated whenever a CSV or the
# bake script changes.
find_package(Python3 QUIET COMPONENTS Interpreter)
set(PONCELET_DATA_INC ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/data_tables.inc)
set(PONCELET_DATA_CSV
  ${CMAKE_CURRENT_SOURCE_DIR}/data/class_defaults.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/data/projectiles.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/data/ball_profiles.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/data/materials.csv)
if(Python3_FOUND)
  add_custom_command(
    OUTPUT ${PONCELET_DATA_INC}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/bake_data.py
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/bake_data.py ${PONCELET_DATA_CSV}
    COMMENT "poncelet: baking data/*.csv -> data_tables.inc"
    VERBATIM)
  add_custom_target(poncelet_bake_data DEPENDS ${PONCELET_DATA_INC})
endif()

add_library(poncelet STATIC ${PONCELET_SOURCES})
add_library(poncelet::poncelet ALIAS poncelet)
if(Python3_FOUND)
  add_dependencies(poncelet poncelet_bake_data)
endif()
target_include_directories(poncelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(poncelet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# The public headers use <optional>/<string_view> — a consumer inherits C++17.
target_compile_features(poncelet PUBLIC cxx_std_17)

if(PONCELET_SHARED)
  add_library(poncelet_shared SHARED ${PONCELET_SOURCES})
  target_include_directories(poncelet_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(poncelet_shared PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_features(poncelet_shared PUBLIC cxx_std_17)
  target_compile_definitions(poncelet_shared PUBLIC PONCELET_SHARED
                                            PRIVATE PONCELET_BUILD)
  if(Python3_FOUND)
    add_dependencies(poncelet_shared poncelet_bake_data)
  endif()
  # NB: keep a distinct output name — an import lib called poncelet.lib would
  # collide with the static archive in a shared build dir.
  set_target_properties(poncelet_shared PROPERTIES OUTPUT_NAME poncelet_shared)
endif()

# --- Determinism prep (docs/ballistics-phase-plan.md §3.6): FP-sensitive TUs
# get no-fast-math, contraction off, applied per-file so a global flag change
# elsewhere can't silently break it.
#   * integrate.cpp — the hot-path integrator.
#   * drag_tables.cpp / ball_profiles.cpp — compile_drag_lut()'s interp_curve()
#     does `a + (b - a) * f` (textbook fused-multiply-add bait). First
#     hypothesis for the macOS-only poncelet_bitexact_golden mismatch; kept as
#     good practice but a follow-up CI run with the same digest afterward
#     proved this specific fix had zero effect on the actual bug.
#   * sim.cpp — the REAL cause (found via a per-shot digest diff: only the
#     SixDOF and SpinDrift+Coriolis shots mismatched on macOS, and those are
#     the only two that reach Vec3::cross()/Quat::operator* — both textbook
#     `a*b - c*d` FMA-contraction bait, defined in types.hpp and inlined
#     wherever included). This TU never had the flag; on ARM64 (native FMA)
#     the compiler folds those into a single fused instruction with a
#     different last bit than x86-64's separate mul+sub, and the resulting
#     1-bit orientation/right-vector difference cascades into a different
#     folded hash. Confirmed by two shift-free Fx32 arithmetic reference
#     tests passing on macOS CI (ruling out the fixed-point core itself)
#     before landing on this.
#   * guidance.cpp — compute_guidance() (bitexact-guidance-law-plan.md): every
#     dot()/cross() in the PN/APN/Pursuit laws is a genuine FMA-contraction
#     risk of the same shape as sim.cpp's, feeding the hashed accel_mps2/
#     losRate_radps/closingSpeed_mps output directly.
if(MSVC)
  set_source_files_properties(src/integrate.cpp src/drag_tables.cpp src/ball_profiles.cpp
                               src/sim.cpp src/guidance.cpp
    PROPERTIES COMPILE_OPTIONS "/fp:precise;/fp:except-")
else()
  set_source_files_properties(src/integrate.cpp src/drag_tables.cpp src/ball_profiles.cpp
                               src/sim.cpp src/guidance.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-fast-math")
endif()

if(MSVC)
  target_compile_options(poncelet PRIVATE /W4 /permissive-)
else()
  target_compile_options(poncelet PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --- Install ------------------------------------------------------------------
# An installed poncelet is consumable three equivalent ways:
#   add_subdirectory(poncelet)                         -> poncelet::poncelet
#   FetchContent_MakeAvailable(poncelet)               -> poncelet::poncelet
#   find_package(poncelet CONFIG REQUIRED)             -> poncelet::poncelet
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS poncelet EXPORT ponceletTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
if(PONCELET_SHARED)
  install(TARGETS poncelet_shared EXPORT ponceletTargets
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(PONCELET_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/poncelet)
install(EXPORT ponceletTargets
        NAMESPACE poncelet::
        FILE ponceletTargets.cmake
        DESTINATION ${PONCELET_CMAKEDIR})
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ponceletConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ponceletConfig.cmake
  INSTALL_DESTINATION ${PONCELET_CMAKEDIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ponceletConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ponceletConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ponceletConfigVersion.cmake
  DESTINATION ${PONCELET_CMAKEDIR})

# pkg-config, for Make / Meson / hand-rolled builds that don't consume the
# CMake package directly.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/poncelet.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/poncelet.pc
  @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/poncelet.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# --- Tests / bench -----------------------------------------------------------
if(PONCELET_BUILD_TESTS)
  enable_testing()
  add_executable(poncelet_tests tests/test_main.cpp)
  target_link_libraries(poncelet_tests PRIVATE poncelet)
  target_include_directories(poncelet_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
    ${CMAKE_CURRENT_SOURCE_DIR}/src)   # G1/G7 table unit checks
  add_test(NAME poncelet_tests COMMAND poncelet_tests)
  # The validation suite (published-data reproduction + determinism + fuzz,
  # WORKPLAN item 13 / §6) as its own gate — the harness filters by name.
  add_test(NAME poncelet_validation COMMAND poncelet_tests validation_)
  if(Python3_FOUND)
    add_test(NAME data_tables_baked_fresh
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/bake_data.py --check)
    add_test(NAME fixed_lut_tables_fresh
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fixed_lut.py --check)
    add_test(NAME amalgamation_fresh
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/amalgamate.py --check)
  endif()

  # Single-header amalgamation: compile dist/poncelet_single.hpp on its own (no
  # link against the poncelet library) from two TUs — one defining
  # PONCELET_SINGLE_IMPLEMENTATION, one plain include — so a stale or broken
  # amalgamation fails CI.
  add_executable(poncelet_single_smoke
    tests/single_header_main.cpp tests/single_header_tu2.cpp)
  target_include_directories(poncelet_single_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dist)
  set_target_properties(poncelet_single_smoke PROPERTIES LINKER_LANGUAGE CXX)
  add_test(NAME poncelet_single_header COMMAND poncelet_single_smoke)
endif()

if(PONCELET_BUILD_BENCH)
  add_executable(poncelet_bench bench/bench_main.cpp)
  target_link_libraries(poncelet_bench PRIVATE poncelet)
  # Perf gate: fails CI on a gross throughput regression (loose ceilings).
  if(PONCELET_BUILD_TESTS)
    add_test(NAME poncelet_perf_gate COMMAND poncelet_bench --check)
    # Determinism gate: one fixed scene run twice from scratch must fold to the
    # same per-frame stateHash() digest. CI runs it on every OS/config, so it
    # also catches an optimisation level or platform libm breaking repeatability.
    add_test(NAME poncelet_det_check COMMAND poncelet_bench --det-check)
    # Cross-platform BitExact gate: the same scene under Determinism::BitExact
    # runs entirely on the Q32.32 fixed-point core, so its folded digest is
    # integer-derived and must equal the committed golden on every OS / compiler
    # / optimisation level. A mismatch is a regression or a portability bug —
    # never a tolerance to relax.
    add_test(NAME poncelet_bitexact_golden COMMAND poncelet_bench --bitexact-golden)
    # AdaptiveRKF45 under BitExact: its step-size controller now routes
    # ratio^0.2 through a fixed-point LUT instead of std::pow (see
    # integrate.cpp's pow_ratio02), closing the one libm call that used to
    # keep this precision flag out of the golden scenario above. Same-machine
    # repeatability gate (not a cross-platform golden — this flag isn't in
    # the 3-OS-verified scenario, so there's no committed reference digest
    # for it yet); still a real regression gate for this specific fix.
    add_test(NAME poncelet_bitexact_rkf45_check COMMAND poncelet_bench --bitexact-rkf45-check)
    # Guidance law under BitExact (bitexact-guidance-law-plan.md Phase 4):
    # compute_guidance() now runs GuidanceCore<Fx32>. Same-machine repeatability
    # gate, same reasoning as the RKF45 check above — no committed cross-
    # platform golden for a guided shot yet, so this stays its own ctest.
    add_test(NAME poncelet_bitexact_guidance_check COMMAND poncelet_bench --bitexact-guidance-check)
  endif()
endif()

# --- Examples (docs/examples/*.cpp) — built so they cannot rot; each is also a
# smoke test that the public headers compile and the loop runs.
if(PONCELET_BUILD_EXAMPLES)
  enable_language(C)  # for the C-ABI example only; the library is pure C++
  foreach(ex basic_shot.cpp custom_world.cpp curveball.cpp wind_and_media.cpp
             precision_long_range.cpp moving_target.cpp terminal_ballistics.cpp
             special_media.cpp catalog_shot.cpp bullet_cam.cpp sixdof_flight.cpp
             explosive_warhead.cpp fragmentation.cpp shaped_charge.cpp
             guided_missile.cpp destruction_coupling.cpp aim_preview.cpp
             stock_worlds.cpp dope_table.cpp
             c_api_shot.c)
    get_filename_component(ex_name ${ex} NAME_WE)
    add_executable(poncelet_example_${ex_name} docs/examples/${ex})
    target_link_libraries(poncelet_example_${ex_name} PRIVATE poncelet)
    set_target_properties(poncelet_example_${ex_name} PROPERTIES LINKER_LANGUAGE CXX)
    if(PONCELET_BUILD_TESTS)
      # dope_table takes required args (--round); a fixed invocation is the
      # ctest, not the no-arg default every other example runs with.
      if(ex_name STREQUAL "dope_table")
        add_test(NAME example_dope_table
                 COMMAND poncelet_example_dope_table
                 --round 762x51_175gr_smk --zero 100 --to 500 --step 100)
      else()
        add_test(NAME example_${ex_name} COMMAND poncelet_example_${ex_name})
      endif()
    endif()
  endforeach()
endif()

# --- API reference (Doxygen) -------------------------------------------------
# `cmake --build build --target poncelet_docs` -> docs/api/html/index.html.
# The public headers use plain `//`; docs/Doxyfile runs tools/doxygen_filter.py
# to surface that prose (needs python3). Off unless Doxygen is installed.
option(PONCELET_BUILD_DOCS "Add the poncelet_docs (Doxygen) target" ${PONCELET_STANDALONE})
if(PONCELET_BUILD_DOCS)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    add_custom_target(poncelet_docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "poncelet: generating API reference -> docs/api/html"
      VERBATIM)
  endif()
endif()
