# poncelet — Godot 4 GDExtension sample.
# SPDX-License-Identifier: MIT
#
#   git clone -b 4.3 https://github.com/godotengine/godot-cpp   (match your Godot 4.x)
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#
# then copy build/libponcelet_godot.* next to demo/poncelet.gdextension and open
# demo/ in Godot. See README.md.
cmake_minimum_required(VERSION 3.20)
project(poncelet_godot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# godot-cpp — clone it next to this file (or set GODOT_CPP_DIR).
set(GODOT_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/godot-cpp" CACHE PATH "godot-cpp checkout")
if(NOT EXISTS "${GODOT_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "godot-cpp not found at ${GODOT_CPP_DIR} — clone it there "
                      "(a recent 4.x-compatible checkout) or pass -DGODOT_CPP_DIR=…")
endif()
# godot-cpp master needs the target API version named; it bundles the JSON.
# Override -DGODOTCPP_API_VERSION=4.x to match your Godot.
if(NOT DEFINED GODOTCPP_API_VERSION OR GODOTCPP_API_VERSION STREQUAL "")
  set(GODOTCPP_API_VERSION "4.7" CACHE STRING "Godot API version to target" FORCE)
endif()
add_subdirectory(${GODOT_CPP_DIR} godot-cpp)

# poncelet core (two levels up), static, no tests/bench/examples.
set(PONCELET_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_BENCH   OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_DOCS    OFF CACHE BOOL "" FORCE)
add_subdirectory(../.. poncelet)

add_library(poncelet_godot SHARED
  src/poncelet_sim.cpp
  src/register_types.cpp)
target_link_libraries(poncelet_godot PRIVATE godot-cpp poncelet::poncelet)
set_target_properties(poncelet_godot PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  PREFIX "lib")
