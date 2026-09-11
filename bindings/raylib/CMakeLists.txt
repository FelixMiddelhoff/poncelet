# poncelet — raylib sample.
# SPDX-License-Identifier: MIT
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ./build/poncelet_raylib      (Debug/Release subfolder on multi-config generators)
#
# See README.md.
cmake_minimum_required(VERSION 3.20)
project(poncelet_raylib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# raylib — fetched, not vendored (unlike godot-cpp's manual-clone convention:
# raylib's own CMake build is self-contained, no API-version file to match).
FetchContent_Declare(raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

# poncelet core (two levels up), static, no tests/bench/examples.
set(PONCELET_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_BENCH   OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_DOCS    OFF CACHE BOOL "" FORCE)
add_subdirectory(../.. poncelet)

add_executable(poncelet_raylib main.cpp)
target_link_libraries(poncelet_raylib PRIVATE poncelet::poncelet raylib)
