# poncelet — copy-paste quickstart. SPDX-License-Identifier: MIT
#
# Deliberately outside the main docs/examples/ build loop and its ctest
# entries (PONCELET_BUILD_EXAMPLES) — the whole point of this directory is
# "pretend you don't have the rest of the poncelet repo," so it pulls
# poncelet in via FetchContent like a real first-time consumer would.
cmake_minimum_required(VERSION 3.20)
project(poncelet_quickstart LANGUAGES CXX)

include(FetchContent)

# poncelet has no public remote yet — set PONCELET_QUICKSTART_SOURCE_DIR to
# a local checkout to try this today. Once a real remote + tagged release
# exists, replace the GIT_TAG line below with it; nothing else here changes.
if(DEFINED ENV{PONCELET_QUICKSTART_SOURCE_DIR})
  # file(TO_CMAKE_PATH ...) so a Windows backslash path doesn't get read as a
  # CMake string escape sequence (e.g. "\C" is invalid).
  file(TO_CMAKE_PATH "$ENV{PONCELET_QUICKSTART_SOURCE_DIR}" PONCELET_LOCAL_SOURCE_DIR)
  FetchContent_Declare(poncelet SOURCE_DIR "${PONCELET_LOCAL_SOURCE_DIR}")
else()
  FetchContent_Declare(poncelet
    GIT_REPOSITORY https://github.com/<owner>/poncelet.git # TODO: real remote
    GIT_TAG        v1.0.0)
endif()

# A pure consumer only needs the library, not poncelet's own test/bench/example
# programs.
set(PONCELET_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_BENCH OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PONCELET_BUILD_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(poncelet)

add_executable(quickstart main.cpp)
target_link_libraries(quickstart PRIVATE poncelet::poncelet)
