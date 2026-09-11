# poncelet — vcpkg overlay port.
# SPDX-License-Identifier: MIT
#
# Use today (poncelet has a public remote — https://github.com/FelixMiddelhoff/poncelet
# — but no tagged release yet):
#   vcpkg install poncelet --overlay-ports=<repo>/vcpkg-ports
#
# `vcpkg_from_git` below defaults to cloning THIS repo checkout itself over a
# `file://` URL (git supports local clones offline — no network needed) so
# the port is usable without waiting on a tagged release. Once a `vX.Y.Z`
# tag exists, switch the default below from the `file://` clone to the real
#   URL https://github.com/FelixMiddelhoff/poncelet.git
#   REF v1.0.0
# — nothing else in this file changes. Override PONCELET_OVERLAY_GIT_URL /
# PONCELET_OVERLAY_GIT_REF (environment variables) to point at a different
# checkout or ref without editing the port (e.g. the real remote today).

if(DEFINED ENV{PONCELET_OVERLAY_GIT_URL})
    set(PONCELET_GIT_URL "$ENV{PONCELET_OVERLAY_GIT_URL}")
else()
    get_filename_component(PONCELET_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    set(PONCELET_GIT_URL "file://${PONCELET_REPO_ROOT}")
endif()

if(DEFINED ENV{PONCELET_OVERLAY_GIT_REF})
    set(PONCELET_GIT_FETCH_REF "$ENV{PONCELET_OVERLAY_GIT_REF}")
else()
    set(PONCELET_GIT_FETCH_REF "main")
endif()

# vcpkg_from_git's REF must be a commit SHA (it errors on a named branch/tag —
# "REF must be a commit SHA" — since the ABI hash it caches against needs a
# fixed point, not a moving one). Resolve PONCELET_GIT_FETCH_REF to its SHA
# via `git ls-remote`, which works against both the local `file://` clone
# used for local testing and a real remote URL, without a full clone.
find_program(GIT NAMES git)
if(NOT GIT)
    message(FATAL_ERROR "git not found on PATH — needed to resolve ${PONCELET_GIT_FETCH_REF} to a commit SHA")
endif()
execute_process(
    COMMAND "${GIT}" ls-remote "${PONCELET_GIT_URL}" "${PONCELET_GIT_FETCH_REF}"
    OUTPUT_VARIABLE PONCELET_LS_REMOTE_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PONCELET_LS_REMOTE_RESULT
)
if(NOT PONCELET_LS_REMOTE_RESULT EQUAL 0 OR PONCELET_LS_REMOTE_OUTPUT STREQUAL "")
    message(FATAL_ERROR "git ls-remote '${PONCELET_GIT_URL}' '${PONCELET_GIT_FETCH_REF}' "
                        "found nothing — check PONCELET_OVERLAY_GIT_URL/_REF.")
endif()
string(REGEX MATCH "^[0-9a-f]+" PONCELET_GIT_SHA "${PONCELET_LS_REMOTE_OUTPUT}")

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL "${PONCELET_GIT_URL}"
    REF "${PONCELET_GIT_SHA}"
    FETCH_REF "${PONCELET_GIT_FETCH_REF}"
)

vcpkg_check_linkage(ONLY_STATIC_LIBRARY)   # see PONCELET_SHARED note in ../README.md

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DPONCELET_BUILD_TESTS=OFF
        -DPONCELET_BUILD_BENCH=OFF
        -DPONCELET_BUILD_EXAMPLES=OFF
        -DPONCELET_BUILD_DOCS=OFF
        -DPONCELET_SHARED=OFF
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME poncelet CONFIG_PATH lib/cmake/poncelet)
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(INSTALL "${SOURCE_PATH}/LICENSE" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}" RENAME copyright)
