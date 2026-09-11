# poncelet — vcpkg overlay port.
# SPDX-License-Identifier: MIT
#
# Use today:
#   vcpkg install poncelet --overlay-ports=<repo>/vcpkg-ports
#
# Defaults to the real remote (https://github.com/FelixMiddelhoff/poncelet)
# pinned to the v1.0.0 tag. Override PONCELET_OVERLAY_GIT_URL /
# PONCELET_OVERLAY_GIT_REF (environment variables) to point at a different
# checkout/ref instead — e.g. a local `file://` clone of this repo, or a
# later tag/branch — without editing the port.

if(DEFINED ENV{PONCELET_OVERLAY_GIT_URL})
    set(PONCELET_GIT_URL "$ENV{PONCELET_OVERLAY_GIT_URL}")
else()
    set(PONCELET_GIT_URL "https://github.com/FelixMiddelhoff/poncelet.git")
endif()

if(DEFINED ENV{PONCELET_OVERLAY_GIT_REF})
    set(PONCELET_GIT_FETCH_REF "$ENV{PONCELET_OVERLAY_GIT_REF}")
else()
    set(PONCELET_GIT_FETCH_REF "v1.0.0")
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
