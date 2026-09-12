# poncelet — vcpkg overlay port.
# SPDX-License-Identifier: MIT
#
# Use today:
#   vcpkg install poncelet --overlay-ports=<repo>/vcpkg-ports
#
# Prepared in registry-submission format: a fixed tag + SHA512 checksum,
# matching what a real microsoft/vcpkg ports/poncelet/portfile.cmake needs.
# The checksum is re-derived whenever the pinned REF changes — see
# ../README.md for how.

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO FelixMiddelhoff/poncelet
    REF "v1.0.0"
    SHA512 ba9a3798e63e3911f2771ee1e5053db33b235d524a323123a95e0a8251260bd22918e2806858bdcbd9e490ccd9c3219ee359ec16230986aeaf4956ccaa505a0e
    HEAD_REF main
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
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
file(INSTALL "${SOURCE_PATH}/LICENSE" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}" RENAME copyright)
