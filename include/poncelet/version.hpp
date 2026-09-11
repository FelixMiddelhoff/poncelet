// poncelet — projectile & terminal-ballistics library.
// SPDX-License-Identifier: MIT
#pragma once

#define PONCELET_VERSION_MAJOR 1
#define PONCELET_VERSION_MINOR 0
#define PONCELET_VERSION_PATCH 0
#define PONCELET_VERSION_STRING "1.0.0"

namespace pon {

struct Version {
    int major = PONCELET_VERSION_MAJOR;
    int minor = PONCELET_VERSION_MINOR;
    int patch = PONCELET_VERSION_PATCH;
};

// Returns the version of the linked library (may differ from the header this
// TU compiled against).
Version library_version();
const char* library_version_string();

} // namespace pon
