// SPDX-License-Identifier: MIT
#include "poncelet/version.hpp"

namespace pon {

Version library_version() { return Version{}; }
const char* library_version_string() { return PONCELET_VERSION_STRING; }

} // namespace pon
