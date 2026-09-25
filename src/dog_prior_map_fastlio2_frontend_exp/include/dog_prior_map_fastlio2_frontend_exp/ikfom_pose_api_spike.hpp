#pragma once

#include <string>

namespace dog_prior_map_fastlio2_frontend_exp {

// Stable, dependency-light test facade. The pinned use-ikfom.hpp header is
// intentionally excluded from public headers and included by one source file.
bool runIkfomPoseApiSpike(std::string* failure_reason);

}  // namespace dog_prior_map_fastlio2_frontend_exp
