#pragma once

#include "VolvoDiagOptions.hpp"

namespace volvodiag {

// Parses command-line options; prints usage and returns false on error.
bool parseOptions(int argc, const char* argv[], RunOptions& options);

} // namespace volvodiag
