#pragma once

#include <stdexcept>
#include <string>

namespace volvodiag {

// Stable exit codes for scripts and CI.
enum class ExitCode : int {
    Success          = 0, // command completed; any assertions passed
    GenericFailure   = 1, // unclassified error
    UsageError       = 2, // invalid CLI arguments / usage
    DeviceError      = 3, // J2534 device or channel could not be selected/opened
    TransportTimeout = 4, // no response within the timeout (transport level)
    NrcError         = 5, // ECU replied with a negative response (UDS NRC)
    ValidationError  = 6, // response received but failed an expect/assertion
};

// Error with an explicit process exit code.
class DiagError : public std::runtime_error {
public:
    DiagError(ExitCode code, const std::string& message)
        : std::runtime_error(message), _code{code} {}

    ExitCode code() const noexcept { return _code; }

private:
    ExitCode _code;
};

// Maps an exception to a process exit code.
int classifyExitCode(const std::exception& ex);

} // namespace volvodiag
