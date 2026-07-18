#include "ExitCodes.hpp"

#include <common/Util.hpp>
#include <common/protocols/UDSError.hpp>
#include <common/protocols/UDSRequest.hpp>

namespace volvodiag {

int classifyExitCode(const std::exception& ex)
{
    // Prefer an explicit code.
    if (const auto* diag = dynamic_cast<const DiagError*>(&ex)) {
        return static_cast<int>(diag->code());
    }
    // Use exception types, not message text.
    if (dynamic_cast<const common::UDSError*>(&ex)) {
        return static_cast<int>(ExitCode::NrcError);
    }
    if (dynamic_cast<const common::UDSRequestRxTimeout*>(&ex)) {
        return static_cast<int>(ExitCode::TransportTimeout);
    }
    if (dynamic_cast<const common::UDSRequestTxError*>(&ex)) {
        return static_cast<int>(ExitCode::DeviceError);
    }
    if (dynamic_cast<const common::DeviceSelectionError*>(&ex)) {
        return static_cast<int>(ExitCode::DeviceError);
    }
    return static_cast<int>(ExitCode::GenericFailure);
}

} // namespace volvodiag
