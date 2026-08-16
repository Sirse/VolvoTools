#include "common/Gateway.hpp"

#include "common/protocols/UDSService.hpp"

#include <stdexcept>

namespace common {

bool resolveGatewayRoute(const ConfigurationInfo& configuration, uint32_t targetEcuId,
                         GatewayRoute& route)
{
    (void)targetEcuId;
    // The gateway/sub-tester split is a P3-era feature. Configurations without an endpoint
    // (P1/P2 and friends) have no gateway route at all.
    if (configuration.gatewaySubTester.ecuAddress == 0 && configuration.gatewaySubTester.canId == 0) {
        return false;
    }
    route.endpoint = configuration.gatewaySubTester;
    return true;
}

std::vector<uint8_t> sdaRoutineRequest(bool start, uint16_t routineId,
                                       const std::vector<uint8_t>& optionBytes)
{
    std::vector<uint8_t> request{
        static_cast<uint8_t>(uds::ServiceId::RoutineControl),
        static_cast<uint8_t>(start ? uds::RoutineControlSubFunction::StartRoutine
                                   : uds::RoutineControlSubFunction::StopRoutine),
        static_cast<uint8_t>(routineId >> 8),
        static_cast<uint8_t>(routineId)};
    if (start) {
        request.insert(request.end(), optionBytes.cbegin(), optionBytes.cend());
    }
    return request;
}

void validateRoutineControlResponse(const std::vector<uint8_t>& payload, bool start,
                                    uint16_t routineId)
{
    if (payload.size() < 4
        || payload[0] != static_cast<uint8_t>(uds::PositiveResponseId::RoutineControl)
        || payload[1] != static_cast<uint8_t>(start ? uds::RoutineControlSubFunction::StartRoutine
                                                    : uds::RoutineControlSubFunction::StopRoutine)
        || payload[2] != static_cast<uint8_t>(routineId >> 8)
        || payload[3] != static_cast<uint8_t>(routineId)) {
        throw std::runtime_error("Unexpected RoutineControl response");
    }
}

std::string sdaRoutineName(SdaRoutine routine)
{
    switch (routine) {
    case SdaRoutine::ActivateSBL:
        return "ActivateSBL";
    case SdaRoutine::EraseMemory:
        return "EraseMemory";
    case SdaRoutine::CheckProgrammingDependencies:
        return "CheckProgrammingDependencies";
    case SdaRoutine::CheckValidApplication:
        return "CheckValidApplication";
    case SdaRoutine::GatewayStateAccess:
        return "GatewayStateAccess";
    }
    return "unknown";
}

} // namespace common
