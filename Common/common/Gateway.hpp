#pragma once

#include "ConfigurationInfo.hpp"
#include "GatewayEndpoint.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace common {

// The route SDA uses to reach a target bus through the gateway: which tester endpoint to talk
// to, and the sub-network/sub-node selector bytes carried in the GatewayStateAccess request.
// The exact option record layout is not pinned yet (no reference SDA trace), so the codec
// carries them as explicit raw bytes rather than guessing from the bus name.
struct GatewayRoute {
    GatewayEndpoint endpoint;
    uint32_t subnetwork = 0;
    uint32_t subnodeAddress = 0;
    // Raw option bytes appended to the GatewayStateAccess request when the caller wants to
    // drive the exact wire content. Empty means "use subnetwork/subnodeAddress".
    std::vector<uint8_t> rawOptions;
};

// A named SDA routine, expressed over the generic RoutineControl codec.
enum class SdaRoutine {
    ActivateSBL,
    EraseMemory,
    CheckProgrammingDependencies,
    CheckValidApplication,
    GatewayStateAccess,
};

// Resolves the gateway route for a target ECU: the configured Gateway_SubTester is the
// endpoint that physically bridges to the target bus, so it is used whenever present. The
// sub-network/sub-node values come from explicit overrides; without them the route carries
// zeroes and the caller must supply raw option bytes or explicit values.
// Returns false when the configuration has no gateway endpoint (the P1/P2 world predates it).
bool resolveGatewayRoute(const ConfigurationInfo& configuration, uint32_t targetEcuId,
                         GatewayRoute& route);

// Builds a RoutineControl (31) request for a named SDA routine. RID 0x0300 is only a
// hypothesis (the numeric id was not found in the binaries), so the codec takes an explicit
// routine id and option bytes. start=true -> 31 01 <rid> <options...>, false -> 31 02 <rid>.
std::vector<uint8_t> sdaRoutineRequest(bool start, uint16_t routineId,
                                       const std::vector<uint8_t>& optionBytes);

// Validates a positive RoutineControl response against the request it answers.
// Throws std::runtime_error on a non-71 prefix or mismatched sub-function/routine id.
void validateRoutineControlResponse(const std::vector<uint8_t>& payload, bool start,
                                    uint16_t routineId);

// Human-readable name for a named SDA routine (for logs/CLI output).
std::string sdaRoutineName(SdaRoutine routine);

} // namespace common
