#pragma once

#include "ECUInfo.hpp"

#include <string>
#include <vector>

namespace common {

struct BusConfiguration {
    std::string name;
    uint32_t protocolId;
    uint32_t baudrate;
    uint32_t canIdBitSize;
    // CAN bit sample point in percent, as shipped in the vehicle configuration.
    // 0 means "not specified" and callers fall back to deriving it from the baudrate.
    // Most buses are 68/80, but P1 CAN MS wants 60, which the baudrate alone can't tell you.
    uint32_t samplePoint = 0;
    // Vehicle-configuration values that were present in data.yaml but previously ignored.
    // P4CANMax: maximum response-pending budget in ms (hardcoded as kResponsePendingTimeout today).
    uint32_t p4CanMax = 0;
    // SWDLSpecification: the software-download specification version the bus was validated against.
    uint32_t swdlSpecification = 0;
    std::vector<ECUInfo> ecuInfo;
};

} // namespace common
