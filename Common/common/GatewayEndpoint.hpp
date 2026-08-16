#pragma once

#include <cstdint>
#include <string>

namespace common {

// A tester endpoint for the SDA gateway session, as read from the vehicle configuration
// (Gateway_SubTester / SubTester). ecuAddress is the diagnostic address of the module that
// physically sits on the target bus; canId is the CAN request identifier used to reach it.
struct GatewayEndpoint {
    uint32_t ecuAddress = 0;
    uint32_t canId = 0;
    std::string name;
};

} // namespace common
