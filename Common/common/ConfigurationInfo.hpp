#pragma once

#include "BusConfiguration.hpp"
#include "GatewayEndpoint.hpp"

#include <vector>

namespace common {

struct ConfigurationInfo {
    std::string name;
    std::vector<BusConfiguration> busInfo;
    // The tester endpoints used to reach the target bus through the gateway. In the original
    // sda.xml these live under ModelGroup/Gateway_SubTester and ModelGroup/SubTester; in the
    // YAML they sit at the Configuration level next to Bus and Name.
    GatewayEndpoint gatewaySubTester;
    GatewayEndpoint subTester;
};

} // namespace common
