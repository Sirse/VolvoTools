#pragma once

#include "BusConfiguration.hpp"
#include "GatewayEndpoint.hpp"

#include <vector>

namespace common {

struct ConfigurationInfo {
    std::string name;
    // The value accepted by --platform / parseCarPlatform. In the v2 schema this replaces the
    // textual match of configuration.name against getCarPlatformName(); it is required and unique.
    std::string cliName;
    std::vector<BusConfiguration> busInfo;
    // The tester endpoints used to reach the target bus through the gateway. In the original
    // sda.xml these live under ModelGroup/Gateway_SubTester and ModelGroup/SubTester; in the
    // YAML they sit at the Configuration level next to Bus and Name.
    GatewayEndpoint gatewaySubTester;
    GatewayEndpoint subTester;
};

} // namespace common
