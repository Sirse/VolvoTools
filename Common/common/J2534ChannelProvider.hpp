#pragma once

#include "CarPlatform.hpp"
#include "ConfigurationInfo.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace j2534 {
class J2534;
class J2534Channel;
} // namespace j2534

namespace common {

class J2534ChannelProvider {
public:
    J2534ChannelProvider(J2534ChannelProvider&&) = delete;
    J2534ChannelProvider(const J2534ChannelProvider&) = delete;
    J2534ChannelProvider(j2534::J2534& j2534, CarPlatform carPlatform,
                         std::optional<uint32_t> baudrateOverride = std::nullopt);
    ~J2534ChannelProvider();

    J2534ChannelProvider& operator=(const J2534ChannelProvider&) = delete;

    j2534::J2534& getJ2534() const;
    std::vector<std::unique_ptr<j2534::J2534Channel>> getAllChannels(uint32_t ecuId) const;
    std::vector<std::unique_ptr<j2534::J2534Channel>> getUdsChannels(uint32_t ecuId) const;
    // One ISO15765 channel per UDS bus paired with its bus config, restricted to busNameFilter
    // when non-empty. The flow-control filter is keyed to ecuId on each bus.
    std::vector<std::pair<BusConfiguration, std::unique_ptr<j2534::J2534Channel>>>
    getUdsChannelsByBus(uint32_t ecuId, const std::string& busNameFilter = "") const;
    std::unique_ptr<j2534::J2534Channel> getChannelForEcu(uint32_t ecuId) const;

private:
    j2534::J2534& _j2534;
    CarPlatform _carPlatform;
    std::optional<uint32_t> _baudrateOverride;
    std::unique_ptr<j2534::J2534Channel> _bridgeChannel;
};

} // namespace common
