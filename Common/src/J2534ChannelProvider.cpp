#include "common/J2534ChannelProvider.hpp"

#include "common/CommonData.hpp"
#include "common/Util.hpp"

#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <easylogging++.h>

#include <stdexcept>

namespace common {

namespace {

std::unique_ptr<j2534::J2534Channel> createChannelByBusConf(j2534::J2534& j2534,
                                                            BusConfiguration bus,
                                                            uint32_t canId = 0,
                                                            std::optional<uint32_t> baudrateOverride = std::nullopt)
{
    if (baudrateOverride.has_value() && bus.baudrate != *baudrateOverride) {
        bus.baudrate = *baudrateOverride;
        // The configured sample point belongs to the configured baudrate, so drop it
        // and let the channel fall back to the baudrate-derived default.
        bus.samplePoint = 0;
    }
    // The fork is P3-only, so every bus is ISO15765. The CAN and ISO14230/TP20 branches were
    // unreachable once the non-P3 configurations were removed and are gone.
    return openUDSChannel(j2534, bus.baudrate, canId, bus.samplePoint);
}

}

J2534ChannelProvider::J2534ChannelProvider(j2534::J2534& j2534, CarPlatform carPlatform,
                                           std::optional<uint32_t> baudrateOverride)
    : _j2534{ j2534 }
    , _carPlatform{ carPlatform }
    , _baudrateOverride{ baudrateOverride }
{
    LOG(INFO) << "J2534ChannelProvider init platform=" << static_cast<int>(_carPlatform);
    if (_baudrateOverride.has_value()) {
        LOG(INFO) << "J2534ChannelProvider baudrate override=" << *_baudrateOverride;
    }
}

J2534ChannelProvider::~J2534ChannelProvider()
{
}

j2534::J2534& J2534ChannelProvider::getJ2534() const
{
    return _j2534;
}

std::vector<std::unique_ptr<j2534::J2534Channel>> J2534ChannelProvider::getAllChannels(uint32_t ecuId) const
{
    std::vector<std::unique_ptr<j2534::J2534Channel>> result;
    const auto conf{ getConfigurationInfoByCarPlatform(_carPlatform) };
    LOG(INFO) << "J2534ChannelProvider getAllChannels enter ecu=0x" << std::hex
        << ecuId << " buses=" << std::dec << conf.busInfo.size();
    for(const auto& bus: conf.busInfo) {
        uint32_t canId{};
        for(const auto& ecu: bus.ecuInfo) {
            if(ecu.ecuId == ecuId) {
                canId = ecu.canId;
            }
        }
        LOG(INFO) << "J2534ChannelProvider opening bus protocol=" << protocolName(bus.protocolId)
            << " baudrate=" << bus.baudrate << " canId=0x" << std::hex << canId;
        result.emplace_back(createChannelByBusConf(_j2534, bus, canId, _baudrateOverride));
        LOG(INFO) << "J2534ChannelProvider bus opened, total=" << std::dec << result.size();
    }
    LOG(INFO) << "J2534ChannelProvider getAllChannels exit count=" << result.size();
    return result;
}

std::vector<std::unique_ptr<j2534::J2534Channel>> J2534ChannelProvider::getUdsChannels(uint32_t ecuId) const
{
    std::vector<std::unique_ptr<j2534::J2534Channel>> result;
    const auto conf{ getConfigurationInfoByCarPlatform(_carPlatform) };
    LOG(INFO) << "J2534ChannelProvider getUdsChannels enter ecu=0x" << std::hex
        << ecuId << " buses=" << std::dec << conf.busInfo.size();
    for(const auto& bus: conf.busInfo) {
        if (bus.protocolId != ISO15765) {
            continue;
        }
        uint32_t canId{};
        for(const auto& ecu: bus.ecuInfo) {
            if(ecu.ecuId == ecuId) {
                canId = ecu.canId;
            }
        }
        LOG(INFO) << "J2534ChannelProvider opening UDS bus baudrate=" << bus.baudrate
            << " canId=0x" << std::hex << canId;
        result.emplace_back(createChannelByBusConf(_j2534, bus, canId, _baudrateOverride));
        LOG(INFO) << "J2534ChannelProvider UDS bus opened, total=" << std::dec << result.size();
    }
    LOG(INFO) << "J2534ChannelProvider getUdsChannels exit count=" << result.size();
    return result;
}

std::vector<std::pair<BusConfiguration, std::unique_ptr<j2534::J2534Channel>>>
J2534ChannelProvider::getUdsChannelsByBus(uint32_t ecuId, const std::string& busNameFilter) const
{
    std::vector<std::pair<BusConfiguration, std::unique_ptr<j2534::J2534Channel>>> result;
    const auto conf{ getConfigurationInfoByCarPlatform(_carPlatform) };
    for(const auto& bus: conf.busInfo) {
        if (bus.protocolId != ISO15765) {
            continue;
        }
        if (!busNameFilter.empty() && bus.name != busNameFilter) {
            continue;
        }
        uint32_t canId{};
        for(const auto& ecu: bus.ecuInfo) {
            if(ecu.ecuId == ecuId) {
                canId = ecu.canId;
            }
        }
        LOG(INFO) << "J2534ChannelProvider opening UDS bus=" << bus.name
            << " baudrate=" << bus.baudrate << " canId=0x" << std::hex << canId;
        result.emplace_back(bus, createChannelByBusConf(_j2534, bus, canId, _baudrateOverride));
    }
    return result;
}

std::unique_ptr<j2534::J2534Channel> J2534ChannelProvider::getChannelForEcu(uint32_t ecuId) const
{
    const auto ecuInfo{ getEcuInfoByEcuId(_carPlatform, ecuId) };
    auto channel = createChannelByBusConf(_j2534, std::get<0>(ecuInfo), std::get<1>(ecuInfo).canId, _baudrateOverride);
    LOG(INFO) << "J2534ChannelProvider getChannelForEcu ecu=0x" << std::hex << ecuId
        << " protocol=" << (channel ? protocolName(channel->getProtocolId()) : "none")
        << " baudrate=" << std::get<0>(ecuInfo).baudrate
        << " canId=0x" << std::hex << std::get<1>(ecuInfo).canId;
    return channel;
}

} // namespace common
