#pragma once

#include "VolvoDiagOptions.hpp"

#include <common/CarPlatform.hpp>
#include <common/ConfigurationInfo.hpp>
#include <common/DeviceInfo.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace j2534 {
    class J2534;
    class J2534Channel;
} // namespace j2534

namespace volvodiag {

// Throws if a CAN id does not fit the 29-bit addressing range.
void ensureCanIdFits(uint32_t canId, const std::string& argumentName);

// True for supported UDS (ISO15765) platforms.
bool isUdsPlatform(common::CarPlatform carPlatform);

// Throws when the platform is not a supported UDS platform.
void ensureUdsPlatform(common::CarPlatform carPlatform);

// Opens the selected J2534 device, including the DiCE name quirk.
std::unique_ptr<j2534::J2534> openDevice(const j2534::DeviceInfo& device);

// Resolves the request CAN id configured for an ECU id on the platform.
uint32_t ecuCanId(common::CarPlatform carPlatform, uint8_t ecuId);

// Uses the named CAN bus, or the ECU's bus by default.
common::BusConfiguration selectedRawCanBus(const RunOptions& options);

// Opens the OBD-II ISO15765 channel (0x7E0/0x7E8) on the powertrain bus.
std::unique_ptr<j2534::J2534Channel> openObdChannel(j2534::J2534& j2534, const RunOptions& options);

// Sends a UDS request and returns the raw response frame.
std::vector<uint8_t> processUds(const j2534::J2534Channel& channel, uint32_t canId,
                                const std::vector<uint8_t>& requestData, size_t timeoutMs = 1000);

// Sends a UDS request without waiting for a response.
void sendUdsNoWait(const j2534::J2534Channel& channel, uint32_t canId,
                   const std::vector<uint8_t>& requestData, size_t timeoutMs = 1000);

// Throws when the UDS payload does not start with the expected prefix.
void ensurePayloadPrefix(const std::vector<uint8_t>& payload, const std::vector<uint8_t>& expected);

// Expected outcome for one UDS request.
struct UdsExpectation {
    std::vector<uint8_t> prefix;   // required positive-response prefix (may be empty)
    bool positive{false};          // require a positive (non-0x7F) response
    std::optional<uint8_t> nrc;    // require this specific negative response code
    bool timeout{false};           // require no response (RX timeout)
};

// Sends a UDS request and checks its expected outcome.
std::vector<uint8_t> processUdsExpecting(const j2534::J2534Channel& channel, uint32_t canId,
                                         const std::vector<uint8_t>& request, size_t timeoutMs,
                                         const UdsExpectation& expect);

// Returns the selected ECU or all configured UDS ECUs.
std::vector<common::ECUInfo> getSelectedUdsEcus(const RunOptions& options);

// Reads raw CAN frames from a monitor-format CSV (time_ms,can_id,dlc,data).
std::vector<ReplayFrame> readReplayFrames(const std::string& path);

// Enables UDS tracing; an empty path disables it.
void setUdsTracePath(const std::string& path);

} // namespace volvodiag
