#pragma once

#include <array>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <j2534/J2534_v0404.h>

#include "DeviceInfo.hpp"
#include "CarPlatform.hpp"
#include "ConfigurationInfo.hpp"

namespace j2534 {
    class J2534;
    class J2534Channel;
} // namespace j2534

namespace common {

    std::wstring toWstring(const std::string& str);
    std::string toString(const std::wstring& str);

    uint32_t encodeBigEndian(uint8_t byte1);
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2);
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3);
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
    uint32_t encodeLittleEndian(uint8_t byte1);
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2);
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3);
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
    uint32_t encodeBigEndian(const std::vector<uint8_t>& data);
    uint32_t encodeLittleEndian(const std::vector<uint8_t>& data);

    std::vector<uint8_t> toVector(uint16_t value);
    std::vector<uint8_t> toVector(uint32_t value);
    std::string toHexString(const std::vector<uint8_t>& data);
    std::string formatHexBytesLower(const std::vector<uint8_t>& data);
    std::string j2534StatusToString(unsigned long status);

    std::string toLower(std::string data);

#ifdef UNICODE
    std::wstring toPlatformString(const std::string& str);
    std::string fromPlatformString(const std::wstring& str);
#else
    std::string toPlatformString(const std::string& str);
    std::string fromPlatformString(const std::string& str);
#endif

    std::vector<j2534::DeviceInfo> getAvailableDevices();

    std::vector<j2534::DeviceInfo> matchDevices(const std::vector<j2534::DeviceInfo>& devices,
                                                const std::string& deviceName);

    // Thrown by selectSingleDevice when device selection cannot resolve exactly one
    // device (none found, or an ambiguous match). A distinct type lets callers map
    // the "device/channel" failure class to a dedicated exit code without parsing
    // the message text.
    class DeviceSelectionError : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    // Returns the single matching device, or throws DeviceSelectionError with a
    // descriptive message when zero or more than one device matches the (possibly
    // empty) name substring.
    j2534::DeviceInfo selectSingleDevice(const std::vector<j2534::DeviceInfo>& devices,
                                         const std::string& deviceName);

    std::string trim(const std::string& input);

    // Parses whitespace/comma/semicolon/colon separated hex bytes (optional 0x prefix per
    // token), e.g. "22 F1 90" or "0x10,0x03". Also accepts compact even-length hex
    // without separators, e.g. "22F190". Throws std::runtime_error on malformed input.
    std::vector<uint8_t> parseHexBytes(const std::string& input);

    // Parses a single hex integer (optional 0x prefix), e.g. "7E0". Throws on malformed input.
    uint32_t parseHexU32(const std::string& input);

    // Strips the 4-byte CAN header from a UDS response frame, returning the SID+data
    // payload. Throws std::runtime_error if the frame is too short to contain a payload.
    std::vector<uint8_t> udsPayload(const std::vector<uint8_t>& response);

    std::unique_ptr<j2534::J2534Channel>
        openChannel(j2534::J2534& j2534, unsigned long ProtocolID, unsigned long Flags,
            unsigned long Baudrate, bool AdditionalConfiguration = false);

    std::unique_ptr<j2534::J2534Channel>
        openUDSChannel(j2534::J2534& j2534, unsigned long Baudrate, uint32_t canId = 0);

    bool prepareUDSChannel(const j2534::J2534Channel& channel, uint32_t canId);
    bool prepareTP20Channel(const j2534::J2534Channel& channel, uint32_t canId);

    std::unique_ptr<j2534::J2534Channel>
    openTP20Channel(j2534::J2534& j2534, unsigned long Baudrate, uint32_t canId = 0);

    std::unique_ptr<j2534::J2534Channel> openLowSpeedChannel(j2534::J2534& j2534,
        unsigned long Flags);

    std::unique_ptr<j2534::J2534Channel> openBridgeChannel(j2534::J2534& j2534);

    PASSTHRU_MSG makePassThruMsg(unsigned long ProtocolID, unsigned long Flags,
        const std::vector<uint8_t>& data);

    // Encodes a raw CAN frame (4-byte big-endian id header + data).
    std::vector<uint8_t> makeCanFrame(uint32_t canId, const std::vector<uint8_t>& data);

    // Extracts the CAN id from a received frame. Throws if the frame is shorter than 4 bytes.
    uint32_t canIdFromFrame(const PASSTHRU_MSG& msg);

    // Opens a raw CAN (or CAN_PS at 125k) channel for the given bus with a pass-all filter,
    // for sniffing/sending arbitrary frames. Honours an optional baudrate override.
    std::unique_ptr<j2534::J2534Channel> openRawCanChannel(j2534::J2534& j2534,
        const BusConfiguration& bus, std::optional<uint32_t> baudrateOverride = std::nullopt);

    std::vector<uint8_t> readMessageCheckAndGet(
        const j2534::J2534Channel& channel,
        const std::vector<uint8_t> msgId,
        const std::vector<uint8_t>& toCheck,
        size_t retryCount = 10);

    bool readMessageAndCheck(
        const j2534::J2534Channel& channel,
        const std::vector<uint8_t> msgId,
        const std::vector<uint8_t>& toCheck,
        size_t retryCount = 10);

    CarPlatform getPlatfromFromVIN(const std::string& vin);

    CarPlatform parseCarPlatform(std::string input);

    ConfigurationInfo getConfigurationInfoByCarPlatform(CarPlatform carPlatform);

    std::tuple<BusConfiguration, ECUInfo> getEcuInfoByEcuId(CarPlatform carPlatform, uint32_t ecuId);

    // Finds a configured bus by case-insensitive name. Throws std::runtime_error (with the
    // list of available buses) when the name is unknown or ambiguous.
    BusConfiguration getBusByName(CarPlatform carPlatform, const std::string& busName);

    // Validates that a CAN id fits the bus addressing width (11- or 29-bit). Throws otherwise.
    void ensureCanIdFitsBus(uint32_t canId, const BusConfiguration& bus);

    j2534::J2534Channel& getChannelByEcuId(CarPlatform carPlatform, uint32_t ecuId,
                                           const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels);

    size_t getChannelIndexByEcuId(CarPlatform carPlatform, uint32_t ecuId,
                                  const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels);

    std::vector<ConfigurationInfo> loadConfiguration(std::istream& input);
    std::vector<ConfigurationInfo> loadConfiguration(const std::string& input);

    void checkTP20Error(uint8_t requestId, const uint8_t* data, size_t dataSize);
    void checkUDSError(uint8_t requestId, const uint8_t* data, size_t dataSize);
    void checkD2Error(uint8_t ecuId, const std::vector<uint8_t>& requestId, const uint8_t* data, size_t dataSize);

    CarPlatform parseCarPlatform(std::string input);

    std::array<uint8_t, 5> getPinArray(uint64_t pin);

    void initLogger(const std::string& logFilename, bool debugLogging = false);

    uint16_t crc16(const uint8_t* data_p, size_t length);

} // namespace common
