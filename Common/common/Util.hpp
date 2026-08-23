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

    // Human-readable name of a J2534 protocol id (CAN, ISO15765, ..._PS variants and the
    // raw-CAN id space). Unknown ids render as "protocol=0x<hex>" so the value is never
    // silently dropped.
    std::string protocolName(unsigned long protocolId);

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
        openUDSChannel(j2534::J2534& j2534, unsigned long Baudrate, uint32_t canId = 0,
            unsigned long SamplePoint = 0);

    bool prepareUDSChannel(const j2534::J2534Channel& channel, uint32_t canId);

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

    void checkUDSError(uint8_t requestId, const uint8_t* data, size_t dataSize);

    CarPlatform parseCarPlatform(std::string input);

    std::array<uint8_t, 5> getPinArray(uint64_t pin);

    // Parses a SecurityAccess PIN given as 5 hex bytes, either "AA BB CC DD EE" or "AABBCCDDEE".
    // Returns the bytes big-endian (byte[0] is the most significant), ready to feed authorize.
    // Throws std::runtime_error on malformed input. A 5-byte key does not fit a uint32 --pin, so
    // this must never round-trip through getPinArray.
    std::array<uint8_t, 5> parseSecurityPin(const std::string& hex);

    // True when any of the 5 bytes is non-zero (i.e. a real key, not the "no key" sentinel).
    bool hasSecurityPin(const std::array<uint8_t, 5>& pin);

    // Interprets the 5-byte big-endian array as an unsigned integer. 5 bytes = 40 bits fits a
    // uint64, so this is the numeric starting point for the PIN bruteforcer.
    uint64_t securityPinToUint64(const std::array<uint8_t, 5>& pin);

    void initLogger(const std::string& logFilename, bool debugLogging = false);

    uint16_t crc16(const uint8_t* data_p, size_t length);

    // Incremental CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, non-reflected, no xorout): feed
    // the running value from crc16()/crc16Init() one byte at a time. Keeps the same result as
    // crc16() over the same byte sequence.
    uint16_t crc16Init();
    uint16_t crc16Update(uint16_t crc, uint8_t byte);

} // namespace common
