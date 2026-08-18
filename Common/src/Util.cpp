#include "common/Util.hpp"

#include "common/CommonData.hpp"
#include "common/BusConfiguration.hpp"
#include "common/protocols/UDSError.hpp"
#include "common/ECUInfo.hpp"

#include <j2534/J2534Channel.hpp>
#include <j2534/J2534_v0404.h>

#include <Windows.h>

#include <yaml-cpp/yaml.h>

#include <easylogging++.h>

#include <algorithm>
#include <cctype>
#include <codecvt>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <fstream>

namespace common {
namespace {

    // Mirrors log output to stderr on top of the file sink. Used under --debug so the
    // channel/CAN-id resolution and other INFO lines are visible on the console without
    // touching stdout (VolvoDiag keeps stdout for structured CSV output).
    class StderrLogDispatch : public el::LogDispatchCallback {
    protected:
        void handle(const el::LogDispatchData* data) override {
            const el::LogMessage* message = data->logMessage();
            std::cerr << message->logger()->logBuilder()->build(message,
                data->dispatchAction() == el::base::DispatchAction::NormalLog);
        }
    };

    constexpr const char* supportedCarPlatforms =
        "p3, p3_y413, p3_y283_iam, p3_y283_icm, p3_p313_icm, p3_p313_iam, "
        "p3_y555_iam, p3_y555_icm, p3_y312h_iam, p3_y312h_icm";

    CarPlatform parseCarPlatformImpl(std::string input, bool throwOnUnknown)
    {
        const auto originalInput = input;
        input = toLower(input);
        if ("p3" == input)
            return common::CarPlatform::P3;
        else if ("p3_y413" == input)
            return common::CarPlatform::P3_Y413;
        else if ("p3_y283_iam" == input)
            return common::CarPlatform::P3_Y283_IAM;
        else if ("p3_y283_icm" == input)
            return common::CarPlatform::P3_Y283_ICM;
        else if ("p3_p313_icm" == input)
            return common::CarPlatform::P3_P313_ICM;
        else if ("p3_p313_iam" == input)
            return common::CarPlatform::P3_P313_IAM;
        else if ("p3_y555_iam" == input)
            return common::CarPlatform::P3_Y555_IAM;
        else if ("p3_y555_icm" == input)
            return common::CarPlatform::P3_Y555_ICM;
        else if ("p3_y312h_iam" == input)
            return common::CarPlatform::P3_Y312H_IAM;
        else if ("p3_y312h_icm" == input)
            return common::CarPlatform::P3_Y312H_ICM;
        if (throwOnUnknown) {
            throw std::runtime_error("Unknown car platform \"" + originalInput + "\". Supported values: " + supportedCarPlatforms);
        }
        return common::CarPlatform::Undefined;
    }

} // namespace

    std::wstring toWstring(const std::string& str) {
        using convert_type = std::codecvt_utf8<wchar_t>;
        std::wstring_convert<convert_type, wchar_t> converter;
        return converter.from_bytes(str);
    }

    std::string toString(const std::wstring& str) {
        using convert_type = std::codecvt_utf8<wchar_t>;
        std::wstring_convert<convert_type, wchar_t> converter;
        return converter.to_bytes(str);
    }

    uint32_t encodeBigEndian(uint8_t byte1) { return byte1; }
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2) {
        return (static_cast<uint32_t>(byte1) << 8) | byte2;
    }
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3) {
        return (static_cast<uint32_t>(byte1) << 16) | (static_cast<uint32_t>(byte2) << 8) | byte3;
    }
    uint32_t encodeBigEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
        return (static_cast<uint32_t>(byte1) << 24) | (static_cast<uint32_t>(byte2) << 16)
            | (static_cast<uint32_t>(byte3) << 8) | byte4;
    }

    uint32_t encodeLittleEndian(uint8_t byte1) { return byte1; }
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2) {
        return byte1 | (static_cast<uint32_t>(byte2) << 8);
    }
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3) {
        return byte1 | (static_cast<uint32_t>(byte2) << 8) | (static_cast<uint32_t>(byte3) << 16);
    }
    uint32_t encodeLittleEndian(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
        return byte1 | (static_cast<uint32_t>(byte2) << 8) | (static_cast<uint32_t>(byte3) << 16)
            | (static_cast<uint32_t>(byte4) << 24);
    }

    uint32_t encodeBigEndian(const std::vector<uint8_t>& data) {
        uint32_t result{};
        const auto size = std::min(data.size(), sizeof(result));
        for (size_t i = 0; i < size; ++i) {
            const auto byte = data[i];
            result = (result << 8) | byte;
        }
        return result;
    }

    uint32_t encodeLittleEndian(const std::vector<uint8_t>& data) {
        uint32_t result{};
        const auto size = std::min(data.size(), sizeof(result));
        for (size_t i = 0; i < size; ++i) {
            result |= static_cast<uint32_t>(data[i]) << (i * 8);
        }
        return result;
    }

    std::vector<uint8_t> toVector(uint16_t value) {
        const uint8_t byte1 = (value & 0xFF00) >> 8;
        const uint8_t byte2 = (value & 0xFF);
        return { byte1, byte2 };
    }

    std::vector<uint8_t> toVector(uint32_t value) {
        const uint8_t byte1 = (value & 0xFF000000) >> 24;
        const uint8_t byte2 = (value & 0xFF0000) >> 16;
        const uint8_t byte3 = (value & 0xFF00) >> 8;
        const uint8_t byte4 = (value & 0xFF);
        return { byte1, byte2, byte3, byte4 };
    }

    std::string toHexString(const std::vector<uint8_t>& data)
    {
        std::stringstream ss;
        ss << std::uppercase << std::hex << std::setfill('0');
        for (const auto byte: data) {
            ss << std::setw(2) << static_cast<int>(byte) << ' ';
        }
        return ss.str();
    }

    std::string formatHexBytesLower(const std::vector<uint8_t>& data)
    {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < data.size(); ++i) {
            if (i > 0) {
                ss << ' ';
            }
            ss << std::setw(2) << static_cast<unsigned>(data[i]);
        }
        return ss.str();
    }

    std::string j2534StatusToString(unsigned long status)
    {
        switch (status) {
        case 0x00:
            return "STATUS_NOERROR";
        case 0x01:
            return "ERR_NOT_SUPPORTED";
        case 0x02:
            return "ERR_INVALID_CHANNEL_ID";
        case 0x03:
            return "ERR_INVALID_PROTOCOL_ID";
        case 0x04:
            return "ERR_NULL_PARAMETER";
        case 0x05:
            return "ERR_INVALID_IOCTL_VALUE";
        case 0x06:
            return "ERR_INVALID_FLAGS";
        case 0x07:
            return "ERR_FAILED";
        case 0x08:
            return "ERR_DEVICE_NOT_CONNECTED";
        case 0x09:
            return "ERR_TIMEOUT";
        case 0x0A:
            return "ERR_INVALID_MSG";
        case 0x0B:
            return "ERR_INVALID_TIME_INTERVAL";
        case 0x0C:
            return "ERR_EXCEEDED_LIMIT";
        case 0x0D:
            return "ERR_INVALID_MSG_ID";
        case 0x0E:
            return "ERR_DEVICE_IN_USE";
        case 0x0F:
            return "ERR_INVALID_IOCTL_ID";
        case 0x10:
            return "ERR_BUFFER_EMPTY";
        case 0x11:
            return "ERR_BUFFER_FULL";
        case 0x12:
            return "ERR_BUFFER_OVERFLOW";
        case 0x13:
            return "ERR_PIN_INVALID";
        case 0x14:
            return "ERR_CHANNEL_IN_USE";
        case 0x15:
            return "ERR_MSG_PROTOCOL_ID";
        case 0x16:
            return "ERR_INVALID_FILTER_ID";
        case 0x17:
            return "ERR_NO_FLOW_CONTROL";
        case 0x18:
            return "ERR_NOT_UNIQUE";
        case 0x19:
            return "ERR_INVALID_BAUDRATE";
        case 0x1A:
            return "ERR_INVALID_DEVICE_ID";
        default:
            std::stringstream ss;
            ss << "J2534_STATUS_0x" << std::uppercase << std::hex << status;
            return ss.str();
        }
    }

    std::string protocolName(unsigned long protocolId)
    {
        switch (protocolId) {
        case CAN:
            return "CAN";
        case ISO14230:
            return "ISO14230";
        case ISO15765:
            return "ISO15765";
        case CAN_XON_XOFF:
            return "CAN_XON_XOFF";
        case ISO14230_PS:
            return "ISO14230_PS";
        case CAN_PS:
            return "CAN_PS";
        case ISO15765_PS:
            return "ISO15765_PS";
        case SW_CAN_PS:
            return "SW_CAN_PS";
        case SW_ISO15765_PS:
            return "SW_ISO15765_PS";
        case CAN_XON_XOFF_PS:
            return "CAN_XON_XOFF_PS";
        default: {
            std::stringstream ss;
            ss << "protocol=0x" << std::uppercase << std::hex << protocolId;
            return ss.str();
        }
        }
    }

    std::string toLower(std::string data) {
        std::transform(data.begin(), data.end(), data.begin(),
            [](unsigned char c) { return std::tolower(c); });
        return data;
    }

    std::vector<j2534::DeviceInfo> matchDevices(const std::vector<j2534::DeviceInfo>& devices,
                                                const std::string& deviceName)
    {
        std::vector<j2534::DeviceInfo> result;
        for (const auto& device : devices) {
            if (deviceName.empty() || device.deviceName.find(deviceName) != std::string::npos) {
                result.push_back(device);
            }
        }
        return result;
    }

    j2534::DeviceInfo selectSingleDevice(const std::vector<j2534::DeviceInfo>& devices,
                                         const std::string& deviceName)
    {
        const auto matched = matchDevices(devices, deviceName);
        if (matched.empty()) {
            throw DeviceSelectionError(deviceName.empty()
                ? "No J2534 devices found."
                : "No J2534 devices matched --device \"" + deviceName + "\".");
        }
        if (matched.size() > 1) {
            std::stringstream ss;
            ss << (deviceName.empty()
                ? "Multiple J2534 devices found. Specify --device."
                : "Multiple J2534 devices matched --device \"" + deviceName + "\".");
            for (const auto& device : matched) {
                ss << "\n    " << device.deviceName;
            }
            throw DeviceSelectionError(ss.str());
        }
        return matched.front();
    }

    std::string trim(const std::string& input)
    {
        const auto begin = std::find_if_not(input.cbegin(), input.cend(), [](unsigned char ch) {
            return std::isspace(ch);
        });
        const auto end = std::find_if_not(input.crbegin(), input.crend(), [](unsigned char ch) {
            return std::isspace(ch);
        }).base();
        return begin < end ? std::string(begin, end) : std::string{};
    }

    std::vector<uint8_t> parseHexBytes(const std::string& input)
    {
        std::string normalized;
        normalized.reserve(input.size());
        bool hasSeparator = false;
        for (const auto ch : input) {
            if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',' || ch == ';' || ch == ':') {
                hasSeparator = true;
            }
            normalized.push_back((ch == ',' || ch == ';' || ch == ':') ? ' ' : ch);
        }

        const auto trimmed = trim(normalized);
        if (!hasSeparator && trimmed.size() > 2) {
            std::string compact = trimmed;
            if (compact.size() > 2 && compact[0] == '0'
                && (compact[1] == 'x' || compact[1] == 'X')) {
                compact = compact.substr(2);
            }
            if ((compact.size() % 2) != 0) {
                throw std::runtime_error("Compact hex payload must contain an even number of digits: " + input);
            }
            std::vector<uint8_t> result;
            result.reserve(compact.size() / 2);
            for (size_t offset = 0; offset < compact.size(); offset += 2) {
                const auto token = compact.substr(offset, 2);
                size_t processedChars = 0;
                unsigned long value = 0;
                try {
                    value = std::stoul(token, &processedChars, 16);
                }
                catch (const std::exception&) {
                    throw std::runtime_error("Invalid hex byte: " + token);
                }
                if (processedChars != token.size() || value > 0xFF) {
                    throw std::runtime_error("Invalid hex byte: " + token);
                }
                result.push_back(static_cast<uint8_t>(value));
            }
            if (result.empty()) {
                throw std::runtime_error("Hex payload is empty");
            }
            return result;
        }

        std::stringstream ss{normalized};
        std::string token;
        std::vector<uint8_t> result;
        while (ss >> token) {
            if (token.size() > 2 && token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) {
                token = token.substr(2);
            }
            if (token.empty() || token.size() > 2) {
                throw std::runtime_error("Invalid hex byte: " + token);
            }
            size_t processedChars = 0;
            unsigned long value = 0;
            try {
                value = std::stoul(token, &processedChars, 16);
            }
            catch (const std::exception&) {
                throw std::runtime_error("Invalid hex byte: " + token);
            }
            if (processedChars != token.size() || value > 0xFF) {
                throw std::runtime_error("Invalid hex byte: " + token);
            }
            result.push_back(static_cast<uint8_t>(value));
        }
        if (result.empty()) {
            throw std::runtime_error("Hex payload is empty");
        }
        return result;
    }

    uint32_t parseHexU32(const std::string& input)
    {
        const auto trimmed = trim(input);
        size_t processedChars = 0;
        unsigned long value = 0;
        try {
            value = std::stoul(trimmed, &processedChars, 16);
        }
        catch (const std::exception&) {
            // stoul throws on non-hex or overflow before the checks below run.
            throw std::runtime_error("Invalid hex value: " + input);
        }
        if (trimmed.empty() || processedChars != trimmed.size()) {
            throw std::runtime_error("Invalid hex value: " + input);
        }
        return static_cast<uint32_t>(value);
    }

    std::vector<uint8_t> udsPayload(const std::vector<uint8_t>& response)
    {
        if (response.size() < 5) {
            throw std::runtime_error("Short UDS response");
        }
        return {response.cbegin() + 4, response.cend()};
    }

#ifdef UNICODE
    std::wstring toPlatformString(const std::string& str) { return toWstring(str); }

    std::string fromPlatformString(const std::wstring& str) {
        return toString(str);
    }
#else
    std::string toPlatformString(const std::string& str) { return str; }

    std::string fromPlatformString(const std::string& str) { return str; }
#endif

    bool queryRegistryString(HKEY key, const wchar_t* valueName, std::wstring& value)
    {
        DWORD type = 0;
        DWORD byteCount = 0;
        if (RegQueryValueExW(key, valueName, nullptr, &type, nullptr, &byteCount) != ERROR_SUCCESS
            || (type != REG_SZ && type != REG_EXPAND_SZ) || byteCount == 0) {
            return false;
        }
        std::vector<wchar_t> buffer(byteCount / sizeof(wchar_t) + 1, L'\0');
        if (RegQueryValueExW(key, valueName, nullptr, &type,
                             reinterpret_cast<LPBYTE>(buffer.data()), &byteCount) != ERROR_SUCCESS) {
            return false;
        }
        value.assign(buffer.data());
        return !value.empty();
    }

    struct RegistryHandle {
        HKEY value{};
        RegistryHandle() = default;
        explicit RegistryHandle(HKEY handle) : value{handle} {}
        RegistryHandle(const RegistryHandle&) = delete;
        RegistryHandle& operator=(const RegistryHandle&) = delete;
        RegistryHandle(RegistryHandle&&) = delete;
        RegistryHandle& operator=(RegistryHandle&&) = delete;
        ~RegistryHandle() { if (value) RegCloseKey(value); }
        operator HKEY() const { return value; }
    };

    void appendJ2534Device(std::vector<j2534::DeviceInfo>& result,
                           std::string libraryPath, std::string deviceName)
    {
        const auto duplicate = std::find_if(result.cbegin(), result.cend(),
            [&](const auto& device) {
                return device.libraryName == libraryPath && device.deviceName == deviceName;
            });
        if (duplicate == result.cend()) {
            result.push_back({std::move(libraryPath), std::move(deviceName)});
        }
    }

    void collectJ2534RegistryView(REGSAM view, std::vector<j2534::DeviceInfo>& result)
    {
        constexpr wchar_t rootPath[] = L"Software\\PassThruSupport.04.04";
        HKEY root = nullptr;
        if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, rootPath, 0, KEY_READ | view, &root) != ERROR_SUCCESS) {
            return;
        }
        const RegistryHandle closeRoot{root};
        for (DWORD index = 0;; ++index) {
            wchar_t vendorName[256]{};
            DWORD vendorNameLength = static_cast<DWORD>(sizeof(vendorName) / sizeof(vendorName[0]));
            if (RegEnumKeyExW(root, index, vendorName, &vendorNameLength, nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS) {
                break;
            }
            std::wstring vendorPath = std::wstring(rootPath) + L"\\" + vendorName;
            HKEY vendor = nullptr;
            if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, vendorPath.c_str(), 0, KEY_READ | view, &vendor) != ERROR_SUCCESS) {
                continue;
            }
            const RegistryHandle closeVendor{vendor};
            std::wstring libraryPath;
            if (!queryRegistryString(vendor, L"FunctionLibrary", libraryPath)) {
                continue;
            }
            // Standard PassThruSupport layout keeps Name/FunctionLibrary directly on this key
            // (handled by the fallback below). The nested loop covers rare vendors that group
            // devices in subkeys; those are assumed to share this key's FunctionLibrary.
            bool foundDevice = false;
            for (DWORD deviceIndex = 0;; ++deviceIndex) {
                wchar_t deviceKeyName[256]{};
                DWORD deviceKeyNameLength = static_cast<DWORD>(sizeof(deviceKeyName) / sizeof(deviceKeyName[0]));
                if (RegEnumKeyExW(vendor, deviceIndex, deviceKeyName, &deviceKeyNameLength,
                                  nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS) {
                    break;
                }
                std::wstring devicePath = vendorPath + L"\\" + deviceKeyName;
                HKEY device = nullptr;
                if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, devicePath.c_str(), 0, KEY_READ | view, &device) != ERROR_SUCCESS) {
                    continue;
                }
                const RegistryHandle closeDevice{device};
                std::wstring deviceName;
                if (queryRegistryString(device, L"Name", deviceName)) {
                    appendJ2534Device(result, toString(libraryPath), toString(deviceName));
                    foundDevice = true;
                }
            }
            if (!foundDevice) {
                std::wstring deviceName;
                if (queryRegistryString(vendor, L"Name", deviceName)) {
                    appendJ2534Device(result, toString(libraryPath), toString(deviceName));
                }
            }
        }
    }

    std::vector<j2534::DeviceInfo> getAvailableDevices() {
        std::vector<j2534::DeviceInfo> result;
        // Enumerate both registry views: J2534 drivers on x64 typically register only under
        // the 32-bit WOW6432Node. On a 32-bit build the KEY_WOW64_* flags are ignored and both
        // passes hit the same view, so appendJ2534Device's dedup is functional, not cosmetic.
        collectJ2534RegistryView(KEY_WOW64_64KEY, result);
        collectJ2534RegistryView(KEY_WOW64_32KEY, result);
        return result;
    }

    PASSTHRU_MSG makePassThruMsg(unsigned long ProtocolID,
        unsigned long Flags,
        const std::vector<unsigned char>& data) {
        PASSTHRU_MSG result;
        result.ProtocolID = ProtocolID;
        result.RxStatus = 0;
        result.TxFlags = Flags;
        result.Timestamp = 0;
        result.ExtraDataIndex = 0;
        result.DataSize = data.size();
        std::copy(data.begin(), data.end(), result.Data);
        return result;
    }

    static std::vector<PASSTHRU_MSG>
        makePassThruMsgs(unsigned long ProtocolID, unsigned long Flags,
            const std::vector<std::vector<unsigned char>>& data) {
        std::vector<PASSTHRU_MSG> result;
        for (const auto msgData : data) {
            PASSTHRU_MSG msg;
            msg.ProtocolID = ProtocolID;
            msg.RxStatus = 0;
            msg.TxFlags = Flags;
            msg.Timestamp = 0;
            msg.ExtraDataIndex = 0;
            msg.DataSize = msgData.size();
            std::copy(msgData.begin(), msgData.end(), msg.Data);
            result.emplace_back(std::move(msg));
        }
        return result;
    }


    // Fallback used when the bus configuration carries no explicit sample point.
    // Matches the shipped vehicle configuration everywhere except P1 CAN MS (which wants 60),
    // so prefer the configured value whenever the caller has one.
    static unsigned long defaultSamplePoint(unsigned long baudrate)
    {
        return baudrate == 500000 ? 80 : 68;
    }

    static void setupChannelParameters(j2534::J2534Channel& channel, unsigned long samplePoint = 0)
    {
        const auto effectiveSamplePoint = samplePoint != 0
            ? samplePoint
            : defaultSamplePoint(channel.getBaudrate());
        std::vector<SCONFIG> config(3);
        config[0].Parameter = DATA_RATE;
        config[0].Value = channel.getBaudrate();
        config[1].Parameter = LOOPBACK;
        config[1].Value = 0;
        config[2].Parameter = BIT_SAMPLE_POINT;
        config[2].Value = effectiveSamplePoint;
        channel.setConfig(config);
    }

    static void setupChannelIso15765Parameters(j2534::J2534Channel& channel)
    {
        if (channel.getProtocolId() != ISO15765 && channel.getProtocolId() != ISO15765_PS) {
            return;
        }
        // The adapter used to be left entirely on driver defaults. Ask for no inter-frame delay
        // on our own consecutive frames (STMIN=0) and no block-size cap (BS=0, send all CF after
        // one FC) so we never stall on the tester side; the ECU's own FC still governs the
        // response timing. Failing to set these used to be silent, so log the result.
        std::vector<SCONFIG> config(2);
        config[0].Parameter = ISO15765_STMIN;
        config[0].Value = 0;
        config[1].Parameter = ISO15765_BS;
        config[1].Value = 0;
        const auto status = channel.setConfig(config);
        LOG(INFO) << "ISO15765 channel params: STMIN=0 BS=0 status=" << j2534StatusToString(status);
    }

    static void setupChannelPins(j2534::J2534Channel& channel)
    {
        if (channel.getProtocolId() == ISO15765_PS) {
            std::vector<SCONFIG> config(1);
            config[0].Parameter = J1962_PINS;
            config[0].Value = 0x030B;
            channel.setConfig(config);
        }
    }

    std::unique_ptr<j2534::J2534Channel>
        openUDSChannel(j2534::J2534& j2534, unsigned long baudrate, uint32_t canId,
            unsigned long samplePoint) {

        std::unique_ptr<j2534::J2534Channel> channel;
        const std::vector<unsigned long> SupportedProtocols = { ISO15765_PS, ISO15765 };
        for (const auto& protocolId : SupportedProtocols) {
            if(protocolId == ISO15765_PS && baudrate != 125000) {
                continue;
            }
            unsigned long flags = 0;
            unsigned long txFlags = ISO15765_FRAME_PAD;
            if (protocolId == ISO15765 && baudrate == 125000)
                flags |= PHYSICAL_CHANNEL;
            try {
                channel = std::make_unique<j2534::J2534Channel>(
                    j2534, protocolId, flags, baudrate, txFlags);
            }
            catch (...) {
                continue;
            }

            setupChannelParameters(*channel, samplePoint);
            setupChannelIso15765Parameters(*channel);
            setupChannelPins(*channel);

            if (canId) {
                if (!prepareUDSChannel(*channel, canId)) {
                    LOG(ERROR) << "Failed to set up UDS flow-control filter for canId=0x"
                        << std::hex << canId;
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            // The fallback loop tries ISO15765_PS first, then ISO15765 (with PHYSICAL_CHANNEL at
            // 125k). Say which one actually won, so a bench log can distinguish the two instead of
            // guessing from the bus config.
            LOG(INFO) << "Opened UDS channel protocol=" << protocolName(channel->getProtocolId())
                      << " flags=0x" << std::hex << channel->getFlags() << " baudrate=" << std::dec << baudrate;

            return std::move(channel);
        }
        LOG(ERROR) << "Failed to open UDS channel: no usable protocol/filter combination";
        return {};
    }

    bool prepareUDSChannel(const j2534::J2534Channel& channel, uint32_t canId) {
        const uint32_t responseCanId = canId + 0x8;
        unsigned long msgId;
        PASSTHRU_MSG maskMsg =
            makePassThruMsg(channel.getProtocolId(), channel.getTxFlags(), { 0xFF, 0xFF, 0xFF, 0xFF });
        PASSTHRU_MSG patternMsg =
            makePassThruMsg(channel.getProtocolId(), channel.getTxFlags(), toVector(responseCanId));
        PASSTHRU_MSG flowMsg =
            makePassThruMsg(channel.getProtocolId(), channel.getTxFlags(), toVector(canId));
        return channel.startMsgFilter(FLOW_CONTROL_FILTER, &maskMsg, &patternMsg, &flowMsg, msgId) == STATUS_NOERROR;
    }


    static void startRawCanPassAllFilter(const j2534::J2534Channel& channel)
    {
        unsigned long msgId = 0;
        auto maskMsg = makePassThruMsg(channel.getProtocolId(), channel.getTxFlags(), { 0x00, 0x00, 0x00, 0x00 });
        auto patternMsg = makePassThruMsg(channel.getProtocolId(), channel.getTxFlags(), { 0x00, 0x00, 0x00, 0x00 });
        const auto status = channel.startMsgFilter(PASS_FILTER, &maskMsg, &patternMsg, nullptr, msgId);
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to start raw CAN pass-all filter: " + j2534StatusToString(status));
        }
    }

    std::unique_ptr<j2534::J2534Channel> openRawCanChannel(j2534::J2534& j2534,
        const BusConfiguration& bus, std::optional<uint32_t> baudrateOverride)
    {
        const auto baudrate = baudrateOverride.value_or(bus.baudrate);
        // The configured sample point only applies at the configured baudrate.
        const unsigned long samplePoint = baudrate == bus.baudrate ? bus.samplePoint : 0;
        const unsigned long flags = bus.canIdBitSize == 29 ? CAN_29BIT_ID : 0;

        std::vector<unsigned long> protocolIds{ CAN };
        if (baudrate == 125000) {
            protocolIds.insert(protocolIds.begin(), CAN_PS);
        }

        for (const auto protocolId : protocolIds) {
            auto localFlags = flags;
            if (protocolId == CAN && baudrate == 125000) {
                localFlags |= PHYSICAL_CHANNEL;
            }
            try {
                auto channel = std::make_unique<j2534::J2534Channel>(j2534, protocolId, localFlags, baudrate, flags);
                setupChannelParameters(*channel, samplePoint);
                if (protocolId == CAN_PS) {
                    std::vector<SCONFIG> pinsConfig(1);
                    pinsConfig[0].Parameter = J1962_PINS;
                    pinsConfig[0].Value = 0x030B;
                    const auto pinStatus = channel->setConfig(pinsConfig);
                    if (pinStatus != STATUS_NOERROR) {
                        throw std::runtime_error("Failed to configure raw CAN pins: " + j2534StatusToString(pinStatus));
                    }
                }
                startRawCanPassAllFilter(*channel);
                LOG(INFO) << "Opened raw CAN channel bus=" << bus.name
                    << " protocol=" << protocolName(channel->getProtocolId())
                    << " baudrate=" << baudrate
                    << " flags=0x" << std::hex << localFlags;
                return channel;
            }
            catch (const std::exception& ex) {
                LOG(WARNING) << "Failed to open raw CAN channel protocol=" << protocolName(protocolId)
                    << " baudrate=" << std::dec << baudrate << ": " << ex.what();
            }
        }

        throw std::runtime_error("Failed to open raw CAN channel");
    }

    std::vector<uint8_t> makeCanFrame(uint32_t canId, const std::vector<uint8_t>& data)
    {
        std::vector<uint8_t> frame;
        frame.reserve(4 + data.size());
        frame.push_back(static_cast<uint8_t>((canId >> 24) & 0xFF));
        frame.push_back(static_cast<uint8_t>((canId >> 16) & 0xFF));
        frame.push_back(static_cast<uint8_t>((canId >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(canId & 0xFF));
        frame.insert(frame.end(), data.cbegin(), data.cend());
        return frame;
    }

    uint32_t canIdFromFrame(const PASSTHRU_MSG& msg)
    {
        if (msg.DataSize < 4) {
            throw std::runtime_error("Short CAN frame");
        }
        return (static_cast<uint32_t>(msg.Data[0]) << 24)
            | (static_cast<uint32_t>(msg.Data[1]) << 16)
            | (static_cast<uint32_t>(msg.Data[2]) << 8)
            | static_cast<uint32_t>(msg.Data[3]);
    }

    static bool readCheckAndGetImpl(
        const j2534::J2534Channel& channel,
        const std::vector<uint8_t> msgId,
        const std::vector<uint8_t>& toCheck,
        std::vector<uint8_t>& result,
        size_t retryCount) {
        for (size_t i = 0; i < retryCount; ++i) {
            std::vector<PASSTHRU_MSG> read_msgs;
            read_msgs.resize(1);
            if (channel.readMsgs(read_msgs, 10000) != STATUS_NOERROR || read_msgs.empty())
            {
                continue;
            }
            const auto& msg = read_msgs[0];
            if (msg.DataSize < msgId.size() + 4) {
                continue;
            }
            uint32_t checkOffset = 4;
            const auto areMessagesEqual = std::equal(msgId.cbegin(), msgId.cend(), msg.Data + checkOffset);
            if (!areMessagesEqual) {
                continue;
            }
            checkOffset += msgId.size();
            const auto areResultEqual = std::equal(toCheck.cbegin(), toCheck.cend(), msg.Data + checkOffset);
            if (!areResultEqual) {
                return false;
            }
            else {
                checkOffset += toCheck.size();
                result.insert(result.end(), msg.Data + checkOffset, msg.Data + msg.DataSize);
                return true;
            }
        }
        return false;
    }

    std::vector<uint8_t> readMessageCheckAndGet(
        const j2534::J2534Channel& channel,
        const std::vector<uint8_t> msgId,
        const std::vector<uint8_t>& toCheck,
        size_t retryCount) {
        std::vector<uint8_t> result;
        readCheckAndGetImpl(channel, msgId, toCheck, result, retryCount);
        return result;
    }

    bool readMessageAndCheck(
        const j2534::J2534Channel& channel,
        const std::vector<uint8_t> msgId,
        const std::vector<uint8_t>& toCheck,
        size_t retryCount)
    {
        std::vector<uint8_t> result;
        return readCheckAndGetImpl(channel, msgId, toCheck, result, retryCount);
    }

    CarPlatform getPlatfromFromVIN(const std::string& vin)
    {
        // The fork is P3-only. A Volvo P3 VIN starts with YV1 and has A or B in position 4
        // (world manufacturer's designator). Anything else is not a platform we support, so
        // return Undefined rather than guess a P1/P2/Ford/Haval that cannot be used.
        const std::string volvoPrefix = "YV1";
        if (vin.find(volvoPrefix) == 0)
        {
            switch (vin[3])
            {
            case 'A':
            case 'B':
                return CarPlatform::P3;
            }
        }
        return CarPlatform::Undefined;
    }

    CarPlatform parseCarPlatform(std::string input)
    {
        return parseCarPlatformImpl(std::move(input), true);
    }

    static std::string getCarPlatformName(CarPlatform carPlatform)
    {
        // Returns the cliName (the --platform value) for the enum. getConfigurationInfoByCarPlatform
        // matches configurations on this, replacing the old textual match against the display name.
        switch (carPlatform) {
        case CarPlatform::P3:
            return "p3";
        case CarPlatform::P3_Y413:
            return "p3_y413";
        case CarPlatform::P3_Y283_IAM:
            return "p3_y283_iam";
        case CarPlatform::P3_Y283_ICM:
            return "p3_y283_icm";
        case CarPlatform::P3_P313_ICM:
            return "p3_p313_icm";
        case CarPlatform::P3_P313_IAM:
            return "p3_p313_iam";
        case CarPlatform::P3_Y555_IAM:
            return "p3_y555_iam";
        case CarPlatform::P3_Y555_ICM:
            return "p3_y555_icm";
        case CarPlatform::P3_Y312H_IAM:
            return "p3_y312h_iam";
        case CarPlatform::P3_Y312H_ICM:
            return "p3_y312h_icm";
        }
        return {};
    }

    const std::vector<ConfigurationInfo>& staticConfiguration()
    {
        static const std::vector<ConfigurationInfo> configuration(loadConfiguration(CommonData::commonConfiguration));
        return configuration;
    }

    ConfigurationInfo getConfigurationInfoByCarPlatform(CarPlatform carPlatform)
    {
        const auto& configurationInfo{staticConfiguration()};
        const auto platformName = getCarPlatformName(carPlatform);
        const auto confIt = std::find_if(configurationInfo.cbegin(), configurationInfo.cend(), [&platformName](const ConfigurationInfo& info) {
            return info.cliName == platformName;
        });
        if (confIt == configurationInfo.cend()) {
            throw std::runtime_error("Unknown platform " + platformName);
        }
        return *confIt;
    }

    std::tuple<BusConfiguration, ECUInfo> getEcuInfoByEcuId(CarPlatform carPlatform, uint32_t ecuId)
    {
        const auto conf = getConfigurationInfoByCarPlatform(carPlatform);
        for (const auto& busInfo : conf.busInfo) {
            for (const auto& ecuInfo : busInfo.ecuInfo) {
                if (ecuInfo.ecuId == ecuId) {
                    return { busInfo, ecuInfo };
                }
            }
        }
        const auto platformName = getCarPlatformName(carPlatform);
        throw std::runtime_error((std::stringstream() << "Can't find ECU with id = " << ecuId << ", for platform = " << platformName).str());
    }

    BusConfiguration getBusByName(CarPlatform carPlatform, const std::string& busName)
    {
        const auto configuration = getConfigurationInfoByCarPlatform(carPlatform);
        const auto normalizedBusName = toLower(trim(busName));
        std::vector<BusConfiguration> matches;
        for (const auto& bus : configuration.busInfo) {
            if (toLower(bus.name) == normalizedBusName) {
                matches.push_back(bus);
            }
        }
        if (matches.empty()) {
            std::stringstream ss;
            ss << "Can't find bus \"" << busName << "\". Available buses:";
            for (const auto& bus : configuration.busInfo) {
                ss << "\n    " << bus.name;
            }
            throw std::runtime_error(ss.str());
        }
        if (matches.size() > 1) {
            throw std::runtime_error("Bus name is ambiguous: " + busName);
        }
        return matches.front();
    }

    void ensureCanIdFitsBus(uint32_t canId, const BusConfiguration& bus)
    {
        const uint32_t maxId = bus.canIdBitSize == 29 ? 0x1FFFFFFFu : 0x7FFu;
        if (canId > maxId) {
            std::stringstream ss;
            ss << "CAN id 0x" << std::hex << canId << " does not fit bus \"" << bus.name
               << "\" (" << std::dec << bus.canIdBitSize << "-bit)";
            throw std::runtime_error(ss.str());
        }
    }

    size_t getChannelIndexByEcuId(CarPlatform carPlatform, uint32_t ecuId,
        const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
    {
        const auto [busInfo, ecuInfo] = getEcuInfoByEcuId(carPlatform, ecuId);
        for(size_t i = 0; i < channels.size(); ++i) {
            if (busInfo.baudrate == channels[i]->getBaudrate()) {
                return i;
            }
        }
        // A baudrate override collapses every channel onto the same speed, so the
        // configured bus baudrate no longer identifies a channel. When only one
        // channel is open there is no ambiguity; otherwise fail loudly.
        if (channels.size() == 1) {
            return 0;
        }
        throw std::runtime_error((std::stringstream() << "Can't find opened channel with baudrate = " << busInfo.baudrate
            << " (a baudrate override may have made channels indistinguishable by speed)").str());
    }

    j2534::J2534Channel& getChannelByEcuId(CarPlatform carPlatform, uint32_t ecuId,
        const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
    {
        return *channels[getChannelIndexByEcuId(carPlatform, ecuId, channels)];
    }

    static std::string getNonEmptyHexIntString(const std::string& input)
    {
        return input.empty() ? "0" : input;
    }

    static CompressionType getEcuCompression(const YAML::Node& node)
    {
        const std::string tag = "CompressionType";
        if(!node[tag].IsDefined()) {
            return CompressionType::None;
        }
        const auto compressionType = toLower(node[tag].as<std::string>());
        if(compressionType == "bosch") {
            return CompressionType::Bosch;
        }
        else if(compressionType == "lzss") {
            return CompressionType::LZSS;
        }
        return CompressionType::None;
    }

    static EncryptionType getEcuEncryption(const YAML::Node& node)
    {
        const std::string tag = "EncryptionType";
        if(!node[tag].IsDefined()) {
            return EncryptionType::None;
        }
        const auto compressionType = toLower(node[tag].as<std::string>());
        if(compressionType == "xor") {
            return EncryptionType::XOR;
        }
        else if(compressionType == "aes") {
            return EncryptionType::AES;
        }
        return EncryptionType::None;
    }

    static bool getFlag(const YAML::Node& node, const std::string& tag)
    {
        if (!node[tag].IsDefined()) {
            return false;
        }
        // v1 wrote flags as integers (SBLInPBL: 1), v2 as booleans (sblInPbl: true). Accept both.
        if (node[tag].as<uint32_t>(0) != 0) {
            return true;
        }
        try {
            return node[tag].as<bool>();
        }
        catch (...) {
            return false;
        }
    }

    static ECUInfo processEcuNode(const YAML::Node& node)
    {
        ECUInfo ecuInfo;
        ecuInfo.name = node["Name"].as<std::string>();
        ecuInfo.ecuId = std::stoi(node["Address"].as<std::string>(), 0, 16);
        ecuInfo.canId = std::stoi(getNonEmptyHexIntString(node["CANIdentifier"].as<std::string>("")), 0, 16);
        ecuInfo.compressionType = getEcuCompression(node);
        ecuInfo.encryptionType = getEcuEncryption(node);
        ecuInfo.sblInPBL = getFlag(node, "SBLInPBL");
        ecuInfo.swdlIssue = getFlag(node, "SwdlIssue");
        ecuInfo.masterEcu = getFlag(node, "MasterECU");
        return ecuInfo;
    }

    static GatewayEndpoint processEndpointNode(const YAML::Node& node, const std::string& tag)
    {
        GatewayEndpoint endpoint;
        if (!node[tag].IsDefined()) {
            return endpoint;
        }
        const auto& entry = node[tag];
        endpoint.ecuAddress = std::stoi(getNonEmptyHexIntString(entry["ECUAddress"].as<std::string>("")), 0, 16);
        endpoint.canId = std::stoi(getNonEmptyHexIntString(entry["CANIdentifier"].as<std::string>("")), 0, 16);
        endpoint.name = entry["Name"].as<std::string>("");
        return endpoint;
    }

    static uint32_t getCanProtocol(const std::string& input)
    {
        if (input == "CAN")
            return CAN;
        else if (input == "15765-2")
            return ISO15765;
        else if (input == "14230-3")
            return ISO14230;
        return CAN;
    }

    static ECUInfo processEcuNodeV2(const YAML::Node& node)
    {
        ECUInfo ecuInfo;
        ecuInfo.name = node["name"].as<std::string>();
        ecuInfo.ecuId = std::stoi(getNonEmptyHexIntString(node["address"].as<std::string>("")), 0, 16);
        ecuInfo.canId = std::stoi(getNonEmptyHexIntString(node["canId"].as<std::string>("")), 0, 16);
        ecuInfo.compressionType = getEcuCompression(node);
        ecuInfo.encryptionType = getEcuEncryption(node);
        ecuInfo.sblInPBL = getFlag(node, "sblInPbl");
        ecuInfo.swdlIssue = getFlag(node, "swdlIssue");
        ecuInfo.masterEcu = getFlag(node, "masterEcu");
        if (node["securityPin"].IsDefined()) {
            ecuInfo.securityPin = parseSecurityPin(node["securityPin"].as<std::string>());
        }
        return ecuInfo;
    }

    static GatewayEndpoint processEndpointNodeV2(const YAML::Node& node, const std::string& tag)
    {
        GatewayEndpoint endpoint;
        if (!node[tag].IsDefined()) {
            return endpoint;
        }
        const auto& entry = node[tag];
        endpoint.ecuAddress = std::stoi(getNonEmptyHexIntString(entry["ecuAddress"].as<std::string>("")), 0, 16);
        endpoint.canId = std::stoi(getNonEmptyHexIntString(entry["canId"].as<std::string>("")), 0, 16);
        endpoint.name = entry["name"].as<std::string>("");
        return endpoint;
    }

    static std::vector<ConfigurationInfo> loadConfigurationImpl(const YAML::Node& node)
    {
        // v2 schema: a shared ECU pool plus defaults and configurations that reference pool ids.
        if (node["version"].IsDefined() && node["version"].as<int>() == 2) {
            std::vector<ConfigurationInfo> result;

            const auto& defaultsNode = node["defaults"];
            const auto& defaultsBus = defaultsNode["bus"];
            const auto& defaultsSwdl = defaultsNode["swdl"];

            std::map<std::string, ECUInfo> pool;
            for (const auto& kv : node["ecus"]) {
                pool.emplace(kv.first.as<std::string>(), processEcuNodeV2(kv.second));
            }

            for (const auto& confNode : node["configurations"]) {
                ConfigurationInfo info;
                info.name = confNode["name"].as<std::string>();
                info.cliName = confNode["cliName"].as<std::string>();
                if (info.cliName.empty()) {
                    throw std::runtime_error("Configuration \"" + info.name + "\" is missing a required unique cliName");
                }
                info.gatewaySubTester = processEndpointNodeV2(confNode, "gatewaySubTester");
                info.subTester = processEndpointNodeV2(confNode, "subTester");

                for (const auto& bus : confNode["buses"]) {
                    BusConfiguration busConf;
                    const auto busName = bus["name"].as<std::string>();
                    busConf.name = busName;
                    const auto& busDefault = defaultsBus[busName];
                    busConf.baudrate = busDefault["baudRate"].as<uint32_t>() * 1000;
                    busConf.canIdBitSize = busDefault["canIdBitSize"].as<uint32_t>();
                    busConf.samplePoint = busDefault["samplePoint"].as<uint32_t>(0);
                    busConf.p4CanMax = defaultsSwdl["p4CanMax"].as<uint32_t>(0);
                    busConf.swdlSpecification = std::stoul(defaultsSwdl["specification"].as<std::string>("0"));
                    busConf.protocolId = getCanProtocol(defaultsSwdl["protocol"].as<std::string>(""));

                    for (const auto& idNode : bus["ecus"]) {
                        const auto id = idNode.as<std::string>();
                        const auto it = pool.find(id);
                        if (it == pool.end()) {
                            throw std::runtime_error("Unknown ECU id \"" + id + "\" referenced by configuration \""
                                + info.name + "\" bus \"" + busName + "\"");
                        }
                        busConf.ecuInfo.emplace_back(it->second);
                    }
                    info.busInfo.emplace_back(std::move(busConf));
                }
                result.emplace_back(std::move(info));
            }
            return result;
        }

        // Legacy v1 schema (pre-pool, pre-defaults), kept until the golden equivalence test is
        // green and the migration is accepted; removed by a follow-up commit.
        std::vector<ConfigurationInfo> result;
        auto confNodes = node["Configuration"];
        for (const auto& confNode : confNodes) {
            ConfigurationInfo info;
            info.name = confNode["Name"].as<std::string>();
            info.gatewaySubTester = processEndpointNode(confNode, "Gateway_SubTester");
            info.subTester = processEndpointNode(confNode, "SubTester");
            for (const auto& bus : confNode["Bus"]) {
                BusConfiguration busConf;
                busConf.baudrate = bus["BaudRate"].as<uint32_t>() * 1000;
                busConf.canIdBitSize = bus["CANIdBitSize"].as<uint32_t>();
                busConf.samplePoint = bus["SamplePoint"].as<uint32_t>(0);
                busConf.p4CanMax = bus["P4CANMax"].as<uint32_t>(0);
                busConf.swdlSpecification = bus["SWDLSpecification"].as<uint32_t>(0);
                busConf.protocolId = getCanProtocol(bus["SWDLProtocol"].as<std::string>());
                busConf.name = bus["Name"].as<std::string>();
                const auto& nodes = bus["Node"];
                if (nodes.IsDefined()) {
                    if (nodes.IsSequence()) {
                        for (const auto& node : bus["Node"]) {
                            busConf.ecuInfo.emplace_back(processEcuNode(node));
                        }
                    }
                    else {
                        busConf.ecuInfo.emplace_back(processEcuNode(nodes));
                    }
                }
                info.busInfo.emplace_back(std::move(busConf));
            }
            result.emplace_back(std::move(info));
        }
        return result;
    }

    std::vector<ConfigurationInfo> loadConfiguration(std::istream& input)
    {
        return loadConfigurationImpl(YAML::Load(input));
    }

    std::vector<ConfigurationInfo> loadConfiguration(const std::string& input)
    {
        return loadConfigurationImpl(YAML::Load(input));
    }

    void checkUDSError(uint8_t requestId, const uint8_t* data, size_t dataSize)
    {
        if(dataSize < 7 || data[4] != 0x7F || data[5] != requestId) {
            return;
        }
        const uint32_t responseCanId = encodeBigEndian(data[0], data[1], data[2], data[3]);
        throw UDSError(data[6], data[5], responseCanId);
    }

    CarPlatform parsePlatform(std::string input)
    {
        return parseCarPlatformImpl(std::move(input), false);
    }

    std::array<uint8_t, 5> getPinArray(uint64_t pin)
    {
        return {
            static_cast<uint8_t>((pin >> 32) & 0xFF),
            static_cast<uint8_t>((pin >> 24) & 0xFF),
            static_cast<uint8_t>((pin >> 16) & 0xFF),
            static_cast<uint8_t>((pin >> 8) & 0xFF),
            static_cast<uint8_t>(pin & 0xFF)
        };
    }

    std::array<uint8_t, 5> parseSecurityPin(const std::string& hex)
    {
        std::string compact;
        compact.reserve(hex.size());
        for (const auto ch : hex) {
            if (ch != ' ' && ch != '\t') {
                compact.push_back(ch);
            }
        }
        if (compact.size() != 10) {
            throw std::runtime_error(
                "SecurityAccess PIN must be exactly 5 hex bytes (10 hex digits), got \"" + hex + "\"");
        }
        std::array<uint8_t, 5> pin{};
        const auto hexVal = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            throw std::runtime_error("SecurityAccess PIN contains a non-hex character");
        };
        for (size_t i = 0; i < 5; ++i) {
            pin[i] = static_cast<uint8_t>((hexVal(compact[2 * i]) << 4) | hexVal(compact[2 * i + 1]));
        }
        return pin;
    }

    bool hasSecurityPin(const std::array<uint8_t, 5>& pin)
    {
        for (const auto byte : pin) {
            if (byte != 0) {
                return true;
            }
        }
        return false;
    }

    uint64_t securityPinToUint64(const std::array<uint8_t, 5>& pin)
    {
        uint64_t value = 0;
        for (const auto byte : pin) {
            value = (value << 8) | byte;
        }
        return value;
    }

    void initLogger(const std::string& logFilename, bool debugLogging)
    {
        el::Configurations defaultConf;
        defaultConf.setToDefault();
        defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime %level %msg");
        defaultConf.setGlobally(el::ConfigurationType::Filename, logFilename);
        // Keep stdout clean for structured (CSV) command output; logs go to file only.
        // Fatal/error conditions are still surfaced to stderr by the command layer.
        defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
        defaultConf.set(el::Level::Debug, el::ConfigurationType::Enabled, debugLogging ? "true" : "false");
        defaultConf.set(el::Level::Trace, el::ConfigurationType::Enabled, debugLogging ? "true" : "false");
        el::Loggers::reconfigureAllLoggers(defaultConf);
        el::Loggers::setDefaultConfigurations(defaultConf, true);
        // Under --debug also echo logs to stderr so the operator can see channel/CAN-id
        // resolution live; stdout stays clean for command output.
        if (debugLogging) {
            el::Helpers::installLogDispatchCallback<StderrLogDispatch>("StderrLogDispatch");
        }
    }

    uint16_t crc16(const uint8_t* data_p, size_t length)
    {
        uint16_t crc = crc16Init();

        while (length--) {
            crc = crc16Update(crc, *data_p++);
        }
        return crc;
    }

    uint16_t crc16Init()
    {
        return 0xFFFF;
    }

    uint16_t crc16Update(uint16_t crc, uint8_t byte)
    {
        uint8_t x = (crc >> 8) ^ byte;
        x ^= x >> 4;
        return static_cast<uint16_t>((crc << 8) ^ (static_cast<uint16_t>(x << 12))
            ^ (static_cast<uint16_t>(x << 5)) ^ static_cast<uint16_t>(x));
    }

} // namespace common
