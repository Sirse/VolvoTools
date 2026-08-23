#include "UdsCommands.hpp"

#include "DiagContext.hpp"
#include "ExitCodes.hpp"
#include "IsoTpFrame.hpp"
#include "OutputFormat.hpp"

#include <common/J2534ChannelProvider.hpp>
#include <common/Gateway.hpp>
#include <common/Util.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/protocols/UDSDid.hpp>
#include <common/protocols/UDSDtc.hpp>
#include <common/protocols/UDSError.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <common/protocols/UDSService.hpp>
#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <easylogging++.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>

namespace volvodiag {

namespace {

using common::selectSingleDevice;
using common::udsPayload;
using common::formatDtcCode;
using common::decodeDtcStatus;
using common::didName;
using common::decodeDidValue;
using common::parseDidResponse;
using common::parseDtcRecordResponse;

// Converts a UDS/OBD service to its wire byte.
template <typename Enum, typename = std::enable_if_t<std::is_enum_v<Enum>>>
constexpr uint8_t byte(Enum value)
{
    return static_cast<uint8_t>(value);
}

constexpr uint8_t kAllDtcGroupByte = 0xFF;

void writeDtcRow(std::ostream& output, const common::ECUInfo& ecu, const std::string& dtc,
                 const std::string& status, const std::string& statusFlags, const std::string& raw)
{
    output << hexNumber(ecu.ecuId, 2)
           << "," << hexNumber(ecu.canId, 3)
           << "," << csvEscape(ecu.name)
           << "," << csvEscape(dtc)
           << "," << csvEscape(status)
           << "," << csvEscape(statusFlags)
           << "," << csvEscape(raw)
           << std::endl;
}

void writeDtcClearRow(std::ostream& output, const common::ECUInfo& ecu, const std::string& status,
                      const std::string& response)
{
    output << hexNumber(ecu.ecuId, 2)
           << "," << hexNumber(ecu.canId, 3)
           << "," << csvEscape(ecu.name)
           << "," << csvEscape(status)
           << "," << csvEscape(response)
           << std::endl;
}

void writeScanRow(std::ostream& output, const common::ECUInfo& ecu, const std::string& status,
                  const std::string& response)
{
    output << hexNumber(ecu.ecuId, 2)
           << "," << hexNumber(ecu.canId, 3)
           << "," << csvEscape(status)
           << "," << csvEscape(ecu.name)
           << "," << csvEscape(response)
           << std::endl;
}

void writeDidScanRow(std::ostream& output, uint16_t did, const std::string& status,
                     const std::string& name, const std::string& value, const std::string& raw)
{
    output << hexNumber(did, 4)
           << "," << csvEscape(status)
           << "," << csvEscape(name)
           << "," << csvEscape(value)
           << "," << csvEscape(raw)
           << std::endl;
}

// Switches to extended session (10 03); true only for response 50 03.
bool enterExtendedSession(const j2534::J2534Channel& channel, uint32_t canId, size_t timeoutMs)
{
    const std::vector<uint8_t> request{
        byte(common::uds::ServiceId::DiagnosticSessionControl),
        byte(common::uds::DiagnosticSessionType::Extended)};
    try {
        const auto payload = udsPayload(processUds(channel, canId, request, timeoutMs));
        return payload.size() >= 2
            && payload[0] == byte(common::uds::PositiveResponseId::DiagnosticSessionControl)
            && payload[1] == byte(common::uds::DiagnosticSessionType::Extended);
    }
    catch (const std::exception& ex) {
        LOG(WARNING) << "Failed to enter/refresh extended session: " << ex.what();
        return false;
    }
}

void writeRoutineScanRow(std::ostream& output, uint16_t routine, const std::string& status,
                         const std::string& response)
{
    output << hexNumber(routine, 4)
           << "," << csvEscape(status)
           << "," << csvEscape(response)
           << std::endl;
}

void writeObdRow(std::ostream& output, const std::string& kind, const std::string& value,
                 const std::string& raw)
{
    output << csvEscape(kind)
           << "," << csvEscape(value)
           << "," << csvEscape(raw)
           << std::endl;
}

std::vector<uint8_t> readDidRequest(uint16_t did)
{
    return {byte(common::uds::ServiceId::ReadDataByIdentifier),
            static_cast<uint8_t>(did >> 8),
            static_cast<uint8_t>(did)};
}

void sendWakeBurst(j2534::J2534& j2534, const RunOptions& options)
{
    const auto busInfo = selectedRawCanBus(options);
    ensureCanIdFits(options.wakeCanId, "--wake can-id");
    auto rawChannel = common::openRawCanChannel(j2534, busInfo, options.baudrateOverride);
    const auto wakeData = makeIsoTpSingleFrame(options.wakePayload, /*padToEight=*/true);
    const auto wakeFrame = common::makeCanFrame(options.wakeCanId, wakeData);
    for (size_t i = 0; i < options.wakeBurstCount && !stopRequested.load(); ++i) {
        const auto status = rawChannel->writeMsg(wakeFrame, static_cast<unsigned long>(options.timeoutMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to send prelude wake frame: "
                                     + common::j2534StatusToString(status));
        }
        if (options.wakeGapMs > 0 && i + 1 < options.wakeBurstCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.wakeGapMs));
        }
    }
}

bool enterDiagnosticSession(const j2534::J2534Channel& channel, uint32_t canId,
                            uint8_t sessionType, size_t timeoutMs)
{
    const std::vector<uint8_t> request{
        byte(common::uds::ServiceId::DiagnosticSessionControl),
        sessionType};
    try {
        const auto payload = udsPayload(processUds(channel, canId, request, timeoutMs));
        return payload.size() >= 2
            && payload[0] == byte(common::uds::PositiveResponseId::DiagnosticSessionControl)
            && payload[1] == sessionType;
    }
    catch (const std::exception& ex) {
        LOG(WARNING) << "Failed to enter diagnostic session 0x" << std::hex
                     << static_cast<unsigned>(sessionType) << ": " << ex.what();
        return false;
    }
}

std::array<uint8_t, 3> generateSecurityKey(const std::vector<uint8_t>& pin,
                                           const std::array<uint8_t, 3>& seed)
{
    if (pin.size() != 5) {
        throw std::runtime_error("Security key input must contain exactly 5 bytes");
    }
    // Single source of truth: the same algorithm the flasher and the bruteforcer use,
    // pinned by reference vectors in SecurityKeyTests.
    const std::array<uint8_t, 5> pinArray{pin[0], pin[1], pin[2], pin[3], pin[4]};
    const uint32_t result = common::UDSProtocolCommonSteps::generateSecurityKey(pinArray, seed);
    return {static_cast<uint8_t>(result >> 16),
            static_cast<uint8_t>(result >> 8),
            static_cast<uint8_t>(result)};
}

void unlockSecurityAccess(const j2534::J2534Channel& channel, uint32_t canId,
                          const std::vector<uint8_t>& pin, size_t timeoutMs)
{
    const auto seedPayload = udsPayload(processUds(channel, canId, {0x27, 0x01}, timeoutMs));
    if (seedPayload.size() < 5 || seedPayload[0] != 0x67 || seedPayload[1] != 0x01) {
        throw std::runtime_error("Unexpected SecurityAccess seed response: " + formatBytes(seedPayload));
    }
    const std::array<uint8_t, 3> seed{seedPayload[2], seedPayload[3], seedPayload[4]};
    const auto key = generateSecurityKey(pin, seed);
    const std::vector<uint8_t> keyRequest{0x27, 0x02, key[0], key[1], key[2]};
    const auto keyPayload = udsPayload(processUds(channel, canId, keyRequest, timeoutMs));
    if (keyPayload.size() < 2 || keyPayload[0] != 0x67 || keyPayload[1] != 0x02) {
        throw std::runtime_error("Unexpected SecurityAccess key response: " + formatBytes(keyPayload));
    }
}

void sendTesterPresentKeepalive(const j2534::J2534Channel& channel, uint32_t canId,
                                size_t timeoutMs)
{
    const std::vector<uint8_t> request{
        byte(common::uds::ServiceId::TesterPresent),
        0x00};
    const auto payload = udsPayload(processUds(channel, canId, request, timeoutMs));
    if (payload.size() < 2
        || payload[0] != byte(common::uds::PositiveResponseId::TesterPresent)
        || payload[1] != 0x00) {
        throw std::runtime_error("Unexpected TesterPresent keepalive response: " + formatBytes(payload));
    }
}

void applyUdsPrelude(j2534::J2534& j2534, const j2534::J2534Channel& channel,
                     const RunOptions& options, uint32_t canId)
{
    (void)j2534;
    if (options.preludeSessionType != 0x00) {
        if (!enterDiagnosticSession(channel, canId, options.preludeSessionType, options.timeoutMs)) {
            throw std::runtime_error("Could not enter requested diagnostic session (10 "
                                     + hexNumber(options.preludeSessionType, 2) + ")");
        }
    }
    if (!options.preludeSecurityKey.empty()) {
        unlockSecurityAccess(channel, canId, options.preludeSecurityKey, options.timeoutMs);
    }
}

std::string decodeObdDtc(uint8_t high, uint8_t low)
{
    if (high == 0x00 && low == 0x00) {
        return {};
    }
    return common::formatDtcBaseCode(high, low);
}

std::vector<std::string> parseObdDtcPayload(const std::vector<uint8_t>& payload)
{
    if (payload.empty() || payload[0] != byte(common::obd::PositiveResponseId::ShowStoredDTCs)) {
        throw std::runtime_error("Unexpected OBD DTC response: " + formatBytes(payload));
    }

    std::vector<std::string> result;
    for (size_t offset = 1; offset + 1 < payload.size(); offset += 2) {
        const auto dtc = decodeObdDtc(payload[offset], payload[offset + 1]);
        if (!dtc.empty()) {
            result.push_back(dtc);
        }
    }
    return result;
}

std::string parseObdVinPayload(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 3
        || payload[0] != byte(common::obd::PositiveResponseId::RequestVehicleInformation)
        || payload[1] != byte(common::obd::VehicleInformationPid::Vin)) {
        throw std::runtime_error("Unexpected OBD VIN response: " + formatBytes(payload));
    }

    std::string vin;
    for (size_t i = 3; i < payload.size(); ++i) {
        if (payload[i] >= 0x20 && payload[i] <= 0x7E) {
            vin.push_back(static_cast<char>(payload[i]));
        }
    }
    if (vin.empty()) {
        throw std::runtime_error("OBD VIN response contains no printable VIN bytes");
    }
    return vin;
}

size_t writeDtcRecords(std::ostream& output, const common::ECUInfo& ecu,
                       const common::DtcReadResult& parsed, bool confirmedOnly)
{
    size_t count = 0;
    for (const auto& record : parsed.records) {
        if (confirmedOnly && (record.status & common::uds::DtcStatusMask::Confirmed) == 0) {
            continue;
        }
        writeDtcRow(output, ecu, formatDtcCode(record.high, record.middle, record.low),
                    hexNumber(record.status, 2),
                    decodeDtcStatus(record.status, parsed.availabilityMask),
                    hexNumber((static_cast<uint32_t>(record.high) << 24)
                        | (static_cast<uint32_t>(record.middle) << 16)
                        | (static_cast<uint32_t>(record.low) << 8)
                        | record.status, 8));
        ++count;
    }
    return count;
}

void writeDtcRecordRow(std::ostream& output, const common::ECUInfo& ecu, const std::string& dtc,
                       const std::string& status, const std::string& statusFlags,
                       const std::string& record, const std::string& data)
{
    output << hexNumber(ecu.ecuId, 2)
           << "," << hexNumber(ecu.canId, 3)
           << "," << csvEscape(ecu.name)
           << "," << csvEscape(dtc)
           << "," << csvEscape(status)
           << "," << csvEscape(statusFlags)
           << "," << csvEscape(record)
           << "," << csvEscape(data)
           << std::endl;
}

// Combines explicit DTCs with codes from 19 02 when requested.
std::vector<uint32_t> collectDtcTargets(const j2534::J2534Channel& channel, uint32_t canId,
                                        const RunOptions& options)
{
    std::vector<uint32_t> targets = options.dtcTargets;
    if (options.dtcAllStored) {
        const std::vector<uint8_t> request{
            byte(common::uds::ServiceId::ReadDTCInformation),
            byte(common::uds::ReadDTCSubFunction::ReportDTCByStatusMask),
            kAllDtcGroupByte};
        const auto parsed = common::parseDtcByStatusMaskResponse(
            processUds(channel, canId, request, options.timeoutMs));
        for (const auto& record : parsed.records) {
            targets.push_back((static_cast<uint32_t>(record.high) << 16)
                | (static_cast<uint32_t>(record.middle) << 8) | record.low);
        }
    }
    return targets;
}

// Shared 19 04 / 19 06 reader; record payloads stay raw.
void runDtcRecordsImpl(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options,
                       common::uds::ReadDTCSubFunction subFunction, const char* name)
{
    DiagOutput output{options.dtcRecordOutputPath, std::string("DTC ") + name + " output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    const auto ecu = std::get<1>(common::getEcuInfoByEcuId(options.carPlatform, options.ecuId));
    LOG(INFO) << "VolvoDiag dtc " << name << " device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId);
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);

    output.line("ecu,can_id,name,dtc,status,status_flags,record,data");
    const auto record = hexNumber(options.dtcRecordNumber, 2);
    for (const auto target : collectDtcTargets(*channel, canId, options)) {
        if (stopRequested.load()) {
            return;
        }
        if (options.keepalive) {
            sendTesterPresentKeepalive(*channel, canId, options.timeoutMs);
        }
        const auto high = static_cast<uint8_t>(target >> 16);
        const auto middle = static_cast<uint8_t>(target >> 8);
        const auto low = static_cast<uint8_t>(target);
        const std::vector<uint8_t> request{
            byte(common::uds::ServiceId::ReadDTCInformation), byte(subFunction),
            high, middle, low, options.dtcRecordNumber};
        try {
            const auto parsed = parseDtcRecordResponse(
                processUds(*channel, canId, request, options.timeoutMs), byte(subFunction));
            output.each([&](std::ostream& os) {
                writeDtcRecordRow(os, ecu, formatDtcCode(parsed.high, parsed.middle, parsed.low),
                    hexNumber(parsed.status, 2), decodeDtcStatus(parsed.status, 0),
                    record, formatBytes(parsed.data));
            });
        }
        catch (const std::exception& ex) {
            output.each([&](std::ostream& os) {
                writeDtcRecordRow(os, ecu, formatDtcCode(high, middle, low),
                    "error", "", record, ex.what());
            });
        }
    }
}

} // namespace

void runBuses(const RunOptions& options)
{
    const auto configuration = common::getConfigurationInfoByCarPlatform(options.carPlatform);
    std::cout << "bus,protocol,baudrate,can_id_bits,ecu_count,ecus" << std::endl;
    for (const auto& bus : configuration.busInfo) {
        std::stringstream ecus;
        for (size_t i = 0; i < bus.ecuInfo.size(); ++i) {
            if (i > 0) {
                ecus << "; ";
            }
            ecus << hexNumber(bus.ecuInfo[i].ecuId, 2) << " " << bus.ecuInfo[i].name;
        }
        std::cout << csvEscape(bus.name)
                  << "," << bus.protocolId
                  << "," << bus.baudrate
                  << "," << bus.canIdBitSize
                  << "," << bus.ecuInfo.size()
                  << "," << csvEscape(ecus.str())
                  << std::endl;
    }
}

void runSend(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag send device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId);
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);
    UdsExpectation expect;
    expect.prefix = options.expectedData;
    expect.nrc = options.expectNrc;
    expect.timeout = options.expectTimeout;

    const auto ecuLabel = hexNumber(options.ecuId, 2);
    for (size_t i = 0; i < options.repeatCount; ++i) {
        const std::string command = options.repeatCount > 1
            ? "send[" + std::to_string(i + 1) + "]"
            : "send";
        runReported(command, ecuLabel, options.requestData, [&]() -> std::vector<uint8_t> {
            // Expected timeouts and NRCs have no payload to print.
            if (expect.timeout || expect.nrc) {
                processUdsExpecting(*channel, canId, options.requestData, options.timeoutMs, expect);
                return {};
            }
            const auto response = processUds(*channel, canId, options.requestData, options.timeoutMs);
            const auto payload = udsPayload(response);
            ensurePayloadPrefix(payload, options.expectedData);
            return options.outputMode == "payload" ? payload : response;
        }, [&](const std::vector<uint8_t>&) -> std::string {
            if (expect.timeout) return "no response (expected)";
            if (expect.nrc) return "NRC " + hexNumber(*expect.nrc, 2) + " (expected)";
            return {};
        });
    }
}

void runIdent(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.identOutputPath, "ident output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag ident device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId);
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);

    const std::vector<uint16_t> identDids{
        0xF190, // VIN
        0xF18C, // ECU serial number
        0xF191, // ECU hardware number
        0xF194, // ECU software number
        0xF195, // ECU software version
        0xF18A, // System supplier id
        0xF197, // System name
    };
    for (const auto did : identDids) {
        const auto request = readDidRequest(did);
        try {
            const auto response = processUds(*channel, canId, request);
            const auto decoded = parseDidResponse(response);
            output.line(didName(decoded.did) + " (" + hexNumber(decoded.did, 4) + "): "
                        + decodeDidValue(decoded.did, decoded.data));
        }
        catch (const std::exception& ex) {
            output.line(didName(did) + " (" + hexNumber(did, 4) + "): " + ex.what());
        }
    }
}

void runDidRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.didOutputPath, "DID output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag did device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId);
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);

    if (!options.didOutputPath.empty()) {
        output.line("did,name,value,raw");
    }
    for (const auto did : options.didIds) {
        if (stopRequested.load()) {
            return;
        }
        const auto request = readDidRequest(did);
        try {
            const auto response = processUds(*channel, canId, request);
            const auto decoded = parseDidResponse(response);
            const auto name = didName(decoded.did);
            const auto value = decodeDidValue(decoded.did, decoded.data);
            const auto raw = formatBytes(decoded.data);
            if (options.didOutputPath.empty()) {
                std::cout << name << " (" << hexNumber(decoded.did, 4) << "): "
                          << value << "    [" << raw << "]" << std::endl;
            } else {
                output.each([&](std::ostream& os) {
                    os << hexNumber(decoded.did, 4) << ","
                       << csvEscape(name) << ","
                       << csvEscape(value) << ","
                       << csvEscape(raw) << std::endl;
                });
            }
        }
        catch (const std::exception& ex) {
            const auto name = didName(did);
            if (options.didOutputPath.empty()) {
                std::cout << name << " (" << hexNumber(did, 4) << "): " << ex.what() << std::endl;
            } else {
                output.each([&](std::ostream& os) {
                    os << hexNumber(did, 4) << "," << csvEscape(name)
                       << ",error," << csvEscape(ex.what()) << std::endl;
                });
            }
        }
    }
}

void runDidWrite(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "did-write modifies ECU memory; pass --yes to proceed");
    }

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag did-write device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " did=0x" << options.didWriteId
              << " session=" << (options.didWriteExtendedSession ? "ext" : "default");
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);

    if (options.didWriteExtendedSession
        && !enterExtendedSession(*channel, canId, options.timeoutMs)) {
        throw std::runtime_error("Could not enter extended session (10 03); aborting write so it "
                                 "does not land in the default session");
    }

    const auto didHigh = static_cast<uint8_t>(options.didWriteId >> 8);
    const auto didLow = static_cast<uint8_t>(options.didWriteId);
    std::vector<uint8_t> request{byte(common::uds::ServiceId::WriteDataByIdentifier), didHigh, didLow};
    request.insert(request.end(), options.requestData.cbegin(), options.requestData.cend());

    runReported("did-write", hexNumber(options.ecuId, 2), request, [&]() {
        const auto payload = udsPayload(processUds(*channel, canId, request, options.timeoutMs));
        if (payload.size() < 3 || payload[0] != byte(common::uds::PositiveResponseId::WriteDataByIdentifier)
            || payload[1] != didHigh || payload[2] != didLow) {
            throw DiagError(ExitCode::ValidationError,
                "Unexpected WriteDataByIdentifier response: " + formatBytes(payload));
        }
        return payload;
    });
}

void runDidScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.didScanOutputPath, "DID scan output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag did-scan device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " from=0x" << options.didScanFrom
              << " to=0x" << options.didScanTo;
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);

    output.line("did,status,name,value,raw");

    for (uint32_t did = options.didScanFrom; did <= options.didScanTo && !stopRequested.load(); ++did) {
        if (options.keepalive) {
            sendTesterPresentKeepalive(*channel, canId, options.timeoutMs);
        }
        const auto request = readDidRequest(static_cast<uint16_t>(did));
        try {
            const auto response = processUds(*channel, canId, request, options.timeoutMs);
            const auto decoded = parseDidResponse(response);
            const auto value = decodeDidValue(decoded.did, decoded.data);
            const auto raw = formatBytes(decoded.data);
            output.each([&](std::ostream& os) {
                writeDidScanRow(os, decoded.did, "ok", didName(decoded.did), value, raw);
            });
        }
        catch (const std::exception& ex) {
            LOG(DEBUG) << "DID scan miss did=0x" << std::hex << did << ": " << ex.what();
        }
        if (options.didScanGapMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.didScanGapMs));
        }
    }
}

void runScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.scanOutputPath, "scan output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag scan device=" << device.deviceName
              << " platform=" << options.platformName;
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};

    const auto configuration = common::getConfigurationInfoByCarPlatform(options.carPlatform);
    output.line("ecu,can_id,status,name,response");
    std::set<uint32_t> scannedEcus;
    for (const auto& bus : configuration.busInfo) {
        if (bus.protocolId != ISO15765) {
            continue;
        }
        for (const auto& ecu : bus.ecuInfo) {
            if (stopRequested.load()) {
                return;
            }
            if (ecu.ecuId == 0 || ecu.ecuId > 0xFF) {
                continue;
            }
            if (!scannedEcus.insert(ecu.ecuId).second) {
                continue;
            }
            try {
                auto channel = provider.getChannelForEcu(ecu.ecuId);
                const auto response = processUds(*channel, ecu.canId, readDidRequest(0xF190));
                const auto payload = formatBytes(udsPayload(response));
                output.each([&](std::ostream& os) { writeScanRow(os, ecu, "ok", payload); });
            }
            catch (const std::exception& ex) {
                output.each([&](std::ostream& os) { writeScanRow(os, ecu, "error", ex.what()); });
            }
        }
    }
}

void runSession(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag session device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " type=0x" << static_cast<unsigned>(options.sessionType)
              << " suppress=" << options.sessionSuppress;
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    const auto subFunction = static_cast<uint8_t>(options.sessionType | (options.sessionSuppress ? 0x80 : 0x00));
    const std::vector<uint8_t> request{
        byte(common::uds::ServiceId::DiagnosticSessionControl), subFunction};

    const auto ecuLabel = hexNumber(options.ecuId, 2);
    if (options.sessionSuppress) {
        const auto start = std::chrono::steady_clock::now();
        sendUdsNoWait(*channel, canId, request, options.timeoutMs);
        ResultLine line;
        line.command = "session";
        line.ecu = ecuLabel;
        line.request = request;
        line.detail = "suppressed";
        line.durationMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        printResultLine(line);
        return;
    }

    runReported("session", ecuLabel, request, [&]() {
        const auto payload = udsPayload(processUds(*channel, canId, request, options.timeoutMs));
        if (payload.size() < 2
            || payload[0] != byte(common::uds::PositiveResponseId::DiagnosticSessionControl)
            || payload[1] != options.sessionType) {
            throw DiagError(ExitCode::ValidationError,
                "Unexpected DiagnosticSessionControl response: " + formatBytes(payload));
        }
        return payload;
    });
}

void runTesterPresent(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag tester-present device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " count=" << std::dec << options.testerPresentCount
              << " interval_ms=" << options.testerPresentIntervalMs
              << " suppress=" << options.testerPresentSuppress;
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    const auto subFunction = static_cast<uint8_t>(options.testerPresentSuppress ? 0x80 : 0x00);
    const std::vector<uint8_t> request{byte(common::uds::ServiceId::TesterPresent), subFunction};

    const auto ecuLabel = hexNumber(options.ecuId, 2);
    size_t sent = 0;
    while (!stopRequested.load() && (options.testerPresentCount == 0 || sent < options.testerPresentCount)) {
        ++sent;
        // Add the counter to repeated-send labels.
        std::string command = "tester-present";
        if (options.testerPresentCount != 1) {
            command += " [" + std::to_string(sent);
            if (options.testerPresentCount > 0) {
                command += "/" + std::to_string(options.testerPresentCount);
            }
            command += "]";
        }
        if (options.testerPresentSuppress) {
            const auto start = std::chrono::steady_clock::now();
            sendUdsNoWait(*channel, canId, request, options.timeoutMs);
            ResultLine line;
            line.command = command;
            line.ecu = ecuLabel;
            line.request = request;
            line.detail = "suppressed";
            line.durationMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            printResultLine(line);
        } else {
            runReported(command, ecuLabel, request, [&]() {
                const auto payload = udsPayload(processUds(*channel, canId, request, options.timeoutMs));
                if (payload.size() < 2
                    || payload[0] != byte(common::uds::PositiveResponseId::TesterPresent)
                    || payload[1] != 0x00) {
                    throw DiagError(ExitCode::ValidationError,
                        "Unexpected TesterPresent response: " + formatBytes(payload));
                }
                return payload;
            });
        }

        if (options.testerPresentCount == 0 || sent < options.testerPresentCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.testerPresentIntervalMs));
        }
    }
}

void runObdVin(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.obdOutputPath, "OBD output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    auto j2534 = openDevice(device);
    auto channel = openObdChannel(*j2534, options);
    const std::vector<uint8_t> request{
        byte(common::obd::ServiceId::RequestVehicleInformation),
        byte(common::obd::VehicleInformationPid::Vin)};
    std::string vin;
    runReported("obd-vin", formatCanId(common::obd::FunctionalRequestCanId), request, [&]() {
        const auto payload = udsPayload(processUds(*channel, common::obd::FunctionalRequestCanId,
            request, options.timeoutMs));
        vin = parseObdVinPayload(payload);
        if (output.hasFile()) {
            output.fileOnly([](std::ostream& os) { os << "kind,value,raw\n"; });
            output.fileOnly([&](std::ostream& os) { writeObdRow(os, "vin", vin, formatBytes(payload)); });
        }
        return payload;
    }, [&](const std::vector<uint8_t>&) { return "vin=" + vin; });
}

void runObdPid(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.obdOutputPath, "OBD output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    auto j2534 = openDevice(device);
    auto channel = openObdChannel(*j2534, options);
    const std::vector<uint8_t> request{byte(common::obd::ServiceId::ShowCurrentData), options.obdPid};
    const auto kind = "pid_" + hexNumber(options.obdPid, 2);
    std::string value;
    runReported("obd-pid", formatCanId(common::obd::FunctionalRequestCanId), request, [&]() {
        const auto payload = udsPayload(processUds(*channel, common::obd::FunctionalRequestCanId,
            request, options.timeoutMs));
        if (payload.size() < 2 || payload[0] != byte(common::obd::PositiveResponseId::ShowCurrentData)
            || payload[1] != options.obdPid) {
            throw DiagError(ExitCode::ValidationError,
                "Unexpected OBD PID response: " + formatBytes(payload));
        }
        value = formatBytes(std::vector<uint8_t>{payload.cbegin() + 2, payload.cend()});
        if (output.hasFile()) {
            output.fileOnly([](std::ostream& os) { os << "kind,value,raw\n"; });
            output.fileOnly([&](std::ostream& os) { writeObdRow(os, kind, value, formatBytes(payload)); });
        }
        return payload;
    }, [&](const std::vector<uint8_t>&) { return kind + "=" + value; });
}

void runObdDtcRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.obdOutputPath, "OBD output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    auto j2534 = openDevice(device);
    auto channel = openObdChannel(*j2534, options);
    const auto payload = udsPayload(processUds(*channel, common::obd::FunctionalRequestCanId,
        {byte(common::obd::ServiceId::ShowStoredDTCs)}, options.timeoutMs));
    const auto dtcs = parseObdDtcPayload(payload);

    output.line("kind,value,raw");
    if (dtcs.empty()) {
        output.each([&](std::ostream& os) { writeObdRow(os, "dtc", "none", formatBytes(payload)); });
        return;
    }
    for (const auto& dtc : dtcs) {
        output.each([&](std::ostream& os) { writeObdRow(os, "dtc", dtc, formatBytes(payload)); });
    }
}

void runObdDtcClear(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "OBD DTC clear requires --yes");
    }

    DiagOutput output{options.obdOutputPath, "OBD output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    auto j2534 = openDevice(device);
    auto channel = openObdChannel(*j2534, options);
    const std::vector<uint8_t> request{byte(common::obd::ServiceId::ClearDTCs)};
    runReported("obd-dtc-clear", formatCanId(common::obd::FunctionalRequestCanId), request, [&]() {
        const auto payload = udsPayload(processUds(*channel, common::obd::FunctionalRequestCanId,
            request, options.timeoutMs));
        if (payload.empty() || payload[0] != byte(common::obd::PositiveResponseId::ClearDTCs)) {
            throw DiagError(ExitCode::ValidationError,
                "Unexpected OBD DTC clear response: " + formatBytes(payload));
        }
        if (output.hasFile()) {
            output.fileOnly([](std::ostream& os) { os << "kind,value,raw\n"; });
            output.fileOnly([&](std::ostream& os) { writeObdRow(os, "dtc_clear", "ok", formatBytes(payload)); });
        }
        return payload;
    });
}

void runDtcRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.dtcOutputPath, "DTC output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag dtc read device=" << device.deviceName
              << " platform=" << options.platformName
              << (options.allEcus ? " all_ecus=true" : " all_ecus=false");
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};

    output.line("ecu,can_id,name,dtc,status,status_flags,raw");
    const std::vector<uint8_t> readRequest{
        byte(common::uds::ServiceId::ReadDTCInformation),
        byte(common::uds::ReadDTCSubFunction::ReportDTCByStatusMask),
        options.dtcStatusMask};
    for (const auto& ecu : getSelectedUdsEcus(options)) {
        if (stopRequested.load()) {
            return;
        }
        if (ecu.ecuId > 0xFF) {
            continue;
        }
        try {
            auto channel = provider.getChannelForEcu(ecu.ecuId);
            applyUdsPrelude(*j2534, *channel, options, ecu.canId);
            if (options.keepalive) {
                sendTesterPresentKeepalive(*channel, ecu.canId, options.timeoutMs);
            }
            const auto response = processUds(*channel, ecu.canId, readRequest);
            const auto parsed = common::parseDtcByStatusMaskResponse(response);
            size_t count = 0;
            output.each([&](std::ostream& os) {
                count = writeDtcRecords(os, ecu, parsed, options.dtcConfirmedOnly);
            });
            if (count == 0) {
                output.each([&](std::ostream& os) { writeDtcRow(os, ecu, "none", "", "", ""); });
            }
        }
        catch (const std::exception& ex) {
            output.each([&](std::ostream& os) { writeDtcRow(os, ecu, "error", "", "", ex.what()); });
        }
    }
}

void runDtcClear(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "DTC clear requires --yes");
    }

    DiagOutput output{options.dtcClearOutputPath, "DTC clear output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag dtc clear device=" << device.deviceName
              << " platform=" << options.platformName
              << (options.allEcus ? " all_ecus=true" : " all_ecus=false");
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};

    output.line("ecu,can_id,name,status,response");
    for (const auto& ecu : getSelectedUdsEcus(options)) {
        if (stopRequested.load()) {
            return;
        }
        if (ecu.ecuId > 0xFF) {
            continue;
        }
        try {
            auto channel = provider.getChannelForEcu(ecu.ecuId);
            const auto clearResponse = udsPayload(processUds(*channel, ecu.canId,
                {byte(common::uds::ServiceId::ClearDiagnosticInformation),
                 kAllDtcGroupByte, kAllDtcGroupByte, kAllDtcGroupByte}));
            if (clearResponse.empty()
                || clearResponse[0] != byte(common::uds::PositiveResponseId::ClearDiagnosticInformation)) {
                throw std::runtime_error("Unexpected clear response: " + formatBytes(clearResponse));
            }

            output.each([&](std::ostream& os) { writeDtcClearRow(os, ecu, "ok", formatBytes(clearResponse)); });
        }
        catch (const std::exception& ex) {
            output.each([&](std::ostream& os) { writeDtcClearRow(os, ecu, "error", ex.what()); });
        }
    }
}

void runDtcSnapshot(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    runDtcRecordsImpl(devices, options,
        common::uds::ReadDTCSubFunction::ReportDTCSnapshotRecordByDTCNumber, "snapshot");
}

void runDtcExtended(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    runDtcRecordsImpl(devices, options,
        common::uds::ReadDTCSubFunction::ReportDTCExtendedDataRecordByDTCNumber, "extended");
}

void runReset(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "ECU reset requires --yes");
    }

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    const uint8_t subFunction = static_cast<uint8_t>(
        options.resetType | (options.resetSuppress ? 0x80 : 0x00));
    LOG(INFO) << "VolvoDiag reset device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " type=0x" << static_cast<unsigned>(options.resetType)
              << " functional=" << options.resetFunctional
              << " suppress=" << options.resetSuppress
              << " bus=" << (options.busName.empty() ? "(all)" : options.busName);
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};

    const std::vector<uint8_t> request{byte(common::uds::ServiceId::ECUReset), subFunction};
    const auto ecuLabel = hexNumber(options.ecuId, 2);

    // Reset is best-effort: silence is expected for suppressed and functional requests.
    const auto sendReset = [&](const j2534::J2534Channel& channel, uint32_t canId,
                               const std::string& where) {
        const auto start = std::chrono::steady_clock::now();
        const auto elapsedMs = [&]() {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        };
        ResultLine line;
        line.command = "reset";
        line.ecu = ecuLabel;
        line.request = request;
        if (options.resetSuppress) {
            sendUdsNoWait(channel, canId, request, options.timeoutMs);
            line.detail = where + ", suppressed";
            line.durationMs = elapsedMs();
            printResultLine(line);
            return;
        }
        try {
            const auto payload = udsPayload(processUds(channel, canId, request, options.timeoutMs));
            line.response = payload;
            line.durationMs = elapsedMs();
            if (payload.empty() || payload[0] != byte(common::uds::PositiveResponseId::ECUReset)) {
                line.ok = false;
                line.detail = where + ", unexpected response";
            } else {
                line.detail = where;
            }
            printResultLine(line);
        }
        catch (const common::UDSRequestRxTimeout&) {
            line.detail = where + ", no response";
            line.durationMs = elapsedMs();
            printResultLine(line);
        }
    };

    if (!options.resetFunctional) {
        auto channel = provider.getChannelForEcu(options.ecuId);
        sendReset(*channel, ecuCanId(options.carPlatform, options.ecuId), "addressed");
        return;
    }

    // Functional reset is effectively fire-and-forget; use --suppress.
    if (!options.resetSuppress) {
        LOG(WARNING) << "functional reset without --suppress: responses on 0x7DF are best-effort";
        std::cerr << "Warning: functional reset response capture is best-effort; "
                     "use --suppress for a clean fire-and-forget broadcast." << std::endl;
    }
    auto channels = provider.getUdsChannelsByBus(options.ecuId, options.busName);
    if (channels.empty()) {
        throw std::runtime_error(options.busName.empty()
            ? "No ISO15765 buses configured for functional reset"
            : "No ISO15765 bus named: " + options.busName);
    }
    for (auto& [bus, channel] : channels) {
        sendReset(*channel, common::obd::FunctionalRequestCanId,
                  "functional 0x7DF on " + bus.name);
    }
}

void runRoutine(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "routine start/stop modifies ECU state; pass --yes to proceed");
    }

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag routine device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " sub=0x" << static_cast<unsigned>(options.routineSubFunction)
              << " routine=0x" << options.routineId
              << " session=" << (options.routineExtendedSession ? "ext" : "default")
              << " suppress=" << options.routineSuppress;
    auto j2534 = openDevice(device);
    if (options.preludeWake) {
        sendWakeBurst(*j2534, options);
    }
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);
    applyUdsPrelude(*j2534, *channel, options, canId);

    if (options.routineExtendedSession
        && !enterExtendedSession(*channel, canId, options.timeoutMs)) {
        throw std::runtime_error("Could not enter extended session (10 03); aborting routine");
    }

    const auto routineHigh = static_cast<uint8_t>(options.routineId >> 8);
    const auto routineLow = static_cast<uint8_t>(options.routineId);
    const auto subFunction = static_cast<uint8_t>(
        options.routineSubFunction | (options.routineSuppress ? 0x80 : 0x00));
    std::vector<uint8_t> request{
        byte(common::uds::ServiceId::RoutineControl),
        subFunction,
        routineHigh,
        routineLow};
    request.insert(request.end(), options.routineData.cbegin(), options.routineData.cend());

    const auto ecuLabel = hexNumber(options.ecuId, 2);
    if (options.routineSuppress) {
        const auto start = std::chrono::steady_clock::now();
        sendUdsNoWait(*channel, canId, request, options.timeoutMs);
        ResultLine line;
        line.command = "routine";
        line.ecu = ecuLabel;
        line.request = request;
        line.detail = "suppressed";
        line.durationMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        printResultLine(line);
        return;
    }

    runReported("routine", ecuLabel, request, [&]() {
        const auto payload = udsPayload(processUds(*channel, canId, request, options.timeoutMs));
        try {
            common::validateRoutineControlResponse(payload, options.routineSubFunction == 0x01,
                                                   options.routineId);
        }
        catch (const std::exception&) {
            throw DiagError(ExitCode::ValidationError,
                "Unexpected RoutineControl response: " + formatBytes(payload));
        }
        return payload;
    });
}

void runRoutineScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    if (!options.confirmDestructive) {
        throw DiagError(ExitCode::UsageError, "routine-scan actively starts routines on the ECU; pass --yes to proceed");
    }

    DiagOutput output{options.routineScanOutputPath, "routine scan output"};

    ensureUdsPlatform(options.carPlatform);
    const auto device = selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag routine-scan device=" << device.deviceName
              << " platform=" << options.platformName
              << " ecu=0x" << std::hex << static_cast<unsigned>(options.ecuId)
              << " sub=0x" << static_cast<unsigned>(options.routineSubFunction)
              << " from=0x" << options.routineScanFrom << " to=0x" << options.routineScanTo
              << " session=" << (options.routineExtendedSession ? "ext" : "default");
    auto j2534 = openDevice(device);
    common::J2534ChannelProvider provider{*j2534, options.carPlatform, options.baudrateOverride};
    auto channel = provider.getChannelForEcu(options.ecuId);
    const auto canId = ecuCanId(options.carPlatform, options.ecuId);

    output.line("routine,status,response");

    // Refresh extended session during long scans; abort if it cannot be entered.
    constexpr size_t kSessionRefreshEvery = 20;
    size_t sinceRefresh = 0;
    if (options.routineExtendedSession
        && !enterExtendedSession(*channel, canId, options.timeoutMs)) {
        output.each([&](std::ostream& os) {
            writeRoutineScanRow(os, 0, "session_error", "failed to enter extended session (10 03)");
        });
        return;
    }
    for (uint32_t routine = options.routineScanFrom;
         routine <= options.routineScanTo && !stopRequested.load(); ++routine) {
        if (options.keepalive) {
            sendTesterPresentKeepalive(*channel, canId, options.timeoutMs);
        }
        if (options.routineExtendedSession && ++sinceRefresh > kSessionRefreshEvery) {
            if (!enterExtendedSession(*channel, canId, options.timeoutMs)) {
                output.each([&](std::ostream& os) {
                    writeRoutineScanRow(os, static_cast<uint16_t>(routine), "session_error",
                        "lost extended session (10 03); stopping so later rows stay in ext");
                });
                return;
            }
            sinceRefresh = 0;
        }
        const std::vector<uint8_t> request{
            byte(common::uds::ServiceId::RoutineControl), options.routineSubFunction,
            static_cast<uint8_t>(routine >> 8), static_cast<uint8_t>(routine)};
        try {
            const auto payload = udsPayload(processUds(*channel, canId, request, options.timeoutMs));
            output.each([&](std::ostream& os) {
                writeRoutineScanRow(os, static_cast<uint16_t>(routine), "ok", formatBytes(payload));
            });
        }
        catch (const common::UDSError& ex) {
            // 0x31 means the routine is absent; report other NRCs.
            if (ex.getErrorCode() == common::UDSError::ErrorCode::RequestOutOfRange) {
                LOG(DEBUG) << "routine-scan absent routine=0x" << std::hex << routine;
            } else {
                output.each([&](std::ostream& os) {
                    writeRoutineScanRow(os, static_cast<uint16_t>(routine), "rejected",
                        "NRC 0x" + hexNumber(ex.getErrorCode(), 2) + " " + ex.what());
                });
            }
        }
        catch (const std::exception& ex) {
            LOG(DEBUG) << "routine-scan miss routine=0x" << std::hex << routine << ": " << ex.what();
        }
        if (options.routineScanGapMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.routineScanGapMs));
        }
    }
}

} // namespace volvodiag
