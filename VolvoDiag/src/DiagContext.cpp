#include "DiagContext.hpp"

#include "ExitCodes.hpp"
#include "OutputFormat.hpp"

#include <common/CliSupport.hpp>
#include <common/Util.hpp>
#include <common/protocols/UDSError.hpp>
#include <common/protocols/UDSMessage.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>

namespace volvodiag {

namespace {

class UdsTraceWriter {
public:
    explicit UdsTraceWriter(const std::string& path)
        : _start{std::chrono::steady_clock::now()}
    {
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        _output.open(path);
        if (!_output) {
            throw std::runtime_error("Failed to open UDS trace: " + path);
        }
        _output << "time_ms,can_id,direction,sid,payload,result" << std::endl;
    }

    void write(uint32_t canId, const std::string& direction, const std::vector<uint8_t>& payload,
               const std::string& result)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - _start);
        const auto sid = payload.empty() ? std::string{} : hexNumber(payload.front(), 2);
        _output << elapsed.count()
                << "," << formatCanId(canId)
                << "," << csvEscape(direction)
                << "," << csvEscape(sid)
                << "," << csvEscape(formatBytes(payload))
                << "," << csvEscape(result)
                << std::endl;
    }

private:
    std::ofstream _output;
    std::chrono::steady_clock::time_point _start;
};

std::unique_ptr<UdsTraceWriter> gUdsTrace;

uint32_t canIdFromRawFrame(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 4) {
        throw std::runtime_error("Short CAN frame");
    }
    return (static_cast<uint32_t>(frame[0]) << 24)
        | (static_cast<uint32_t>(frame[1]) << 16)
        | (static_cast<uint32_t>(frame[2]) << 8)
        | static_cast<uint32_t>(frame[3]);
}

} // namespace

void ensureCanIdFits(uint32_t canId, const std::string& argumentName)
{
    if (canId > 0x1FFFFFFF) {
        throw std::runtime_error(argumentName + " must fit in 29 bits");
    }
}

bool isUdsPlatform(common::CarPlatform carPlatform)
{
    using common::CarPlatform;
    return carPlatform == CarPlatform::P3
        || carPlatform == CarPlatform::P3_Y413
        || carPlatform == CarPlatform::P3_Y283_IAM
        || carPlatform == CarPlatform::P3_Y283_ICM
        || carPlatform == CarPlatform::P3_P313_ICM
        || carPlatform == CarPlatform::P3_P313_IAM
        || carPlatform == CarPlatform::P3_Y555_IAM
        || carPlatform == CarPlatform::P3_Y555_ICM
        || carPlatform == CarPlatform::P3_Y312H_IAM
        || carPlatform == CarPlatform::P3_Y312H_ICM;
}

void ensureUdsPlatform(common::CarPlatform carPlatform)
{
    // Only UDS (ISO15765) platforms are supported here.
    if (!isUdsPlatform(carPlatform)) {
        throw std::runtime_error("VolvoDiag currently supports UDS platforms only (e.g. P3)");
    }
}

std::unique_ptr<j2534::J2534> openDevice(const j2534::DeviceInfo& device)
{
    try {
        // The DiCE naming quirk lives in the shared helper.
        return common::openJ2534Device(device);
    }
    catch (const DiagError&) {
        throw;
    }
    catch (const std::exception& ex) {
        // Treat library and PassThruOpen failures as device errors.
        throw DiagError(ExitCode::DeviceError, ex.what());
    }
}

uint32_t ecuCanId(common::CarPlatform carPlatform, uint8_t ecuId)
{
    return std::get<1>(common::getEcuInfoByEcuId(carPlatform, ecuId)).canId;
}

common::BusConfiguration selectedRawCanBus(const RunOptions& options)
{
    if (!options.busName.empty()) {
        return common::getBusByName(options.carPlatform, options.busName);
    }
    return std::get<0>(common::getEcuInfoByEcuId(options.carPlatform, options.ecuId));
}

std::unique_ptr<j2534::J2534Channel> openObdChannel(j2534::J2534& j2534, const RunOptions& options)
{
    // Use 0x7E0/0x7E8 for filters and flow control; send requests to 0x7DF.
    constexpr uint8_t kPowertrainEcuId = 0x10;
    constexpr uint32_t kObdEcmRequestCanId = 0x7E0;
    const auto bus = std::get<0>(common::getEcuInfoByEcuId(options.carPlatform, kPowertrainEcuId));
    const auto baudrate = options.baudrateOverride.value_or(bus.baudrate);
    // The configured sample point only applies at the configured baudrate.
    const unsigned long samplePoint = baudrate == bus.baudrate ? bus.samplePoint : 0;
    return common::openUDSChannel(j2534, baudrate, kObdEcmRequestCanId, samplePoint);
}

std::vector<uint8_t> processUds(const j2534::J2534Channel& channel, uint32_t canId,
                                const std::vector<uint8_t>& requestData, size_t timeoutMs)
{
    if (gUdsTrace) {
        gUdsTrace->write(canId, "tx", requestData, "");
    }
    common::UDSRequest request{canId, requestData};
    try {
        auto response = request.process(channel, timeoutMs);
        if (gUdsTrace) {
            gUdsTrace->write(canId, "tx_ok", requestData, "");
            const auto result = response.empty() ? "empty" : "positive";
            if (response.size() >= 5) {
                gUdsTrace->write(canIdFromRawFrame(response), "rx", common::udsPayload(response), result);
            } else {
                gUdsTrace->write(0, "rx", response, result);
            }
        }
        return response;
    }
    catch (const std::exception& ex) {
        if (gUdsTrace) {
            // An ECU negative response is a real RX frame, not a local/transport error.
            // Log it as an rx row with the raw 7F <sid> <nrc> bytes on the response can id,
            // never attributed to the tx id.
            if (const auto* nrc = dynamic_cast<const common::UDSError*>(&ex)) {
                const auto rxCanId = nrc->hasFrameContext() ? nrc->getResponseCanId() : canId;
                gUdsTrace->write(rxCanId, "rx", nrc->negativeResponse(),
                    "NRC " + hexNumber(nrc->getErrorCode(), 2) + " (" + ex.what() + ")");
            } else {
                gUdsTrace->write(canId, "error", {}, ex.what());
            }
        }
        throw;
    }
}

void sendUdsNoWait(const j2534::J2534Channel& channel, uint32_t canId,
                   const std::vector<uint8_t>& requestData, size_t timeoutMs)
{
    if (gUdsTrace) {
        gUdsTrace->write(canId, "tx", requestData, "no_wait");
    }
    common::UDSMessage message{canId, requestData};
    unsigned long numMsgs = 0;
    const auto writeStatus = channel.writeMsgs(message, numMsgs, timeoutMs);
    if (writeStatus != STATUS_NOERROR || numMsgs < 1) {
        throw common::UDSRequestTxError(writeStatus, numMsgs,
            "Failed to send CAN message: " + common::j2534StatusToString(writeStatus));
    }
    if (gUdsTrace) {
        gUdsTrace->write(canId, "tx_ok", requestData, "no_wait");
    }
}

void ensurePayloadPrefix(const std::vector<uint8_t>& payload, const std::vector<uint8_t>& expected)
{
    if (expected.empty()) {
        return;
    }
    if (payload.size() < expected.size() || !std::equal(expected.cbegin(), expected.cend(), payload.cbegin())) {
        // A prefix mismatch is a validation error, not a transport error.
        throw DiagError(ExitCode::ValidationError,
            "Unexpected UDS response payload: " + common::toHexString(payload)
            + " expected prefix: " + common::toHexString(expected));
    }
}

std::vector<uint8_t> processUdsExpecting(const j2534::J2534Channel& channel, uint32_t canId,
                                         const std::vector<uint8_t>& request, size_t timeoutMs,
                                         const UdsExpectation& expect)
{
    if (expect.timeout) {
        try {
            const auto payload = common::udsPayload(processUds(channel, canId, request, timeoutMs));
            throw DiagError(ExitCode::ValidationError,
                "expected no response (timeout) but got: " + common::toHexString(payload));
        }
        catch (const common::UDSRequestRxTimeout&) {
            return {}; // no response, as expected
        }
        catch (const common::UDSError& ex) {
            // An NRC is a response, not a timeout.
            throw DiagError(ExitCode::ValidationError,
                "expected no response (timeout) but got NRC " + hexNumber(ex.getErrorCode(), 2)
                + " (" + ex.what() + ")");
        }
    }
    if (expect.nrc) {
        try {
            const auto payload = common::udsPayload(processUds(channel, canId, request, timeoutMs));
            throw DiagError(ExitCode::ValidationError,
                "expected NRC " + hexNumber(*expect.nrc, 2) + " but got positive response: "
                + common::toHexString(payload));
        }
        catch (const common::UDSError& ex) {
            if (ex.getErrorCode() != *expect.nrc) {
                throw DiagError(ExitCode::ValidationError,
                    "expected NRC " + hexNumber(*expect.nrc, 2) + " but got "
                    + hexNumber(ex.getErrorCode(), 2) + " (" + ex.what() + ")");
            }
            return {}; // matching NRC, as expected
        }
        // Unexpected timeouts remain transport errors.
    }
    const auto payload = common::udsPayload(processUds(channel, canId, request, timeoutMs));
    ensurePayloadPrefix(payload, expect.prefix);
    if (expect.positive && (payload.empty() || payload[0] == 0x7F)) {
        throw DiagError(ExitCode::ValidationError, "expected positive response, got: "
            + common::toHexString(payload));
    }
    return payload;
}

std::vector<common::ECUInfo> getSelectedUdsEcus(const RunOptions& options)
{
    if (!isUdsPlatform(options.carPlatform)) {
        throw std::runtime_error("DTC commands support UDS platforms only");
    }
    if (!options.allEcus) {
        return {std::get<1>(common::getEcuInfoByEcuId(options.carPlatform, options.ecuId))};
    }

    const auto configuration = common::getConfigurationInfoByCarPlatform(options.carPlatform);
    std::vector<common::ECUInfo> ecus;
    std::set<uint32_t> seenIds;
    for (const auto& bus : configuration.busInfo) {
        if (bus.protocolId != ISO15765) {
            continue;
        }
        for (const auto& ecu : bus.ecuInfo) {
            if (ecu.ecuId == 0 || !seenIds.insert(ecu.ecuId).second) {
                continue;
            }
            ecus.push_back(ecu);
        }
    }
    return ecus;
}

std::vector<ReplayFrame> readReplayFrames(const std::string& path)
{
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("Failed to open replay input: " + path);
    }

    std::vector<ReplayFrame> frames;
    std::string line;
    size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (common::trim(line).empty()) {
            continue;
        }
        const auto cells = splitCsvLine(line);
        if (lineNumber == 1 && !cells.empty() && common::toLower(cells[0]) == "time_ms") {
            continue;
        }
        if (cells.size() != 4) {
            throw std::runtime_error("Invalid replay CSV line " + std::to_string(lineNumber)
                + ": expected time_ms,can_id,dlc,data");
        }
        try {
            ReplayFrame frame;
            frame.timeMs = parseDecimalU64(cells[0], "time_ms");
            frame.canId = common::parseHexU32(cells[1]);
            ensureCanIdFits(frame.canId, "can_id");
            const auto dlc = parseDecimalU64(cells[2], "dlc");
            if (dlc > 8) {
                throw std::runtime_error("dlc must be at most 8");
            }
            frame.data = dlc == 0 ? std::vector<uint8_t>{} : common::parseHexBytes(cells[3]);
            if (frame.data.size() != dlc) {
                throw std::runtime_error("dlc does not match data byte count");
            }
            frames.push_back(std::move(frame));
        }
        catch (const std::exception& ex) {
            throw std::runtime_error("Invalid replay CSV line " + std::to_string(lineNumber)
                + ": " + ex.what());
        }
    }
    if (frames.empty()) {
        throw std::runtime_error("Replay input contains no frames: " + path);
    }
    return frames;
}

void setUdsTracePath(const std::string& path)
{
    if (path.empty()) {
        gUdsTrace.reset();
        return;
    }
    gUdsTrace = std::make_unique<UdsTraceWriter>(path);
}

} // namespace volvodiag
