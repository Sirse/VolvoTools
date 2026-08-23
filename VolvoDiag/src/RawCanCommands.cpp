#include "RawCanCommands.hpp"

#include "DiagContext.hpp"
#include "IsoTpReassemble.hpp"
#include "OutputFormat.hpp"

#include <common/Util.hpp>
#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

namespace volvodiag {

namespace {

using common::ensureCanIdFitsBus;

bool hasRawFilters(const RawCanFilters& filters)
{
    return !filters.ids.empty() || !filters.ranges.empty() || !filters.masks.empty();
}

bool matchesRawFilters(uint32_t canId, const RawCanFilters& filters)
{
    if (!hasRawFilters(filters)) {
        return true;
    }
    if (std::find(filters.ids.cbegin(), filters.ids.cend(), canId) != filters.ids.cend()) {
        return true;
    }
    for (const auto& range : filters.ranges) {
        if (canId >= range.first && canId <= range.second) {
            return true;
        }
    }
    for (const auto& mask : filters.masks) {
        if ((canId & mask.first) == mask.second) {
            return true;
        }
    }
    return false;
}

bool shouldDropTxEcho(uint32_t canId, const TxEchoFilter& filter)
{
    return filter.enabled
        && std::find(filter.ids.cbegin(), filter.ids.cend(), canId) != filter.ids.cend();
}

bool shouldPrintFrame(const PASSTHRU_MSG& msg, const RawCanFilters& filters, const TxEchoFilter& txEchoFilter)
{
    if (msg.DataSize < 4) {
        return false;
    }
    const auto canId = common::canIdFromFrame(msg);
    return !shouldDropTxEcho(canId, txEchoFilter) && matchesRawFilters(canId, filters);
}

// Empty reads and timeouts are normal; real read failures throw.
std::vector<PASSTHRU_MSG> readCanBatch(const j2534::J2534Channel& channel, unsigned long timeoutMs)
{
    std::vector<PASSTHRU_MSG> msgs(32);
    const auto readStatus = channel.readMsgs(msgs, timeoutMs);
    if (msgs.empty() && readStatus != STATUS_NOERROR && readStatus != ERR_TIMEOUT
        && readStatus != ERR_BUFFER_EMPTY) {
        throw std::runtime_error("Failed to read raw CAN frames: " + common::j2534StatusToString(readStatus));
    }
    return msgs;
}

unsigned long remainingMs(std::chrono::steady_clock::time_point deadline,
                          std::chrono::steady_clock::time_point now)
{
    return static_cast<unsigned long>(
        std::max<std::chrono::milliseconds::rep>(
            1,
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));
}

size_t readRawFrames(const j2534::J2534Channel& channel, DiagOutput& output,
                     const RawCanFilters& filters, const TxEchoFilter& txEchoFilter,
                     size_t count, size_t timeoutMs, size_t durationMs)
{
    const auto start = std::chrono::steady_clock::now();
    // Total wait budget: the response timeout counts from the transmit, not per read
    // batch - otherwise a slow ECU answering fewer frames than --count loops forever.
    const auto responseDeadline = start + std::chrono::milliseconds(timeoutMs);
    const auto durationDeadline = durationMs > 0
        ? start + std::chrono::milliseconds(durationMs)
        : std::chrono::steady_clock::time_point::max();
    const auto deadline = std::min(responseDeadline, durationDeadline);
    size_t matchedCount = 0;
    while (!stopRequested.load() && matchedCount < count) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            break;
        }
        const auto msgs = readCanBatch(channel, remainingMs(deadline, now));
        for (const auto& msg : msgs) {
            if (!shouldPrintFrame(msg, filters, txEchoFilter)) {
                continue;
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            output.each([&elapsed, &msg](std::ostream& os) { writeFrame(os, elapsed, msg); });
            ++matchedCount;
            if (matchedCount >= count) {
                break;
            }
        }
    }
    return matchedCount;
}

std::vector<uint8_t> isoTpSingleFramePayload(const std::vector<uint8_t>& payload)
{
    if (payload.size() > 7) {
        throw std::runtime_error("ISO-TP single frame payload must be at most 7 bytes");
    }
    std::vector<uint8_t> frame;
    frame.reserve(1 + payload.size());
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.cbegin(), payload.cend());
    return frame;
}

std::string formatCanFramePayload(const PASSTHRU_MSG& msg)
{
    if (msg.DataSize <= 4) {
        return {};
    }
    std::vector<uint8_t> payload;
    payload.reserve(msg.DataSize - 4);
    for (size_t i = 4; i < msg.DataSize; ++i) {
        payload.push_back(msg.Data[i]);
    }
    return formatBytes(payload);
}

struct ProbeMatch {
    PASSTHRU_MSG frame;
    std::vector<uint8_t> payload;
    // Non-zero when the payload came from first-frame reassembly: the total length the
    // FF declared. A shorter payload means the reassembly was truncated (timeout or
    // sequence gap) and must not be presented as a complete response.
    size_t reassembledLength{ 0 };
};

bool matchesProbeResponse(uint32_t responseId, uint32_t requestId, const RunOptions& options)
{
    if (options.probeUseResponseRange) {
        return responseId >= options.probeResponseRange.first
            && responseId <= options.probeResponseRange.second;
    }
    return responseId == requestId + options.probeResponseOffset;
}

std::optional<ProbeMatch> readProbeResponse(const j2534::J2534Channel& channel,
                                            uint32_t requestId,
                                            const RunOptions& options)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(options.timeoutMs);
    while (!stopRequested.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return std::nullopt;
        }
        const auto msgs = readCanBatch(channel, remainingMs(deadline, now));
        for (const auto& msg : msgs) {
            if (msg.DataSize < 4) {
                continue;
            }
            const auto responseId = common::canIdFromFrame(msg);
            if (options.txEchoFilter.enabled && responseId == requestId) {
                continue;
            }
            if (matchesProbeResponse(responseId, requestId, options)) {
                std::vector<uint8_t> payload;
                payload.reserve(msg.DataSize > 4 ? msg.DataSize - 4 : 0);
                for (size_t i = 4; i < msg.DataSize; ++i) {
                    payload.push_back(msg.Data[i]);
                }
                return ProbeMatch{msg, std::move(payload)};
            }
        }
    }
    return std::nullopt;
}

std::optional<ProbeMatch> readProbeResponseReassembled(const j2534::J2534Channel& channel,
                                                       uint32_t requestId,
                                                       const RunOptions& options)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(options.timeoutMs);
    while (!stopRequested.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return std::nullopt;
        }
        const auto msgs = readCanBatch(channel, remainingMs(deadline, now));
        for (const auto& msg : msgs) {
            if (msg.DataSize < 4) {
                continue;
            }
            const auto responseId = common::canIdFromFrame(msg);
            if (options.txEchoFilter.enabled && responseId == requestId) {
                continue;
            }
            if (!matchesProbeResponse(responseId, requestId, options)) {
                continue;
            }

            std::vector<uint8_t> framePayload;
            framePayload.reserve(msg.DataSize - 4);
            for (size_t i = 4; i < msg.DataSize; ++i) {
                framePayload.push_back(msg.Data[i]);
            }
            if (framePayload.empty()) {
                return ProbeMatch{msg, std::move(framePayload)};
            }

            const auto pci = framePayload[0];
            if ((pci & 0xF0) == 0x00 || (pci & 0xF0) != 0x10 || framePayload.size() < 2) {
                return ProbeMatch{msg, std::move(framePayload)};
            }

            const size_t expectedLength = ((static_cast<size_t>(pci & 0x0F) << 8) | framePayload[1]);
            std::vector<uint8_t> payload;
            payload.reserve(expectedLength);
            for (size_t i = 2; i < framePayload.size() && payload.size() < expectedLength; ++i) {
                payload.push_back(framePayload[i]);
            }
            // ISO-TP: the first CF carries (FF SN + 1), then the sequence rolls 0..15.
            unsigned expectedSequence = ((static_cast<unsigned>(pci) & 0x0Fu) + 1u) & 0x0Fu;
            bool sequenceGap = false;

            while (payload.size() < expectedLength && !stopRequested.load()) {
                const auto now2 = std::chrono::steady_clock::now();
                if (now2 >= deadline) {
                    break;
                }
                const auto followUpMsgs = readCanBatch(channel, remainingMs(deadline, now2));
                for (const auto& followMsg : followUpMsgs) {
                    if (followMsg.DataSize < 5) {
                        // 4-byte CAN id + at least one payload byte; anything shorter
                        // cannot carry a consecutive-frame PCI.
                        continue;
                    }
                    const auto followResponseId = common::canIdFromFrame(followMsg);
                    if (options.txEchoFilter.enabled && followResponseId == requestId) {
                        continue;
                    }
                    if (followResponseId != responseId) {
                        continue;
                    }
                    // Only the byte right after the CAN id is a PCI candidate; the rest is
                    // data copied verbatim. A sequence gap means frames were lost - report
                    // and stop instead of gluing mismatched pieces together.
                    const auto cfStatus = appendConsecutiveFrame(payload, expectedSequence,
                        followMsg.Data + 4, followMsg.DataSize - 4, expectedLength);
                    if (cfStatus == ConsecutiveFrameStatus::SequenceGap) {
                        LOG(WARNING) << "probe reassembly sequence gap on response id=0x"
                                     << std::hex << responseId << ": got SN "
                                     << static_cast<unsigned>(followMsg.Data[4] & 0x0F)
                                     << ", expected " << std::dec << expectedSequence;
                        sequenceGap = true;
                        break;
                    }
                }
                if (sequenceGap) {
                    break;
                }
            }

            ProbeMatch match{ msg, std::move(payload), expectedLength };
            return match;
        }
    }
    return std::nullopt;
}

void writeProbeRow(std::ostream& output, uint32_t requestId, const std::string& responseId,
                   const std::string& status, const std::string& response)
{
    output << formatCanId(requestId)
           << "," << csvEscape(responseId)
           << "," << csvEscape(status)
           << "," << csvEscape(response)
           << std::endl;
}

std::string frameKey(const PASSTHRU_MSG& msg, BaselineKey mode)
{
    const auto canId = common::canIdFromFrame(msg);
    if (mode == BaselineKey::Id) {
        return formatCanId(canId);
    }
    return formatCanId(canId) + ":" + formatCanFramePayload(msg);
}

std::set<std::string> loadBaselineKeys(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open baseline file: " + path);
    }
    std::set<std::string> keys;
    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
            line.pop_back();
        }
        if (!line.empty()) {
            keys.insert(line);
        }
    }
    return keys;
}

void writeBaselineKeys(const std::string& path, const std::set<std::string>& keys)
{
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to write baseline file: " + path);
    }
    for (const auto& key : keys) {
        out << key << "\n";
    }
}

} // namespace

void runCanSend(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag can-send device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " canId=" << formatCanId(options.canId);
    ensureCanIdFitsBus(options.canId, busInfo);
    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);
    const auto frame = common::makeCanFrame(options.canId, options.requestData);

    if (options.txEchoFilter.enabled) {
        std::cerr << "Warning: can send ignores --drop-tx-echo because it does not read RX" << std::endl;
    }

    for (size_t i = 0; i < options.repeatCount; ++i) {
        const auto status = channel->writeMsg(frame, static_cast<unsigned long>(options.timeoutMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to send raw CAN frame: " + common::j2534StatusToString(status));
        }
        std::cout << "TX";
        if (options.repeatCount > 1) {
            std::cout << "[" << (i + 1) << "]";
        }
        std::cout << ": can_id=" << formatCanId(options.canId)
                  << " dlc=" << options.requestData.size()
                  << " data=" << formatBytes(options.requestData)
                  << std::endl;
    }
}

void runCanRequest(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.canOutputPath, "raw CAN output"};

    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag can-request device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " canId=" << formatCanId(options.canId);
    ensureCanIdFitsBus(options.canId, busInfo);
    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);
    const auto frame = common::makeCanFrame(options.canId, options.requestData);

    if (!hasRawFilters(options.rawFilters)) {
        const auto message = "Warning: no --filter-id/--filter-range/--filter-mask set; captured frames may be unrelated "
                             "bus traffic, not the ECU response. Pass a filter (e.g. --filter-id 7E8) to target it.";
        LOG(WARNING) << "can-request without raw filter";
        std::cerr << message << std::endl;
    }

    output.line("time_ms,can_id,dlc,data");

    channel->clearRx();
    const auto status = channel->writeMsg(frame, static_cast<unsigned long>(options.timeoutMs));
    if (status != STATUS_NOERROR) {
        throw std::runtime_error("Failed to send raw CAN request frame: " + common::j2534StatusToString(status));
    }

    const auto received = readRawFrames(*channel, output, options.rawFilters, options.txEchoFilter,
                                        options.responseCount, options.responseTimeoutMs,
                                        options.monitorDurationMs);
    if (received == 0) {
        LOG(WARNING) << "can-request timed out without matching frames";
    }
}

void runCanPeriodic(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag can-periodic device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " canId=0x" << options.canId;
    ensureCanIdFitsBus(options.canId, busInfo);
    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);
    const auto frame = common::makeCanFrame(options.canId, options.requestData);

    // Hardware periodic TX. --count is approximate: there is no frame callback.
    auto periodicMsg = common::makePassThruMsg(channel->getProtocolId(), channel->getTxFlags(), frame);
    unsigned long msgId = 0;
    const auto status = channel->startPeriodicMsg(periodicMsg, msgId,
                                                  static_cast<unsigned long>(options.intervalMs));
    if (status != STATUS_NOERROR) {
        throw std::runtime_error("Failed to start periodic raw CAN message: " + common::j2534StatusToString(status));
    }

    std::cout << "Periodic TX started: can_id=" << formatCanId(options.canId)
              << " dlc=" << options.requestData.size()
              << " data=" << formatBytes(options.requestData)
              << " interval_ms=" << options.intervalMs;
    if (options.repeatCount > 0) {
        std::cout << " count=" << options.repeatCount;
    } else {
        std::cout << " (Ctrl-C to stop)";
    }
    std::cout << std::endl;

    const auto start = std::chrono::steady_clock::now();
    while (!stopRequested.load()) {
        if (options.repeatCount > 0) {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (static_cast<size_t>(elapsedMs) >= options.repeatCount * options.intervalMs) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // Drain frames collected by the pass-all filter.
        channel->clearRx();
    }

    channel->stopPeriodicMsg(msgId);
    std::cout << "Periodic TX stopped" << std::endl;
}

void runCanReplay(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    const auto frames = readReplayFrames(options.replayInputPath);
    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag can-replay device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " frames=" << frames.size();
    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);

    for (size_t i = 0; i < frames.size() && !stopRequested.load(); ++i) {
        if (options.preserveReplayTiming && i > 0 && frames[i].timeMs > frames[i - 1].timeMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(frames[i].timeMs - frames[i - 1].timeMs));
        }
        const auto frame = common::makeCanFrame(frames[i].canId, frames[i].data);
        const auto status = channel->writeMsg(frame, static_cast<unsigned long>(options.timeoutMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to replay raw CAN frame: " + common::j2534StatusToString(status));
        }
        std::cout << "TX[" << (i + 1) << "/" << frames.size() << "]: can_id="
                  << formatCanId(frames[i].canId)
                  << " dlc=" << frames[i].data.size()
                  << " data=" << formatBytes(frames[i].data)
                  << std::endl;
    }
}

void runWake(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag wake device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " canId=" << formatCanId(options.wakeCanId)
              << " burstCount=" << options.wakeBurstCount
              << " hold=" << options.wakeHold;
    ensureCanIdFitsBus(options.wakeCanId, busInfo);

    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);

    const auto wakeData = isoTpSingleFramePayload(options.wakePayload);
    const auto wakeFrame = common::makeCanFrame(options.wakeCanId, wakeData);
    for (size_t i = 0; i < options.wakeBurstCount && !stopRequested.load(); ++i) {
        const auto status = channel->writeMsg(wakeFrame, static_cast<unsigned long>(options.timeoutMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to send wake frame: " + common::j2534StatusToString(status));
        }
        std::cout << "WAKE[" << (i + 1) << "/" << options.wakeBurstCount << "]: can_id="
                  << formatCanId(options.wakeCanId)
                  << " data=" << formatBytes(wakeData)
                  << std::endl;
        if (options.wakeGapMs > 0 && i + 1 < options.wakeBurstCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.wakeGapMs));
        }
    }

    unsigned long holdMsgId = 0;
    bool holdStarted = false;
    if (options.wakeHold && !stopRequested.load()) {
        const auto holdData = isoTpSingleFramePayload(options.wakeHoldPayload);
        const auto holdFrame = common::makeCanFrame(options.wakeCanId, holdData);
        auto periodicMsg = common::makePassThruMsg(channel->getProtocolId(), channel->getTxFlags(), holdFrame);
        const auto status = channel->startPeriodicMsg(periodicMsg, holdMsgId,
                                                      static_cast<unsigned long>(options.intervalMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to start wake hold frame: " + common::j2534StatusToString(status));
        }
        holdStarted = true;
        std::cout << "HOLD started: can_id=" << formatCanId(options.wakeCanId)
                  << " data=" << formatBytes(holdData)
                  << " interval_ms=" << options.intervalMs;
        if (options.wakeHoldMs > 0) {
            std::cout << " hold_ms=" << options.wakeHoldMs;
        } else {
            std::cout << " (Ctrl-C to stop)";
        }
        std::cout << std::endl;

        const auto start = std::chrono::steady_clock::now();
        while (!stopRequested.load()) {
            if (options.wakeHoldMs > 0) {
                const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (static_cast<size_t>(elapsedMs) >= options.wakeHoldMs) {
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            channel->clearRx();
        }
        channel->stopPeriodicMsg(holdMsgId);
        holdStarted = false;
        std::cout << "HOLD stopped" << std::endl;
    }

    if (holdStarted) {
        channel->stopPeriodicMsg(holdMsgId);
    }

    if (options.wakeTeardown && !stopRequested.load()) {
        const auto teardownData = isoTpSingleFramePayload(options.wakeTeardownPayload);
        const auto teardownFrame = common::makeCanFrame(options.wakeCanId, teardownData);
        const auto status = channel->writeMsg(teardownFrame, static_cast<unsigned long>(options.timeoutMs));
        if (status != STATUS_NOERROR) {
            throw std::runtime_error("Failed to send wake teardown frame: " + common::j2534StatusToString(status));
        }
        std::cout << "TEARDOWN: can_id=" << formatCanId(options.wakeCanId)
                  << " data=" << formatBytes(teardownData)
                  << std::endl;
    }
}

void runProbe(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    DiagOutput output{options.probeOutputPath, "probe output"};

    const auto busInfo = selectedRawCanBus(options);
    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag probe device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " from=0x" << std::hex << options.probeFrom
              << " to=0x" << options.probeTo;
    ensureCanIdFitsBus(options.probeFrom, busInfo);
    ensureCanIdFitsBus(options.probeTo, busInfo);

    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);
    const auto probeData = isoTpSingleFramePayload(options.requestData);

    output.line("request_can_id,response_can_id,status,response");

    for (uint32_t requestId = options.probeFrom; requestId <= options.probeTo && !stopRequested.load(); ++requestId) {
        channel->clearRx();
        const auto frame = common::makeCanFrame(requestId, probeData);
        const auto writeStatus = channel->writeMsg(frame, static_cast<unsigned long>(options.timeoutMs));
        if (writeStatus != STATUS_NOERROR) {
            const auto error = common::j2534StatusToString(writeStatus);
            output.each([&](std::ostream& os) { writeProbeRow(os, requestId, "", "tx_error", error); });
            continue;
        }

        const auto response = options.probeReassemble
            ? readProbeResponseReassembled(*channel, requestId, options)
            : readProbeResponse(*channel, requestId, options);
        if (!response) {
            output.each([&](std::ostream& os) { writeProbeRow(os, requestId, "", "timeout", ""); });
        } else {
            const auto responseId = common::canIdFromFrame(response->frame);
            const auto responseIdText = formatCanId(responseId);
            const auto payload = options.probeReassemble
                ? formatBytes(response->payload)
                : formatCanFramePayload(response->frame);
            // A reassembled payload shorter than what the first frame declared was
            // truncated (timeout or sequence gap) - never present it as a complete answer.
            const bool incomplete = options.probeReassemble && response->reassembledLength != 0
                && response->payload.size() < response->reassembledLength;
            output.each([&](std::ostream& os) {
                writeProbeRow(os, requestId, responseIdText, incomplete ? "incomplete" : "ok", payload);
            });
        }

        if (options.probeGapMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.probeGapMs));
        }
    }
}

void runMonitor(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    const auto busInfo = selectedRawCanBus(options);

    DiagOutput output{options.monitorOutputPath, "monitor output"};

    const auto device = common::selectSingleDevice(devices, options.deviceName);
    LOG(INFO) << "VolvoDiag raw CAN monitor device=" << device.deviceName
              << " platform=" << options.platformName
              << " bus=" << busInfo.name
              << " baudrate=" << std::dec << options.baudrateOverride.value_or(busInfo.baudrate);
    auto j2534 = openDevice(device);
    auto channel = common::openRawCanChannel(*j2534, busInfo, options.baudrateOverride);
    channel->clearRx();

    // Compare skips known frames; record saves seen keys on exit.
    std::set<std::string> baseline;
    if (!options.baselinePath.empty()) {
        baseline = loadBaselineKeys(options.baselinePath);
    }
    const bool comparing = !options.baselinePath.empty();
    const bool recording = !options.baselineOutPath.empty();
    std::set<std::string> recorded;

    output.line("time_ms,can_id,dlc,data");
    const auto start = std::chrono::steady_clock::now();
    size_t matchedCount = 0;
    const auto deadline = options.monitorDurationMs > 0
        ? start + std::chrono::milliseconds(options.monitorDurationMs)
        : std::chrono::steady_clock::time_point::max();
    while (!stopRequested.load()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            break;
        }
        const auto msgs = readCanBatch(*channel, 250);
        for (const auto& msg : msgs) {
            if (!shouldPrintFrame(msg, options.rawFilters, options.txEchoFilter)) {
                continue;
            }
            if (comparing || recording) {
                const auto key = frameKey(msg, options.baselineKey);
                if (recording) {
                    recorded.insert(key);
                }
                if (comparing && baseline.count(key) != 0) {
                    continue;
                }
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            output.each([&elapsed, &msg](std::ostream& os) { writeFrame(os, elapsed, msg); });
            ++matchedCount;
            if (options.monitorCount > 0 && matchedCount >= options.monitorCount) {
                break;
            }
        }
        if (options.monitorCount > 0 && matchedCount >= options.monitorCount) {
            break;
        }
    }

    if (options.monitorDurationMs > 0 && matchedCount == 0) {
        LOG(INFO) << "monitor exited after duration without matching frames";
    }

    if (recording) {
        writeBaselineKeys(options.baselineOutPath, recorded);
        std::cout << "Baseline written: " << options.baselineOutPath
                  << " (" << recorded.size() << " keys)" << std::endl;
    }
}

} // namespace volvodiag
