#include "SoftIsoTp.hpp"

#include <common/Util.hpp>
#include <j2534/J2534Channel.hpp>
#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace volvodiag {

namespace {

struct FrameStamp {
    long long t;
    long long dt;
};

FrameStamp frameStamp()
{
    static const auto start = std::chrono::steady_clock::now();
    static auto last = start;
    const auto now = std::chrono::steady_clock::now();
    FrameStamp stamp{
        std::chrono::duration_cast<std::chrono::microseconds>(now - start).count(),
        std::chrono::duration_cast<std::chrono::microseconds>(now - last).count()};
    last = now;
    return stamp;
}

std::vector<uint8_t> frameData(const PASSTHRU_MSG& message)
{
    if (message.DataSize < 4) return {};
    return {message.Data + 4, message.Data + message.DataSize};
}

std::vector<uint8_t> paddedFrame(uint32_t canId, const std::vector<uint8_t>& data,
                                 uint8_t padding, bool padToEight)
{
    if (data.size() > 8) throw std::runtime_error("ISO-TP CAN frame exceeds 8 bytes");
    auto frame = common::makeCanFrame(canId, data);
    if (padToEight && data.size() < 8) {
        frame.resize(12, padding);
    }
    return frame;
}

unsigned long remainingMs(std::chrono::steady_clock::time_point deadline)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return 0;
    return static_cast<unsigned long>(std::max<int64_t>(1,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));
}

std::vector<uint8_t> readMatching(j2534::J2534Channel& raw, uint32_t rxId,
                                  std::chrono::steady_clock::time_point deadline,
                                  std::deque<std::vector<uint8_t>>& pending,
                                  std::vector<SoftIsoTpRawFrame>& nonIsoTpFrames)
{
    constexpr unsigned long kReadSliceMs = 5;
    while (remainingMs(deadline) != 0) {
        if (!pending.empty()) {
            auto data = std::move(pending.front());
            pending.pop_front();
            return data;
        }
        std::vector<PASSTHRU_MSG> messages(32);
        const auto status = raw.readMsgs(messages, std::min<unsigned long>(
            remainingMs(deadline), kReadSliceMs));
        if (status != STATUS_NOERROR && status != ERR_TIMEOUT && status != ERR_BUFFER_EMPTY) {
            throw std::runtime_error("ISO-TP raw read failed: " + common::j2534StatusToString(status));
        }
        for (const auto& message : messages) {
            if (message.DataSize < 4) continue;
            const auto canId = common::canIdFromFrame(message);
            auto data = frameData(message);
            if (canId != rxId) {
                if (!data.empty() && (data[0] >> 4) > 3)
                    nonIsoTpFrames.push_back({canId, message.Timestamp, std::move(data)});
                continue;
            }
            const auto stamp = frameStamp();
            LOG(DEBUG) << "SoftIsoTp RX id=0x" << std::hex << rxId
                       << " dlc=" << std::dec << data.size()
                       << " data=" << common::toHexString(data)
                       << " t=" << stamp.t << "us dt=" << stamp.dt << "us"
                       << " drv=" << message.Timestamp;
            if (!data.empty() && (data[0] >> 4) <= 3) {
                pending.push_back(std::move(data));
            } else if (!data.empty()) {
                nonIsoTpFrames.push_back({canId, message.Timestamp, std::move(data)});
            }
        }
        if (!pending.empty()) {
            auto data = std::move(pending.front());
            pending.pop_front();
            return data;
        }
    }
    throw std::runtime_error("ISO-TP response timeout");
}

void sendFrame(j2534::J2534Channel& raw, uint32_t id, const std::vector<uint8_t>& data,
               uint8_t padding, bool padToEight, size_t timeoutMs)
{
    const auto frame = paddedFrame(id, data, padding, padToEight);
    const auto stamp = frameStamp();
    LOG(DEBUG) << "SoftIsoTp TX id=0x" << std::hex << id
               << " dlc=" << std::dec << (frame.size() - 4)
               << " data=" << common::toHexString(std::vector<uint8_t>(frame.begin() + 4, frame.end()))
               << " t=" << stamp.t << "us dt=" << stamp.dt << "us";
    const auto status = raw.writeMsg(frame, static_cast<unsigned long>(timeoutMs));
    if (status != STATUS_NOERROR) {
        throw std::runtime_error("ISO-TP raw write failed: " + common::j2534StatusToString(status));
    }
}

unsigned stminMs(uint8_t value)
{
    if (value <= 0x7F) return value;
    if (value >= 0xF1 && value <= 0xF9) return 1; // round microseconds up to 1 ms
    throw std::runtime_error("Unsupported ISO-TP STmin");
}

bool isResponsePending(const std::vector<uint8_t>& frame)
{
    return frame.size() >= 4
        && (frame[0] >> 4) == 0
        && (frame[0] & 0x0F) >= 3
        && frame[1] == 0x7F
        && frame[3] == 0x78;
}

} // namespace

SoftIsoTp::SoftIsoTp(j2534::J2534Channel& raw, uint32_t txId, uint32_t rxId,
                     SoftIsoTpOptions options)
    : _raw(raw), _txId(txId), _rxId(rxId), _options(options) {}

void SoftIsoTp::sendRequest(const std::vector<uint8_t>& payload, size_t timeoutMs)
{
    if (payload.empty() || payload.size() > 4095) throw std::runtime_error("Invalid ISO-TP payload length");
    // Drop any ISO-TP frames left over from a prior transaction so a stray frame
    // cannot desync this request/response pair.
    _rxFrames.clear();
    _nonIsoTpFrames.clear();
    if (payload.size() <= 7) {
        std::vector<uint8_t> data{static_cast<uint8_t>(payload.size())};
        data.insert(data.end(), payload.begin(), payload.end());
        sendFrame(_raw, _txId, data, _options.padding, _options.padToEight, timeoutMs);
        return;
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    std::vector<uint8_t> first{static_cast<uint8_t>(0x10 | ((payload.size() >> 8) & 0x0F)),
        static_cast<uint8_t>(payload.size())};
    first.insert(first.end(), payload.begin(), payload.begin() + 6);
    sendFrame(_raw, _txId, first, _options.padding, _options.padToEight, timeoutMs);
    auto readFlowControl = [&]() {
        auto frame = readMatching(_raw, _rxId, deadline, _rxFrames, _nonIsoTpFrames);
        while (isResponsePending(frame)) {
            LOG(DEBUG) << "SoftIsoTp ignoring responsePending while waiting for Flow Control";
            deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
            frame = readMatching(_raw, _rxId, deadline, _rxFrames, _nonIsoTpFrames);
        }
        return frame;
    };
    auto flow = readFlowControl();
    if (flow.size() < 3) throw std::runtime_error("ISO-TP Flow Control frame is truncated");
    while ((flow[0] >> 4) == 3 && (flow[0] & 0x0F) == 1) {
        flow = readFlowControl();
        if (flow.size() < 3) throw std::runtime_error("ISO-TP Flow Control frame is truncated");
    }
    if ((flow[0] >> 4) != 3 || (flow[0] & 0x0F) == 2)
        throw std::runtime_error("ISO-TP Flow Control overflow or invalid frame");
    if (flow[0] & 0x0F) throw std::runtime_error("ISO-TP Flow Control status not CTS");
    unsigned blockSize = flow[1];
    unsigned delay = stminMs(flow[2]);
    size_t offset = 6;
    uint8_t sequence = 1;
    size_t inBlock = 0;
    while (offset < payload.size()) {
        if (blockSize != 0 && inBlock == blockSize) {
            auto nextFlow = readFlowControl();
            if (nextFlow.size() < 3)
                throw std::runtime_error("ISO-TP Flow Control frame is truncated");
            while ((nextFlow[0] >> 4) == 3 && (nextFlow[0] & 0x0F) == 1) {
                nextFlow = readFlowControl();
                if (nextFlow.size() < 3)
                    throw std::runtime_error("ISO-TP Flow Control frame is truncated");
            }
            if ((nextFlow[0] >> 4) != 3 || (nextFlow[0] & 0x0F) != 0)
                throw std::runtime_error("ISO-TP Flow Control overflowed or invalid");
            // Every CTS reprograms BS/STmin for the blocks that follow it.
            blockSize = nextFlow[1];
            delay = stminMs(nextFlow[2]);
            inBlock = 0;
        }
        if (delay != 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        const auto count = std::min<size_t>(7, payload.size() - offset);
        std::vector<uint8_t> data{static_cast<uint8_t>(0x20 | (sequence & 0x0F))};
        data.insert(data.end(), payload.begin() + offset, payload.begin() + offset + count);
        sendFrame(_raw, _txId, data, _options.padding, _options.padToEight, timeoutMs);
        offset += count;
        sequence = static_cast<uint8_t>((sequence + 1) & 0x0F);
        ++inBlock;
    }
}

std::vector<uint8_t> SoftIsoTp::receiveResponse(size_t timeoutMs)
{
    const auto initial = std::chrono::milliseconds(timeoutMs);
    auto deadline = std::chrono::steady_clock::now() + initial;
    unsigned pendingCount = 0;
    for (;;) {
        const auto first = readMatching(_raw, _rxId, deadline, _rxFrames, _nonIsoTpFrames);
        const auto type = first[0] >> 4;
        if (type == 0) {
            const auto length = first[0] & 0x0F;
            if (length == 0) throw std::runtime_error("ISO-TP Single Frame declares zero length");
            if (length > first.size() - 1) throw std::runtime_error("Truncated ISO-TP Single Frame");
            std::vector<uint8_t> result(first.begin() + 1, first.begin() + 1 + length);
            if (result.size() >= 3 && result[0] == 0x7F && result[2] == 0x78) {
                if (++pendingCount > 3) throw std::runtime_error("Too many ISO-TP responsePending replies");
                deadline = std::chrono::steady_clock::now() + initial;
                continue;
            }
            return result;
        }
        if (type != 1 || first.size() < 2) throw std::runtime_error("Invalid ISO-TP First Frame");
        // A low nibble of 0 marks the ISO-TP escape form with a 32-bit length in bytes
        // 2..5; parsing it as a 12-bit length would silently misread the frame.
        if ((first[0] & 0x0F) == 0)
            throw std::runtime_error("ISO-TP First Frame with escaped 32-bit length is not supported");
        const size_t length = ((first[0] & 0x0F) << 8) | first[1];
        std::vector<uint8_t> result(first.begin() + 2, first.end());
        sendFrame(_raw, _txId, {0x30, _options.flowControlBlockSize, _options.flowControlStmin},
            _options.padding, _options.padToEight, timeoutMs);
        uint8_t sequence = 1;
        while (result.size() < length) {
            const auto next = readMatching(_raw, _rxId, deadline, _rxFrames, _nonIsoTpFrames);
            if ((next[0] >> 4) != 2 || (next[0] & 0x0F) != sequence)
                throw std::runtime_error("ISO-TP Consecutive Frame sequence mismatch");
            result.insert(result.end(), next.begin() + 1, next.end());
            sequence = static_cast<uint8_t>((sequence + 1) & 0x0F);
        }
        result.resize(length);
        return result;
    }
}

std::vector<SoftIsoTpRawFrame> SoftIsoTp::takeNonIsoTpFrames()
{
    auto frames = std::move(_nonIsoTpFrames);
    _nonIsoTpFrames.clear();
    return frames;
}

} // namespace volvodiag
