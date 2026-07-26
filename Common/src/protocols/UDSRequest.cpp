#include "common/protocols/UDSRequest.hpp"

#include "common/protocols/UDSError.hpp"
#include "common/Util.hpp"

#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <iterator>

namespace common {

UDSRequestTxError::UDSRequestTxError(unsigned long status, unsigned long written, const std::string& message)
    : std::runtime_error{ message }
    , _status{ status }
    , _written{ written }
{
}

unsigned long UDSRequestTxError::status() const
{
    return _status;
}

unsigned long UDSRequestTxError::written() const
{
    return _written;
}

UDSRequestRxTimeout::UDSRequestRxTimeout(const std::string& message)
    : std::runtime_error{ message }
{
}

namespace {

constexpr char kChannelReadTimeoutText[] = "Timed out waiting for data from CAN channel";

// Runs the channel read and translates a channel-level timeout into a UDS one. When the ECU
// answered 7F <sid> 78 during the attempt it asked for more time, so grant it another full read
// window instead of failing, until the pending budget is spent. Nothing is retransmitted: the
// request is still outstanding, we are only waiting longer for its final answer. A silent ECU
// never sets pendingSeen and so still fails after a single ordinary timeout.
template <typename Reader>
void readMsgsWithPendingBudget(Reader&& reader, bool& pendingSeen, size_t pendingTimeout)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(pendingTimeout);
    for (;;) {
        pendingSeen = false;
        try {
            reader();
            return;
        }
        catch (const std::runtime_error& ex) {
            if (std::string(ex.what()) != kChannelReadTimeoutText) {
                throw;
            }
            if (!pendingSeen) {
                throw UDSRequestRxTimeout("UDS RX timeout waiting for matching response");
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                throw UDSRequestRxTimeout("UDS RX timeout: response pending budget of "
                    + std::to_string(pendingTimeout) + " ms exhausted");
            }
            LOG(INFO) << "UDS response pending, extending read window";
        }
    }
}

uint8_t getRequestId(const std::vector<uint8_t>& data)
{
    if(data.empty()) {
        throw std::runtime_error("Can't get request id from empty request");
    }
    return data[0];
}

std::vector<uint8_t> payloadFromFrame(const uint8_t* data, size_t dataSize)
{
    if (dataSize <= 4) {
        return {};
    }
    return { data + 4, data + dataSize };
}

std::vector<uint8_t> frameToVector(const uint8_t* data, size_t dataSize)
{
    return { data, data + dataSize };
}

}

UDSRequest::UDSRequest(uint32_t canId, const std::vector<uint8_t>& data)
    : _canId{ canId }
    , _requestId{ getRequestId(data) }
    , _data{ data }
    , _message{ canId, data }
{
}

UDSRequest::UDSRequest(uint32_t canId, std::vector<uint8_t>&& data)
    : _canId{ canId }
    , _requestId{ getRequestId(data) }
    , _data{ std::move(data) }
    , _message{ canId, _data }
{
}

std::vector<uint8_t> UDSRequest::process(const j2534::J2534Channel& channel, size_t timeout,
                                         size_t pendingTimeout)
{
    unsigned long numMsgs = 0;
    LOG(DEBUG) << "UDS TX can=0x" << std::hex << _canId
               << " data=" << toHexString(_data) << " timeout=" << std::dec << timeout;
    const auto writeStatus = channel.writeMsgs(_message, numMsgs, timeout);
    if(writeStatus != STATUS_NOERROR || numMsgs < 1) {
        LOG(ERROR) << "UDS TX failed can=0x" << std::hex << _canId
                   << " status=" << j2534StatusToString(writeStatus) << " written=" << std::dec << numMsgs;
        throw UDSRequestTxError(writeStatus, numMsgs, "UDS TX failed: " + j2534StatusToString(writeStatus));
    }
    std::vector<uint8_t> result;
    bool pendingSeen = false;
    readMsgsWithPendingBudget([&]() {
    channel.readMsgs([&result, &pendingSeen, this](const uint8_t* data, size_t dataSize) {
        LOG(DEBUG) << "UDS RX raw=" << toHexString(frameToVector(data, dataSize));
        try {
            checkUDSError(_requestId, data, dataSize);
        }
        catch (const UDSError& ex) {
            LOG(WARNING) << "UDS NRC request=0x" << std::hex << static_cast<int>(_requestId)
                         << " nrc=0x" << static_cast<int>(ex.getErrorCode()) << " " << ex.what();
            if (ex.getErrorCode() == UDSError::ErrorCode::RequestReceivedResponsePending) {
                pendingSeen = true;
                return true;
            }
            throw;
        }
        if(dataSize < 5) {
            return true;
        }
        if(data[4] != _requestId + 0x40) {
            return true;
        }
        result.reserve(result.size() + dataSize);
        std::copy(data, data + dataSize, std::back_inserter(result));
        LOG(DEBUG) << "UDS RX accepted payload=" << toHexString(payloadFromFrame(data, dataSize));
        return false;
    }, timeout);
    }, pendingSeen, pendingTimeout);
    if(result.empty()) {
        LOG(WARNING) << "UDS RX completed without payload/no matching response can=0x" << std::hex << _canId
                     << " request=0x" << static_cast<int>(_requestId);
    }
    return result;
}

std::vector<uint8_t> UDSRequest::process(const j2534::J2534Channel& channel,
                                         const std::vector<uint8_t>& checkData,
                                         size_t retryCount, size_t timeout, size_t pendingTimeout)
{
    channel.clearRx();
    unsigned long numMsgs = 0;
    LOG(DEBUG) << "UDS TX can=0x" << std::hex << _canId
               << " data=" << toHexString(_data) << " expect=" << toHexString(checkData)
               << " timeout=" << std::dec << timeout;
    const auto writeStatus = channel.writeMsgs(_message, numMsgs, timeout);
    if(writeStatus != STATUS_NOERROR || numMsgs < 1) {
        LOG(ERROR) << "UDS TX failed can=0x" << std::hex << _canId
                   << " status=" << j2534StatusToString(writeStatus) << " written=" << std::dec << numMsgs;
        throw UDSRequestTxError(writeStatus, numMsgs, "UDS TX failed: " + j2534StatusToString(writeStatus));
    }
    std::vector<uint8_t> result;
    bool acceptedResponse = false;
    bool pendingSeen = false;
    readMsgsWithPendingBudget([&]() {
    channel.readMsgs([&result, &checkData, &retryCount, &acceptedResponse, &pendingSeen, this](const uint8_t* data, size_t dataSize) {
        LOG(DEBUG) << "UDS RX raw=" << toHexString(frameToVector(data, dataSize));
        try {
            checkUDSError(_requestId, data, dataSize);
        }
        catch (const UDSError& ex) {
            LOG(WARNING) << "UDS NRC request=0x" << std::hex << static_cast<int>(_requestId)
                         << " nrc=0x" << static_cast<int>(ex.getErrorCode()) << " " << ex.what();
            if (ex.getErrorCode() == UDSError::ErrorCode::RequestReceivedResponsePending) {
                pendingSeen = true;
                return true;
            }
            throw;
        }
        size_t dataOffset = 4;
        if(dataSize < dataOffset + 1 + checkData.size()) {
            return true;
        }
        if(data[dataOffset] != _requestId + 0x40) {
            return true;
        }
        ++dataOffset;
        const auto areResultEqual{std::equal(checkData.cbegin(), checkData.cend(), data + dataOffset)};
        if (!areResultEqual) {
            const std::vector<uint8_t> actualCheck{ data + dataOffset, data + dataOffset + checkData.size() };
            LOG(WARNING) << "UDS RX positive response check mismatch can=0x" << std::hex << _canId
                         << " request=0x" << static_cast<int>(_requestId)
                         << " expected=" << toHexString(checkData)
                         << " actual=" << toHexString(actualCheck)
                         << " raw=" << toHexString(frameToVector(data, dataSize));
            if(--retryCount == 0) {
                throw std::runtime_error("Failed to receive correct answer");
            }
            return true;
        }
        dataOffset += checkData.size();
        acceptedResponse = true;
        result.reserve(result.size() + dataSize);
        std::copy(data + dataOffset, data + dataSize, std::back_inserter(result));
        LOG(DEBUG) << "UDS RX accepted positive response can=0x" << std::hex << _canId
                   << " request=0x" << static_cast<int>(_requestId)
                   << " strippedSid=0x" << static_cast<int>(_requestId + 0x40)
                   << " strippedCheck=" << toHexString(checkData)
                   << " payloadAfterStrip=" << toHexString(result)
                   << " rawPayload=" << toHexString(payloadFromFrame(data, dataSize));
        return false;
    }, timeout);
    }, pendingSeen, pendingTimeout);
    if (result.empty() && acceptedResponse) {
        LOG(DEBUG) << "UDS RX accepted response without payload can=0x" << std::hex << _canId
                   << " request=0x" << static_cast<int>(_requestId);
    }
    else if(result.empty()) {
        LOG(WARNING) << "UDS RX completed without payload/no matching response can=0x" << std::hex << _canId
                     << " request=0x" << static_cast<int>(_requestId);
    }
    return result;
}

} // namespace common
