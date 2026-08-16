#pragma once

#include "UDSMessage.hpp"

#include "j2534/J2534Channel.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace common {

class UDSRequestTxError : public std::runtime_error {
public:
    UDSRequestTxError(unsigned long status, unsigned long written, const std::string& message);

    unsigned long status() const;
    unsigned long written() const;

private:
    unsigned long _status;
    unsigned long _written;
};

class UDSRequestRxTimeout : public std::runtime_error {
public:
    explicit UDSRequestRxTimeout(const std::string& message);
};

// Total extra time an ECU may buy itself by answering 7F <sid> 78 (RequestReceivedResponsePending).
// This is the P4CANMax of the vehicle configuration (300000 ms, confirmed in common/data.yaml on
// every bus) and is deliberately separate from the per-read timeout: erase and transfer
// legitimately keep an ECU busy for minutes, while the plain timeout is sized for a normal round
// trip. The budget only keeps ticking while the ECU actually emits pending responses, so a silent
// ECU still fails after one ordinary timeout.
//
// Do not "align" this with the SDA trace value P2*CAN: 4500 - that is the server's P2* (initial
// response-pending) timing, a different parameter. P4CANMax is the tester-side total pending
// budget and the correct value here.
constexpr size_t kResponsePendingTimeout = 300000;

class UDSRequest {
public:
    UDSRequest(uint32_t canId, const std::vector<uint8_t>& data);
    UDSRequest(uint32_t canId, std::vector<uint8_t>&& data);

    std::vector<uint8_t> process(const j2534::J2534Channel& channel, size_t timeout = 1000,
                                 size_t pendingTimeout = kResponsePendingTimeout);
    std::vector<uint8_t> process(const j2534::J2534Channel& channel, const std::vector<uint8_t>& checkData,
                                 size_t retryCount = 1, size_t timeout = 1000,
                                 size_t pendingTimeout = kResponsePendingTimeout);

private:
    uint32_t _canId;
    uint8_t _requestId;
    std::vector<uint8_t> _data;
    UDSMessage _message;
};

} // namespace common
