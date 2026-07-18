#pragma once

#include <cstdint>
#include <deque>
#include <vector>

namespace j2534 { class J2534Channel; }

namespace volvodiag {

struct SoftIsoTpRawFrame {
    uint32_t canId{0};
    uint32_t driverTimestamp{0};
    std::vector<uint8_t> data;
};

struct SoftIsoTpOptions {
    uint8_t padding{0x00};
    bool padToEight{false};
    uint8_t flowControlBlockSize{0};
    uint8_t flowControlStmin{0};
};

class SoftIsoTp final {
public:
    SoftIsoTp(j2534::J2534Channel& raw, uint32_t txId, uint32_t rxId,
              SoftIsoTpOptions options = {});

    void sendRequest(const std::vector<uint8_t>& payload, size_t timeoutMs);
    std::vector<uint8_t> receiveResponse(size_t timeoutMs);
    std::vector<SoftIsoTpRawFrame> takeNonIsoTpFrames();

private:
    j2534::J2534Channel& _raw;
    uint32_t _txId;
    uint32_t _rxId;
    SoftIsoTpOptions _options;
    std::deque<std::vector<uint8_t>> _rxFrames;
    std::vector<SoftIsoTpRawFrame> _nonIsoTpFrames;
};

} // namespace volvodiag
