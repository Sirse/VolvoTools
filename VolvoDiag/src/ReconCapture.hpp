#pragma once

#include "VolvoDiagOptions.hpp"

#include <common/DeviceInfo.hpp>

#include <cstdint>
#include <vector>

namespace volvodiag {

struct ReconFrame {
    uint64_t timeMs{0};
    uint32_t canId{0};
    std::vector<uint8_t> data;
};

struct ReconImage {
    uint32_t address{0};
    std::vector<uint8_t> bytes;
    std::vector<uint8_t> sequence;
};

ReconImage reconstructReconImage(const std::vector<ReconFrame>& frames,
                                 uint32_t markerId,
                                 const std::vector<uint8_t>& signature,
                                 size_t frameCount);

void runReconCapture(const std::vector<j2534::DeviceInfo>& devices,
                     const RunOptions& options);

} // namespace volvodiag
