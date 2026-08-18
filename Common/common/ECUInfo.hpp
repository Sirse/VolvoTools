#pragma once

#include "compression/CompressionType.hpp"
#include "encryption/EncryptionType.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace common {

struct ECUInfo {
    uint32_t ecuId;
    uint32_t canId;
    std::string name;
    CompressionType compressionType;
    EncryptionType encryptionType;
    // Vehicle-configuration flags that were present in data.yaml but previously ignored.
    // SBLInPBL: the ECU's bootloader lives in the PBL flash region.
    bool sblInPBL = false;
    // SwdlIssue: the ECU is marked as a known software-download problem in the config.
    bool swdlIssue = false;
    // MasterECU: the CEM-like coordinator of the software-download session.
    bool masterEcu = false;
    // Publicly known SecurityAccess (0x27) PIN, 5 bytes big-endian as fed to authorize.
    // All zeros means "no known PIN" and callers fall back to the current behaviour.
    std::array<uint8_t, 5> securityPin{};

    bool hasSecurityPin() const
    {
        for (const auto byte : securityPin) {
            if (byte != 0) {
                return true;
            }
        }
        return false;
    }
};

} // namespace common
