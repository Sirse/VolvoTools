#pragma once

#include "compression/CompressionType.hpp"
#include "encryption/EncryptionType.hpp"

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
};

} // namespace common
