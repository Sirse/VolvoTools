#pragma once

#include "FlasherBase.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace j2534 {
    class J2534;
}

namespace flasher {

    struct UDSReaderParameters {
        const std::array<uint8_t, 5> pin;
        bool skipFallAsleep = false;
        bool attachRunningSbl = false;
        // Skip the SBL-directed SecurityAccess (27 01). A minimal resident/read SBL that
        // only implements 35/36/37 upload does not answer 0x27, so authorizing against it
        // just fails and triggers the ECUReset cleanup. The target-ECU authorize needed to
        // load a fresh SBL is unaffected.
        bool noSblAuth = false;
        uint32_t startAddress = 0;
        uint32_t dataSize = 0;
    };

    class UDSReader : public FlasherBase {
    public:
        UDSReader(j2534::J2534& j2534, FlasherParameters&& flasherParameters,
            UDSReaderParameters&& udsReaderParameters, std::vector<uint8_t>& output);
        ~UDSReader();

    private:
        std::vector<std::unique_ptr<j2534::J2534Channel>> openChannels() override;
        void startImpl(std::vector<std::unique_ptr<j2534::J2534Channel>>& channels) override;

    private:
        UDSReaderParameters _udsReaderParameters;
        std::vector<uint8_t>& _output;
    };

} // namespace flasher
