#pragma once

#include "FlasherBase.hpp"

#include <common/protocols/UploadIntegrity.hpp>

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

    // Commits the upload payload into the caller's output buffer BEFORE any failure decision,
    // then reports whether the read itself succeeded. Integrity verdicts (Unverified,
    // NotChecked, NoTransferExit) are never a failure: a dump whose bytes arrived and passed the
    // size check is kept no matter what the closing 77 said. UDSReader calls this; it is
    // extracted so the ordering regression (payload dropped after a throwing setFailure) is
    // covered by a unit test without hardware.
    bool commitUploadResult(const common::UploadReadResult& upload, std::vector<uint8_t>& output);

    class UDSReader : public FlasherBase {
    public:
        UDSReader(j2534::J2534& j2534, FlasherParameters&& flasherParameters,
            UDSReaderParameters&& udsReaderParameters, std::vector<uint8_t>& output);
        ~UDSReader();

        // The last upload result, including the integrity verdict and both computed CRCs. Filled
        // after startImpl reaches the read step; empty/default otherwise. The CLI uses it to
        // write <dump>.integrity.txt next to the artifact.
        const common::UploadReadResult& getLastUploadResult() const { return _lastUploadResult; }

    private:
        std::vector<std::unique_ptr<j2534::J2534Channel>> openChannels() override;
        void startImpl(std::vector<std::unique_ptr<j2534::J2534Channel>>& channels) override;

    private:
        UDSReaderParameters _udsReaderParameters;
        std::vector<uint8_t>& _output;
        common::UploadReadResult _lastUploadResult;
    };

} // namespace flasher
