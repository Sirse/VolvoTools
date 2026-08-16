#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace common {

// The known upload integrity algorithms. Only the two modelled checks survive here; the profile
// is an internal label for the report, not a user-selectable policy. Bare-77 ("None") and
// "Auto" used to be modes: now every read is checked against both algorithms and the verdict is
// diagnostic, never fatal.
enum class UploadIntegrityProfile {
    // 77 <crc_hi> <crc_lo> over the reconstructed memory image (the stripped payload bytes).
    ImageCrc16CcittFalse,
    // 77 <crc_hi> <crc_lo> over SDA 4.7's filtered concatenation of complete positive 0x36
    // responses (including 0x76 and the block counter). Not the same as the image CRC.
    Sda47WireCrc16,
};

std::string uploadIntegrityProfileName(UploadIntegrityProfile profile);

// The outcome of the RequestTransferExit integrity check. None of these is fatal: the dump is
// kept and the exit code stays zero. Warnings are logged and written to the integrity report.
enum class IntegrityStatus {
    NotChecked,     // response was bare 77: SBL sent no CRC, nothing to compare
    Ok,             // the returned CRC matched at least one known algorithm
    Unverified,     // CRC present but matched no known algorithm - keep the dump, flag it
    NoTransferExit, // no positive 77 response at all
};

std::string integrityStatusName(IntegrityStatus status);

// Incrementally computes both the reconstructed-image CRC16-CCITT-FALSE and the SDA 4.7 wire
// CRC in a single pass over complete positive 0x36 responses. Tracks the flattened offset
// (needed for the SDA i mod 64 exclusion), counts blocks, and ignores a duplicate replay of
// the immediately previous block without advancing any state.
class UploadIntegrityAccumulator {
public:
    UploadIntegrityAccumulator();

    void reset();

    // payload is the full positive 0x36 response UDS payload: [0x76, blockCounter, data...].
    // The 0x76 and counter are part of the SDA wire CRC but not of the image CRC. A response
    // whose block counter repeats the immediately previous one is a duplicate replay and is
    // ignored entirely (no CRC advance, no offset advance).
    void addResponse(const std::vector<uint8_t>& payload);

    uint16_t imageCrc() const { return _imageCrc; }
    uint16_t sdaWireCrc() const { return _sdaWireCrc; }
    size_t flattenedOffset() const { return _flattenedOffset; }
    size_t blockCount() const { return _blockCount; }
    uint8_t lastBlockCounter() const { return _lastCounter; }
    bool hasPreviousBlock() const { return _hasPrevious; }

private:
    uint16_t _imageCrc;
    uint16_t _sdaWireCrc;
    size_t _flattenedOffset;
    size_t _blockCount;
    uint8_t _lastCounter;
    bool _hasPrevious;
};

// Parsed RequestTransferExit response and its integrity verdict.
struct TransferExitResult {
    bool responsePresent = false;
    bool crcPresent = false;
    uint16_t returnedCrc = 0;
    IntegrityStatus status = IntegrityStatus::NotChecked;
    // When status == Ok, which algorithm matched. If both matched (the normal case on even
    // 64-byte blocks), this is reported as ImageCrc16CcittFalse per the compatibility rule.
    UploadIntegrityProfile profileUsed = UploadIntegrityProfile::ImageCrc16CcittFalse;
    uint16_t computedImageCrc = 0;
    uint16_t computedSdaWireCrc = 0;
};

// Result of a full 0x35/0x36/0x37 upload. success reflects only that the full requested volume
// was collected and its size matched - the integrity verdict never touches it. The report fields
// carry everything needed to write <dump>.integrity.txt next to the artifact.
struct UploadReadResult {
    bool success = false;
    std::vector<uint8_t> payload;
    std::vector<uint8_t> transferExitResponse;
    IntegrityStatus integrityStatus = IntegrityStatus::NotChecked;
    // True when the 77 response actually carried two CRC bytes. Distinct from integrityStatus:
    // NotChecked is a bare 77 (crcPresent=false), while Unverified means a CRC arrived but
    // matched nothing. The report must not print "ecu_crc: 0x0000" for a response that had none.
    bool crcPresent = false;
    // Meaningful only when integrityStatus == Ok; report as n/a otherwise.
    UploadIntegrityProfile integrityProfileUsed = UploadIntegrityProfile::ImageCrc16CcittFalse;
    uint16_t returnedCrc = 0;
    uint16_t computedImageCrc = 0;
    uint16_t computedSdaWireCrc = 0;
    size_t blockCount = 0;
    uint32_t startAddress = 0;
    uint32_t dataSize = 0;
};

// Parses a full 0x37 response (4-byte CAN id header + payload, matching what UDSRequest
// returns). Sets responsePresent/crcPresent/returnedCrc; leaves the verdict to resolveTransferExit.
TransferExitResult parseTransferExit(const std::vector<uint8_t>& response);

// Applies the integrity verdict over the accumulator's computed CRCs:
//   no 77 -> NoTransferExit; bare 77 -> NotChecked; 77 <crc> matching at least one algorithm ->
//   Ok (both matching is reported as image-crc16); matching none -> Unverified.
TransferExitResult resolveTransferExit(const std::vector<uint8_t>& response,
                                       const UploadIntegrityAccumulator& accumulator);

// Formats the <dump>.integrity.txt body for a completed upload. algorithm is printed only when
// the verdict is Ok (something actually matched); ecu_crc only when the 77 response carried a
// CRC (a bare 77 means the ECU returned nothing - not "returned zero"). Both are n/a otherwise,
// so the report never contradicts its own status line. Extracted for unit testing.
std::string formatIntegrityReport(const UploadReadResult& upload);

} // namespace common
