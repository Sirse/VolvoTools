#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace common {

// Formats the 2-byte OBD/UDS base DTC code, e.g. P0123.
std::string formatDtcBaseCode(uint8_t high, uint8_t middle);

// Formats a 3-byte UDS DTC as base code + failure type, e.g. P0123-45.
std::string formatDtcCode(uint8_t high, uint8_t middle, uint8_t low);

// Decodes the ISO 14229 statusOfDTC byte into a pipe-separated flag list. availabilityMask
// (from the 59 02 response) restricts decoding to bits the ECU actually maintains; pass 0
// to decode every set bit.
std::string decodeDtcStatus(uint8_t status, uint8_t availabilityMask);

struct DtcRecord {
    uint8_t high;
    uint8_t middle;
    uint8_t low;
    uint8_t status;
};

struct DtcReadResult {
    uint8_t availabilityMask;
    std::vector<DtcRecord> records;
};

// Parses a reportDTCByStatusMask (19 02) positive response frame. Drops all-zero padding
// records. Throws std::runtime_error if the response is not a valid 59 02 payload.
DtcReadResult parseDtcByStatusMaskResponse(const std::vector<uint8_t>& response);

struct DtcRecordData {
    uint8_t high;
    uint8_t middle;
    uint8_t low;
    uint8_t status;
    std::vector<uint8_t> data; // raw record payload after the status byte
};

// Parses a 59 04 / 59 06 response: 59 <sub> <dtc hi mid lo> <status> <record bytes...>.
// Per-DID lengths inside the record are ECU-specific, so the payload is returned raw.
// Throws std::runtime_error on a malformed or mismatched subfunction response.
DtcRecordData parseDtcRecordResponse(const std::vector<uint8_t>& response, uint8_t subFunction);

} // namespace common
