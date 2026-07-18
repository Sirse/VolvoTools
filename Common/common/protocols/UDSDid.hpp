#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace common {

enum class DidFormat {
    Ascii,
    Hex,
};

struct KnownDid {
    uint16_t id;
    const char* name;
    DidFormat format;
};

// Looks up a standard ISO 14229-1 identification DID, or nullptr if unknown.
const KnownDid* findKnownDid(uint16_t did);

// Human-readable name of a DID ("VIN", "ECU serial number", ...) or "DID 0x...." if unknown.
std::string didName(uint16_t did);

// Returns the printable string if every (non trailing-padding) byte is printable ASCII.
std::optional<std::string> decodeAscii(const std::vector<uint8_t>& data);

// Human-readable value for a DID payload. Known ASCII DIDs (and unknown DIDs that
// happen to be printable) render as text; everything else falls back to hex.
std::string decodeDidValue(uint16_t did, const std::vector<uint8_t>& data);

struct DidResponse {
    uint16_t did;
    std::vector<uint8_t> data;
};

// Parses a ReadDataByIdentifier (0x22) positive response frame: payload is
// 62 <did_hi> <did_lo> <data...>. Throws std::runtime_error on malformed input.
DidResponse parseDidResponse(const std::vector<uint8_t>& response);

} // namespace common
