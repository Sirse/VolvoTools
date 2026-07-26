#include "common/protocols/UDSDid.hpp"

#include "common/Util.hpp"
#include "common/protocols/UDSService.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace common {

namespace {

// Standard ISO 14229-1 identification DIDs (0xF1xx). Most are ASCII strings.
const KnownDid kKnownDids[] = {
    {0xF180, "Boot software id", DidFormat::Ascii},
    {0xF181, "Application software id", DidFormat::Ascii},
    {0xF186, "Active diagnostic session", DidFormat::Hex},
    {0xF187, "Spare part number", DidFormat::Ascii},
    {0xF188, "ECU software number", DidFormat::Ascii},
    {0xF189, "ECU software version", DidFormat::Ascii},
    {0xF18A, "System supplier id", DidFormat::Ascii},
    {0xF18B, "ECU manufacturing date", DidFormat::Hex},
    {0xF18C, "ECU serial number", DidFormat::Ascii},
    {0xF190, "VIN", DidFormat::Ascii},
    {0xF191, "ECU hardware number", DidFormat::Ascii},
    {0xF192, "Supplier hardware number", DidFormat::Ascii},
    {0xF193, "Supplier hardware version", DidFormat::Ascii},
    {0xF194, "ECU software number (supplier)", DidFormat::Ascii},
    {0xF195, "ECU software version (supplier)", DidFormat::Ascii},
    {0xF197, "System name", DidFormat::Ascii},
    {0xF198, "Repair shop / tester id", DidFormat::Ascii},
    {0xF199, "Programming date", DidFormat::Hex},
    {0xF19D, "ECU installation date", DidFormat::Hex},
};

// Volvo-specific identifiers used by the factory software download tooling. The assembly and
// part numbers are marked Ascii because ECUs are inconsistent about them: some answer with text,
// some with packed digits, and Ascii already falls back to hex when the bytes aren't printable.
// The genuinely binary ones stay Hex so a stray printable byte can't render as garbage text.
const KnownDid kVolvoDids[] = {
    {0xD100, "Diagnostic session type", DidFormat::Hex},
    {0xF101, "PBL configuration", DidFormat::Hex},
    {0xF102, "Security constant", DidFormat::Hex},
    {0xF109, "Boot software version number", DidFormat::Ascii},
    {0xF111, "ECU core assembly number", DidFormat::Ascii},
    {0xF113, "ECU delivery assembly number", DidFormat::Ascii},
    {0xF162, "Software download specification version", DidFormat::Ascii},
    {0xF1A3, "KDP core assembly number", DidFormat::Ascii},
    {0xF1A4, "KDP delivery assembly number", DidFormat::Ascii},
    {0xF1AF, "Boot software id (KDP)", DidFormat::Ascii},
    {0xF1B0, "ECU hardware number (KDP)", DidFormat::Ascii},
};

} // namespace

const KnownDid* findKnownDid(uint16_t did)
{
    for (const auto& known : kKnownDids) {
        if (known.id == did) {
            return &known;
        }
    }
    for (const auto& known : kVolvoDids) {
        if (known.id == did) {
            return &known;
        }
    }
    return nullptr;
}

std::string didName(uint16_t did)
{
    if (const auto* known = findKnownDid(did)) {
        return known->name;
    }
    std::stringstream ss;
    ss << "DID 0x" << std::hex << std::setw(4) << std::setfill('0') << did;
    return ss.str();
}

std::optional<std::string> decodeAscii(const std::vector<uint8_t>& data)
{
    size_t end = data.size();
    while (end > 0 && (data[end - 1] == 0x00 || data[end - 1] == 0x20)) {
        --end;
    }
    if (end == 0) {
        return std::nullopt;
    }
    std::string result;
    result.reserve(end);
    for (size_t i = 0; i < end; ++i) {
        const auto byte = data[i];
        if (byte < 0x20 || byte > 0x7E) {
            return std::nullopt;
        }
        result.push_back(static_cast<char>(byte));
    }
    return result;
}

std::string decodeDidValue(uint16_t did, const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        return "(empty)";
    }
    const auto* known = findKnownDid(did);
    const bool preferAscii = !known || known->format == DidFormat::Ascii;
    if (preferAscii) {
        if (const auto ascii = decodeAscii(data)) {
            return *ascii;
        }
    }
    return formatHexBytesLower(data);
}

DidResponse parseDidResponse(const std::vector<uint8_t>& response)
{
    const auto payload = udsPayload(response);
    if (payload.size() < 3 || payload[0] != static_cast<uint8_t>(uds::PositiveResponseId::ReadDataByIdentifier)) {
        throw std::runtime_error("Unexpected ReadDataByIdentifier response: " + formatHexBytesLower(payload));
    }
    const uint16_t did = static_cast<uint16_t>((payload[1] << 8) | payload[2]);
    return {did, {payload.cbegin() + 3, payload.cend()}};
}

} // namespace common
