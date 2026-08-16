#include "common/protocols/UploadIntegrity.hpp"

#include "common/Util.hpp"

#include <stdexcept>

namespace common {

namespace {

uint16_t initialCrc()
{
    return 0xFFFF;
}

// Compact uppercase hex without separators, e.g. 8A3F, for the integrity report.
std::string compactHex(uint16_t value)
{
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string out(4, '0');
    out[0] = kDigits[(value >> 12) & 0xF];
    out[1] = kDigits[(value >> 8) & 0xF];
    out[2] = kDigits[(value >> 4) & 0xF];
    out[3] = kDigits[value & 0xF];
    return out;
}

std::string compactHex32(uint32_t value)
{
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string out(8, '0');
    for (int i = 0; i < 8; ++i) {
        out[7 - i] = kDigits[value & 0xF];
        value >>= 4;
    }
    return out;
}

}

std::string uploadIntegrityProfileName(UploadIntegrityProfile profile)
{
    switch (profile) {
    case UploadIntegrityProfile::ImageCrc16CcittFalse:
        return "image-crc16";
    case UploadIntegrityProfile::Sda47WireCrc16:
        return "sda47-wire-crc16";
    }
    return "unknown";
}

std::string integrityStatusName(IntegrityStatus status)
{
    switch (status) {
    case IntegrityStatus::NotChecked:
        return "not-checked";
    case IntegrityStatus::Ok:
        return "ok";
    case IntegrityStatus::Unverified:
        return "unverified";
    case IntegrityStatus::NoTransferExit:
        return "no-transfer-exit";
    }
    return "unknown";
}

UploadIntegrityAccumulator::UploadIntegrityAccumulator()
    : _imageCrc{ initialCrc() }
    , _sdaWireCrc{ initialCrc() }
    , _flattenedOffset{ 0 }
    , _blockCount{ 0 }
    , _lastCounter{ 0 }
    , _hasPrevious{ false }
{
}

void UploadIntegrityAccumulator::reset()
{
    _imageCrc = initialCrc();
    _sdaWireCrc = initialCrc();
    _flattenedOffset = 0;
    _blockCount = 0;
    _lastCounter = 0;
    _hasPrevious = false;
}

void UploadIntegrityAccumulator::addResponse(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 2 || payload[0] != 0x76) {
        // Malformed positive TransferData response. Fail loud rather than guess.
        throw std::runtime_error("UploadIntegrityAccumulator: expected a positive 0x76 response");
    }
    const uint8_t counter = payload[1];
    if (_hasPrevious && counter == _lastCounter) {
        // Duplicate replay of the previous block: no CRC advance, no offset advance.
        return;
    }
    _lastCounter = counter;
    _hasPrevious = true;
    ++_blockCount;

    // Image CRC covers the reconstructed memory image: the payload bytes after 0x76/counter.
    for (size_t i = 2; i < payload.size(); ++i) {
        _imageCrc = crc16Update(_imageCrc, payload[i]);
    }

    // SDA 4.7 wire CRC covers the flattened concatenation of complete positive responses:
    // flat[i] for i = 2 .. len(flat)-1, except where (i mod 64) is 0 or 1. The first two bytes
    // of the whole stream are excluded by the i >= 2 rule; the per-response 0x76/counter pair
    // is excluded by the modulo rule whenever responses are 64 bytes long, but a shorter
    // boundary response shifts the offset and the exclusion lands elsewhere.
    for (size_t j = 0; j < payload.size(); ++j) {
        const size_t i = _flattenedOffset + j;
        if (i < 2) {
            continue;
        }
        if (i % 64 == 0 || i % 64 == 1) {
            continue;
        }
        _sdaWireCrc = crc16Update(_sdaWireCrc, payload[j]);
    }
    _flattenedOffset += payload.size();
}

TransferExitResult parseTransferExit(const std::vector<uint8_t>& response)
{
    TransferExitResult result;
    if (response.size() < 5 || response[4] != 0x77) {
        return result;
    }
    result.responsePresent = true;
    if (response.size() >= 7) {
        result.crcPresent = true;
        result.returnedCrc = (static_cast<uint16_t>(response[5]) << 8) | response[6];
    }
    return result;
}

TransferExitResult resolveTransferExit(const std::vector<uint8_t>& response,
                                       const UploadIntegrityAccumulator& accumulator)
{
    TransferExitResult result = parseTransferExit(response);
    result.computedImageCrc = accumulator.imageCrc();
    result.computedSdaWireCrc = accumulator.sdaWireCrc();

    if (!result.responsePresent) {
        result.status = IntegrityStatus::NoTransferExit;
        return result;
    }
    if (!result.crcPresent) {
        result.status = IntegrityStatus::NotChecked;
        return result;
    }

    const bool imageMatches = result.returnedCrc == result.computedImageCrc;
    const bool sdaMatches = result.returnedCrc == result.computedSdaWireCrc;
    if (imageMatches && sdaMatches) {
        // On even 64-byte blocks the two sums coincide by construction; that is the normal
        // healthy answer, not an ambiguity. Report it as the image CRC per the compatibility rule.
        result.status = IntegrityStatus::Ok;
        result.profileUsed = UploadIntegrityProfile::ImageCrc16CcittFalse;
    }
    else if (imageMatches) {
        result.status = IntegrityStatus::Ok;
        result.profileUsed = UploadIntegrityProfile::ImageCrc16CcittFalse;
    }
    else if (sdaMatches) {
        result.status = IntegrityStatus::Ok;
        result.profileUsed = UploadIntegrityProfile::Sda47WireCrc16;
    }
    else {
        result.status = IntegrityStatus::Unverified;
    }
    return result;
}

std::string formatIntegrityReport(const UploadReadResult& upload)
{
    std::string report;
    report += "status: " + integrityStatusName(upload.integrityStatus) + "\n";
    report += "algorithm: "
        + (upload.integrityStatus == IntegrityStatus::Ok
            ? uploadIntegrityProfileName(upload.integrityProfileUsed)
            : "n/a")
        + "\n";
    if (upload.crcPresent) {
        report += "ecu_crc: 0x" + compactHex(upload.returnedCrc) + "\n";
    }
    else {
        report += "ecu_crc: n/a\n";
    }
    report += "image_crc16: 0x" + compactHex(upload.computedImageCrc) + "\n";
    report += "sda47_wire_crc16: 0x" + compactHex(upload.computedSdaWireCrc) + "\n";
    report += "blocks: " + std::to_string(upload.blockCount) + "\n";
    report += "size: 0x" + compactHex32(upload.dataSize) + "\n";
    report += "start_address: 0x" + compactHex32(upload.startAddress) + "\n";
    return report;
}

} // namespace common
