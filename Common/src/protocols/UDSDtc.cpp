#include "common/protocols/UDSDtc.hpp"

#include "common/Util.hpp"
#include "common/protocols/UDSService.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace common {

std::string formatDtcBaseCode(uint8_t high, uint8_t middle)
{
    char family = 'P';
    switch (high & 0xC0) {
    case 0x40:
        family = 'C';
        break;
    case 0x80:
        family = 'B';
        break;
    case 0xC0:
        family = 'U';
        break;
    default:
        family = 'P';
        break;
    }

    std::stringstream ss;
    ss << family
       << std::uppercase << std::hex
       << ((high & 0x30) >> 4)
       << (high & 0x0F)
       << std::setw(2) << std::setfill('0') << static_cast<unsigned>(middle);
    return ss.str();
}

std::string formatDtcCode(uint8_t high, uint8_t middle, uint8_t low)
{
    // 2-byte base code (family + 4 hex digits) plus the UDS 3rd byte as failure type,
    // e.g. P0123-45, matching how scan tools present 3-byte UDS DTCs.
    std::stringstream ss;
    ss << formatDtcBaseCode(high, middle)
       << "-" << std::uppercase << std::hex
       << std::setw(2) << std::setfill('0') << static_cast<unsigned>(low);
    return ss.str();
}

std::string decodeDtcStatus(uint8_t status, uint8_t availabilityMask)
{
    struct StatusBit {
        uint8_t mask;
        const char* name;
    };
    static const StatusBit statusBits[] = {
        {uds::DtcStatusMask::TestFailed, "testFailed"},
        {uds::DtcStatusMask::TestFailedThisCycle, "testFailedThisCycle"},
        {uds::DtcStatusMask::Pending, "pending"},
        {uds::DtcStatusMask::Confirmed, "confirmed"},
        {uds::DtcStatusMask::TestNotCompletedSinceClear, "testNotCompletedSinceClear"},
        {uds::DtcStatusMask::TestFailedSinceClear, "testFailedSinceClear"},
        {uds::DtcStatusMask::TestNotCompletedThisCycle, "testNotCompletedThisCycle"},
        {uds::DtcStatusMask::WarningIndicator, "warningIndicator"},
    };

    std::string result;
    for (const auto& bit : statusBits) {
        if (availabilityMask != 0 && (availabilityMask & bit.mask) == 0) {
            continue;
        }
        if (status & bit.mask) {
            if (!result.empty()) {
                result += "|";
            }
            result += bit.name;
        }
    }
    return result.empty() ? "none" : result;
}

DtcReadResult parseDtcByStatusMaskResponse(const std::vector<uint8_t>& response)
{
    const auto payload = udsPayload(response);
    if (payload.size() < 3
        || payload[0] != static_cast<uint8_t>(uds::PositiveResponseId::ReadDTCInformation)
        || payload[1] != static_cast<uint8_t>(uds::ReadDTCSubFunction::ReportDTCByStatusMask)
        || (payload.size() - 3) % 4 != 0) {
        throw std::runtime_error("Unexpected DTC response: " + formatHexBytesLower(payload));
    }

    DtcReadResult result;
    result.availabilityMask = payload[2];
    for (size_t offset = 3; offset + 3 < payload.size(); offset += 4) {
        const auto high = payload[offset];
        const auto middle = payload[offset + 1];
        const auto low = payload[offset + 2];
        const auto status = payload[offset + 3];
        if (middle == 0x00 && low == 0x00) {
            continue;
        }
        result.records.push_back({high, middle, low, status});
    }
    return result;
}

DtcRecordData parseDtcRecordResponse(const std::vector<uint8_t>& response, uint8_t subFunction)
{
    const auto payload = udsPayload(response);
    // 59 <sub> <dtc hi mid lo> <status> = 6 bytes minimum.
    if (payload.size() < 6 || payload[0] != 0x59 || payload[1] != subFunction) {
        throw std::runtime_error("Unexpected DTC record response: " + formatHexBytesLower(payload));
    }

    DtcRecordData result;
    result.high = payload[2];
    result.middle = payload[3];
    result.low = payload[4];
    result.status = payload[5];
    result.data.assign(payload.cbegin() + 6, payload.cend());
    return result;
}

} // namespace common
