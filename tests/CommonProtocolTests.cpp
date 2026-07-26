#include <common/Util.hpp>
#include <common/protocols/UDSDid.hpp>
#include <common/protocols/UDSDtc.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

using namespace common;

namespace {

// UDS responses reach the parsers as full frames: 4-byte CAN-id header + payload.
std::vector<uint8_t> frame(std::vector<uint8_t> payload)
{
    std::vector<uint8_t> f{0x00, 0x00, 0x07, 0xE8};
    f.insert(f.end(), payload.begin(), payload.end());
    return f;
}

} // namespace

// ---- hex parsing ----------------------------------------------------------

TEST(HexParse, ParsesValueAndBytes)
{
    EXPECT_EQ(parseHexU32("7E0"), 0x7E0u);
    EXPECT_EQ(parseHexU32("0"), 0u);
    EXPECT_EQ(parseHexBytes("01 02 FF"), (std::vector<uint8_t>{0x01, 0x02, 0xFF}));
    EXPECT_EQ(parseHexBytes("DCD5447AE6"),
              (std::vector<uint8_t>{0xDC, 0xD5, 0x44, 0x7A, 0xE6}));
    EXPECT_EQ(parseHexBytes("0xDCD5447AE6"),
              (std::vector<uint8_t>{0xDC, 0xD5, 0x44, 0x7A, 0xE6}));
}

TEST(HexParse, RejectsInvalidInput)
{
    EXPECT_THROW(parseHexU32("ZZ"), std::exception);
    EXPECT_THROW(parseHexBytes("ZZ"), std::exception);
    EXPECT_THROW(parseHexBytes("ABC"), std::exception);
}

// ---- DTC formatting -------------------------------------------------------

TEST(DtcFormat, BaseCodeFamilies)
{
    EXPECT_EQ(formatDtcBaseCode(0x01, 0x23), "P0123");
    EXPECT_EQ(formatDtcBaseCode(0xC0, 0x00), "U0000");
    EXPECT_EQ(formatDtcBaseCode(0x41, 0x23), "C0123");
    EXPECT_EQ(formatDtcBaseCode(0x81, 0x23), "B0123");
}

TEST(DtcFormat, ThreeByteCodeHasFailureType)
{
    EXPECT_EQ(formatDtcCode(0x01, 0x23, 0x45), "P0123-45");
}

TEST(DtcStatus, DecodesBits)
{
    EXPECT_EQ(decodeDtcStatus(0x00, 0x00), "none");
    EXPECT_EQ(decodeDtcStatus(0x08, 0x00), "confirmed");
    EXPECT_EQ(decodeDtcStatus(0x09, 0x00), "testFailed|confirmed");
}

TEST(DtcStatus, AvailabilityMaskRestrictsBits)
{
    // testFailed(0x01) is set but not maintained per the mask, so only confirmed shows.
    EXPECT_EQ(decodeDtcStatus(0x09, 0x08), "confirmed");
}

// ---- DTC by status mask (19 02) -------------------------------------------

TEST(DtcByStatusMask, ParsesRecordsAndDropsPadding)
{
    const auto resp = frame({0x59, 0x02, 0xFF,
                             0x01, 0x23, 0x45, 0x08,   // real DTC
                             0x00, 0x00, 0x00, 0x00}); // all-zero padding -> dropped
    const auto parsed = parseDtcByStatusMaskResponse(resp);
    EXPECT_EQ(parsed.availabilityMask, 0xFF);
    ASSERT_EQ(parsed.records.size(), 1u);
    EXPECT_EQ(parsed.records[0].high, 0x01);
    EXPECT_EQ(parsed.records[0].status, 0x08);
}

TEST(DtcByStatusMask, RejectsWrongService)
{
    EXPECT_THROW(parseDtcByStatusMaskResponse(frame({0x7F, 0x19, 0x31})), std::exception);
}

TEST(DtcByStatusMask, RejectsTruncatedRecord)
{
    EXPECT_THROW(parseDtcByStatusMaskResponse(
                     frame({0x59, 0x02, 0xFF, 0x01, 0x23, 0x45})),
                 std::exception);
}

// ---- DTC record (19 04 / 19 06) -------------------------------------------

TEST(DtcRecord, ParsesPrefixAndRawPayload)
{
    const auto resp = frame({0x59, 0x04, 0x01, 0x23, 0x45, 0x08, 0xAA, 0xBB});
    const auto parsed = parseDtcRecordResponse(resp, 0x04);
    EXPECT_EQ(parsed.high, 0x01);
    EXPECT_EQ(parsed.middle, 0x23);
    EXPECT_EQ(parsed.low, 0x45);
    EXPECT_EQ(parsed.status, 0x08);
    EXPECT_EQ(parsed.data, (std::vector<uint8_t>{0xAA, 0xBB}));
}

TEST(DtcRecord, RejectsShortAndWrongSubfunction)
{
    EXPECT_THROW(parseDtcRecordResponse(frame({0x59, 0x04, 0x01, 0x23}), 0x04), std::exception);
    EXPECT_THROW(parseDtcRecordResponse(frame({0x59, 0x06, 0x01, 0x23, 0x45, 0x08}), 0x04),
                 std::exception);
}

// ---- DID (22) -------------------------------------------------------------

TEST(Did, ParsesResponse)
{
    const auto resp = frame({0x62, 0xF1, 0x90, 'A', 'B', 'C'});
    const auto parsed = parseDidResponse(resp);
    EXPECT_EQ(parsed.did, 0xF190);
    EXPECT_EQ(parsed.data, (std::vector<uint8_t>{'A', 'B', 'C'}));
}

TEST(Did, NameAndAsciiValue)
{
    EXPECT_EQ(didName(0xF190), "VIN");
    EXPECT_EQ(decodeDidValue(0xF190, {'A', 'B', 'C'}), "ABC");
}

// ---- bus configuration ----------------------------------------------------

namespace {

const BusConfiguration& busByName(const ConfigurationInfo& conf, const std::string& name)
{
    for (const auto& bus : conf.busInfo) {
        if (bus.name == name) {
            return bus;
        }
    }
    throw std::runtime_error("No such bus: " + name);
}

} // namespace

// The sample point used to be derived from the baudrate alone (500k -> 80, else 68), which is
// wrong for P1 CAN MS: the shipped vehicle configuration asks for 60 there. Make sure the value
// now comes from the configuration.
TEST(BusConfig, SamplePointComesFromConfiguration)
{
    const auto p1 = getConfigurationInfoByCarPlatform(CarPlatform::P1);
    EXPECT_EQ(busByName(p1, "CAN MS").samplePoint, 60u);
    EXPECT_EQ(busByName(p1, "CAN HS").samplePoint, 80u);

    const auto p2 = getConfigurationInfoByCarPlatform(CarPlatform::P2);
    EXPECT_EQ(busByName(p2, "CAN MS").samplePoint, 68u);
}
