#include <common/Util.hpp>
#include <common/VBFParser.hpp>
#include <common/CommonData.hpp>
#include <common/Gateway.hpp>
#include <common/protocols/UDSDid.hpp>
#include <common/protocols/UDSDtc.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
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

// The sample point used to be derived from the baudrate alone (500k -> 80, else 68). Make sure
// the value now comes from the configuration, on the P3 family that the fork supports.
TEST(BusConfig, SamplePointComesFromConfiguration)
{
    const auto p3 = getConfigurationInfoByCarPlatform(CarPlatform::P3);
    EXPECT_EQ(busByName(p3, "CAN MS").samplePoint, 68u);
    EXPECT_EQ(busByName(p3, "CAN HS").samplePoint, 80u);

    const auto y413 = getConfigurationInfoByCarPlatform(CarPlatform::P3_Y413);
    EXPECT_EQ(busByName(y413, "CAN MS").samplePoint, 68u);
    EXPECT_EQ(busByName(y413, "CAN HS").samplePoint, 80u);
}

// ---- previously ignored vehicle-configuration fields -----------------------

// The vehicle configuration carries P4CANMax and SWDLSpecification on every bus. They were
// present in data.yaml but never read; the response-pending budget was hardcoded instead.
TEST(BusConfig, P4CanMaxAndSwdlSpecificationComeFromConfiguration)
{
    const auto p3 = getConfigurationInfoByCarPlatform(CarPlatform::P3);
    EXPECT_EQ(busByName(p3, "CAN HS").p4CanMax, 300000u);
    EXPECT_EQ(busByName(p3, "CAN HS").swdlSpecification, 31808456u);
}

// SBLInPBL / SwdlIssue / MasterECU are per-ECU flags in the configuration.
TEST(EcuConfig, PerEcuFlagsComeFromConfiguration)
{
    const auto configs = loadConfiguration(R"(
Configuration:
  - 
    Name: TEST
    Bus:
      - 
        BaudRate: 500
        CANIdBitSize: 11
        Name: CAN HS
        SWDLProtocol: 15765-2
        Node:
          - 
            Address: '10'
            CANIdentifier: '7E0'
            Name: ECM - Engine Control Module
            SBLInPBL: 1
          - 
            Address: '50'
            CANIdentifier: '7B0'
            Name: CEM - Central Electronic Module
            MasterECU: 1
            SwdlIssue: 1
          - 
            Address: '18'
            CANIdentifier: '7E1'
            Name: TCM - Transmission Control Module
)");
    ASSERT_EQ(configs.size(), 1u);
    ASSERT_EQ(configs[0].busInfo.size(), 1u);
    const auto& ecuInfo = configs[0].busInfo[0].ecuInfo;
    ASSERT_EQ(ecuInfo.size(), 3u);

    EXPECT_TRUE(ecuInfo[0].sblInPBL);
    EXPECT_FALSE(ecuInfo[0].masterEcu);
    EXPECT_FALSE(ecuInfo[0].swdlIssue);

    EXPECT_TRUE(ecuInfo[1].masterEcu);
    EXPECT_TRUE(ecuInfo[1].swdlIssue);
    EXPECT_FALSE(ecuInfo[1].sblInPBL);

    EXPECT_FALSE(ecuInfo[2].sblInPBL);
    EXPECT_FALSE(ecuInfo[2].masterEcu);
}

// Gateway_SubTester / SubTester live at the Configuration level next to Bus and Name.
TEST(Config, GatewayAndSubTesterEndpoints)
{
    const auto configs = loadConfiguration(R"(
Configuration:
  - 
    Name: TEST
    Gateway_SubTester:
      CANIdentifier: 784
      ECUAddress: 61
      Name: ICM - Infotainment Control Module
    SubTester:
      CANIdentifier: 784
      ECUAddress: 80
      Name: IAM - Integrated Audio Module
    Bus:
      - 
        BaudRate: 500
        CANIdBitSize: 11
        Name: CAN HS
        SWDLProtocol: 15765-2
)");
    ASSERT_EQ(configs.size(), 1u);
    EXPECT_EQ(configs[0].gatewaySubTester.ecuAddress, 0x61u);
    EXPECT_EQ(configs[0].gatewaySubTester.canId, 0x784u);
    EXPECT_EQ(configs[0].gatewaySubTester.name, "ICM - Infotainment Control Module");
    EXPECT_EQ(configs[0].subTester.ecuAddress, 0x80u);
    EXPECT_EQ(configs[0].subTester.canId, 0x784u);
}

// A configuration with no endpoint entries leaves them as "not set".
TEST(Config, MissingEndpointsAreUnset)
{
    const auto configs = loadConfiguration(R"(
Configuration:
  - 
    Name: TEST
    Bus:
      - 
        BaudRate: 500
        CANIdBitSize: 11
        Name: CAN HS
        SWDLProtocol: 15765-2
)");
    ASSERT_EQ(configs.size(), 1u);
    EXPECT_EQ(configs[0].gatewaySubTester.ecuAddress, 0u);
    EXPECT_EQ(configs[0].gatewaySubTester.canId, 0u);
    EXPECT_TRUE(configs[0].gatewaySubTester.name.empty());
    EXPECT_EQ(configs[0].subTester.ecuAddress, 0u);
}

// The embedded configuration must stay identical to the checked-in data.yaml: data.yaml is the
// single source of truth and CommonData.cpp is generated from it at configure time. A divergence
// here means someone hand-edited one without the other.
TEST(Config, EmbeddedConfigurationMatchesDataYaml)
{
    std::ifstream yaml(VOLVOTOOLS_DATA_YAML, std::ios::binary);
    ASSERT_TRUE(yaml.good()) << "can't open data.yaml at " << VOLVOTOOLS_DATA_YAML;
    std::string onDisk((std::istreambuf_iterator<char>(yaml)), std::istreambuf_iterator<char>());
    ASSERT_FALSE(onDisk.empty());

    // The embedded string was generated from data.yaml as-is; only line endings may differ
    // depending on how the file is checked out. Compare after normalising CRLF to LF.
    auto normalize = [](const std::string& text) {
        std::string result;
        result.reserve(text.size());
        for (size_t i = 0; i < text.size(); ++i) {
            if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') {
                continue;
            }
            result.push_back(text[i]);
        }
        return result;
    };
    EXPECT_EQ(normalize(CommonData::commonConfiguration), normalize(onDisk));
}

// Volvo's own identifiers, taken from the factory tooling's published interface. They used to
// fall through to the "DID 0x...." placeholder.
TEST(Did, KnowsVolvoSpecificIdentifiers)
{
    EXPECT_EQ(didName(0xF162), "Software download specification version");
    EXPECT_EQ(didName(0xF1B0), "ECU hardware number (KDP)");
    EXPECT_EQ(didName(0xD100), "Diagnostic session type");

    // Assembly numbers come back either as text or as packed digits. Text renders as text;
    // packed digits aren't printable and fall back to hex instead of turning into garbage.
    EXPECT_EQ(decodeDidValue(0xF111, {'3', '1', '8', '0', '8', '4', '5', '6'}), "31808456");
    EXPECT_EQ(decodeDidValue(0xF111, {0x31, 0x80, 0x84, 0x56}), "31 80 84 56");

    // Binary identifiers must never be guessed as text.
    EXPECT_EQ(decodeDidValue(0xF102, {0x41, 0x42}), "41 42");
}

// ---- CRC16-CCITT-FALSE ----------------------------------------------------

// The download block CRC is CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection,
// no xorout). The canonical check value keeps the implementation honest.
TEST(Crc16, CanonicalCheckValue)
{
    const uint8_t check[] = "123456789";
    EXPECT_EQ(crc16(check, 9), 0x29B1u);
}

// The known download fixture: the ME9 factory SBL closed its block with 77 A9 F3
// (verified_live), and the 26 KiB payload's CRC16-CCITT-FALSE is exactly 0xA9F3. A small
// equivalent vector below reproduces the same algorithm over a fixed byte stream.
TEST(Crc16, DownloadFixturePinsAlgorithm)
{
    // The ME9 factory SBL closed its block with 77 A9 F3 (verified_live); the 26 KiB payload's
    // CRC16-CCITT-FALSE is exactly 0xA9F3. The full payload is out-of-repo, so this synthetic
    // stream of the same length pins the exact algorithm: a regression in the polynomial/init/
    // reflection would break the value.
    std::vector<uint8_t> data;
    for (size_t i = 0; i < 26474; ++i) {
        data.push_back(static_cast<uint8_t>((i * 13 + 11) & 0xFF));
    }
    EXPECT_EQ(crc16(data.data(), data.size()), 0xADE7u);
}

// ---- VBF header -----------------------------------------------------------

namespace {

// A minimal but complete VBF header; the body is irrelevant for header-level checks.
std::string vbfHeader(const std::string& extraEntries)
{
    return "vbf_version = 2.6;\nheader {\n"
           " sw_part_number = \"31808832\";\n"
           " sw_part_type = EXE;\n"
           " network = CAN_HS;\n"
           " ecu_address = 0x7A;\n"
           + extraEntries +
           " file_checksum = 0x1234;\n}\n";
}

VBFHeader parseHeader(const std::string& text)
{
    VBFParser parser;
    return parser.parse(std::vector<char>(text.begin(), text.end())).header;
}

} // namespace

// data_format_identifier used to be an unknown key, which killed the whole parse.
TEST(VbfHeader, ParsesDataFormatIdentifier)
{
    EXPECT_EQ(parseHeader(vbfHeader(" data_format_identifier = 0x00;\n")).dataFormatIdentifier, 0x00);
    EXPECT_EQ(parseHeader(vbfHeader(" data_format_identifier = 0x10;\n")).dataFormatIdentifier, 0x10);
}

// Anything but 0x00 means packed payload, which we must never write to an ECU.
TEST(VbfHeader, FlagsPackedDataFormat)
{
    EXPECT_FALSE(isPackedDataFormat(parseHeader(vbfHeader(" data_format_identifier = 0x00;\n"))));
    EXPECT_TRUE(isPackedDataFormat(parseHeader(vbfHeader(" data_format_identifier = 0x10;\n"))));
    // Absent means plain data, which is the historical assumption.
    EXPECT_FALSE(isPackedDataFormat(parseHeader(vbfHeader(""))));
}

// Volvo writes some scalar entries as brace lists. ecu_address used to fail the parse outright,
// and sw_part_number silently kept the raw "{...}" text as the part number.
TEST(VbfHeader, AcceptsBraceListForms)
{
    const auto listed = parseHeader(
        "vbf_version = 2.6;\nheader {\n"
        " sw_part_number = {\"YW4T-13B525-AB\", \"31808832\"};\n"
        " sw_part_type = EXE;\n network = CAN_HS;\n"
        " ecu_address = { 0x723, 0x00, 0xff };\n"
        " file_checksum = 0x1234;\n}\n");
    EXPECT_EQ(listed.swPartNumber, "YW4T-13B525-AB");
    EXPECT_EQ(listed.ecuAddress, 0x723u);

    // The scalar forms must keep working.
    const auto scalar = parseHeader(vbfHeader(""));
    EXPECT_EQ(scalar.swPartNumber, "31808832");
    EXPECT_EQ(scalar.ecuAddress, 0x7Au);
}

// The header used to be a closed alternation, so one unknown key killed the whole file.
TEST(VbfHeader, SkipsUnknownEntriesInsteadOfFailing)
{
    const auto header = parseHeader(vbfHeader(
        " some_future_volvo_key = 0x42;\n"
        " another_one = { 1, 2, 3 };\n"
        " call = 0x8000;\n"));
    // Unknown entries are skipped, and the known ones around them still land.
    EXPECT_EQ(header.call, 0x8000u);
    EXPECT_EQ(header.swPartNumber, "31808832");
    EXPECT_EQ(header.ecuAddress, 0x7Au);
}

// The catch-all must not shadow keys we do model.
TEST(VbfHeader, KnownEntriesStillWinOverCatchAll)
{
    const auto header = parseHeader(vbfHeader(
        " sw_version = AB;\n session_type = programming;\n"
        " security_access_level = 0x11;\n erase = {{0x8000, 0x1000}};\n"));
    EXPECT_EQ(header.swVersion, "AB");
    EXPECT_EQ(header.sessionType, SessionType::PROGRAMMING);
    EXPECT_EQ(header.securityAccessLevel, 0x11);
    ASSERT_EQ(header.eraseBlocks.size(), 1u);
    EXPECT_EQ(header.eraseBlocks[0].startAddr, 0x8000u);
    EXPECT_EQ(header.eraseBlocks[0].length, 0x1000u);
}

// ---- gateway route + GSA codec --------------------------------------------

TEST(GatewayRoute, ResolvesFromConfiguration)
{
    const auto configs = loadConfiguration(R"(
Configuration:
  - 
    Name: TEST
    Gateway_SubTester:
      CANIdentifier: 784
      ECUAddress: 61
      Name: ICM - Infotainment Control Module
    Bus:
      - 
        BaudRate: 500
        CANIdBitSize: 11
        Name: CAN HS
        SWDLProtocol: 15765-2
)");
    ASSERT_EQ(configs.size(), 1u);
    GatewayRoute route;
    ASSERT_TRUE(resolveGatewayRoute(configs[0], 0x10, route));
    EXPECT_EQ(route.endpoint.ecuAddress, 0x61u);
    EXPECT_EQ(route.endpoint.canId, 0x784u);
}

// Configurations without a gateway endpoint (P1/P2 world) resolve to no route.
TEST(GatewayRoute, AbsentEndpointMeansNoRoute)
{
    const auto configs = loadConfiguration(R"(
Configuration:
  - 
    Name: TEST
    Bus:
      - 
        BaudRate: 500
        CANIdBitSize: 11
        Name: CAN HS
        SWDLProtocol: 15765-2
)");
    GatewayRoute route;
    EXPECT_FALSE(resolveGatewayRoute(configs[0], 0x10, route));
}

// The GSA RID is only a hypothesis, so the codec builds 31 01/31 02 with the explicit id and
// raw option bytes; it does not hardcode 0x0300.
TEST(GsaCodec, BuildsStartAndStopRequests)
{
    const auto start = sdaRoutineRequest(true, 0x0300, { 0x02, 0x01, 0x50 });
    EXPECT_EQ(start, (std::vector<uint8_t>{ 0x31, 0x01, 0x03, 0x00, 0x02, 0x01, 0x50 }));

    const auto stop = sdaRoutineRequest(false, 0x0300, {});
    EXPECT_EQ(stop, (std::vector<uint8_t>{ 0x31, 0x02, 0x03, 0x00 }));
}

TEST(GsaCodec, ValidatesPositiveResponse)
{
    EXPECT_NO_THROW(validateRoutineControlResponse(
        { 0x71, 0x01, 0x03, 0x00 }, true, 0x0300));
    EXPECT_NO_THROW(validateRoutineControlResponse(
        { 0x71, 0x02, 0x03, 0x00 }, false, 0x0300));
    EXPECT_THROW(validateRoutineControlResponse({ 0x71, 0x02, 0x03, 0x00 }, true, 0x0300),
                 std::exception);
    EXPECT_THROW(validateRoutineControlResponse({ 0x71, 0x01, 0x03, 0x01 }, true, 0x0300),
                 std::exception);
    EXPECT_THROW(validateRoutineControlResponse({ 0x62, 0x01, 0x03, 0x00 }, true, 0x0300),
                 std::exception);
}

TEST(GsaCodec, RoutineNames)
{
    EXPECT_EQ(sdaRoutineName(SdaRoutine::ActivateSBL), "ActivateSBL");
    EXPECT_EQ(sdaRoutineName(SdaRoutine::GatewayStateAccess), "GatewayStateAccess");
}

// ---- P3-only platform parsing ---------------------------------------------

// Every value of the P3 family resolves, and legacy p3 still means Y285/Y286/Y381.
TEST(P3Only, ParsesEveryP3FamilyMember)
{
    EXPECT_EQ(parseCarPlatform("p3"), CarPlatform::P3);
    EXPECT_EQ(parseCarPlatform("P3"), CarPlatform::P3);
    EXPECT_EQ(parseCarPlatform("p3_y413"), CarPlatform::P3_Y413);
    EXPECT_EQ(parseCarPlatform("p3_y283_iam"), CarPlatform::P3_Y283_IAM);
    EXPECT_EQ(parseCarPlatform("p3_y283_icm"), CarPlatform::P3_Y283_ICM);
    EXPECT_EQ(parseCarPlatform("p3_p313_icm"), CarPlatform::P3_P313_ICM);
    EXPECT_EQ(parseCarPlatform("p3_p313_iam"), CarPlatform::P3_P313_IAM);
    EXPECT_EQ(parseCarPlatform("p3_y555_iam"), CarPlatform::P3_Y555_IAM);
    EXPECT_EQ(parseCarPlatform("p3_y555_icm"), CarPlatform::P3_Y555_ICM);
    EXPECT_EQ(parseCarPlatform("p3_y312h_iam"), CarPlatform::P3_Y312H_IAM);
    EXPECT_EQ(parseCarPlatform("p3_y312h_icm"), CarPlatform::P3_Y312H_ICM);
}

// The old values are gone: a clear error naming the P3 family, never a silent guess.
TEST(P3Only, RejectsNonP3Platforms)
{
    EXPECT_THROW(parseCarPlatform("p2"), std::exception);
    EXPECT_THROW(parseCarPlatform("p80"), std::exception);
    EXPECT_THROW(parseCarPlatform("spa"), std::exception);
    EXPECT_THROW(parseCarPlatform("ford_uds"), std::exception);
    EXPECT_THROW(parseCarPlatform("ford_kwp"), std::exception);
    EXPECT_THROW(parseCarPlatform("haval_uds"), std::exception);
    EXPECT_THROW(parseCarPlatform("vag_med91"), std::exception);
    EXPECT_THROW(parseCarPlatform("p1"), std::exception);
}

// Every remaining enum value must resolve to a configuration; this catches the drift between
// the enum and the string names held together by textual matching.
TEST(P3Only, EveryPlatformResolvesToConfiguration)
{
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y413));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y283_IAM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y283_ICM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_P313_ICM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_P313_IAM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y555_IAM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y555_ICM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y312H_IAM));
    EXPECT_NO_THROW((void)getConfigurationInfoByCarPlatform(CarPlatform::P3_Y312H_ICM));
}

// ---- data.yaml v2 golden equivalence ----------------------------------------

namespace {

std::string readTextFile(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in.good()) {
        throw std::runtime_error("can't open " + path);
    }
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// Compares everything the parser reads out of the ECU records. securityPin is deliberately
// excluded: it is new to the v2 schema, so the v1 snapshot cannot carry it.
bool sameEcu(const ECUInfo& a, const ECUInfo& b)
{
    return a.ecuId == b.ecuId && a.canId == b.canId && a.name == b.name
        && a.sblInPBL == b.sblInPBL && a.swdlIssue == b.swdlIssue && a.masterEcu == b.masterEcu;
}

bool sameEndpoint(const GatewayEndpoint& a, const GatewayEndpoint& b)
{
    return a.ecuAddress == b.ecuAddress && a.canId == b.canId && a.name == b.name;
}

bool sameBus(const BusConfiguration& a, const BusConfiguration& b)
{
    if (a.name != b.name || a.protocolId != b.protocolId || a.baudrate != b.baudrate
        || a.canIdBitSize != b.canIdBitSize || a.samplePoint != b.samplePoint
        || a.p4CanMax != b.p4CanMax || a.swdlSpecification != b.swdlSpecification
        || a.ecuInfo.size() != b.ecuInfo.size()) {
        return false;
    }
    for (size_t i = 0; i < a.ecuInfo.size(); ++i) {
        if (!sameEcu(a.ecuInfo[i], b.ecuInfo[i])) {
            return false;
        }
    }
    return true;
}

bool sameConfiguration(const ConfigurationInfo& a, const ConfigurationInfo& b)
{
    // cliName is new to the v2 schema (the v1 file cannot carry it), so it is excluded here the
    // same way securityPin is; it is asserted separately by EveryConfigurationHasUniqueCliName.
    if (a.name != b.name || !sameEndpoint(a.gatewaySubTester, b.gatewaySubTester)
        || !sameEndpoint(a.subTester, b.subTester) || a.busInfo.size() != b.busInfo.size()) {
        return false;
    }
    for (size_t i = 0; i < a.busInfo.size(); ++i) {
        if (!sameBus(a.busInfo[i], b.busInfo[i])) {
            return false;
        }
    }
    return true;
}

} // namespace

// The v2 schema (shared ECU pool + defaults + cliName) must expand to exactly the same
// ConfigurationInfo as the pre-migration v1 file parsed through the v1 code path: the same number
// of configurations, the same buses, the same ECUs in the same order (order decides which channel
// opens first), and the same fields. This is the acceptance gate for the migration -- without it
// the v2 data.yaml must not merge.
TEST(ConfigV2, ExpandsIdenticallyToV1)
{
    const auto v1 = loadConfiguration(readTextFile(VOLVOTOOLS_DATA_V1_FIXTURE));
    const auto v2 = loadConfiguration(readTextFile(VOLVOTOOLS_DATA_YAML));

    ASSERT_EQ(v1.size(), v2.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        EXPECT_TRUE(sameConfiguration(v1[i], v2[i]))
            << "configuration index " << i << " differs between v1 and v2";
    }
}

// The v2 config must carry a unique cliName for every configuration (the value --platform takes).
TEST(ConfigV2, EveryConfigurationHasUniqueCliName)
{
    const auto configs = loadConfiguration(readTextFile(VOLVOTOOLS_DATA_YAML));
    ASSERT_FALSE(configs.empty());
    std::set<std::string> seen;
    for (const auto& conf : configs) {
        EXPECT_FALSE(conf.cliName.empty()) << "configuration \"" << conf.name << "\" lacks cliName";
        EXPECT_FALSE(seen.count(conf.cliName)) << "duplicate cliName " << conf.cliName;
        seen.insert(conf.cliName);
    }
}

// A known SecurityAccess PIN in the pool must survive parsing and reach getEcuInfoByEcuId as the
// exact 5 bytes, on the platform that carries the ECU.
TEST(ConfigV2, SecurityPinReachesEcuInfo)
{
    // IAM (0x80) on P3 CAN MS has a known PIN 06 4E 9A E1 DE in the pool.
    const auto [bus, ecu] = getEcuInfoByEcuId(CarPlatform::P3, 0x80);
    (void)bus;
    EXPECT_TRUE(ecu.hasSecurityPin());
    EXPECT_EQ(ecu.securityPin, (std::array<uint8_t, 5>{ 0x06, 0x4E, 0x9A, 0xE1, 0xDE }));
}

// ECUs without a known PIN report "no pin" (all-zero array), so callers fall back to current
// behaviour instead of sending a bogus key.
TEST(ConfigV2, EcuWithoutKnownPinHasEmptySecurityPin)
{
    // CEM (0x52) has no known key (individual per vehicle).
    const auto [bus, ecu] = getEcuInfoByEcuId(CarPlatform::P3, 0x52);
    (void)bus;
    EXPECT_FALSE(ecu.hasSecurityPin());
}

// parseSecurityPin accepts both spaced and packed hex, and rejects malformed input.
TEST(SecurityPin, ParsesSpacedAndPackedHex)
{
    EXPECT_EQ(common::parseSecurityPin("AA BB CC DD EE"),
        (std::array<uint8_t, 5>{ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE }));
    EXPECT_EQ(common::parseSecurityPin("AABBCCDDEE"),
        (std::array<uint8_t, 5>{ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE }));
    EXPECT_THROW(common::parseSecurityPin("AABBCCDD"), std::exception);      // 4 bytes
    EXPECT_THROW(common::parseSecurityPin("AABBCCDDEEFF"), std::exception);  // 6 bytes
    EXPECT_THROW(common::parseSecurityPin("AABBCCDDEG"), std::exception);    // non-hex
}

// The 5-byte key is 40 bits, so it round-trips through a uint64 as the big-endian integer.
TEST(SecurityPin, RoundTripsThroughUint64)
{
    const auto pin = common::parseSecurityPin("06 4E 9A E1 DE");
    EXPECT_EQ(common::securityPinToUint64(pin), 0x064E9AE1DEull);
    EXPECT_EQ(common::getPinArray(common::securityPinToUint64(pin)), pin);
}

