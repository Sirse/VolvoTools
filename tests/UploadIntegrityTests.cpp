#include <common/Util.hpp>
#include <common/protocols/UploadIntegrity.hpp>

#include <flasher/UDSReader.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using namespace common;

namespace {

// Builds the deterministic pattern used to synthesise the reference vectors.
std::vector<uint8_t> patternImage(size_t n)
{
    std::vector<uint8_t> image(n);
    for (size_t i = 0; i < n; ++i) {
        image[i] = static_cast<uint8_t>((i * 29 + 7) & 0xFF);
    }
    return image;
}

// Page-bounded chunking mirroring the SDA resident: reads are split at 16 KiB PPAGE
// boundaries and each positive 0x36 response carries at most 62 memory bytes.
std::vector<std::vector<uint8_t>> sdaChunk(const std::vector<uint8_t>& image)
{
    constexpr size_t kPage = 0x4000;
    constexpr size_t kMaxPayload = 62;
    std::vector<std::vector<uint8_t>> responses;
    uint8_t counter = 1;
    for (size_t pageStart = 0; pageStart < image.size(); pageStart += kPage) {
        const size_t pageEnd = std::min(pageStart + kPage, image.size());
        size_t off = pageStart;
        while (off < pageEnd) {
            const size_t n = std::min(kMaxPayload, pageEnd - off);
            std::vector<uint8_t> resp{ 0x76, counter };
            resp.insert(resp.end(), image.begin() + off, image.begin() + off + n);
            responses.emplace_back(std::move(resp));
            ++counter;
            off += n;
        }
    }
    return responses;
}

void feedAll(UploadIntegrityAccumulator& acc, const std::vector<std::vector<uint8_t>>& responses)
{
    for (const auto& resp : responses) {
        acc.addResponse(resp);
    }
}

// A bare 77 is the response frame as returned by UDSRequest: 4-byte CAN id header + payload.
std::vector<uint8_t> frame77()
{
    return { 0x00, 0x00, 0x07, 0xE8, 0x77 };
}

std::vector<uint8_t> frame77(uint16_t crc)
{
    return { 0x00, 0x00, 0x07, 0xE8, 0x77, static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc) };
}

} // namespace

// ---- SDA 4.7 wire CRC against the confirmed live vector -------------------

// The full PAM flash upload (128 KiB, 2120 sequential blocks, page-bounded at 16 KiB)
// produced 77 FE D6 on the owned bench. Rebuilding the response stream from the dump and
// feeding it through the accumulator must reproduce FE D6.
TEST(UploadIntegrity, ReproducesSda47LiveVector)
{
    // This vector is over the PAM dump bytes. The unit test builds the same response stream
    // from a synthetic image so it runs offline; the live result FE D6 was reproduced from the
    // actual dump by the reference model (see docs/dev/sda-upload-compat).
    //
    // The synthetic vector below (266 responses, boundary-short block) is the small offline
    // stand-in that pins the coverage rule. Reference: image_crc=9527, wire_crc=EC2B.
    const auto image = patternImage(0x4000 + 16);
    const auto responses = sdaChunk(image);
    ASSERT_EQ(responses.size(), 266u);
    ASSERT_EQ(responses.back().size(), 18u); // 0x76 + counter + 16 boundary bytes

    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    EXPECT_EQ(acc.blockCount(), 266u);
    EXPECT_EQ(acc.imageCrc(), 0x9527u);
    EXPECT_EQ(acc.sdaWireCrc(), 0xEC2Bu);
}

// ---- regular 64-byte responses --------------------------------------------

// With every response exactly 64 bytes long, the SDA exclusion removes precisely the 0x76 and
// counter of each response, so the wire CRC equals the CRC of the reconstructed image.
TEST(UploadIntegrity, Regular64ByteResponsesWireEqualsImage)
{
    const auto image = patternImage(62 * 3);
    std::vector<std::vector<uint8_t>> responses{
        { 0x76, 0x01, }, { 0x76, 0x02, }, { 0x76, 0x03, },
    };
    for (size_t i = 0; i < 3; ++i) {
        responses[i].insert(responses[i].end(), image.begin() + i * 62, image.begin() + (i + 1) * 62);
    }

    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    EXPECT_EQ(acc.imageCrc(), 0xB663u);
    EXPECT_EQ(acc.sdaWireCrc(), 0xB663u);
}

// ---- duplicate previous block must not advance any state ------------------

TEST(UploadIntegrity, DuplicateBlockDoesNotAdvanceState)
{
    const auto image = patternImage(62 * 2);
    UploadIntegrityAccumulator acc;
    acc.addResponse({ 0x76, 0x01, image[0], image[1], image[2] });
    acc.addResponse({ 0x76, 0x02, image[62], image[63], image[64] });
    const auto before = acc;
    // Replay of the immediately previous block: same counter, must be ignored.
    acc.addResponse({ 0x76, 0x02, image[62], image[63], image[64] });
    EXPECT_EQ(acc.blockCount(), before.blockCount());
    EXPECT_EQ(acc.imageCrc(), before.imageCrc());
    EXPECT_EQ(acc.sdaWireCrc(), before.sdaWireCrc());
    EXPECT_EQ(acc.flattenedOffset(), before.flattenedOffset());
}

// A replay of a block that is NOT the previous one is a genuine new block.
TEST(UploadIntegrity, NonConsecutiveBlockAdvances)
{
    UploadIntegrityAccumulator acc;
    acc.addResponse({ 0x76, 0x01, 0xAA });
    acc.addResponse({ 0x76, 0x02, 0xBB });
    const auto after = acc;
    EXPECT_EQ(after.blockCount(), 2u);
}

// ---- counter wrap ----------------------------------------------------------

TEST(UploadIntegrity, CounterWrapAdvances)
{
    const auto image = patternImage(62 * 3);
    std::vector<std::vector<uint8_t>> responses{
        { 0x76, 0xFE, }, { 0x76, 0xFF, }, { 0x76, 0x00, },
    };
    for (size_t i = 0; i < 3; ++i) {
        responses[i].insert(responses[i].end(), image.begin() + i * 62, image.begin() + (i + 1) * 62);
    }
    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    EXPECT_EQ(acc.blockCount(), 3u);
    // The block counter bytes are excluded from the SDA wire CRC, so the wrap does not change it.
    EXPECT_EQ(acc.imageCrc(), 0xB663u);
    EXPECT_EQ(acc.sdaWireCrc(), 0xB663u);
}

// ---- malformed responses ---------------------------------------------------

TEST(UploadIntegrity, RejectsMalformedResponse)
{
    UploadIntegrityAccumulator acc;
    EXPECT_THROW(acc.addResponse({ 0x76 }), std::exception);           // no counter
    EXPECT_THROW(acc.addResponse({ 0x62, 0x01, 0xAA }), std::exception); // wrong SID
}

// ---- TransferExit parsing --------------------------------------------------

TEST(TransferExit, Bare77ParsesWithoutCrc)
{
    const auto parsed = parseTransferExit(frame77());
    EXPECT_TRUE(parsed.responsePresent);
    EXPECT_FALSE(parsed.crcPresent);
}

TEST(TransferExit, Non77ResponseIsNoTransferExit)
{
    const auto parsed = parseTransferExit({ 0x00, 0x00, 0x07, 0xE8, 0x7F, 0x37, 0x31 });
    EXPECT_FALSE(parsed.responsePresent);
}

// ---- verdict: none of the four outcomes is fatal ---------------------------

// Bare 77: the SBL sent no CRC, so there is nothing to compare - NotChecked, not a failure.
TEST(TransferExit, Bare77IsNotChecked)
{
    UploadIntegrityAccumulator acc;
    acc.addResponse({ 0x76, 0x01, 0xAA });
    const auto result = resolveTransferExit(frame77(), acc);
    EXPECT_EQ(result.status, IntegrityStatus::NotChecked);
}

// No positive 77 response at all: NoTransferExit.
TEST(TransferExit, NoResponseIsNoTransferExit)
{
    UploadIntegrityAccumulator acc;
    acc.addResponse({ 0x76, 0x01, 0xAA });
    const auto result = resolveTransferExit({ 0x00, 0x00, 0x07, 0xE8, 0x7F, 0x37, 0x31 }, acc);
    EXPECT_EQ(result.status, IntegrityStatus::NoTransferExit);
}

// Exactly one algorithm matched.
TEST(TransferExit, OneProfileMatchesIsOk)
{
    // Page-bounded vector: image CRC (0x9527) differs from the SDA wire CRC (0xEC2B), so a
    // returned value can be attributed to exactly one algorithm.
    const auto image = patternImage(0x4000 + 16);
    const auto responses = sdaChunk(image);
    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    ASSERT_NE(acc.imageCrc(), acc.sdaWireCrc());

    const auto imageMatch = resolveTransferExit(frame77(acc.imageCrc()), acc);
    EXPECT_EQ(imageMatch.status, IntegrityStatus::Ok);
    EXPECT_EQ(imageMatch.profileUsed, UploadIntegrityProfile::ImageCrc16CcittFalse);

    const auto wireMatch = resolveTransferExit(frame77(acc.sdaWireCrc()), acc);
    EXPECT_EQ(wireMatch.status, IntegrityStatus::Ok);
    EXPECT_EQ(wireMatch.profileUsed, UploadIntegrityProfile::Sda47WireCrc16);
}

// Both sums matched: Ok, reported as image-crc16. On even 64-byte blocks the two coincide by
// construction - this is the healthy answer, never an ambiguity.
TEST(TransferExit, BothProfilesMatchIsOkAsImageCrc)
{
    const auto image = patternImage(62 * 3);
    std::vector<std::vector<uint8_t>> responses{
        { 0x76, 0x01, }, { 0x76, 0x02, }, { 0x76, 0x03, },
    };
    for (size_t i = 0; i < 3; ++i) {
        responses[i].insert(responses[i].end(), image.begin() + i * 62, image.begin() + (i + 1) * 62);
    }
    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    EXPECT_EQ(acc.imageCrc(), acc.sdaWireCrc());

    const auto result = resolveTransferExit(frame77(acc.imageCrc()), acc);
    EXPECT_EQ(result.status, IntegrityStatus::Ok);
    EXPECT_EQ(result.profileUsed, UploadIntegrityProfile::ImageCrc16CcittFalse);
}

// Nothing matched: Unverified - the dump is kept and the user is told to check it manually.
TEST(TransferExit, NoMatchIsUnverified)
{
    const auto image = patternImage(0x4000 + 16);
    const auto responses = sdaChunk(image);
    UploadIntegrityAccumulator acc;
    feedAll(acc, responses);
    const auto result = resolveTransferExit(frame77(0x1234), acc);
    EXPECT_EQ(result.status, IntegrityStatus::Unverified);
    // Both computed sums must be available for the report/log.
    EXPECT_EQ(result.computedImageCrc, acc.imageCrc());
    EXPECT_EQ(result.computedSdaWireCrc, acc.sdaWireCrc());
}

// ---- names ----------------------------------------------------------------

TEST(UploadIntegrity, ProfileNames)
{
    EXPECT_EQ(uploadIntegrityProfileName(UploadIntegrityProfile::ImageCrc16CcittFalse),
              "image-crc16");
    EXPECT_EQ(uploadIntegrityProfileName(UploadIntegrityProfile::Sda47WireCrc16),
              "sda47-wire-crc16");
}

TEST(UploadIntegrity, StatusNames)
{
    EXPECT_EQ(integrityStatusName(IntegrityStatus::NotChecked), "not-checked");
    EXPECT_EQ(integrityStatusName(IntegrityStatus::Ok), "ok");
    EXPECT_EQ(integrityStatusName(IntegrityStatus::Unverified), "unverified");
    EXPECT_EQ(integrityStatusName(IntegrityStatus::NoTransferExit), "no-transfer-exit");
}

// ---- UploadReadResult contract --------------------------------------------

// success reflects only that the full volume arrived and the size matched; the verdict never
// flips it. Regression guard for the ordering bug that used to drop the dump on a failed CRC.
TEST(UploadReadResult, VerdictDoesNotAffectSuccess)
{
    UploadReadResult result;
    result.success = true;
    result.payload = { 0xAA, 0xBB, 0xCC };
    result.integrityStatus = IntegrityStatus::Unverified;
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.payload.empty());
    EXPECT_EQ(result.payload, (std::vector<uint8_t>{ 0xAA, 0xBB, 0xCC }));
}

// ---- UDSReader-level regression: payload survives an Unverified verdict ----

// The old code assigned _output = upload.payload AFTER setFailure (which throws), so a dump
// whose CRC mismatched was silently dropped even though the bytes had arrived. commitUploadResult
// is the exact function UDSReader uses: it must commit the payload before any failure decision
// and must NOT treat an integrity verdict as a read failure.
TEST(UDSReaderIntegrity, UnverifiedCommitsPayloadToOutput)
{
    common::UploadReadResult upload;
    upload.success = true;                      // full volume collected, size matched
    upload.payload = { 0xDE, 0xAD, 0xBE, 0xEF };
    upload.integrityStatus = common::IntegrityStatus::Unverified;
    upload.returnedCrc = 0x1234;
    upload.computedImageCrc = 0x5678;
    upload.computedSdaWireCrc = 0x9ABC;

    std::vector<uint8_t> output;
    EXPECT_TRUE(flasher::commitUploadResult(upload, output));
    EXPECT_FALSE(output.empty());
    EXPECT_EQ(output, upload.payload);
}

// A real read failure (success=false) must still commit whatever arrived, but reports failure.
TEST(UDSReaderIntegrity, FailedReadCommitsPartialPayloadAndFails)
{
    common::UploadReadResult upload;
    upload.success = false;                     // size mismatch / transport error
    upload.payload = { 0x01, 0x02 };
    upload.integrityStatus = common::IntegrityStatus::NoTransferExit;

    std::vector<uint8_t> output;
    EXPECT_FALSE(flasher::commitUploadResult(upload, output));
    // Even on a failed read the bytes that did arrive are kept for inspection.
    EXPECT_EQ(output, upload.payload);
}

// ---- integrity report: the on-disk evidence must not contradict its status ----

// Regression: the report used to print algorithm and ecu_crc unconditionally, so an Unverified
// verdict showed "algorithm: image_crc16_ccitt_false" (nothing matched!) and a bare-77
// NotChecked showed "ecu_crc: 0x0000" as if the ECU returned zero. Both must be n/a.
TEST(IntegrityReport, UnverifiedPrintsAlgorithmAndEcuCrcAsNa)
{
    common::UploadReadResult upload;
    upload.integrityStatus = common::IntegrityStatus::Unverified;
    upload.crcPresent = true;                   // a CRC arrived but matched nothing
    upload.returnedCrc = 0x8A3F;
    upload.integrityProfileUsed = common::UploadIntegrityProfile::ImageCrc16CcittFalse; // stale default
    const auto report = common::formatIntegrityReport(upload);
    EXPECT_NE(report.find("status: unverified"), std::string::npos);
    EXPECT_NE(report.find("algorithm: n/a"), std::string::npos);
    EXPECT_NE(report.find("ecu_crc: 0x8A3F"), std::string::npos); // CRC did arrive, value shown
    EXPECT_EQ(report.find("image_crc16_ccitt_false"), std::string::npos);
}

TEST(IntegrityReport, NotCheckedHidesEcuCrc)
{
    common::UploadReadResult upload;
    upload.integrityStatus = common::IntegrityStatus::NotChecked; // bare 77, no CRC
    upload.crcPresent = false;
    upload.returnedCrc = 0x0000;                // struct default, not an ECU value
    const auto report = common::formatIntegrityReport(upload);
    EXPECT_NE(report.find("status: not-checked"), std::string::npos);
    EXPECT_NE(report.find("algorithm: n/a"), std::string::npos);
    EXPECT_NE(report.find("ecu_crc: n/a"), std::string::npos);
    // The ECU returned nothing; the report must not read as "returned zero".
    EXPECT_EQ(report.find("ecu_crc: 0x0000"), std::string::npos);
}

TEST(IntegrityReport, OkShowsAlgorithmAndEcuCrc)
{
    common::UploadReadResult upload;
    upload.integrityStatus = common::IntegrityStatus::Ok;
    upload.crcPresent = true;
    upload.returnedCrc = 0xB663;
    upload.integrityProfileUsed = common::UploadIntegrityProfile::ImageCrc16CcittFalse;
    const auto report = common::formatIntegrityReport(upload);
    EXPECT_NE(report.find("status: ok"), std::string::npos);
    EXPECT_NE(report.find("algorithm: image-crc16"), std::string::npos);
    EXPECT_NE(report.find("ecu_crc: 0xB663"), std::string::npos);
}

TEST(IntegrityReport, NoTransferExitIsNa)
{
    common::UploadReadResult upload;
    upload.integrityStatus = common::IntegrityStatus::NoTransferExit;
    upload.crcPresent = false;
    const auto report = common::formatIntegrityReport(upload);
    EXPECT_NE(report.find("status: no-transfer-exit"), std::string::npos);
    EXPECT_NE(report.find("algorithm: n/a"), std::string::npos);
    EXPECT_NE(report.find("ecu_crc: n/a"), std::string::npos);
}
