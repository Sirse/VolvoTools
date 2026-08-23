#include "IsoTpReassemble.hpp"

#include <gtest/gtest.h>

#include <vector>

using volvodiag::appendConsecutiveFrame;
using volvodiag::ConsecutiveFrameStatus;

namespace {

constexpr size_t kNoCap = 4096;

} // namespace

TEST(ConsecutiveFrame, AppendsDataAndAdvancesSequence)
{
    std::vector<uint8_t> payload;
    unsigned nextSequence = 1;
    const uint8_t frame[] = { 0x21, 0xAA, 0xBB, 0x20, 0x2F };
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, frame, sizeof(frame), kNoCap),
        ConsecutiveFrameStatus::Appended);
    // Data bytes 0x20/0x2F are data, not PCI: they must survive reassembly verbatim.
    EXPECT_EQ(payload, (std::vector<uint8_t>{ 0xAA, 0xBB, 0x20, 0x2F }));
    EXPECT_EQ(nextSequence, 2u);
}

TEST(ConsecutiveFrame, RollsSequenceOverFrom15To0)
{
    std::vector<uint8_t> payload;
    unsigned nextSequence = 15;
    const uint8_t frame[] = { 0x2F, 0x01 };
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, frame, sizeof(frame), kNoCap),
        ConsecutiveFrameStatus::Appended);
    EXPECT_EQ(nextSequence, 0u);
}

TEST(ConsecutiveFrame, IgnoresNonConsecutiveFrames)
{
    std::vector<uint8_t> payload;
    unsigned nextSequence = 3;
    const uint8_t singleFrame[] = { 0x06, 0x62, 0xF1, 0x90 };
    const uint8_t firstFrame[] = { 0x10, 0x14, 0x62, 0xF1, 0x90 };
    const uint8_t flowFrame[] = { 0x30, 0x00, 0x00 };
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, singleFrame, sizeof(singleFrame), kNoCap),
        ConsecutiveFrameStatus::Ignored);
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, firstFrame, sizeof(firstFrame), kNoCap),
        ConsecutiveFrameStatus::Ignored);
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, flowFrame, sizeof(flowFrame), kNoCap),
        ConsecutiveFrameStatus::Ignored);
    EXPECT_TRUE(payload.empty());
    EXPECT_EQ(nextSequence, 3u);
}

TEST(ConsecutiveFrame, DetectsSequenceGapWithoutTouchingPayload)
{
    std::vector<uint8_t> payload{ 0xDE, 0xAD };
    unsigned nextSequence = 4;
    const uint8_t frame[] = { 0x26, 0xBE, 0xEF };
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, frame, sizeof(frame), kNoCap),
        ConsecutiveFrameStatus::SequenceGap);
    // A lost frame must never be papered over with mismatched pieces.
    EXPECT_EQ(payload, (std::vector<uint8_t>{ 0xDE, 0xAD }));
    EXPECT_EQ(nextSequence, 4u);
}

TEST(ConsecutiveFrame, RespectsDeclaredLengthCap)
{
    std::vector<uint8_t> payload{ 0x01, 0x02, 0x03 };
    unsigned nextSequence = 1;
    const uint8_t frame[] = { 0x21, 0x04, 0x05, 0x06, 0x07 };
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, frame, sizeof(frame), 5),
        ConsecutiveFrameStatus::Appended);
    EXPECT_EQ(payload.size(), 5u);
    // The SN still advances when the cap stopped the copy: the next CF stays in sequence.
    EXPECT_EQ(nextSequence, 2u);
}

TEST(ConsecutiveFrame, EmptyPayloadIsIgnored)
{
    std::vector<uint8_t> payload;
    unsigned nextSequence = 0;
    EXPECT_EQ(appendConsecutiveFrame(payload, nextSequence, nullptr, 0, kNoCap),
        ConsecutiveFrameStatus::Ignored);
    EXPECT_EQ(nextSequence, 0u);
}
