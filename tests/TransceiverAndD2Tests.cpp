#include <common/CanMessagesTransceiver.hpp>
#include <common/protocols/D2Messages.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <set>
#include <vector>

using namespace common;

namespace {

// A frame for processD2Frame: 4-byte ECU-id header (01 20 00 05 = TCM) + packet-type byte +
// payload.
PASSTHRU_MSG d2Frame(uint8_t packetType, std::vector<uint8_t> payload)
{
    PASSTHRU_MSG msg;
    memset(&msg, 0, sizeof(msg));
    std::vector<uint8_t> body{ 0x01, 0x20, 0x00, 0x05, packetType };
    body.insert(body.end(), payload.begin(), payload.end());
    msg.DataSize = body.size();
    memcpy(msg.Data, body.data(), body.size());
    return msg;
}

class RecordingReceiver : public ICanMessagesReceiver {
public:
    bool onCanMessage(const uint8_t* buffer, size_t bufferSize) override
    {
        calls.emplace_back(buffer, buffer + bufferSize);
        return true;
    }
    std::vector<std::vector<uint8_t>> calls;
};

} // namespace

// ---- processD2Frame: packet assembly masks --------------------------------

// Regression: packetType && 0x80 was "non-zero", so a continuation frame (bit 6 only) was
// treated as a fresh packet start, overwriting the accumulated payload. Only bit 7 must start
// a new packet.
TEST(ProcessD2Frame, Bit7StartsAndOverwritesBit6Appends)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    // First frame: bit 7 (start).
    processD2Frame(received, subscribers, d2Frame(0x80, { 0x01, 0x02, 0x03 }));
    // Second frame: bit 6 (continue) - must append, not overwrite.
    processD2Frame(received, subscribers, d2Frame(0x40, { 0x04, 0x05 }));

    const auto ecu = static_cast<uint8_t>(D2Message::getECUType(std::vector<uint8_t>{ 0x01, 0x20, 0x00, 0x05 }));
    ASSERT_EQ(received.count(ecu), 1u);
    EXPECT_EQ(received[ecu], (std::vector<uint8_t>{ 0x01, 0x02, 0x03, 0x04, 0x05 }));

    // A new start frame overwrites the buffer.
    processD2Frame(received, subscribers, d2Frame(0x80, { 0xAA }));
    EXPECT_EQ(received[ecu], (std::vector<uint8_t>{ 0xAA }));
}

// Regression: packetType && 0x40 was "non-zero", so a start frame (bit 7 only) matched the
// begin branch; the continue branch was unreachable because 0x80 is also non-zero.
TEST(ProcessD2Frame, StartBitDoesNotTriggerContinueBranch)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    processD2Frame(received, subscribers, d2Frame(0x80, { 0x01 }));
    processD2Frame(received, subscribers, d2Frame(0x80, { 0x02 }));

    const auto ecu = static_cast<uint8_t>(D2Message::getECUType(std::vector<uint8_t>{ 0x01, 0x20, 0x00, 0x05 }));
    ASSERT_EQ(received.count(ecu), 1u);
    // Second 0x80 must replace, not append: 0x02 only.
    EXPECT_EQ(received[ecu], (std::vector<uint8_t>{ 0x02 }));
}

// A frame with neither bit set does not touch the assembly buffer.
TEST(ProcessD2Frame, NoPacketBitLeavesBufferUntouched)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    const auto ecu = static_cast<uint8_t>(D2Message::getECUType(std::vector<uint8_t>{ 0x01, 0x20, 0x00, 0x05 }));
    processD2Frame(received, subscribers, d2Frame(0x00, { 0x01 }));
    EXPECT_EQ(received.count(ecu), 0u);
}

// Short frames (< 5 bytes, i.e. no packet-type byte) are ignored.
TEST(ProcessD2Frame, ShortFrameIgnored)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    PASSTHRU_MSG msg;
    memset(&msg, 0, sizeof(msg));
    msg.DataSize = 3;
    memcpy(msg.Data, "\x01\x20\x00", 3);

    processD2Frame(received, subscribers, msg);
    EXPECT_TRUE(received.empty());
}

// ---- processD2Frame: subscriber dispatch ----------------------------------

// Regression: the subscriber loop incremented `it` (the frame iterator) instead of `callback`,
// so with a subscriber present it looped past range.second and dereferenced out of range. With
// the fix every subscriber is called exactly once.
TEST(ProcessD2Frame, DispatchesToEverySubscriberOnce)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    RecordingReceiver receiver;
    const auto ecu = static_cast<uint8_t>(D2Message::getECUType(std::vector<uint8_t>{ 0x01, 0x20, 0x00, 0x05 }));
    subscribers.emplace(ecu, &receiver);
    subscribers.emplace(ecu, &receiver); // two entries for the same ECU

    processD2Frame(received, subscribers, d2Frame(0x80, { 0x01, 0x02 }));

    ASSERT_EQ(receiver.calls.size(), 2u);
    // onCanMessage receives the full frame from the packet-type byte onward.
    EXPECT_EQ(receiver.calls[0], (std::vector<uint8_t>{ 0x80, 0x01, 0x02 }));
    EXPECT_EQ(receiver.calls[1], (std::vector<uint8_t>{ 0x80, 0x01, 0x02 }));
}

// Subscribers of other ECUs are not called.
TEST(ProcessD2Frame, DoesNotDispatchToOtherEcu)
{
    ReceivedMessageMap received;
    SubscriberMap subscribers;

    RecordingReceiver receiver;
    subscribers.emplace(static_cast<uint8_t>(ECUType::CEM), &receiver); // not the frame's TCM

    processD2Frame(received, subscribers, d2Frame(0x80, { 0x01 }));
    EXPECT_TRUE(receiver.calls.empty());
}

// ---- D2Messages::createWriteDataMsgs range validation ----------------------

TEST(CreateWriteDataMsgs, ValidatesRange)
{
    std::vector<uint8_t> bin(20, 0xAA);
    EXPECT_THROW(D2Messages::createWriteDataMsgs(0x01, bin, 21, 22), std::out_of_range);
    EXPECT_THROW(D2Messages::createWriteDataMsgs(0x01, bin, 5, 1), std::out_of_range);
    EXPECT_THROW(D2Messages::createWriteDataMsgs(0x01, bin, 0, 21), std::out_of_range);
    // Valid full and partial ranges still work.
    EXPECT_NO_THROW(D2Messages::createWriteDataMsgs(0x01, bin));
    EXPECT_NO_THROW(D2Messages::createWriteDataMsgs(0x01, bin, 0, 0));
    EXPECT_NO_THROW(D2Messages::createWriteDataMsgs(0x01, bin, 4, 20));
}

// The 0,0 empty range produces the trailing no-op message, matching the single-arg call.
TEST(CreateWriteDataMsgs, EmptyRangeProducesNoOp)
{
    std::vector<uint8_t> bin;
    const auto msgs = D2Messages::createWriteDataMsgs(0x01, bin);
    ASSERT_FALSE(msgs.empty());
}
