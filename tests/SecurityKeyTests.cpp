#include <common/protocols/UDSProtocolCommonSteps.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

using common::UDSProtocolCommonSteps;

namespace {

struct SeedKeyVector {
    std::array<uint8_t, 5> pin;
    std::array<uint8_t, 3> seed;
    uint32_t key;
};

// Reference seed->key pairs produced by the P3Tool method_50 port and cross-checked against
// the factory SecurityAccess constants in the VIDA data. They lock the algorithm: flipping
// any of the 0xC541A9 / 0x109028 constants, a round of generateKeyImpl or the final bit
// permutation breaks these vectors - and would silently kill every online PIN search.
const SeedKeyVector kReferenceVectors[] = {
    // PAM
    { { 0xDC, 0xD5, 0x44, 0x7A, 0xE6 }, { 0x01, 0x02, 0x03 }, 0xC0BBC5 },
    // IAM
    { { 0x06, 0x4E, 0x9A, 0xE1, 0xDE }, { 0x01, 0x02, 0x03 }, 0x1FE032 },
    // FSM
    { { 0xAA, 0xCC, 0xCC, 0x33, 0x55 }, { 0x01, 0x02, 0x03 }, 0xE98BBC },
    // DIM
    { { 0x11, 0x23, 0x58, 0x13, 0x21 }, { 0x01, 0x02, 0x03 }, 0xB0B0AD },
    // CCM
    { { 0x45, 0x55, 0x43, 0x44, 0x31 }, { 0x01, 0x02, 0x03 }, 0xF6BB5B },
};

} // namespace

TEST(SecurityKey, ReferenceSeedKeyVectors)
{
    for (const auto& vector : kReferenceVectors) {
        EXPECT_EQ(UDSProtocolCommonSteps::generateSecurityKey(vector.pin, vector.seed), vector.key);
    }
}

TEST(SecurityKey, ResultFits24Bits)
{
    // The wire format is a 3-byte key (27 02 <key_hi> <key_mid> <key_lo>); anything above
    // 24 bits would be truncated on send and never match.
    const std::array<std::array<uint8_t, 3>, 4> seeds{ {
        { 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 0xFF }, { 0x12, 0x34, 0x56 }, { 0xDE, 0xAD, 0xC0 },
    } };
    for (const auto& seed : seeds) {
        for (const auto& vector : kReferenceVectors) {
            EXPECT_LE(UDSProtocolCommonSteps::generateSecurityKey(vector.pin, seed), 0xFFFFFFu);
        }
    }
}
