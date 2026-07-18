#include <common/Util.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace {

TEST(EndianEncoding, TwoBytes)
{
    EXPECT_EQ(common::encodeBigEndian(0x12, 0x34), 0x1234u);
    EXPECT_EQ(common::encodeLittleEndian(0x12, 0x34), 0x3412u);
    EXPECT_EQ(common::encodeBigEndian(std::vector<uint8_t>{0x12, 0x34}), 0x1234u);
    EXPECT_EQ(common::encodeLittleEndian(std::vector<uint8_t>{0x12, 0x34}), 0x3412u);
}

TEST(EndianEncoding, FourBytes)
{
    EXPECT_EQ(common::encodeBigEndian(0x12, 0x34, 0x56, 0x78), 0x12345678u);
    EXPECT_EQ(common::encodeLittleEndian(0x12, 0x34, 0x56, 0x78), 0x78563412u);
    EXPECT_EQ(common::encodeBigEndian(std::vector<uint8_t>{0x12, 0x34, 0x56, 0x78}), 0x12345678u);
    EXPECT_EQ(common::encodeLittleEndian(std::vector<uint8_t>{0x12, 0x34, 0x56, 0x78}), 0x78563412u);
}

} // namespace
