#include "logger/LogParameter.hpp"

#include <gtest/gtest.h>

using logger::DataType;
using logger::LogParameter;

namespace {

LogParameter makeParameter(size_t size, bool isSigned, uint32_t bitmask = 0)
{
    return LogParameter{ "test", 0x1234, size,
        DataType::Int, bitmask, "", isSigned, false, 1.0, 0.0, "" };
}

} // namespace

TEST(LogParameterValue, SignedSizesExtendSignFromTheirWidth)
{
    // 8 bit
    EXPECT_DOUBLE_EQ(makeParameter(1, true).formatValue(0x80), -128.0);
    EXPECT_DOUBLE_EQ(makeParameter(1, true).formatValue(0x7F), 127.0);
    // 16 bit
    EXPECT_DOUBLE_EQ(makeParameter(2, true).formatValue(0x8000), -32768.0);
    EXPECT_DOUBLE_EQ(makeParameter(2, true).formatValue(0x7FFF), 32767.0);
    // 24 bit: the sign lives in bit 23, a plain int32_t cast would give +8388608.
    EXPECT_DOUBLE_EQ(makeParameter(3, true).formatValue(0x800000), -8388608.0);
    EXPECT_DOUBLE_EQ(makeParameter(3, true).formatValue(0xFFFFFFu), -1.0);
    EXPECT_DOUBLE_EQ(makeParameter(3, true).formatValue(0x7FFFFF), 8388607.0);
    // 32 bit
    EXPECT_DOUBLE_EQ(makeParameter(4, true).formatValue(0x80000000u), -2147483648.0);
}

TEST(LogParameterValue, UnsignedSizesNeverGoNegative)
{
    EXPECT_DOUBLE_EQ(makeParameter(3, false).formatValue(0x800000), 8388608.0);
    EXPECT_DOUBLE_EQ(makeParameter(2, false).formatValue(0xFFFF), 65535.0);
}

TEST(LogParameterValue, BitmaskAppliesBeforeSigning)
{
    // Masked low nibble stays positive...
    EXPECT_DOUBLE_EQ(makeParameter(2, true, 0x000F).formatValue(0xF00F), 15.0);
    // ...and a masked byte-sized field still signs as an 8-bit value.
    EXPECT_DOUBLE_EQ(makeParameter(1, true, 0x00FF).formatValue(0xABFF), -1.0);
}

TEST(LogParameterValue, InverseConversionDivides)
{
    const LogParameter inverse{ "inv", 0x1234, 1, DataType::Int, 0, "", false, true, 100.0, 5.0, "" };
    EXPECT_DOUBLE_EQ(inverse.formatValue(45), 2.0);   // 100 / (45 + 5)
}

TEST(LogParameterValue, InverseConversionZeroDenominatorClampsToZero)
{
    // Signed -5 plus offset 5 gives a zero denominator; the contract returns 0 instead
    // of dividing by zero.
    const LogParameter inverseSigned{ "inv", 0x1234, 1, DataType::Int, 0, "", true, true, 100.0, 5.0, "" };
    EXPECT_DOUBLE_EQ(inverseSigned.formatValue(0xFB), 0.0);
}
