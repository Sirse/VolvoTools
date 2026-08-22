#include <common/protocols/PinSearch.hpp>

#include <gtest/gtest.h>

#include <cstdint>

using common::kCemPinWindowCeil;
using common::kCemPinWindowFloor;
using common::PinSearchWindow;

namespace {

// Walks the whole window in the given direction and returns the number of visited
// candidates (including the start).
uint64_t walkCount(const PinSearchWindow& window, uint64_t start, bool upward)
{
    uint64_t count = 1;
    std::optional<uint64_t> current = start;
    while ((current = window.nextCandidate(*current, upward))) {
        ++count;
    }
    return count;
}

} // namespace

TEST(PinSearchWindow, CemConstantsMatchReference)
{
    EXPECT_EQ(kCemPinWindowFloor, 0xFFFF000000ull);
    EXPECT_EQ(kCemPinWindowCeil, 0xFFFFFFFFFFull);
    // Top two bytes fixed at FFFF, low three bytes scanned: exactly 2^24 candidates.
    EXPECT_EQ(kCemPinWindowCeil - kCemPinWindowFloor + 1, 1ull << 24);
}

TEST(PinSearchWindow, DownwardScanCoversWholeCemWindowAndStopsAtFloor)
{
    const PinSearchWindow cem{ kCemPinWindowFloor, kCemPinWindowCeil };
    // First step from the top goes one candidate down, not below the ceiling...
    EXPECT_EQ(cem.nextCandidate(kCemPinWindowCeil, false), kCemPinWindowCeil - 1);
    // ...and the floor itself is the last candidate: stepping below it ends the scan.
    EXPECT_FALSE(cem.nextCandidate(kCemPinWindowFloor, false).has_value());
    EXPECT_EQ(walkCount(cem, kCemPinWindowCeil, false), 1ull << 24);
}

TEST(PinSearchWindow, UpwardScanCoversWholeCemWindowAndStopsAtCeiling)
{
    const PinSearchWindow cem{ kCemPinWindowFloor, kCemPinWindowCeil };
    EXPECT_EQ(cem.nextCandidate(kCemPinWindowFloor, true), kCemPinWindowFloor + 1);
    EXPECT_FALSE(cem.nextCandidate(kCemPinWindowCeil, true).has_value());
    EXPECT_EQ(walkCount(cem, kCemPinWindowFloor, true), 1ull << 24);
}

TEST(PinSearchWindow, PartialWindowVisitsOnlyItsRange)
{
    const PinSearchWindow w{ 0x10, 0x12 };
    EXPECT_TRUE(w.contains(0x10));
    EXPECT_TRUE(w.contains(0x12));
    EXPECT_FALSE(w.contains(0x0F));
    EXPECT_FALSE(w.contains(0x13));
    EXPECT_EQ(walkCount(w, 0x12, false), 3u);
}

TEST(PinSearchWindow, UnboundedSideNeverExhausts)
{
    // Floor set, no ceiling: scanning upward never ends...
    const PinSearchWindow floored{ 0xFF0000, {} };
    // ...and a ceiling without a floor leaves the downward scan unbounded.
    const PinSearchWindow ceiled{ {}, 0xFF0000 };
    for (uint64_t i = 0; i < 100; ++i) {
        EXPECT_TRUE(floored.nextCandidate(0xFF0000 + i, true).has_value());
        EXPECT_TRUE(ceiled.nextCandidate(0xFF0000 - i, false).has_value());
    }
}

TEST(PinSearchWindow, StartOutsideWindowExhaustsOnFirstAdvance)
{
    const PinSearchWindow cem{ kCemPinWindowFloor, kCemPinWindowCeil };
    // A manual start below the floor has nothing left to visit going downward.
    EXPECT_FALSE(cem.nextCandidate(kCemPinWindowFloor - 1, false).has_value());
    EXPECT_FALSE(cem.nextCandidate(kCemPinWindowCeil + 1, true).has_value());
}
