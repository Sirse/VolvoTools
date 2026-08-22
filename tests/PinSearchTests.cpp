#include <common/protocols/PinSearch.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using common::kCemPinWindowCeil;
using common::kCemPinWindowFloor;
using common::AuthResult;
using common::PinSearchDispatcher;
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

// ---- dispatcher: wrong key / transient failure / success -------------------

namespace {

// Feeds a scripted sequence of outcomes to the dispatcher and collects the actions.
std::vector<PinSearchDispatcher::Action> feed(PinSearchDispatcher& d,
    const std::vector<AuthResult>& script)
{
    std::vector<PinSearchDispatcher::Action> actions;
    for (const auto result : script) {
        actions.push_back(d.step(result));
    }
    return actions;
}

} // namespace

TEST(PinSearchDispatcher, WrongKeyAdvancesCandidate)
{
    PinSearchDispatcher d{ { kCemPinWindowFloor, kCemPinWindowCeil }, false,
        kCemPinWindowCeil };
    EXPECT_EQ(d.currentPin(), kCemPinWindowCeil);
    EXPECT_EQ(d.step(AuthResult::WrongKey), PinSearchDispatcher::Action::Continue);
    EXPECT_EQ(d.currentPin(), kCemPinWindowCeil - 1);
}

TEST(PinSearchDispatcher, TransientErrorRepeatsSameCandidate)
{
    PinSearchDispatcher d{ {}, true, 100 };
    EXPECT_EQ(d.step(AuthResult::TransientError),
        PinSearchDispatcher::Action::RetrySameCandidate);
    // A bus failure says nothing about the key: the candidate must not be skipped.
    EXPECT_EQ(d.currentPin(), 100u);
}

TEST(PinSearchDispatcher, ConsecutiveTransientsGiveUp)
{
    PinSearchDispatcher d{ {}, true, 100, /*maxConsecutiveTransientErrors=*/3 };
    EXPECT_EQ(feed(d, { AuthResult::TransientError, AuthResult::TransientError }),
        (std::vector<PinSearchDispatcher::Action>{
            PinSearchDispatcher::Action::RetrySameCandidate,
            PinSearchDispatcher::Action::RetrySameCandidate }));
    EXPECT_EQ(d.step(AuthResult::TransientError), PinSearchDispatcher::Action::GiveUp);
}

TEST(PinSearchDispatcher, SuccessfulAttemptResetsTransientStreak)
{
    PinSearchDispatcher d{ {}, true, 100, /*maxConsecutiveTransientErrors=*/2 };
    EXPECT_EQ(d.step(AuthResult::TransientError),
        PinSearchDispatcher::Action::RetrySameCandidate);
    EXPECT_EQ(d.step(AuthResult::WrongKey), PinSearchDispatcher::Action::Continue);
    // The streak was reset by the wrong-key verdict, so one more transient is survivable.
    EXPECT_EQ(d.step(AuthResult::TransientError),
        PinSearchDispatcher::Action::RetrySameCandidate);
    EXPECT_EQ(d.currentPin(), 101u);
}

TEST(PinSearchDispatcher, UnlockedFinishesWithTheTriedCandidate)
{
    PinSearchDispatcher d{ { kCemPinWindowFloor, kCemPinWindowCeil }, false,
        kCemPinWindowCeil - 5 };
    ASSERT_EQ(d.step(AuthResult::WrongKey), PinSearchDispatcher::Action::Continue);
    ASSERT_EQ(d.step(AuthResult::TransientError),
        PinSearchDispatcher::Action::RetrySameCandidate);
    EXPECT_EQ(d.step(AuthResult::Unlocked), PinSearchDispatcher::Action::Found);
    EXPECT_EQ(d.currentPin(), kCemPinWindowCeil - 6);
}

TEST(PinSearchDispatcher, LastWindowCandidateRejectEndsScanAsExhausted)
{
    PinSearchDispatcher d{ { kCemPinWindowFloor, kCemPinWindowCeil }, false,
        kCemPinWindowFloor + 1 };
    EXPECT_EQ(d.step(AuthResult::WrongKey), PinSearchDispatcher::Action::Continue);
    EXPECT_EQ(d.currentPin(), kCemPinWindowFloor);
    // The floor itself was still tried; stepping below it is the not-found verdict.
    EXPECT_EQ(d.step(AuthResult::WrongKey), PinSearchDispatcher::Action::Exhausted);
}
