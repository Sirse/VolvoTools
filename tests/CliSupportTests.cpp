#include <common/CliSupport.hpp>

#include <gtest/gtest.h>

namespace {

// The suite mutates process-global stop/region state; every test leaves it clean.
class ConsoleStopPolicy : public ::testing::Test {
protected:
    void SetUp() override
    {
        common::resetStopRequested();
        common::endUninterruptibleRegion();
    }

    void TearDown() override
    {
        common::resetStopRequested();
        common::endUninterruptibleRegion();
    }
};

} // namespace

TEST_F(ConsoleStopPolicy, FirstRequestStopsGracefully)
{
    EXPECT_EQ(common::decideCtrlAction(false, false), common::StopDecision::GracefulStop);
    EXPECT_EQ(common::decideCtrlAction(true, false), common::StopDecision::ForceKill);
}

TEST_F(ConsoleStopPolicy, CriticalRegionIgnoresAnyRequest)
{
    EXPECT_EQ(common::decideCtrlAction(false, true), common::StopDecision::Ignore);
    EXPECT_EQ(common::decideCtrlAction(true, true), common::StopDecision::Ignore);
}

TEST_F(ConsoleStopPolicy, RegionScopeActivatesAndCleansUp)
{
    ASSERT_FALSE(common::isUninterruptibleRegionActive());

    // A press racing in just before the region begins...
    common::requestStop();
    {
        const common::UninterruptibleRegion region{ "test destructive phase" };
        EXPECT_TRUE(common::isUninterruptibleRegionActive());
        // ...stays swallowed for as long as the region is active: the handler must not
        // re-set the flag, and the region owner ignores it by design.
        EXPECT_TRUE(common::stopRequested.load());
    }

    // Leaving the region discards the stale request so it cannot leak into later phases.
    EXPECT_FALSE(common::isUninterruptibleRegionActive());
    EXPECT_FALSE(common::stopRequested.load());
}
