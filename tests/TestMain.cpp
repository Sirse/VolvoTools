#include <easylogging++.h>
#include <gtest/gtest.h>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv)
{
    // Keep easylogging quiet during tests.
    el::Configurations conf;
    conf.setToDefault();
    conf.setGlobally(el::ConfigurationType::Enabled, "false");
    el::Loggers::reconfigureAllLoggers(conf);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
