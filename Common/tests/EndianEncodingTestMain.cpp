#include <easylogging++.h>
#include <gtest/gtest.h>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
