project(VolvoToolsTests LANGUAGES CXX)

find_package(GTest REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(VolvoToolsTests
    TestMain.cpp
    CommonProtocolTests.cpp
    CliParserTests.cpp
    ScriptModelTests.cpp
)

if (WIN32)
    target_compile_definitions(VolvoToolsTests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# VolvoDiagCore pulls in Common (protocol/format helpers) and the CLI parser.
target_link_libraries(VolvoToolsTests VolvoDiagCore GTest::gtest)

include(GoogleTest)
gtest_discover_tests(VolvoToolsTests)
