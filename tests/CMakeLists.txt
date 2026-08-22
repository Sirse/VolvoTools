project(VolvoToolsTests LANGUAGES CXX)

find_package(GTest REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(VolvoToolsTests
    TestMain.cpp
    CommonProtocolTests.cpp
    UploadIntegrityTests.cpp
    CliParserTests.cpp
    ScriptModelTests.cpp
    PinSearchTests.cpp
    SecurityKeyTests.cpp
)

if (WIN32)
    target_compile_definitions(VolvoToolsTests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# The embedded vehicle configuration must stay in sync with the checked-in data.yaml (which is
# generated into CommonData.cpp at configure time). Point the sync test at the source YAML.
target_compile_definitions(VolvoToolsTests PRIVATE
    VOLVOTOOLS_DATA_YAML="${CMAKE_SOURCE_DIR}/Common/common/data.yaml"
    # The pre-pool v1 schema snapshot, read by the golden equivalence test: the v2 schema must
    # expand to exactly the same ConfigurationInfo as this file parsed with the v1 code path.
    VOLVOTOOLS_DATA_V1_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/data_v1.yaml"
)

# VolvoDiagCore pulls in Common (protocol/format helpers) and the CLI parser. Flasher is linked
# for the UDSReader-level integrity regression test (commitUploadResult).
target_link_libraries(VolvoToolsTests VolvoDiagCore Flasher GTest::gtest)

include(GoogleTest)
gtest_discover_tests(VolvoToolsTests)
