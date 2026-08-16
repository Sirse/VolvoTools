cmake_minimum_required(VERSION 3.16)
project(Common VERSION 1.0.0 LANGUAGES CXX)

find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Easyloggingpp REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/protocols SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/encryption SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/compression SUB_SOURCES)

# The vehicle configuration ships as common/data.yaml. CommonData.cpp used to be a hand-maintained
# hex blob of the same file, which let the two drift apart (data.yaml itself is not part of any
# CMake target). Regenerate CommonData.cpp from data.yaml as a build step: add_custom_command gives
# the generated file a real dependency on data.yaml and the generator script, so an edit to the YAML
# re-runs the generator on the next cmake --build instead of silently keeping a stale configuration.
set(COMMON_CONFIG_YAML ${CMAKE_CURRENT_SOURCE_DIR}/common/data.yaml)
set(COMMON_DATA_GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_common_data.py)
set(COMMON_DATA_GENERATED ${CMAKE_CURRENT_BINARY_DIR}/generated/CommonData.cpp)
find_package(Python3 REQUIRED)
add_custom_command(
    OUTPUT ${COMMON_DATA_GENERATED}
    COMMAND ${Python3_EXECUTABLE} ${COMMON_DATA_GENERATOR} ${COMMON_CONFIG_YAML} ${COMMON_DATA_GENERATED}
    DEPENDS ${COMMON_DATA_GENERATOR} ${COMMON_CONFIG_YAML}
    COMMENT "Generating CommonData.cpp from data.yaml"
    VERBATIM
)

set(SOURCE
  ${SOURCE}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CliSupport.cpp
  ${COMMON_DATA_GENERATED}
  ${SUB_SOURCES}
)
list(REMOVE_DUPLICATES SOURCE)

FILE(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/common/*.h*)
FILE(GLOB PRIVATE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h*)

add_library(${PROJECT_NAME} ${SOURCE} ${HEADERS} ${PRIVATE_HEADERS})

if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE   
       # Prevents Windows.h from adding unnecessary includes    
       WIN32_LEAN_AND_MEAN  
       # Prevents Windows.h from defining min/max as macros 
       NOMINMAX 
    )   
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp j2534 boost::boost intelhex easyloggingpp::easyloggingpp version)
if (WIN32)
    target_link_libraries(${PROJECT_NAME} advapi32)
endif()

option(VOLVOTOOLS_BUILD_ENDIAN_TESTS "Build endian encoding unit tests" OFF)
if (VOLVOTOOLS_BUILD_ENDIAN_TESTS)
    find_package(GTest REQUIRED)
    add_executable(EndianEncodingTests tests/EndianEncodingTestMain.cpp tests/EndianEncodingTests.cpp)
    target_link_libraries(EndianEncodingTests PRIVATE Common GTest::gtest)
    include(GoogleTest)
    gtest_discover_tests(EndianEncodingTests)
endif()
