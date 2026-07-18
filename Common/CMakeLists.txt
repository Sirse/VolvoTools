cmake_minimum_required(VERSION 3.16)
project(Common VERSION 1.0.0 LANGUAGES CXX)

find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Easyloggingpp REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/protocols SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/encryption SUB_SOURCES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/compression SUB_SOURCES)

set(SOURCE
  ${SOURCE}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CliSupport.cpp
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
