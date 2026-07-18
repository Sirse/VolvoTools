cmake_minimum_required(VERSION 3.16)
project(VolvoDiag VERSION 1.0.0 LANGUAGES CXX)

find_package(Easyloggingpp REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(argparse REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SUB_SOURCES)

# Split into a core library (all logic) and a thin executable (just main), so the
# command/parsing logic can be linked into the test target too.
set(CORE_SOURCES ${SUB_SOURCES})
list(REMOVE_ITEM CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/VolvoDiag.cpp)

FILE(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h*)

add_library(VolvoDiagCore STATIC ${CORE_SOURCES} ${HEADERS})
target_include_directories(VolvoDiagCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(VolvoDiagCore PUBLIC Common j2534 argparse::argparse easyloggingpp::easyloggingpp tomlplusplus::tomlplusplus)
if (WIN32)
    target_link_libraries(VolvoDiagCore PUBLIC bcrypt)
endif()

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/VolvoDiag.cpp)
target_link_libraries(${PROJECT_NAME} VolvoDiagCore)

if (WIN32)
    target_compile_definitions(VolvoDiagCore PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
