cmake_minimum_required(VERSION 3.16)
project(VolvoDiag VERSION 1.0.0 LANGUAGES CXX)

find_package(Easyloggingpp REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(argparse REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Explicit source lists: a glob hides new files until the next reconfigure and keeps
# deleted ones in the build.
set(CORE_SOURCES
    src/CliParser.cpp
    src/CommandRisk.cpp
    src/DiagContext.cpp
    src/ExitCodes.cpp
    src/OutputFormat.cpp
    src/RawCanCommands.cpp
    src/ReconCapture.cpp
    src/ScriptModel.cpp
    src/SoftIsoTp.cpp
    src/UdsCommands.cpp
)

set(CORE_HEADERS
    src/CliParser.hpp
    src/CommandRisk.hpp
    src/DiagContext.hpp
    src/ExitCodes.hpp
    src/IsoTpFrame.hpp
    src/IsoTpReassemble.hpp
    src/OutputFormat.hpp
    src/RawCanCommands.hpp
    src/ReconCapture.hpp
    src/ScriptModel.hpp
    src/SoftIsoTp.hpp
    src/UdsCommands.hpp
    src/VolvoDiagOptions.hpp
)

add_library(VolvoDiagCore STATIC ${CORE_SOURCES} ${CORE_HEADERS})
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
