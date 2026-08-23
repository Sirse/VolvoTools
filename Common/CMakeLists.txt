cmake_minimum_required(VERSION 3.16)
project(Common VERSION 1.0.0 LANGUAGES CXX)

find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Easyloggingpp REQUIRED)

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

# Explicit source lists: a glob hides new files until the next reconfigure and keeps
# deleted ones in the build.
set(COMMON_SOURCES
    src/CliSupport.cpp
    src/Gateway.cpp
    src/J2534ChannelProvider.cpp
    src/RuntimeDiagnostics.cpp
    src/Util.cpp
    src/VBFParser.cpp
    src/protocols/UDSDid.cpp
    src/protocols/UDSDtc.cpp
    src/protocols/UDSError.cpp
    src/protocols/UDSMessage.cpp
    src/protocols/UDSPinFinder.cpp
    src/protocols/UDSProtocolCommonSteps.cpp
    src/protocols/UDSRequest.cpp
    src/protocols/UDSStepType.cpp
    src/protocols/UploadIntegrity.cpp
)

set(COMMON_HEADERS
    common/BusConfiguration.hpp
    common/CarPlatform.hpp
    common/CliSupport.hpp
    common/CommonData.hpp
    common/CompressionType.hpp
    common/ConfigurationInfo.hpp
    common/ECUInfo.hpp
    common/Gateway.hpp
    common/GatewayEndpoint.hpp
    common/J2534ChannelProvider.hpp
    common/ProtocolType.hpp
    common/RuntimeDiagnostics.hpp
    common/Util.hpp
    common/VBF.hpp
    common/VBFChunk.hpp
    common/VBFHeader.hpp
    common/VBFParser.hpp
    common/encryption/EncryptionType.hpp
    common/protocols/PinSearch.hpp
    common/protocols/UDSDid.hpp
    common/protocols/UDSDtc.hpp
    common/protocols/UDSError.hpp
    common/protocols/UDSMessage.hpp
    common/protocols/UDSPinFinder.hpp
    common/protocols/UDSProtocolCommonSteps.hpp
    common/protocols/UDSRequest.hpp
    common/protocols/UDSService.hpp
    common/protocols/UDSStepType.hpp
    common/protocols/UploadIntegrity.hpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${COMMON_SOURCES} ${COMMON_HEADERS})

add_library(${PROJECT_NAME} ${COMMON_SOURCES} ${COMMON_HEADERS} ${COMMON_DATA_GENERATED})

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

if (MSVC)
    target_compile_options(Common PRIVATE /W4)
endif()
