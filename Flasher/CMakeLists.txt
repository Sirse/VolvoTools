cmake_minimum_required(VERSION 3.16)
project(Flasher VERSION 1.0.0 LANGUAGES CXX)

# Explicit source lists: a glob hides new files until the next reconfigure and keeps
# deleted ones in the build.
set(FLASHER_SOURCES
    src/FlasherBase.cpp
    src/SBLProviderVBF.cpp
    src/UDSFlasher.cpp
    src/UDSReader.cpp
)

set(FLASHER_HEADERS
    flasher/FlasherBase.hpp
    flasher/FlasherCallback.hpp
    flasher/FlasherState.hpp
    flasher/SBLProviderBase.hpp
    flasher/SBLProviderVBF.hpp
    flasher/UDSFlasher.hpp
    flasher/UDSReader.hpp
)

add_library(${PROJECT_NAME} ${FLASHER_SOURCES} ${FLASHER_HEADERS})

if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
       # Prevents Windows.h from adding unnecessary includes
       WIN32_LEAN_AND_MEAN
       # Prevents Windows.h from defining min/max as macros
       NOMINMAX
    )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} Common j2534)

if (MSVC)
    target_compile_options(Flasher PRIVATE /W4)
endif()
