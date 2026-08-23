cmake_minimum_required(VERSION 3.16)
project(VolvoLogger VERSION 1.0.0 LANGUAGES CXX)

find_package(Easyloggingpp REQUIRED)
find_package(argparse REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Explicit source list: a glob hides new files until the next reconfigure and keeps
# deleted ones in the build.
add_executable(${PROJECT_NAME}
    src/FileLogWriter.cpp
    src/LoggerApplication.cpp
    src/VolvoLogger.cpp
)

if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
       # Prevents Windows.h from adding unnecessary includes
       WIN32_LEAN_AND_MEAN
       # Prevents Windows.h from defining min/max as macros
       NOMINMAX
    )
endif()
target_link_libraries(${PROJECT_NAME} Logger Common j2534 argparse::argparse easyloggingpp::easyloggingpp)

if (MSVC)
    target_compile_options(VolvoLogger PRIVATE /W4)
endif()
