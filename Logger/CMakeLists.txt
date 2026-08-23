cmake_minimum_required(VERSION 3.16)
project(Logger VERSION 1.0.0 LANGUAGES CXX)

# Explicit source lists: a glob hides new files until the next reconfigure and keeps
# deleted ones in the build.
set(LOGGER_SOURCES
    src/Logger.cpp
    src/LogParameter.cpp
    src/LogParameters.cpp
)

set(LOGGER_HEADERS
    logger/Logger.hpp
    logger/LoggerCallback.hpp
    logger/LogParameter.hpp
    logger/LogParameters.hpp
)

add_library(${PROJECT_NAME} ${LOGGER_SOURCES} ${LOGGER_HEADERS})

if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
       # Prevents Windows.h from adding unnecessary includes
       WIN32_LEAN_AND_MEAN
       # Prevents Windows.h from defining min/max as macros
       NOMINMAX
    )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/fast-cpp-csv-parser)

target_link_libraries(${PROJECT_NAME} Common j2534)

if (MSVC)
    target_compile_options(Logger PRIVATE /W4)
endif()
