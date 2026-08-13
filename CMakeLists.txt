cmake_minimum_required(VERSION 3.20)
project(inav_abstractx CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler Warnings & Hardening
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Werror=vla)
endif()

# Locate Canonical AbstractX Header Directory
set(ABSTRACTX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../AbstractX" CACHE PATH "Path to AbstractX repository")

if(NOT EXISTS "${ABSTRACTX_PATH}/include/asp_tlp64.h")
    message(FATAL_ERROR "AbstractX headers not found at ${ABSTRACTX_PATH}/include. Please set ABSTRACTX_PATH.")
endif()

message(STATUS "Found canonical AbstractX headers at: ${ABSTRACTX_PATH}/include")

# Include Directories
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/abstractx_engine"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msp"
    "${ABSTRACTX_PATH}/include"
)

# Core Executable Target for SITL / Desktop / HIL Verification
add_executable(inav_abstractx_sitl
    src/main.cpp
    src/msp/msp_protocol.cpp
)

target_include_directories(inav_abstractx_sitl PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/abstractx_engine"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msp"
    "${ABSTRACTX_PATH}/include"
)
