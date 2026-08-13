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

if(NOT EXISTS "${ABSTRACTX_PATH}/include/asp_tlp64.hpp")
    message(FATAL_ERROR "AbstractX headers not found at ${ABSTRACTX_PATH}/include. Please set ABSTRACTX_PATH.")
endif()

message(STATUS "Found canonical AbstractX headers at: ${ABSTRACTX_PATH}/include")

# Include Directories
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/abstractx_engine"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flight"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/imu"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/baro"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/mag"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/esc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/rc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/led"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/display"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/logging"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target/sitl"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target/pico2_rp2350"
    "${ABSTRACTX_PATH}/include"
)

# Core Executable Target for SITL / Desktop / HIL Verification
add_executable(inav_abstractx_sitl
    src/main.cpp
    src/config/flash_storage.cpp
    src/msp/msp_protocol.cpp
    src/msp/msp_server.cpp
    src/logging/ctf_stream_server.cpp
    src/target/pico2_rp2350/pico2_target.cpp
)

# Unit Test Runner Executable Target
add_executable(run_unit_tests
    test/test_main.cpp
    src/config/flash_storage.cpp
    src/msp/msp_protocol.cpp
    src/msp/msp_server.cpp
    src/logging/ctf_stream_server.cpp
    src/target/pico2_rp2350/pico2_target.cpp
)

# On-Device Pico 2 W & Linux Hardware Test Target
add_executable(pico2_hw_test
    src/target/pico2_rp2350/pico2_hw_test.cpp
    src/config/flash_storage.cpp
    src/msp/msp_protocol.cpp
    src/msp/msp_server.cpp
    src/logging/ctf_stream_server.cpp
    src/target/pico2_rp2350/pico2_target.cpp
)

# Enable CTest integration
enable_testing()
add_test(NAME UnitTests COMMAND run_unit_tests)
add_test(NAME HardwareTest COMMAND pico2_hw_test)
