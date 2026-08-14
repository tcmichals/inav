cmake_minimum_required(VERSION 3.20)

# Raspberry Pi Pico SDK & Toolchain FetchContent Configuration
if(PICO_BOARD OR BUILD_PICO2)
    include(cmake/pico_sdk_fetch.cmake)
endif()

project(inav_abstractx CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Offline/Disconnected FetchContent (Ensures zero network latency on regular builds)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Disable remote update checks on build" FORCE)

# Default to Debug if CMAKE_BUILD_TYPE is not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
endif()

# Compiler Warnings & Hardening
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Werror=vla)
    add_compile_options($<$<CONFIG:Debug>:-g3> $<$<CONFIG:Debug>:-O0> $<$<CONFIG:Debug>:-DDEBUG>)
    add_compile_options($<$<CONFIG:Release>:-O3> $<$<CONFIG:Release>:-DNDEBUG>)
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
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/gps"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/esc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/rc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/led"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/display"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/logging"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target/linux_common"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target/sitl"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/target/pico2_rp2350"
    "${ABSTRACTX_PATH}/include"

)

# Common source files (portable — no platform I/O)
set(COMMON_SOURCES
    src/config/flash_storage.cpp
    src/msp/msp_protocol.cpp
    src/logging/ctf_stream_server.cpp
    src/target/pico2_rp2350/pico2_target.cpp
)

# ==============================================================================
# Target Definitions: Pico 2 (RP2350) Bare-Metal Target
# ==============================================================================
if(PICO_BOARD OR BUILD_PICO2)
    message(STATUS "Configuring bare-metal firmware for board: ${PICO_BOARD}")

    add_executable(inav_abstractx_pico2w
        src/target/pico2_rp2350/pico2_main.cpp
        ${COMMON_SOURCES}
    )

    target_link_libraries(inav_abstractx_pico2w PRIVATE
        pico_stdlib
        pico_multicore
        pico_cyw43_arch_lwip_poll
        hardware_pio
        hardware_dma
        hardware_spi
        hardware_i2c
        hardware_flash
    )

    # Enable .uf2, .bin, .hex firmware artifacts
    pico_add_extra_outputs(inav_abstractx_pico2w)

# ==============================================================================
# Target Definitions: Host Linux / SITL / SBC / Test Targets
# ==============================================================================
else()
    # Find Boost for Host / SITL TCP/UDP Async Networking
    find_package(Boost 1.74 REQUIRED COMPONENTS system)
    message(STATUS "Found Boost ${Boost_VERSION} at: ${Boost_INCLUDE_DIRS}")

    # Core Production Executable Target for SITL / Desktop / HIL Verification
    add_executable(inav_abstractx_sitl
        src/target/sitl/sitl_main.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(inav_abstractx_sitl PRIVATE Boost::system)

    # Dedicated Debug Executable Target for GDB / VSCode Debugger (-O0 -g3)
    add_executable(inav_sitl_debug
        src/target/sitl/sitl_main.cpp
        ${COMMON_SOURCES}
    )
    target_compile_options(inav_sitl_debug PRIVATE -g3 -O0 -DDEBUG)
    target_link_libraries(inav_sitl_debug PRIVATE Boost::system)

    # Dedicated Linux Quad-Core SBC Executable Target
    add_executable(inav_linux_sbc
        src/target/linux_sbc/linux_sbc_main.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(inav_linux_sbc PRIVATE Boost::system)

    # Locate CppUTest for Embedded Unit Testing & Test Groups
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(CPPUTEST QUIET cpputest)
    endif()

    if(CPPUTEST_FOUND)
        message(STATUS "Found CppUTest ${CPPUTEST_VERSION} (Include: ${CPPUTEST_INCLUDE_DIRS}, Libs: ${CPPUTEST_LIBRARIES})")
        include_directories(${CPPUTEST_INCLUDE_DIRS})
    else()
        find_library(CPPUTEST_LIB NAMES CppUTest PATHS /usr/local/lib /usr/lib)
        find_library(CPPUTEST_EXT_LIB NAMES CppUTestExt PATHS /usr/local/lib /usr/lib)
        find_path(CPPUTEST_INCLUDE_DIR NAMES CppUTest/TestHarness.h PATHS /usr/local/include /usr/include)
        if(CPPUTEST_LIB AND CPPUTEST_INCLUDE_DIR)
            set(CPPUTEST_LIBRARIES ${CPPUTEST_LIB} ${CPPUTEST_EXT_LIB})
            include_directories(${CPPUTEST_INCLUDE_DIR})
            set(CPPUTEST_FOUND TRUE)
            message(STATUS "Found CppUTest manually at ${CPPUTEST_LIB}")
        endif()
    endif()

    # Unit Test Runner Executable Target (Standalone & CppUTest Integration)
    add_executable(run_unit_tests
        test/test_main.cpp
        ${COMMON_SOURCES}
    )
    if(CPPUTEST_FOUND)
        target_link_libraries(run_unit_tests PRIVATE Boost::system ${CPPUTEST_LIBRARIES})
        target_compile_definitions(run_unit_tests PRIVATE HAVE_CPPUTEST=1)
    else()
        target_link_libraries(run_unit_tests PRIVATE Boost::system)
    endif()

    # On-Device Pico 2 W & Linux Hardware Test Target
    add_executable(pico2_hw_test
        src/target/pico2_rp2350/pico2_hw_test.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(pico2_hw_test PRIVATE Boost::system)

    # Enable CTest integration
    enable_testing()
    add_test(NAME UnitTests COMMAND run_unit_tests)
    add_test(NAME HardwareTest COMMAND pico2_hw_test)
endif()

