# cmake/xpack_arm_gcc.cmake
# xPack GNU Arm Embedded GCC Toolchain FetchContent Helper

include(FetchContent)

# Pinned xPack GCC Toolchain Version (Overrideable via -DXPACK_GCC_VERSION=...)
set(XPACK_GCC_VERSION "14.2.1-1.1" CACHE STRING "xPack GNU Arm Embedded GCC Version")

# Prevent CMake from re-checking download on routine builds
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Disable remote update checks on build" FORCE)

if(NOT DEFINED PICO_TOOLCHAIN_PATH AND NOT DEFINED ENV{PICO_TOOLCHAIN_PATH})
    message(STATUS "PICO_TOOLCHAIN_PATH not set. Fetching xPack ARM GCC v${XPACK_GCC_VERSION} binary...")

    FetchContent_Declare(
        xpack_gcc
        URL "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${XPACK_GCC_VERSION}/xpack-arm-none-eabi-gcc-${XPACK_GCC_VERSION}-linux-x64.tar.gz"
        UPDATE_DISCONNECTED ON
    )
    FetchContent_MakeAvailable(xpack_gcc)

    set(PICO_TOOLCHAIN_PATH "${xpack_gcc_SOURCE_DIR}/bin" CACHE PATH "Path to ARM GCC toolchain" FORCE)
else()
    if(DEFINED ENV{PICO_TOOLCHAIN_PATH} AND NOT DEFINED PICO_TOOLCHAIN_PATH)
        set(PICO_TOOLCHAIN_PATH "$ENV{PICO_TOOLCHAIN_PATH}" CACHE PATH "Path to ARM GCC toolchain" FORCE)
    endif()
    message(STATUS "Using existing ARM Toolchain at: ${PICO_TOOLCHAIN_PATH}")
endif()
