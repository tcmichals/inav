# cmake/pico_sdk_fetch.cmake
# Raspberry Pi Pico SDK FetchContent Manager with Pinned Versioning and Offline/Disconnected Builds

include(FetchContent)

# Pinned Pico SDK Version (Overrideable via -DPICO_SDK_VERSION=...)
set(PICO_SDK_VERSION "2.1.1" CACHE STRING "Target Raspberry Pi Pico SDK Version")

# Prevent CMake from querying remote Git repositories on routine builds
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Disable remote update checks on build" FORCE)

if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    message(STATUS "PICO_SDK_PATH not set. Fetching Raspberry Pi Pico SDK v${PICO_SDK_VERSION} via FetchContent...")

    FetchContent_Declare(
        pico_sdk
        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
        GIT_TAG        ${PICO_SDK_VERSION}
        GIT_SHALLOW    TRUE
        GIT_SUBMODULES tinyusb cyw43-driver lwip mbedtls
        UPDATE_DISCONNECTED ON
    )

    FetchContent_MakeAvailable(pico_sdk)
    set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE PATH "Path to the Pico SDK" FORCE)
else()
    if(DEFINED ENV{PICO_SDK_PATH} AND NOT DEFINED PICO_SDK_PATH)
        set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "Path to the Pico SDK" FORCE)
    endif()
    message(STATUS "Using existing Pico SDK at: ${PICO_SDK_PATH}")
endif()

# Initialize Pico SDK Subsystems
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()
