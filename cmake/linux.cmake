message(STATUS "Linux build system detected")
main_sources(LINUX_COMMON_SRC_EXCLUDES
    build/atomic.h
    drivers/system.c
    drivers/time.c
    drivers/timer.c
    drivers/rcc.c
    drivers/persistent.c
    drivers/bus.c

    drivers/display_ug2864hsweg01.c
    drivers/exti.c

)

main_sources(LINUX_SRC
    config/config_streamer_file.c
    target/LINUX/bus.c
    target/LINUX/exti.c

)



set(LINUX_LINK_OPTIONS
    -Wl,-L${STM32_LINKER_DIR}
)

if(${CYGWIN})
    set(LINUX_LINK_OPTIONS ${LINUX_LINK_OPTIONS} "-static-libgcc")
endif()

set(LINUX_LINK_LIBRARIS
    -lpthread
    -lm
    -lc
)

if(NOT MACOSX)
    set(LINUX_LINK_LIBRARIS ${LINUX_LINK_LIBRARIS} -lrt)
endif()

set(LINUX_COMPILE_OPTIONS
    -Wno-format #Fixme: Compile for 32bit, but settings.rb has to be adjusted
    -funsigned-char
)

if(DEBUG)
    message(STATUS "Debug mode enabled. Adding -g to LINUX_COMPILE_OPTIONS.")
    list(APPEND LINUX_COMPILE_OPTIONS -g)
endif()

if(NOT MACOSX)
    set(LINUX_COMPILE_OPTIONS ${LINUX_COMPILE_OPTIONS}
        -Wno-return-local-addr
        -Wno-error=maybe-uninitialized
        -fsingle-precision-constant
    )
    if (CMAKE_COMPILER_IS_GNUCC AND NOT CMAKE_C_COMPILER_VERSION VERSION_LESS 12.0)
        set(LINUX_LINK_OPTIONS ${LINUX_LINK_OPTIONS} "-Wl,--no-warn-rwx-segments")
    endif()
else()
    set(LINUX_COMPILE_OPTIONS ${LINUX_COMPILE_OPTIONS}
    )
endif()

set(LINUX_DEFINITIONS
    LINUX_BUILD
)

function (target_linux name)
    if(CMAKE_VERSION VERSION_GREATER 3.22)
        set(CMAKE_C_STANDARD 17)
    endif()

    if(NOT host STREQUAL TOOLCHAIN)
        return()
    endif()

    exclude(COMMON_SRC "${LINUX_COMMON_SRC_EXCLUDES}")

    set(target_sources)
    list(APPEND target_sources ${LINUX_SRC})
    file(GLOB target_c_sources "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
    file(GLOB target_h_sources "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    list(APPEND target_sources ${target_c_sources} ${target_h_sources})

    set(target_definitions ${COMMON_COMPILE_DEFINITIONS})

    set(hse_mhz ${STM32_DEFAULT_HSE_MHZ})
    math(EXPR hse_value "${hse_mhz} * 1000000")
    list(APPEND target_definitions "HSE_VALUE=${hse_value}")

    if (MSP_UART) 
        list(APPEND target_definitions "MSP_UART=${MSP_UART}")
    endif()

    string(TOLOWER ${PROJECT_NAME} lowercase_project_name)
    set(binary_name ${lowercase_project_name}_${FIRMWARE_VERSION}_${name})
    if(DEFINED BUILD_SUFFIX AND NOT "" STREQUAL "${BUILD_SUFFIX}")
        set(binary_name "${binary_name}_${BUILD_SUFFIX}")
    endif()

    list(APPEND target_definitions ${LINUX_DEFINITIONS})
    set(exe_target ${name}.elf)
    add_executable(${exe_target})
    target_sources(${exe_target} PRIVATE ${target_sources} ${COMMON_SRC})
    target_include_directories(${exe_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_definitions(${exe_target} PRIVATE ${target_definitions})


    if(WARNINGS_AS_ERRORS)
        target_compile_options(${exe_target} PRIVATE -Werror)
    endif()

    target_compile_options(${exe_target} PRIVATE ${LINUX_COMPILE_OPTIONS})

    target_link_libraries(${exe_target} PRIVATE ${LINUX_LINK_LIBRARIS})
    target_link_options(${exe_target} PRIVATE ${LINUX_LINK_OPTIONS})

    set(script_path ${MAIN_SRC_DIR}/target/link/linux.ld)
    if(NOT EXISTS ${script_path})
        message(FATAL_ERROR "linker script ${script_path} doesn't exist")
    endif()
    set_target_properties(${exe_target} PROPERTIES LINK_DEPENDS ${script_path})
    if(NOT MACOSX)
        target_link_options(${exe_target} PRIVATE -T${script_path})
    endif()

    if(${CYGWIN})
        set(exe_filename ${CMAKE_BINARY_DIR}/${binary_name}.exe)
    else()
        set(exe_filename ${CMAKE_BINARY_DIR}/${binary_name})
    endif()

    add_custom_target(${name} ALL
        cmake -E copy $<TARGET_FILE:${exe_target}> ${exe_filename}
    )

    setup_firmware_target(${exe_target} ${name} ${ARGN})
    #clean_<target>
    set(generator_cmd "")
    if (CMAKE_GENERATOR STREQUAL "Unix Makefiles")
        set(generator_cmd "make")
    elseif(CMAKE_GENERATOR STREQUAL "Ninja")
        set(generator_cmd "ninja")
    endif()
    if (NOT generator_cmd STREQUAL "")
        set(clean_target "clean_${name}")
        add_custom_target(${clean_target}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMAND ${generator_cmd} clean
            COMMENT "Removing intermediate files for ${name}")
        set_property(TARGET ${clean_target} PROPERTY
            EXCLUDE_FROM_ALL 1
            EXCLUDE_FROM_DEFAULT_BUILD 1)
    endif()
endfunction()
