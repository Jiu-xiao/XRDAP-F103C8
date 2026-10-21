cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_link_options(-flto)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME XRDAP_F103C8)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION 0.6.0)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Firmware version, reported in DAP_Info: the version above plus the commit it was built
# from, e.g. 0.6.0+g9c0860c (".dirty" for uncommitted changes). A build from a copy
# without .git passes the commit with -DXRDAP_GIT_REV=<short hash>[.dirty].
if(NOT DEFINED XRDAP_GIT_REV)
    execute_process(COMMAND git rev-parse --short=7 HEAD
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                    OUTPUT_VARIABLE XRDAP_GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET RESULT_VARIABLE _git_result)
    if(_git_result EQUAL 0)
        execute_process(COMMAND git status --porcelain --untracked-files=no
                        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                        OUTPUT_VARIABLE _git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
        if(_git_dirty)
            string(APPEND XRDAP_GIT_REV ".dirty")
        endif()
    else()
        set(XRDAP_GIT_REV "")
    endif()
endif()
set(XRDAP_FIRMWARE_VERSION "${PROJECT_VERSION}")
if(XRDAP_GIT_REV)
    string(APPEND XRDAP_FIRMWARE_VERSION "+g${XRDAP_GIT_REV}")
endif()
message(STATUS "XR-DAP firmware version ${XRDAP_FIRMWARE_VERSION}")

set(XR_CLOCK_MODE "72M" CACHE STRING "System clock mode: 72M or 96M")
set_property(CACHE XR_CLOCK_MODE PROPERTY STRINGS 72M 96M)

if(XR_CLOCK_MODE STREQUAL "72M")
    set(XR_CLOCK_MODE_DEFINE XR_CLOCK_MODE=72)
elseif(XR_CLOCK_MODE STREQUAL "96M")
    set(XR_CLOCK_MODE_DEFINE XR_CLOCK_MODE=96)
else()
    message(FATAL_ERROR "Unsupported XR_CLOCK_MODE=${XR_CLOCK_MODE}; expected 72M or 96M")
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "XRDAP-F103C8"
    EXCLUDE_FROM_ALL TRUE
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${XR_CLOCK_MODE_DEFINE}
    XRDAP_FIRMWARE_VERSION="${XRDAP_FIRMWARE_VERSION}"
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add LibXR
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)

function(xr_add_clock_build_target target_name clock_mode)
    set(_clock_build_dir ${CMAKE_BINARY_DIR}/${target_name})
    set(_cfg_args
        -S ${CMAKE_SOURCE_DIR}
        -B ${_clock_build_dir}
        -G ${CMAKE_GENERATOR}
        -DXR_CLOCK_MODE=${clock_mode}
    )

    if(CMAKE_BUILD_TYPE)
        list(APPEND _cfg_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
    endif()

    if(DEFINED CMAKE_TOOLCHAIN_FILE)
        list(APPEND _cfg_args -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
    endif()

    if(DEFINED CMAKE_MAKE_PROGRAM)
        list(APPEND _cfg_args -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
    endif()

    if(DEFINED CMAKE_C_COMPILER)
        list(APPEND _cfg_args -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
    endif()

    if(DEFINED CMAKE_CXX_COMPILER)
        list(APPEND _cfg_args -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    endif()

    if(DEFINED CMAKE_ASM_COMPILER)
        list(APPEND _cfg_args -DCMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER})
    endif()

    add_custom_target(${target_name}
        COMMAND ${CMAKE_COMMAND} ${_cfg_args}
        COMMAND ${CMAKE_COMMAND} --build ${_clock_build_dir} --target ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Build ${CMAKE_PROJECT_NAME} with XR_CLOCK_MODE=${clock_mode}"
    )
endfunction()

xr_add_clock_build_target(${CMAKE_PROJECT_NAME}_clock_72m 72M)
xr_add_clock_build_target(${CMAKE_PROJECT_NAME}_clock_96m 96M)

add_custom_target(${CMAKE_PROJECT_NAME}_clock_all ALL
    DEPENDS
        ${CMAKE_PROJECT_NAME}_clock_72m
        ${CMAKE_PROJECT_NAME}_clock_96m
)
