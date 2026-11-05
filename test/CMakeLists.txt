# Host (PC) tests of the hardware-independent parts of the firmware.
#   cmake -S test -B build-test && cmake --build build-test && ctest --test-dir build-test
cmake_minimum_required(VERSION 3.22)
project(xrdap_f103_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

set(REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(CORE_DIR ${REPO_DIR}/Middlewares/Third_Party/XRDAP-Core)
# Core's dap_cmd.hpp includes daplink_v2_def.hpp from LibXR; these are the LibXR header
# directories that file needs (headers only, nothing of LibXR is compiled).
set(LIBXR_SRC ${REPO_DIR}/Middlewares/Third_Party/LibXR/src)

add_executable(test_led_drive test_led_drive.cpp)
target_include_directories(test_led_drive PRIVATE
    ${REPO_DIR}/User
    ${CORE_DIR}
    ${LIBXR_SRC}/driver/usb/device/dap
    ${LIBXR_SRC}/driver
    ${LIBXR_SRC}/core
    ${LIBXR_SRC}/core/assert
)
target_compile_options(test_led_drive PRIVATE -Wall -Wextra -Werror)
add_test(NAME led_drive COMMAND test_led_drive)
