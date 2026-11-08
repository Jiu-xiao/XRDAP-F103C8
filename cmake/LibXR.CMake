# LibXR
set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER st)
add_subdirectory(Middlewares/Third_Party/LibXR)
target_link_libraries(xr
    PUBLIC stm32cubemx
)

# The product's own headers (User/, including xrdap_port.hpp for XRDAP-Core) are
# private to the firmware target; LibXR gets the CubeMX headers through stm32cubemx.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE User)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    xr
)

file(GLOB LIBXR_USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp")

# XR 厂商扩展、状态灯规则和探针身份 / XR vendor extension, status light rule and probe
# identity.
include(${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/XRDAP-Core/xrdap_core.cmake)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${XRDAP_CORE_INCLUDE_DIRS})


target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE ${LIBXR_USER_SOURCES} ${XRDAP_CORE_SOURCES}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og)
    target_compile_options(xr PRIVATE -O2)
endif()
