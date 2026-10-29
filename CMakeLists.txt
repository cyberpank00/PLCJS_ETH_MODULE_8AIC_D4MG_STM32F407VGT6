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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME PLCJS_ETH_MODULE_8AIC_D4MG_STM32F407VGT6)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------- Module identity (overridable per module variant) ---------------
# Pass -DPRODUCT_ID=0x... / -DHW_REVISION=0xMMmm / -DFW_VERSION=0xMMmm on the
# cmake command line to customise the firmware header for each module variant.
#
# HW_REVISION encoding: (major << 8) | minor  →  e.g. hw:01.02 = 0x0102
# FW_VERSION  encoding: (major << 8) | minor  →  e.g. fw:01.02 = 0x0102
# OTA acceptance: product_id exact match + hw_revision major byte match.
# Identity now lives in Application/fw_header/fw_header.h (single source of
# truth). Override at build time with -DFW_PRODUCT_ID / -DFW_HW_REVISION /
# -DFW_VERSION_VALUE if ever needed.

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Application sources
    Application/app/app.c
    Application/led/led_module.c
    Application/temp/temp_module.c
    Application/button/button_module.c
    Application/ksz8863/ksz8863.c
    Application/net_id/net_id.c
    Application/discovery/discovery.c
    Application/addr_probe/addr_probe.c
    Application/settings/settings.c
    Application/calstore/calstore.c
    Application/modbus/modbus_app.c
    Application/modbus/modbus_tcp_server.c
    Application/fw_header/fw_header.c
    # Analog current-input front-end
    Application/spi/spi_bus.c
    Application/ads1220/ads1220.c
    Application/aic/aic_module.c
    # Third-party
    Application/third_party/nanomodbus/nanomodbus.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/app
    Application/led
    Application/temp
    Application/button
    Application/ksz8863
    Application/net_id
    Application/discovery
    Application/addr_probe
    Application/settings
    Application/calstore
    Application/modbus
    Application/fw_header
    Application/spi
    Application/ads1220
    Application/aic
    Application/third_party/nanomodbus
)

# Disable nanoMODBUS client side - we only need the slave/server.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    NMBS_CLIENT_DISABLED=1
)

# Firmware-header identity (product_id / hw_revision / fw_version) is defined
# in Application/fw_header/fw_header.h; no CMake-passed identity defines needed.

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# ---------- Post-build: generate .hex and .bin images ---------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
