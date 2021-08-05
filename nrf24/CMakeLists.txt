# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/aniket/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(nrf24 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(nrf24 
    main.cpp 
    NRF24.cpp
    )

pico_set_program_name(nrf24 "nrf24")
pico_set_program_version(nrf24 "0.1")

pico_enable_stdio_uart(nrf24 1)
pico_enable_stdio_usb(nrf24 0)

# Add the standard library to the build
target_link_libraries(nrf24 pico_stdlib)

# Add any user requested libraries
target_link_libraries(nrf24
        hardware_spi
	pico_stdlib
        )

pico_add_extra_outputs(nrf24)

