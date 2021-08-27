# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/aniket/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(AHT10 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(AHT10 
main.cpp 
src/AHT10.cpp)

# Add the directories for includes
target_include_directories(AHT10 PUBLIC 
inc 
src)

pico_set_program_name(AHT10 "AHT10")
pico_set_program_version(AHT10 "0.1")

pico_enable_stdio_uart(AHT10 1)
pico_enable_stdio_usb(AHT10 0)

# Add the standard library to the build
target_link_libraries(AHT10 pico_stdlib)

# Add any user requested libraries
target_link_libraries(AHT10
        pico_stdlib
        hardware_spi
        hardware_i2c
        )

pico_add_extra_outputs(AHT10)

