cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/aniket/pico/pico-sdk")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

# add_subdirectory(pico-cpp)
project(FreeRTOS_Blink)

# initialize the Pico SDK
pico_sdk_init()

add_executable(FreeRTOS_Blink
    src/main.cpp
    src/i2c_mpu9250.cpp
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(FreeRTOS_Blink FreeRTOS pico_stdlib hardware_gpio hardware_clocks hardware_spi hardware_i2c hardware_dma)

add_library(FreeRTOS STATIC
    FreeRTOS-Kernel/croutine.c
    FreeRTOS-Kernel/event_groups.c
    FreeRTOS-Kernel/list.c
    FreeRTOS-Kernel/queue.c
    FreeRTOS-Kernel/stream_buffer.c
    FreeRTOS-Kernel/tasks.c
    FreeRTOS-Kernel/timers.c FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c FreeRTOS-Kernel/portable/MemMang/heap_4.c)

target_include_directories(FreeRTOS PUBLIC FreeRTOS-Kernel/include inc/ FreeRTOS-Kernel/portable/GCC/ARM_CM0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(FreeRTOS_Blink)

# enable usb output, disable uart output
pico_enable_stdio_usb(FreeRTOS_Blink 0)
pico_enable_stdio_uart(FreeRTOS_Blink 1)

add_custom_command(TARGET FreeRTOS_Blink
    POST_BUILD
    COMMAND arm-none-eabi-size FreeRTOS_Blink.elf
)
