cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_stepper C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(stepper
        main.cpp
)

pico_generate_pio_header(stepper
        ${CMAKE_CURRENT_LIST_DIR}/pulse.pio
)

# pull in common dependencies
target_link_libraries(stepper
        pico_stdlib
        hardware_pio
#        hardware_dma
)

# enable usb output, disable uart output
pico_enable_stdio_usb(stepper 1)
pico_enable_stdio_uart(stepper 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(stepper)
