cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040zero_swim_slow_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040zero_swim_slow_test
    main.c
)

target_link_libraries(rp2040zero_swim_slow_test
    pico_stdlib
    hardware_gpio
    hardware_clocks
)

pico_enable_stdio_usb(rp2040zero_swim_slow_test 1)
pico_enable_stdio_uart(rp2040zero_swim_slow_test 0)

pico_add_extra_outputs(rp2040zero_swim_slow_test)
