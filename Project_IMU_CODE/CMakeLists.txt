cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pio_ws2812)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(pio_ws2812 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_include_directories(pio_ws2812 PRIVATE ws2812.h)

target_sources(pio_ws2812 PRIVATE ws2812.c)

# pull in common dependencies
target_link_libraries(pio_ws2812 PRIVATE pico_stdlib hardware_pio hardware_adc hardware_pwm)

pico_add_extra_outputs(pio_ws2812)

  # enable usb output, disable uart output
  pico_enable_stdio_usb(pio_ws2812 1)
  pico_enable_stdio_uart(pio_ws2812 0)


# Additionally generate python and hex pioasm outputs for inclusion in the RP2040 datasheet
add_custom_target(pio_ws2812_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
        COMMAND Pioasm -o python ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        )
add_dependencies(pio_ws2812 pio_ws2812_datasheet)