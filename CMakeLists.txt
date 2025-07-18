# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gba-cart-read C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(gba-cart-read
    cartridge.cpp
    filesystem.cpp
    gba-cart-read.cpp
    usb.cpp
    usb_descriptors.c
)

pico_generate_pio_header(gba-cart-read ${CMAKE_CURRENT_LIST_DIR}/gba-cart.pio)

pico_set_program_name(gba-cart-read "gba-cart-read")
pico_set_program_version(gba-cart-read "0.1")

pico_enable_stdio_uart(gba-cart-read 0)
pico_enable_stdio_usb(gba-cart-read 1)

# Add the standard library to the build
target_link_libraries(gba-cart-read
    pico_stdlib
    pico_unique_id
    tinyusb_device
)

# Add any user requested libraries
target_link_libraries(gba-cart-read
    hardware_dma
    hardware_pio
)

target_include_directories(gba-cart-read PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_options(gba-cart-read PRIVATE -Wall)

pico_add_extra_outputs(gba-cart-read)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gba-cart-read.uf2
    DESTINATION bin
)