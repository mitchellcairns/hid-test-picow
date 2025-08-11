# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hid_test_background C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECT_NAME}
        hid_test_background.c
        )

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  CYW43_LWIP=0
)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_flash 
        pico_bootrom
        pico_platform
        hardware_irq
        hardware_flash
        hardware_timer
        hardware_exception
        pico_rand

        pico_btstack_cyw43 
        pico_btstack_hci_transport_cyw43 
        pico_btstack_ble 
        pico_btstack_classic 
        pico_cyw43_arch_threadsafe_background 
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)

# BTStack Loop Implementation

project(hid_test_full C CXX ASM)

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECT_NAME}
        hid_test_full.c
        )

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  CYW43_LWIP=0
)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_flash 
        pico_bootrom
        pico_platform
        hardware_irq
        hardware_flash
        hardware_timer
        hardware_exception
        pico_rand

        pico_btstack_cyw43 
        pico_btstack_hci_transport_cyw43 
        pico_btstack_ble 
        pico_btstack_classic 
        pico_cyw43_arch_poll
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)