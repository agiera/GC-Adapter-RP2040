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
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gc_adapter_rp2040 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(
  include
  common/include
  common/include/devices
  common/include/generated
  common/include/switch
  common/include/usb
  common/include/utilities
  common/include/ll
)

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECT_NAME}
  src/main.c 
  src/joybus_itf.c 
  src/rgb_itf.c 

  common/utilities/rgb.c 
  common/utilities/joybus.c 
  common/utilities/interval.c

  common/ll/adapter_ll_rp2040.c 

  common/usb/desc_bos.c 
  
  common/adapter.c
  common/adapter_tusb.c 

  common/adapter_defines.c 
  common/adapter_settings.c 

  common/usb/xinput_driver.c 
  common/usb/ginput_driver.c

  common/switch/switch_analog.c 
  common/switch/switch_commands.c 
  common/switch/switch_spi.c 

  common/devices/gcinput.c 
  common/devices/swpro.c 
  common/devices/webusb.c 
  common/devices/xinput.c
)

pico_set_program_name(${PROJECT_NAME} "${PROJECT_NAME}")
pico_set_program_version(${PROJECT_NAME} "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 0)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/generated)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/include/generated)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/sk6805_ec15.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/include/generated)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/joybus.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/include/generated)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	PICO_DEFAULT_UART_TX_PIN=12
	PICO_DEFAULT_UART_RX_PIN=13
    ADAPTER_MCU_TYPE=1
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	)

# Pull in pico libraries that we need
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  pico_stdlib
  pico_bootrom
  pico_multicore
  pico_platform
  pico_rand

  hardware_gpio
  hardware_adc
  hardware_flash
  hardware_pio
  hardware_dma
  hardware_pwm
  hardware_spi
  hardware_sync
  hardware_timer
  hardware_irq

  tinyusb_device
  tinyusb_board
)

# Add the standard include files to the build
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(${PROJECT_NAME})

add_custom_command(TARGET ${PROJECT_NAME}
 					POST_BUILD
					COMMAND ${CMAKE_COMMAND} 
					-DTARGET_NAME=${PROJECT_NAME}
					-DTARGET_PATH=${CMAKE_CURRENT_SOURCE_DIR}
					-P ${CMAKE_SOURCE_DIR}/manifest.cmake
					COMMENT "Updating version in manifest.json"
					WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
					)
