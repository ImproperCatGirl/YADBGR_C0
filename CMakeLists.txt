cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32_gcc.cmake)



project(C0_CMSIS_DAP C ASM)
set(MAIN_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32c0xx_hal_msp.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32c0xx_it.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/USB.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/usb_descriptors.c

${CMAKE_CURRENT_SOURCE_DIR}/Src/DAP.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/DAP_vendor.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/SW_DP.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/JTAG_DP.c
${CMAKE_CURRENT_SOURCE_DIR}/Src/DDMI.c
)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMSIS_COMP_LIST "")


set(HAL_COMP_LIST "")

list(APPEND HAL_COMP_LIST STM32C0)

list(APPEND CMSIS_COMP_LIST STM32C0)

find_package(CMSIS COMPONENTS STM32C0 REQUIRED)
find_package(HAL COMPONENTS STM32C0 REQUIRED)

# Find all device specific drivers:
#find_package(HAL COMPONENTS STM32L0 STM32F1 STM32C0 REQUIRED)
# Find drivers for all families:
#find_package(HAL COMPONENTS RCC GPIO CORTEX REQUIRED)
# Find LL driver:
#find_package(HAL COMPONENTS LL_GPIO REQUIRED)
# Find everything:
find_package(HAL REQUIRED)



file(GLOB FreeRTOS_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Src/FreeRTOS/*.c")
file(GLOB FreeRTOS_ASM_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Src/FreeRTOS/*.S")

add_executable(C0_CMSIS_DAP ${MAIN_SOURCE_FILE}  ${FreeRTOS_SOURCE_FILES} ${FreeRTOS_ASM_FILES}
)



target_include_directories(
    C0_CMSIS_DAP PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/FreeRTOS/
)


# Path to the submodule we added
set(TINYUSB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb)

# 1. Create the library target
add_library(tinyusb_upstream STATIC)

# 2. Add all the necessary TinyUSB source files
# This includes the core stack and the RP2040-specific port
file(GLOB_RECURSE TINYUSB_SOURCES
    "${TINYUSB_PATH}/src/*.c"
)

target_sources(tinyusb_upstream PRIVATE ${TINYUSB_SOURCES})


target_link_libraries(tinyusb_upstream
  PUBLIC
    HAL::STM32::C0
    CMSIS::STM32::C071FB
)


target_include_directories(
    tinyusb_upstream PUBLIC
        # This allows your main app to #include "tusb.h"
        "${TINYUSB_PATH}/src" 
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/FreeRTOS/
)






target_link_libraries(C0_CMSIS_DAP
    HAL::STM32::C0::UART
    HAL::STM32::C0::UARTEx
    HAL::STM32::C0::I2C
    HAL::STM32::C0::SPI
    HAL::STM32::C0::I2CEx
    HAL::STM32::C0::GPIO
    HAL::STM32::C0::EXTI
    HAL::STM32::C0::DMA
    HAL::STM32::C0::RCC
    HAL::STM32::C0::RCCEx
    HAL::STM32::C0::FLASH
    HAL::STM32::C0::PWR
    HAL::STM32::C0::PWREx
    HAL::STM32::C0::CORTEX
    HAL::STM32::C0::I2S
    HAL::STM32::C0::TIM
    HAL::STM32::C0::TIMEx
    HAL::STM32::C0::ADC
    HAL::STM32::C0::ADCEx
    CMSIS::STM32::C071FB
    HAL::STM32::C0
    STM32::NoSys
    tinyusb_upstream
)
stm32_print_size_of_target(C0_CMSIS_DAP)

