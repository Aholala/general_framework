cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set the project name
set(CMAKE_PROJECT_NAME general_framework)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(MODULE_USB_COMM_EXTRA_DATA_SIZE 0 CACHE STRING
    "USB vision protocol optional extra data bytes (0..250)")
if(NOT MODULE_USB_COMM_EXTRA_DATA_SIZE MATCHES "^[0-9]+$" OR
   MODULE_USB_COMM_EXTRA_DATA_SIZE GREATER 250)
    message(FATAL_ERROR "MODULE_USB_COMM_EXTRA_DATA_SIZE must be in range 0..250")
endif()

set(MODULE_UART_COMM_DATA_SIZE 8 CACHE STRING
    "UART fixed protocol data bytes (1..252)")
if(NOT MODULE_UART_COMM_DATA_SIZE MATCHES "^[1-9][0-9]*$" OR
   MODULE_UART_COMM_DATA_SIZE GREATER 252)
    message(FATAL_ERROR "MODULE_UART_COMM_DATA_SIZE must be in range 1..252")
endif()

add_executable(${CMAKE_PROJECT_NAME})

function(framework_configure_library target_name)
    target_compile_features(${target_name} PUBLIC c_std_11)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target_name} PRIVATE
            -Wall
            -Wextra
            -Wshadow
            -Wconversion
            -Wdouble-promotion
            -Wundef
            -Werror=return-type
        )
    endif()
endfunction()

# ===========================================================================
# STM32CubeMX generated target support (DO NOT MODIFY THESE PATHS)
# ===========================================================================

set(stm32_definitions
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    HSE_VALUE=24000000UL
    $<$<CONFIG:Debug>:DEBUG>
)

set(stm32_include_directories
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT
)

set(stm32_application_sources
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/freertos.c
    Core/Src/spi.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/Target/usbd_conf.c
    startup_stm32h723xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
)

set(stm32_driver_sources
    Core/Src/system_stm32h7xx.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
)

set(freertos_sources
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

set(usb_device_sources
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${stm32_include_directories})
target_compile_definitions(stm32cubemx INTERFACE ${stm32_definitions})

add_library(stm32_drivers OBJECT ${stm32_driver_sources})
target_link_libraries(stm32_drivers PUBLIC stm32cubemx)

add_library(freertos OBJECT ${freertos_sources})
target_link_libraries(freertos PUBLIC stm32cubemx)

add_library(stm32_usb_device OBJECT ${usb_device_sources})
target_link_libraries(stm32_usb_device PUBLIC stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${stm32_application_sources})

# ===========================================================================
# ECF Framework — Algorithm layer
# ===========================================================================

add_library(alg_crc OBJECT ECF/Algorithm/alg_crc/alg_crc.c)
target_include_directories(alg_crc PUBLIC ECF/Algorithm/alg_crc)
framework_configure_library(alg_crc)

add_library(alg_math OBJECT
    ECF/Algorithm/alg_math/alg_math_scalar.c
    ECF/Algorithm/alg_math/alg_math_statistics.c
    ECF/Algorithm/alg_math/alg_math_vector.c
    ECF/Algorithm/alg_math/alg_math_quaternion.c
    ECF/Algorithm/alg_math/alg_math_matrix.c
)
target_include_directories(alg_math PUBLIC ECF/Algorithm/alg_math)
target_link_libraries(alg_math PUBLIC m)
framework_configure_library(alg_math)

add_library(alg_trajectory OBJECT
    ECF/Algorithm/alg_trajectory/alg_trajectory.c
    ECF/Algorithm/alg_trajectory/alg_trajectory_group.c
)
target_include_directories(alg_trajectory PUBLIC ECF/Algorithm/alg_trajectory)
framework_configure_library(alg_trajectory)

add_library(alg_attitude OBJECT
    ECF/Algorithm/alg_attitude/alg_attitude_core.c
    ECF/Algorithm/alg_attitude/alg_attitude_mahony.c
    ECF/Algorithm/alg_attitude/alg_attitude_madgwick.c
    ECF/Algorithm/alg_attitude/alg_attitude_output.c
)
target_include_directories(alg_attitude PUBLIC ECF/Algorithm/alg_attitude)
target_link_libraries(alg_attitude PUBLIC m)
framework_configure_library(alg_attitude)

add_library(alg_filter OBJECT
    ECF/Algorithm/alg_filter/alg_filter_basic.c
    ECF/Algorithm/alg_filter/alg_filter_window.c
    ECF/Algorithm/alg_filter/alg_filter_fir.c
    ECF/Algorithm/alg_filter/alg_filter_biquad.c
    ECF/Algorithm/alg_filter/alg_filter_complementary.c
)
target_include_directories(alg_filter PUBLIC ECF/Algorithm/alg_filter)
target_link_libraries(alg_filter PUBLIC m)
framework_configure_library(alg_filter)

add_library(alg_kalman OBJECT
    ECF/Algorithm/alg_kalman/alg_kalman_matrix.c
    ECF/Algorithm/alg_kalman/alg_kalman_scalar.c
    ECF/Algorithm/alg_kalman/alg_kalman_linear.c
    ECF/Algorithm/alg_kalman/alg_kalman_extended.c
)
target_include_directories(alg_kalman PUBLIC ECF/Algorithm/alg_kalman)
target_link_libraries(alg_kalman PUBLIC m)
framework_configure_library(alg_kalman)

add_library(alg_imu_ekf OBJECT
    ECF/Algorithm/alg_imu_ekf/alg_imu_ekf_core.c
    ECF/Algorithm/alg_imu_ekf/alg_imu_ekf_model.c
    ECF/Algorithm/alg_imu_ekf/alg_imu_ekf_update.c
    ECF/Algorithm/alg_imu_ekf/alg_imu_ekf_output.c
)
target_include_directories(alg_imu_ekf PUBLIC ECF/Algorithm/alg_imu_ekf)
target_link_libraries(alg_imu_ekf PUBLIC alg_filter alg_kalman m)
framework_configure_library(alg_imu_ekf)

add_library(alg_pid OBJECT
    ECF/Algorithm/alg_pid/alg_pid_core.c
    ECF/Algorithm/alg_pid/alg_pid_incremental.c
    ECF/Algorithm/alg_pid/alg_pid_schedule.c
    ECF/Algorithm/alg_pid/alg_pid_fuzzy.c
    ECF/Algorithm/alg_pid/alg_pid_cascade.c
    ECF/Algorithm/alg_pid/alg_pid_angle.c
)
target_include_directories(alg_pid PUBLIC ECF/Algorithm/alg_pid)
target_link_libraries(alg_pid PUBLIC m)
framework_configure_library(alg_pid)

add_library(alg_lqr OBJECT
    ECF/Algorithm/alg_lqr/alg_lqr_matrix.c
    ECF/Algorithm/alg_lqr/alg_lqr_riccati.c
    ECF/Algorithm/alg_lqr/alg_lqr_controller.c
    ECF/Algorithm/alg_lqr/alg_lqr_model.c
    ECF/Algorithm/alg_lqr/alg_lqr_angle.c
)
target_include_directories(alg_lqr PUBLIC ECF/Algorithm/alg_lqr)
target_link_libraries(alg_lqr PUBLIC m)
framework_configure_library(alg_lqr)

add_library(alg_chassis_motion OBJECT ECF/Algorithm/alg_chassis/alg_chassis_motion.c)
target_include_directories(alg_chassis_motion PUBLIC ECF/Algorithm/alg_chassis)
framework_configure_library(alg_chassis_motion)

add_library(alg_chassis_wheel_monitor OBJECT
    ECF/Algorithm/alg_chassis/alg_chassis_wheel_monitor.c
)
target_include_directories(alg_chassis_wheel_monitor PUBLIC ECF/Algorithm/alg_chassis)
target_link_libraries(alg_chassis_wheel_monitor PUBLIC alg_chassis_motion)
framework_configure_library(alg_chassis_wheel_monitor)

foreach(chassis_algorithm IN ITEMS mecanum omni swerve)
    add_library(alg_${chassis_algorithm} OBJECT
        ECF/Algorithm/alg_${chassis_algorithm}/alg_${chassis_algorithm}.c
    )
    target_include_directories(alg_${chassis_algorithm}
        PUBLIC ECF/Algorithm/alg_${chassis_algorithm}
    )
    target_link_libraries(alg_${chassis_algorithm} PUBLIC alg_chassis_motion)
    framework_configure_library(alg_${chassis_algorithm})
endforeach()

# ===========================================================================
# ECF Framework — BSP layer
# ===========================================================================

set(bsp_include_directories
    Middlewares/Third_Party/SEGGER/Config
    Middlewares/Third_Party/SEGGER/RTT
    ECF/Bsp/bsp_common
    ECF/Bsp/bsp_gpio
    ECF/Bsp/bsp_exti
    ECF/Bsp/bsp_usart
    ECF/Bsp/bsp_spi
    ECF/Bsp/bsp_i2c
    ECF/Bsp/bsp_can
    ECF/Bsp/bsp_fdcan
    ECF/Bsp/bsp_timer
    ECF/Bsp/bsp_pwm
    ECF/Bsp/bsp_encoder
    ECF/Bsp/bsp_adc
    ECF/Bsp/bsp_usb_vcp
    ECF/Bsp/bsp_watchdog
    ECF/Bsp/bsp_dwt
    ECF/Bsp/bsp_rtc
    ECF/Bsp/bsp_log
)

add_library(bsp OBJECT
    ECF/Bsp/bsp_common/bsp_common.c
    ECF/Bsp/bsp_gpio/bsp_gpio.c
    ECF/Bsp/bsp_exti/bsp_exti.c
    ECF/Bsp/bsp_usart/bsp_usart.c
    ECF/Bsp/bsp_spi/bsp_spi.c
    ECF/Bsp/bsp_i2c/bsp_i2c.c
    ECF/Bsp/bsp_can/bsp_can.c
    ECF/Bsp/bsp_can/bsp_can_dispatcher.c
    ECF/Bsp/bsp_fdcan/bsp_fdcan.c
    ECF/Bsp/bsp_fdcan/bsp_fdcan_classic_adapter.c
    ECF/Bsp/bsp_timer/bsp_timer.c
    ECF/Bsp/bsp_pwm/bsp_pwm.c
    ECF/Bsp/bsp_encoder/bsp_encoder.c
    ECF/Bsp/bsp_adc/bsp_adc.c
    ECF/Bsp/bsp_usb_vcp/bsp_usb_vcp.c
    ECF/Bsp/bsp_watchdog/bsp_watchdog.c
    ECF/Bsp/bsp_dwt/bsp_dwt.c
    ECF/Bsp/bsp_rtc/bsp_rtc.c
    ECF/Bsp/bsp_log/bsp_log.c
)
target_include_directories(bsp PUBLIC ${bsp_include_directories})
framework_configure_library(bsp)

# ===========================================================================
# Board Configuration (project-specific — at App/config/)
# ===========================================================================
# NOTE: board_config.c/.h is project-specific hardware glue.
# It lives at App/config/board_config.c and is built here.
# ===========================================================================

add_library(board_config OBJECT
    App/config/board_config.c
)
target_include_directories(board_config
    PUBLIC App/config
    PRIVATE ${bsp_include_directories}
)
target_include_directories(board_config SYSTEM PRIVATE ${stm32_include_directories})
target_compile_definitions(board_config PRIVATE ${stm32_definitions})
target_link_libraries(board_config PUBLIC bsp stm32cubemx)
framework_configure_library(board_config)

# ===========================================================================
# ECF Framework — Module layer
# ===========================================================================

set(module_include_directories
    ECF/Module/module_device
    ECF/Module/module_motor
    ECF/Module/module_dji_motor
    ECF/Module/module_dm_motor
    ECF/Module/module_bmi088
    ECF/Module/module_dr16
    ECF/Module/module_swerve
    ECF/Module/module_board_comm
    ECF/Module/module_uart_comm
    ECF/Module/module_usb_comm
    ECF/Module/module_shooter
    ECF/Module/module_buzzer
    ECF/Module/module_oled
    ECF/Module/module_ws2812
    ECF/Module/module_nrf24l01
    ECF/Module/module_referee
    ECF/Module/module_servo
)

add_library(module OBJECT
    ECF/Module/module_device/module_device.c
    ECF/Module/module_motor/module_motor.c
    ECF/Module/module_motor/module_motor_health.c
    ECF/Module/module_dji_motor/module_dji_motor.c
    ECF/Module/module_dji_motor/module_m2006.c
    ECF/Module/module_dji_motor/module_m3508.c
    ECF/Module/module_dji_motor/module_gm6020.c
    ECF/Module/module_dm_motor/module_dm_motor.c
    ECF/Module/module_dm_motor/module_dm_motor_bus.c
    ECF/Module/module_dm_motor/module_dm4310.c
    ECF/Module/module_bmi088/module_bmi088.c
    ECF/Module/module_dr16/module_dr16.c
    ECF/Module/module_swerve/module_swerve.c
    ECF/Module/module_board_comm/module_board_comm.c
    ECF/Module/module_uart_comm/module_uart_comm.c
    ECF/Module/module_usb_comm/module_usb_comm.c
    ECF/Module/module_shooter/module_shooter.c
    ECF/Module/module_buzzer/module_buzzer.c
    ECF/Module/module_oled/module_oled.c
    ECF/Module/module_ws2812/module_ws2812.c
    ECF/Module/module_nrf24l01/module_nrf24l01.c
    ECF/Module/module_nrf24l01/module_nrf24l01_link.c
    ECF/Module/module_servo/module_servo.c
)
target_include_directories(module PUBLIC ${module_include_directories})
target_compile_definitions(module PUBLIC
    MODULE_USB_COMM_EXTRA_DATA_SIZE=${MODULE_USB_COMM_EXTRA_DATA_SIZE}U
    MODULE_UART_COMM_DATA_SIZE=${MODULE_UART_COMM_DATA_SIZE}U
)
target_link_libraries(module PUBLIC bsp alg_crc alg_pid alg_swerve)
framework_configure_library(module)

# ===========================================================================
# ECF Framework — App layer (generic app logic, NOT project-specific)
# ===========================================================================

add_library(app OBJECT
    ECF/App/app_exchange/app_exchange.c
    ECF/App/app_vision/app_vision.c
    ECF/App/app_command/app_command.c
    ECF/App/app_chassis/app_chassis.c
    ECF/App/app_gimbal/app_gimbal.c
    ECF/App/app_imu/app_imu.c
    ECF/App/app_shooter/app_shooter.c
    ECF/App/app_safety/app_safety.c
)
target_include_directories(app PUBLIC
    ECF/App
    ECF/App/app_exchange
    ECF/App/app_vision
    ECF/App/app_command
    ECF/App/app_chassis
    ECF/App/app_gimbal
    ECF/App/app_imu
    ECF/App/app_shooter
    ECF/App/app_safety
)
target_link_libraries(app PUBLIC bsp module alg_swerve stm32cubemx)
framework_configure_library(app)

# ===========================================================================
# Project App — Task layer (project-specific task wrappers)
# ===========================================================================

add_library(task OBJECT
    App/app_robot.c
    App/task/task_chassis/task_chassis.c
    App/task/task_command/task_command.c
    App/task/task_gimbal/task_gimbal.c
    App/task/task_imu/task_imu.c
    App/task/task_safety/task_safety.c
    App/task/task_shooter/task_shooter.c
)
target_include_directories(task PUBLIC
    App/task/task_chassis
    App/task/task_command
    App/task/task_gimbal
    App/task/task_imu
    App/task/task_safety
    App/task/task_shooter
    App/config
)
target_link_libraries(task PUBLIC app board_config freertos)
framework_configure_library(task)

# ===========================================================================
# Final executable link
# ===========================================================================

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    stm32_drivers
    freertos
    stm32_usb_device
    bsp
    board_config
    alg_crc
    alg_math
    alg_trajectory
    alg_attitude
    alg_filter
    alg_kalman
    alg_imu_ekf
    alg_pid
    alg_lqr
    alg_chassis_wheel_monitor
    alg_mecanum
    alg_omni
    alg_swerve
    module
    app
    task
    ${TOOLCHAIN_LINK_LIBRARIES}
)
