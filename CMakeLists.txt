cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_PROJECT_NAME general_framework)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

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

# ---------------------------------------------------------------------------
# STM32CubeMX generated target support
# ---------------------------------------------------------------------------

set(stm32_definitions
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(stm32_include_directories
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)

set(stm32_application_sources
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    startup_stm32h723xx.s
)

set(stm32_driver_sources
    Core/Src/system_stm32h7xx.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
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
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
)

add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${stm32_include_directories})
target_compile_definitions(stm32cubemx INTERFACE ${stm32_definitions})

add_library(stm32_drivers OBJECT ${stm32_driver_sources})
target_link_libraries(stm32_drivers PUBLIC stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${stm32_application_sources})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES "${CMAKE_PROJECT_NAME}.map"
)

# ---------------------------------------------------------------------------
# Algorithm layer
# ---------------------------------------------------------------------------

add_library(alg_math STATIC
    User/Algorithm/alg_math/alg_math_scalar.c
    User/Algorithm/alg_math/alg_math_statistics.c
    User/Algorithm/alg_math/alg_math_vector.c
    User/Algorithm/alg_math/alg_math_quaternion.c
    User/Algorithm/alg_math/alg_math_matrix.c
)
target_include_directories(alg_math PUBLIC User/Algorithm/alg_math)
target_link_libraries(alg_math PUBLIC m)
framework_configure_library(alg_math)

add_library(alg_motion STATIC User/Algorithm/alg_motion/alg_motion.c)
target_include_directories(alg_motion PUBLIC User/Algorithm/alg_motion)
target_link_libraries(alg_motion PUBLIC m)
framework_configure_library(alg_motion)

add_library(alg_trajectory STATIC User/Algorithm/alg_trajectory/alg_trajectory.c)
target_include_directories(alg_trajectory PUBLIC User/Algorithm/alg_trajectory)
framework_configure_library(alg_trajectory)

add_library(alg_filter STATIC
    User/Algorithm/alg_filter/alg_filter_basic.c
    User/Algorithm/alg_filter/alg_filter_window.c
    User/Algorithm/alg_filter/alg_filter_fir.c
    User/Algorithm/alg_filter/alg_filter_biquad.c
    User/Algorithm/alg_filter/alg_filter_complementary.c
)
target_include_directories(alg_filter PUBLIC User/Algorithm/alg_filter)
target_link_libraries(alg_filter PUBLIC m)
framework_configure_library(alg_filter)

add_library(alg_kalman STATIC
    User/Algorithm/alg_kalman/alg_kalman_matrix.c
    User/Algorithm/alg_kalman/alg_kalman_scalar.c
    User/Algorithm/alg_kalman/alg_kalman_linear.c
    User/Algorithm/alg_kalman/alg_kalman_extended.c
)
target_include_directories(alg_kalman PUBLIC User/Algorithm/alg_kalman)
target_link_libraries(alg_kalman PUBLIC m)
framework_configure_library(alg_kalman)

add_library(alg_imu_ekf STATIC
    User/Algorithm/alg_imu_ekf/alg_imu_ekf_core.c
    User/Algorithm/alg_imu_ekf/alg_imu_ekf_model.c
    User/Algorithm/alg_imu_ekf/alg_imu_ekf_update.c
    User/Algorithm/alg_imu_ekf/alg_imu_ekf_output.c
)
target_include_directories(alg_imu_ekf PUBLIC User/Algorithm/alg_imu_ekf)
target_link_libraries(alg_imu_ekf PUBLIC alg_filter alg_kalman m)
framework_configure_library(alg_imu_ekf)

add_library(alg_pid STATIC
    User/Algorithm/alg_pid/alg_pid_core.c
    User/Algorithm/alg_pid/alg_pid_incremental.c
    User/Algorithm/alg_pid/alg_pid_schedule.c
    User/Algorithm/alg_pid/alg_pid_fuzzy.c
    User/Algorithm/alg_pid/alg_pid_cascade.c
)
target_include_directories(alg_pid PUBLIC User/Algorithm/alg_pid)
target_link_libraries(alg_pid PUBLIC m)
framework_configure_library(alg_pid)

add_library(alg_lqr STATIC
    User/Algorithm/alg_lqr/alg_lqr_matrix.c
    User/Algorithm/alg_lqr/alg_lqr_riccati.c
    User/Algorithm/alg_lqr/alg_lqr_controller.c
    User/Algorithm/alg_lqr/alg_lqr_model.c
)
target_include_directories(alg_lqr PUBLIC User/Algorithm/alg_lqr)
target_link_libraries(alg_lqr PUBLIC m)
framework_configure_library(alg_lqr)

add_library(alg_axis_controller STATIC
    User/Algorithm/alg_axis_controller/alg_axis_controller.c
)
target_include_directories(alg_axis_controller PUBLIC User/Algorithm/alg_axis_controller)
target_link_libraries(alg_axis_controller PUBLIC alg_pid alg_lqr)
framework_configure_library(alg_axis_controller)

add_library(alg_chassis STATIC User/Algorithm/alg_chassis/alg_chassis.c)
target_include_directories(alg_chassis PUBLIC User/Algorithm/alg_chassis)
framework_configure_library(alg_chassis)

add_library(alg_chassis_fault STATIC
    User/Algorithm/alg_chassis_fault/alg_chassis_fault.c
)
target_include_directories(alg_chassis_fault PUBLIC User/Algorithm/alg_chassis_fault)
target_link_libraries(alg_chassis_fault PUBLIC alg_chassis)
framework_configure_library(alg_chassis_fault)

foreach(chassis_algorithm IN ITEMS ackermann differential mecanum omni swerve)
    add_library(alg_${chassis_algorithm} STATIC
        User/Algorithm/alg_${chassis_algorithm}/alg_${chassis_algorithm}.c
    )
    target_include_directories(alg_${chassis_algorithm}
        PUBLIC User/Algorithm/alg_${chassis_algorithm}
    )
    target_link_libraries(alg_${chassis_algorithm} PUBLIC alg_chassis)
    framework_configure_library(alg_${chassis_algorithm})
endforeach()

# ---------------------------------------------------------------------------
# BSP layer
# ---------------------------------------------------------------------------

set(bsp_include_directories
    User/Bsp
    User/Bsp/bsp_common
    User/Bsp/bsp_gpio
    User/Bsp/bsp_exti
    User/Bsp/bsp_usart
    User/Bsp/bsp_spi
    User/Bsp/bsp_i2c
    User/Bsp/bsp_can
    User/Bsp/bsp_fdcan
    User/Bsp/bsp_timer
    User/Bsp/bsp_pwm
    User/Bsp/bsp_encoder
    User/Bsp/bsp_adc
    User/Bsp/bsp_dac
    User/Bsp/bsp_usb_vcp
    User/Bsp/bsp_watchdog
    User/Bsp/bsp_timebase
)

add_library(bsp STATIC
    User/Bsp/bsp_common/bsp_common.c
    User/Bsp/bsp_gpio/bsp_gpio.c
    User/Bsp/bsp_exti/bsp_exti.c
    User/Bsp/bsp_usart/bsp_usart.c
    User/Bsp/bsp_spi/bsp_spi.c
    User/Bsp/bsp_i2c/bsp_i2c.c
    User/Bsp/bsp_can/bsp_can.c
    User/Bsp/bsp_can/bsp_can_dispatcher.c
    User/Bsp/bsp_fdcan/bsp_fdcan.c
    User/Bsp/bsp_fdcan/bsp_fdcan_classic_adapter.c
    User/Bsp/bsp_timer/bsp_timer.c
    User/Bsp/bsp_pwm/bsp_pwm.c
    User/Bsp/bsp_encoder/bsp_encoder.c
    User/Bsp/bsp_adc/bsp_adc.c
    User/Bsp/bsp_dac/bsp_dac.c
    User/Bsp/bsp_usb_vcp/bsp_usb_vcp.c
    User/Bsp/bsp_watchdog/bsp_watchdog.c
    User/Bsp/bsp_timebase/bsp_timebase.c
)
target_include_directories(bsp PUBLIC ${bsp_include_directories})
framework_configure_library(bsp)

# ---------------------------------------------------------------------------
# Module layer
# ---------------------------------------------------------------------------

set(module_include_directories
    User/Module/module_device
    User/Module/module_motor
    User/Module/module_motor_health
    User/Module/module_dji_motor
    User/Module/module_m2006
    User/Module/module_m3508
    User/Module/module_gm6020
    User/Module/module_dm_motor
    User/Module/module_dm4310
    User/Module/module_bmi088
    User/Module/module_dr16
    User/Module/module_swerve
    User/Module/module_robot_link
    User/Module/module_vision
    User/Module/module_shooter
    User/Module/module_buzzer
    User/Module/module_bluetooth
    User/Module/module_oled
    User/Module/module_ws2812
    User/Module/module_nrf24l01
    User/Module/module_referee
    User/Module/module_servo
)

add_library(module STATIC
    User/Module/module_device/module_device.c
    User/Module/module_motor/module_motor.c
    User/Module/module_motor_health/module_motor_health.c
    User/Module/module_dji_motor/module_dji_motor.c
    User/Module/module_m2006/module_m2006.c
    User/Module/module_m3508/module_m3508.c
    User/Module/module_gm6020/module_gm6020.c
    User/Module/module_dm_motor/module_dm_motor.c
    User/Module/module_dm4310/module_dm4310.c
    User/Module/module_bmi088/module_bmi088.c
    User/Module/module_dr16/module_dr16.c
    User/Module/module_swerve/module_swerve.c
    User/Module/module_robot_link/module_robot_link.c
    User/Module/module_vision/module_vision.c
    User/Module/module_shooter/module_shooter.c
    User/Module/module_buzzer/module_buzzer.c
    User/Module/module_bluetooth/module_bluetooth.c
    User/Module/module_oled/module_oled.c
    User/Module/module_ws2812/module_ws2812.c
    User/Module/module_nrf24l01/module_nrf24l01.c
    User/Module/module_referee/module_referee.c
    User/Module/module_referee/module_referee_crc.c
    User/Module/module_servo/module_servo.c
)
target_include_directories(module PUBLIC ${module_include_directories})
target_link_libraries(module PUBLIC bsp alg_pid alg_swerve)
framework_configure_library(module)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    stm32_drivers
    bsp
    alg_math
    alg_motion
    alg_trajectory
    alg_filter
    alg_kalman
    alg_imu_ekf
    alg_pid
    alg_lqr
    alg_axis_controller
    alg_chassis
    alg_chassis_fault
    alg_ackermann
    alg_differential
    alg_mecanum
    alg_omni
    alg_swerve
    module
    ${TOOLCHAIN_LINK_LIBRARIES}
)
