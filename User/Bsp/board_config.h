#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * This is the only board-level configuration header used by BSP adapters.
 * Keep logical device names, GPIO pin mappings, IRQ selections and adapter
 * capacities here. Generic bsp_*.h headers must never include this file.
 */

#define BOARD_CONFIG_BSP_DEFAULT_TIMEOUT_MS (100U)
#define BOARD_CONFIG_BSP_EXTI_MAX_INSTANCES (16U)
#define BOARD_CONFIG_BSP_USART_MAX_INSTANCES (8U)
#define BOARD_CONFIG_BSP_SPI_MAX_INSTANCES (6U)
#define BOARD_CONFIG_BSP_I2C_MAX_INSTANCES (4U)
#define BOARD_CONFIG_BSP_FDCAN_MAX_INSTANCES (3U)
#define BOARD_CONFIG_BSP_TIMER_MAX_INSTANCES (16U)
#define BOARD_CONFIG_BSP_PWM_MAX_INSTANCES (16U)
#define BOARD_CONFIG_BSP_ENCODER_MAX_INSTANCES (8U)
#define BOARD_CONFIG_BSP_ADC_MAX_INSTANCES (16U)
#define BOARD_CONFIG_BSP_DAC_MAX_INSTANCES (4U)
#define BOARD_CONFIG_BSP_USB_VCP_MAX_INSTANCES (2U)

/* No application pin is currently enabled in general_framework.ioc. */
#define BOARD_CONFIG_EXTI_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_USART_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_SPI_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_I2C_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_FDCAN_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_TIMER_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_PWM_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_ENCODER_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_ADC_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_DAC_INSTANCE_COUNT (0U)
#define BOARD_CONFIG_USB_VCP_INSTANCE_COUNT (0U)

/*
 * Add project mappings only in this section after enabling them in CubeMX.
 * Example naming convention (do not create unused mappings):
 *
 * #define BOARD_CONFIG_IMU_INT_GPIO_PORT          GPIOC
 * #define BOARD_CONFIG_IMU_INT_GPIO_PIN           GPIO_PIN_4
 * #define BOARD_CONFIG_IMU_INT_IRQ_NUMBER         EXTI4_IRQn
 * #define BOARD_CONFIG_IMU_SPI_DEVICE_HANDLE      (&platform_spi_1)
 * #define BOARD_CONFIG_DEBUG_USART_DEVICE_HANDLE  (&platform_usart_3)
 * #define BOARD_CONFIG_CONTROL_FDCAN_DEVICE_HANDLE (&platform_fdcan_1)
 * #define BOARD_CONFIG_LEFT_MOTOR_PWM_CHANNEL     (1U)
 * #define BOARD_CONFIG_BATTERY_ADC_CHANNEL        (5U)
 *
 * Vendor types and symbols belong to the platform port or board assembly
 * source file. This header intentionally stays independent of every HAL.
 */

#endif /* BOARD_CONFIG_H */
