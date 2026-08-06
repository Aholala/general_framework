#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_exti.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_usart.h"
#include "bsp_usb_vcp.h"
#include "bsp_watchdog.h"

/*
 * STM32H723VET6 board-level resource map.
 *
 * 引脚、DMA、中断由 CubeMX 生成的 Core/ 文件管理，此文件只定义逻辑外设
 * 索引、驱动操作表和时钟常量。换 MCU 时只需替换本文件和 Core/。
 */

/* ------------- 板级时钟频率（基于 24MHz HSE 晶振） ------------- */
#define BOARD_CONFIG_HSE_FREQUENCY_HZ (24000000UL)           /* HSE 晶振频率 */
#define BOARD_CONFIG_CPU_FREQUENCY_HZ (480000000UL)          /* CPU 主频 480MHz */
#define BOARD_CONFIG_AHB_FREQUENCY_HZ (240000000UL)          /* AHB 总线频率 */
#define BOARD_CONFIG_APB_FREQUENCY_HZ (120000000UL)          /* APB 总线频率（用于外设时钟） */
#define BOARD_CONFIG_FDCAN_KERNEL_FREQUENCY_HZ (120000000UL) /* FDCAN 内核时钟 */
#define BOARD_CONFIG_USB_KERNEL_FREQUENCY_HZ (48000000UL)    /* USB 内核时钟 */

/* ------------- BSP 通用容量限制（用于静态分配检查） ------------- */
#define BOARD_CONFIG_BSP_DEFAULT_TIMEOUT_MS (100U)  /* 默认超时 100ms */
#define BOARD_CONFIG_BSP_EXTI_MAX_INSTANCES (16U)   /* 最大 EXTI 实例数 */
#define BOARD_CONFIG_BSP_USART_MAX_INSTANCES (8U)   /* 最大 USART 实例数 */
#define BOARD_CONFIG_BSP_SPI_MAX_INSTANCES (6U)     /* 最大 SPI 实例数 */
#define BOARD_CONFIG_BSP_I2C_MAX_INSTANCES (4U)     /* 最大 I2C 实例数 */
#define BOARD_CONFIG_BSP_FDCAN_MAX_INSTANCES (3U)   /* 最大 FDCAN 实例数 */
#define BOARD_CONFIG_BSP_TIMER_MAX_INSTANCES (16U)  /* 最大 TIMER 实例数 */
#define BOARD_CONFIG_BSP_PWM_MAX_INSTANCES (16U)    /* 最大 PWM 实例数 */
#define BOARD_CONFIG_BSP_ENCODER_MAX_INSTANCES (8U) /* 最大 Encoder 实例数 */
#define BOARD_CONFIG_BSP_ADC_MAX_INSTANCES (16U)    /* 最大 ADC 实例数 */
#define BOARD_CONFIG_BSP_DAC_MAX_INSTANCES (4U)     /* 最大 DAC 实例数 */
#define BOARD_CONFIG_BSP_USB_VCP_MAX_INSTANCES (2U) /* 最大 USB VCP 实例数 */

/* ------------- 本板实际启用的外设实例数量 ------------- */
#define BOARD_CONFIG_EXTI_INSTANCE_COUNT (2U)    /* 2 个 EXTI（BMI088 中断） */
#define BOARD_CONFIG_USART_INSTANCE_COUNT (1U)   /* DR16 UART5 */
#define BOARD_CONFIG_SPI_INSTANCE_COUNT (1U)     /* SPI2（BMI088） */
#define BOARD_CONFIG_I2C_INSTANCE_COUNT (0U)     /* 未启用 I2C */
#define BOARD_CONFIG_FDCAN_INSTANCE_COUNT (3U)   /* FDCAN1、FDCAN2、FDCAN3 */
#define BOARD_CONFIG_TIMER_INSTANCE_COUNT (1U)   /* TIM1（蜂鸣器 PWM） */
#define BOARD_CONFIG_PWM_INSTANCE_COUNT (1U)     /* TIM1_CH1 蜂鸣器 */
#define BOARD_CONFIG_ENCODER_INSTANCE_COUNT (0U) /* 未启用编码器 */
#define BOARD_CONFIG_ADC_INSTANCE_COUNT (0U)     /* 未启用 ADC */
#define BOARD_CONFIG_DAC_INSTANCE_COUNT (0U)     /* 未启用 DAC */
#define BOARD_CONFIG_USB_VCP_INSTANCE_COUNT (1U) /* USB VCP 一个 */

/*
 * 引脚、DMA 通道、NVIC 优先级由 CubeMX 生成的 Core/Src/fdcan.c / usart.c /
 * spi.c / tim.c 管理。本文件只定义逻辑外设索引、时钟频率和容量上限。
 */

/* ------------- FDCAN 消息 RAM 分配（与 CubeMX .ioc 保持一致） ------------- */
#define BOARD_CONFIG_FDCAN_MESSAGE_RAM_WORD_COUNT (800U)
#define BOARD_CONFIG_CAN1_MESSAGE_RAM_OFFSET_WORDS (0U)
#define BOARD_CONFIG_CAN2_MESSAGE_RAM_OFFSET_WORDS (800U)
#define BOARD_CONFIG_CAN3_MESSAGE_RAM_OFFSET_WORDS (1600U)

/* ------------- 编译期容量检查（防止配置超出 BSP 上限） ------------- */
#if BOARD_CONFIG_EXTI_INSTANCE_COUNT > BOARD_CONFIG_BSP_EXTI_MAX_INSTANCES
#error "EXTI instance count exceeds the BSP capacity"
#endif
#if BOARD_CONFIG_USART_INSTANCE_COUNT > BOARD_CONFIG_BSP_USART_MAX_INSTANCES
#error "USART instance count exceeds the BSP capacity"
#endif
#if BOARD_CONFIG_SPI_INSTANCE_COUNT > BOARD_CONFIG_BSP_SPI_MAX_INSTANCES
#error "SPI instance count exceeds the BSP capacity"
#endif
#if BOARD_CONFIG_FDCAN_INSTANCE_COUNT > BOARD_CONFIG_BSP_FDCAN_MAX_INSTANCES
#error "FDCAN instance count exceeds the BSP capacity"
#endif
#if BOARD_CONFIG_PWM_INSTANCE_COUNT > BOARD_CONFIG_BSP_PWM_MAX_INSTANCES
#error "PWM instance count exceeds the BSP capacity"
#endif

typedef enum
{
    BOARD_CONFIG_CAN_1 = 0,
    BOARD_CONFIG_CAN_2,
    BOARD_CONFIG_CAN_3,
    BOARD_CONFIG_CAN_COUNT
} board_config_can_index_t;

typedef enum
{
    BOARD_CONFIG_UART_DR16 = 0,
    BOARD_CONFIG_USART_COUNT
} board_config_usart_index_t;

typedef enum
{
    BOARD_CONFIG_EXTI_BMI088_GYROSCOPE = 0,
    BOARD_CONFIG_EXTI_BMI088_ACCELEROMETER,
    BOARD_CONFIG_EXTI_COUNT
} board_config_exti_index_t;

typedef enum
{
    BOARD_CONFIG_PWM_BUZZER = 0,
    /* 预留舵机/其他 PWM 外设槽位（取消注释并填入实际定时器即可启用） */
    /* BOARD_CONFIG_PWM_SERVO_1, */
    /* BOARD_CONFIG_PWM_SERVO_2, */
    BOARD_CONFIG_PWM_COUNT
} board_config_pwm_index_t;

typedef struct
{
    bool initialize_watchdog;
    /** @brief 初始化失败时由 board_config_init 填充的具体错误码 */
    bsp_status_t last_error;
    /** @brief 初始化失败时填充的失败步骤名称（如 "can"/"usart"/"spi"） */
    const char *failed_step;
} board_config_init_t;

bsp_status_t board_config_init(board_config_init_t *config);
bsp_can_t *board_config_get_can(board_config_can_index_t index);
bsp_usart_t *board_config_get_usart(board_config_usart_index_t index);
bsp_spi_t *board_config_get_bmi088_spi(void);
bsp_exti_t *board_config_get_exti(board_config_exti_index_t index);
bsp_pwm_t *board_config_get_pwm(board_config_pwm_index_t index);
bsp_usb_vcp_t *board_config_get_usb_vcp(void);
bsp_watchdog_t *board_config_get_watchdog(void);
bsp_dwt_t *board_config_get_dwt(void);
bool board_config_is_initialized(void);

#endif /* BOARD_CONFIG_H */
