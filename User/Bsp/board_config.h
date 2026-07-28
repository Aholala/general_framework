#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * STM32H723VET6 board-level resource map.
 *
 * This file describes logical devices, peripheral instances, pins and
 * alternate functions without including a vendor HAL header. The STM32
 * platform port is responsible for translating these constants to GPIO_TypeDef
 * pointers, HAL handles, IRQ numbers and DMA channels.
 *
 * A pin is encoded as (port_index << 4) | pin_number. Use the access macros
 * below instead of relying on the encoding outside the board assembly code.
 */

/* ------------- GPIO 端口索引宏 ------------- */
#define BOARD_CONFIG_GPIO_PORT_A (0U)
#define BOARD_CONFIG_GPIO_PORT_B (1U)
#define BOARD_CONFIG_GPIO_PORT_C (2U)
#define BOARD_CONFIG_GPIO_PORT_D (3U)
#define BOARD_CONFIG_GPIO_PORT_E (4U)
#define BOARD_CONFIG_GPIO_PORT_H (7U)

/* ------------- GPIO 引脚编码宏 ------------- */
/* 将端口索引和引脚号编码为一个字节：高4位为端口索引，低4位为引脚号 */
#define BOARD_CONFIG_GPIO_PIN(port_index, pin_number)                                              \
    ((((port_index) & 0x0FU) << 4U) | ((pin_number) & 0x0FU))

/* 从编码值中提取端口索引 */
#define BOARD_CONFIG_GPIO_PIN_PORT(encoded_pin) (((encoded_pin) >> 4U) & 0x0FU)

/* 从编码值中提取引脚号 */
#define BOARD_CONFIG_GPIO_PIN_NUMBER(encoded_pin) ((encoded_pin) & 0x0FU)

/* 备用功能：NONE 表示不使用备用功能（作为普通 GPIO） */
#define BOARD_CONFIG_ALTERNATE_FUNCTION_NONE (0xFFU)

/* 引脚有效电平定义 */
#define BOARD_CONFIG_GPIO_ACTIVE_LOW (0U)
#define BOARD_CONFIG_GPIO_ACTIVE_HIGH (1U)

/* ------------- 板级时钟频率（基于 24MHz HSE 晶振） ------------- */
#define BOARD_CONFIG_HSE_FREQUENCY_HZ (24000000UL)          /* HSE 晶振频率 */
#define BOARD_CONFIG_CPU_FREQUENCY_HZ (550000000UL)         /* CPU 主频 550MHz */
#define BOARD_CONFIG_AHB_FREQUENCY_HZ (275000000UL)         /* AHB 总线频率 */
#define BOARD_CONFIG_APB_FREQUENCY_HZ (137500000UL)         /* APB 总线频率（用于外设时钟） */
#define BOARD_CONFIG_FDCAN_KERNEL_FREQUENCY_HZ (24000000UL) /* FDCAN 内核时钟 */
#define BOARD_CONFIG_USB_KERNEL_FREQUENCY_HZ (48000000UL)   /* USB 内核时钟 */

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
#define BOARD_CONFIG_USART_INSTANCE_COUNT (5U)   /* USART2、USART6、UART7、UART8、DR16 UART5 */
#define BOARD_CONFIG_SPI_INSTANCE_COUNT (1U)     /* SPI2（BMI088） */
#define BOARD_CONFIG_I2C_INSTANCE_COUNT (0U)     /* 未启用 I2C */
#define BOARD_CONFIG_FDCAN_INSTANCE_COUNT (3U)   /* FDCAN1、FDCAN2、FDCAN3 */
#define BOARD_CONFIG_TIMER_INSTANCE_COUNT (2U)   /* TIM3、TIM1（PWM） */
#define BOARD_CONFIG_PWM_INSTANCE_COUNT (5U)     /* TIM3 的 4 个通道 + TIM1 的 1 个通道（蜂鸣器） */
#define BOARD_CONFIG_ENCODER_INSTANCE_COUNT (0U) /* 未启用编码器 */
#define BOARD_CONFIG_ADC_INSTANCE_COUNT (0U)     /* 未启用 ADC */
#define BOARD_CONFIG_DAC_INSTANCE_COUNT (0U)     /* 未启用 DAC */
#define BOARD_CONFIG_USB_VCP_INSTANCE_COUNT (1U) /* USB VCP 一个 */

/* ------------------------------------------------------------------------- */
/* USART 和 UART 详细定义                                                    */
/* ------------------------------------------------------------------------- */

/* USART2：逻辑名称 "usart2"，实例号 2，TX PA2，RX PA3，备用功能 AF7 */
#define BOARD_CONFIG_USART2_LOGICAL_NAME "usart2"
#define BOARD_CONFIG_USART2_INSTANCE (2U)
#define BOARD_CONFIG_USART2_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 2U)
#define BOARD_CONFIG_USART2_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 3U)
#define BOARD_CONFIG_USART2_ALTERNATE_FUNCTION (7U)

/* USART6：TX PC6，RX PC7，AF7 */
#define BOARD_CONFIG_USART6_LOGICAL_NAME "usart6"
#define BOARD_CONFIG_USART6_INSTANCE (6U)
#define BOARD_CONFIG_USART6_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_C, 6U)
#define BOARD_CONFIG_USART6_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_C, 7U)
#define BOARD_CONFIG_USART6_ALTERNATE_FUNCTION (7U)

/* UART7（注意命名：STM32H7 中为 UART7，不是 USART7）：TX PB4，RX PB3，AF11 */
#define BOARD_CONFIG_UART7_LOGICAL_NAME "uart7"
#define BOARD_CONFIG_UART7_INSTANCE (7U)
#define BOARD_CONFIG_UART7_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 4U)
#define BOARD_CONFIG_UART7_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 3U)
#define BOARD_CONFIG_UART7_ALTERNATE_FUNCTION (11U)

/* UART8：TX PE1，RX PE0，AF8 */
#define BOARD_CONFIG_UART8_LOGICAL_NAME "uart8"
#define BOARD_CONFIG_UART8_INSTANCE (8U)
#define BOARD_CONFIG_UART8_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 1U)
#define BOARD_CONFIG_UART8_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 0U)
#define BOARD_CONFIG_UART8_ALTERNATE_FUNCTION (8U)

/* DR16 接收专用 UART5：仅 RX，PD2，AF8，波特率 100000，9 位字长，偶校验，2 停止位 */
#define BOARD_CONFIG_DR16_UART_LOGICAL_NAME "dr16_dbus"
#define BOARD_CONFIG_DR16_UART_INSTANCE (5U)
#define BOARD_CONFIG_DR16_UART_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 2U)
#define BOARD_CONFIG_DR16_UART_ALTERNATE_FUNCTION (8U)
#define BOARD_CONFIG_DR16_UART_RECEIVE_ONLY (1U)        /* 仅接收 */
#define BOARD_CONFIG_DR16_UART_BAUD_RATE (100000UL)     /* 100kbps */
#define BOARD_CONFIG_DR16_UART_WORD_LENGTH_BITS (9U)    /* 9 位数据 */
#define BOARD_CONFIG_DR16_UART_EVEN_PARITY_ENABLED (1U) /* 偶校验 */
#define BOARD_CONFIG_DR16_UART_STOP_BITS (2U)           /* 2 停止位 */

/* ------------------------------------------------------------------------- */
/* FDCAN                                                                     */
/* ------------------------------------------------------------------------- */

/* CAN1（逻辑名 "can1"）→ FDCAN1，RX PD0，TX PD1，AF9 */
#define BOARD_CONFIG_CAN1_LOGICAL_NAME "can1"
#define BOARD_CONFIG_CAN1_FDCAN_INSTANCE (1U)
#define BOARD_CONFIG_CAN1_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 0U)
#define BOARD_CONFIG_CAN1_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 1U)
#define BOARD_CONFIG_CAN1_ALTERNATE_FUNCTION (9U)

/* CAN2 → FDCAN2，RX PB5，TX PB6，AF9 */
#define BOARD_CONFIG_CAN2_LOGICAL_NAME "can2"
#define BOARD_CONFIG_CAN2_FDCAN_INSTANCE (2U)
#define BOARD_CONFIG_CAN2_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 5U)
#define BOARD_CONFIG_CAN2_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 6U)
#define BOARD_CONFIG_CAN2_ALTERNATE_FUNCTION (9U)

/* CAN3 → FDCAN3，RX PD12，TX PD13，AF5 */
#define BOARD_CONFIG_CAN3_LOGICAL_NAME "can3"
#define BOARD_CONFIG_CAN3_FDCAN_INSTANCE (3U)
#define BOARD_CONFIG_CAN3_RX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 12U)
#define BOARD_CONFIG_CAN3_TX_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 13U)
#define BOARD_CONFIG_CAN3_ALTERNATE_FUNCTION (5U)

/* FDCAN 全局配置：经典 CAN，1Mbit/s，预分频器、时间段等 */
#define BOARD_CONFIG_FDCAN_NOMINAL_BIT_RATE (1000000UL) /* 1 Mbps */
#define BOARD_CONFIG_FDCAN_NOMINAL_PRESCALER (3U)
#define BOARD_CONFIG_FDCAN_NOMINAL_TIME_SEGMENT_1 (5U)
#define BOARD_CONFIG_FDCAN_NOMINAL_TIME_SEGMENT_2 (2U)
#define BOARD_CONFIG_FDCAN_NOMINAL_SYNC_JUMP_WIDTH (2U)

/* 三个 FDCAN 共享消息 RAM，每个分配 259 个 32 位字 */
#define BOARD_CONFIG_FDCAN_MESSAGE_RAM_WORD_COUNT (259U)
#define BOARD_CONFIG_CAN1_MESSAGE_RAM_OFFSET_WORDS (0U)
#define BOARD_CONFIG_CAN2_MESSAGE_RAM_OFFSET_WORDS (259U)
#define BOARD_CONFIG_CAN3_MESSAGE_RAM_OFFSET_WORDS (518U)

/* ------------------------------------------------------------------------- */
/* USB OTG HS（使用内部全速 PHY）                                            */
/* ------------------------------------------------------------------------- */

#define BOARD_CONFIG_USB_VCP_LOGICAL_NAME "usb_vcp"
#define BOARD_CONFIG_USB_OTG_INSTANCE (2U) /* OTG_HS */
#define BOARD_CONFIG_USB_OTG_ID_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 10U)
#define BOARD_CONFIG_USB_OTG_DM_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 11U)
#define BOARD_CONFIG_USB_OTG_DP_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 12U)
#define BOARD_CONFIG_USB_OTG_ALTERNATE_FUNCTION (10U)
#define BOARD_CONFIG_USB_OTG_ID_CONNECTED (1U) /* ID 引脚接地 */

/* ------------------------------------------------------------------------- */
/* BMI088 惯性传感器：SPI2                                                  */
/* ------------------------------------------------------------------------- */

#define BOARD_CONFIG_BMI088_LOGICAL_NAME "bmi088"
#define BOARD_CONFIG_BMI088_SPI_INSTANCE (2U)                     /* SPI2 */
#define BOARD_CONFIG_BMI088_SPI_ALTERNATE_FUNCTION (5U)           /* AF5 */
#define BOARD_CONFIG_BMI088_SPI_KERNEL_FREQUENCY_HZ (183333333UL) /* SPI 内核时钟 */
#define BOARD_CONFIG_BMI088_SPI_PRESCALER (32U)                   /* 分频系数 */
#define BOARD_CONFIG_BMI088_SPI_FREQUENCY_HZ (5729166UL)          /* 实际 SPI 时钟 ≈ 5.73MHz */
#define BOARD_CONFIG_BMI088_SPI_CLOCK_POLARITY_HIGH (1U)          /* CPOL=1 */
#define BOARD_CONFIG_BMI088_SPI_CLOCK_PHASE_SECOND_EDGE (1U)      /* CPHA=1 */
#define BOARD_CONFIG_BMI088_SPI_SCK_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 10U)
#define BOARD_CONFIG_BMI088_SPI_MISO_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 14U)
#define BOARD_CONFIG_BMI088_SPI_MOSI_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 15U)

/* 加速度计片选：PD8，低电平有效 */
#define BOARD_CONFIG_BMI088_ACCELEROMETER_CHIP_SELECT_PIN                                          \
    BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_D, 8U)
/* 陀螺仪片选：PE15，低电平有效 */
#define BOARD_CONFIG_BMI088_GYROSCOPE_CHIP_SELECT_PIN                                              \
    BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 15U)
#define BOARD_CONFIG_BMI088_CHIP_SELECT_ACTIVE_LEVEL BOARD_CONFIG_GPIO_ACTIVE_LOW

/* 加速度计中断：PE14，EXTI Line 14 */
#define BOARD_CONFIG_BMI088_ACCELEROMETER_INTERRUPT_PIN                                            \
    BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 14U)
#define BOARD_CONFIG_BMI088_ACCELEROMETER_EXTI_LINE (14U)

/* 陀螺仪中断：PE13，EXTI Line 13 */
#define BOARD_CONFIG_BMI088_GYROSCOPE_INTERRUPT_PIN                                                \
    BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 13U)
#define BOARD_CONFIG_BMI088_GYROSCOPE_EXTI_LINE (13U)

/* ------------------------------------------------------------------------- */
/* 定时器与 PWM 输出                                                        */
/* ------------------------------------------------------------------------- */

/* 辅助 PWM 1：TIM3_CH1，PA6，AF2 */
#define BOARD_CONFIG_AUXILIARY_PWM_1_LOGICAL_NAME "auxiliary_pwm_1"
#define BOARD_CONFIG_AUXILIARY_PWM_1_TIMER_INSTANCE (3U)
#define BOARD_CONFIG_AUXILIARY_PWM_1_TIMER_CHANNEL (1U)
#define BOARD_CONFIG_AUXILIARY_PWM_1_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 6U)
#define BOARD_CONFIG_AUXILIARY_PWM_1_ALTERNATE_FUNCTION (2U)

/* 辅助 PWM 2：TIM3_CH2，PA7，AF2 */
#define BOARD_CONFIG_AUXILIARY_PWM_2_LOGICAL_NAME "auxiliary_pwm_2"
#define BOARD_CONFIG_AUXILIARY_PWM_2_TIMER_INSTANCE (3U)
#define BOARD_CONFIG_AUXILIARY_PWM_2_TIMER_CHANNEL (2U)
#define BOARD_CONFIG_AUXILIARY_PWM_2_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_A, 7U)
#define BOARD_CONFIG_AUXILIARY_PWM_2_ALTERNATE_FUNCTION (2U)

/* 辅助 PWM 3：TIM3_CH3，PB0，AF2 */
#define BOARD_CONFIG_AUXILIARY_PWM_3_LOGICAL_NAME "auxiliary_pwm_3"
#define BOARD_CONFIG_AUXILIARY_PWM_3_TIMER_INSTANCE (3U)
#define BOARD_CONFIG_AUXILIARY_PWM_3_TIMER_CHANNEL (3U)
#define BOARD_CONFIG_AUXILIARY_PWM_3_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 0U)
#define BOARD_CONFIG_AUXILIARY_PWM_3_ALTERNATE_FUNCTION (2U)

/* 辅助 PWM 4：TIM3_CH4，PB1，AF2 */
#define BOARD_CONFIG_AUXILIARY_PWM_4_LOGICAL_NAME "auxiliary_pwm_4"
#define BOARD_CONFIG_AUXILIARY_PWM_4_TIMER_INSTANCE (3U)
#define BOARD_CONFIG_AUXILIARY_PWM_4_TIMER_CHANNEL (4U)
#define BOARD_CONFIG_AUXILIARY_PWM_4_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_B, 1U)
#define BOARD_CONFIG_AUXILIARY_PWM_4_ALTERNATE_FUNCTION (2U)

/* 蜂鸣器 PWM：TIM1_CH1，PE9，AF1 */
#define BOARD_CONFIG_BUZZER_PWM_LOGICAL_NAME "buzzer_pwm"
#define BOARD_CONFIG_BUZZER_PWM_TIMER_INSTANCE (1U)
#define BOARD_CONFIG_BUZZER_PWM_TIMER_CHANNEL (1U)
#define BOARD_CONFIG_BUZZER_PWM_PIN BOARD_CONFIG_GPIO_PIN(BOARD_CONFIG_GPIO_PORT_E, 9U)
#define BOARD_CONFIG_BUZZER_PWM_ALTERNATE_FUNCTION (1U)

/* ------------------------------------------------------------------------- */
/* 已知引脚冲突                                                              */
/* ------------------------------------------------------------------------- */

/*
 * PC6 和 PC7 没有硬件 I2C 复用功能，它们已被分配给 USART6。
 * 因此在这些引脚上使用软件 I2C 被禁用。启用硬件 I2C 前需要将 I2C 移到其他空闲引脚。
 */
#define BOARD_CONFIG_SOFTWARE_I2C_PC6_PC7_ENABLED (0U)

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

#endif /* BOARD_CONFIG_H */
