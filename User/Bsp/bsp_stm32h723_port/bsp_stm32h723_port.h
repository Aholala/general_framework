/**
 * @file bsp_stm32h723_port.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief STM32H723 BSP 端口头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 定义了 STM32H723 平台的所有外设索引和端口初始化接口。
 *       通用 BSP 模块通过此端口获取基类指针并注入各业务模块。
 */

#ifndef BSP_STM32H723_PORT_H
#define BSP_STM32H723_PORT_H

#include "bsp_can.h"      // CAN 基类
#include "bsp_exti.h"     // EXTI 基类
#include "bsp_pwm.h"      // PWM 基类
#include "bsp_spi.h"      // SPI 基类
#include "bsp_timebase.h" // Timebase 基类
#include "bsp_usart.h"    // USART 基类
#include "bsp_usb_vcp.h"  // USB VCP 基类
#include "bsp_watchdog.h" // Watchdog 基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ---------- 外设索引枚举 ---------- */

    /**
     * @brief CAN 外设索引
     * @note 对应 FDCAN1~FDCAN3
     */
    typedef enum
    {
        BSP_STM32H723_CAN_1 = 0,
        BSP_STM32H723_CAN_2,
        BSP_STM32H723_CAN_3,
        BSP_STM32H723_CAN_COUNT
    } bsp_stm32h723_can_index_t;

    /**
     * @brief USART 外设索引
     * @note UART_DR16 为 DR16 接收专用（UART5）
     */
    typedef enum
    {
        BSP_STM32H723_USART_2 = 0,
        BSP_STM32H723_USART_6,
        BSP_STM32H723_UART_7,
        BSP_STM32H723_UART_8,
        BSP_STM32H723_UART_DR16, // UART5 用于 DR16
        BSP_STM32H723_USART_COUNT
    } bsp_stm32h723_usart_index_t;

    /**
     * @brief EXTI 外设索引
     * @note BMI088 陀螺仪和加速度计中断
     */
    typedef enum
    {
        BSP_STM32H723_EXTI_BMI088_GYROSCOPE = 0,
        BSP_STM32H723_EXTI_BMI088_ACCELEROMETER,
        BSP_STM32H723_EXTI_COUNT
    } bsp_stm32h723_exti_index_t;

    /**
     * @brief PWM 外设索引
     * @note 前四个通道为 TIM3 的四个输出，最后一个为 TIM1 蜂鸣器
     */
    typedef enum
    {
        BSP_STM32H723_PWM_AUXILIARY_1 = 0,
        BSP_STM32H723_PWM_AUXILIARY_2,
        BSP_STM32H723_PWM_AUXILIARY_3,
        BSP_STM32H723_PWM_AUXILIARY_4,
        BSP_STM32H723_PWM_BUZZER, // TIM1 蜂鸣器
        BSP_STM32H723_PWM_COUNT
    } bsp_stm32h723_pwm_index_t;

    /* ---------- 端口配置结构 ---------- */
    /**
     * @brief 端口初始化配置
     */
    typedef struct
    {
        bool initialize_watchdog; // 是否初始化看门狗
    } bsp_stm32h723_port_config_t;

    /* ---------- 公共 API ---------- */

    /**
     * @brief 端口初始化主函数
     * @param config 端口配置
     * @return 执行状态
     */
    bsp_status_t bsp_stm32h723_port_init(const bsp_stm32h723_port_config_t *config);

    /**
     * @brief 获取 CAN 基类指针
     * @param index CAN 外设索引
     * @return bsp_can_t* 基类指针，若无效则返回 NULL
     */
    bsp_can_t *bsp_stm32h723_port_get_can(bsp_stm32h723_can_index_t index);

    /**
     * @brief 获取 USART 基类指针
     * @param index USART 外设索引
     * @return bsp_usart_t* 基类指针
     */
    bsp_usart_t *bsp_stm32h723_port_get_usart(bsp_stm32h723_usart_index_t index);

    /**
     * @brief 获取 BMI088 SPI 基类指针
     * @return bsp_spi_t* 基类指针
     */
    bsp_spi_t *bsp_stm32h723_port_get_bmi088_spi(void);

    /**
     * @brief 获取 EXTI 基类指针
     * @param index EXTI 外设索引
     * @return bsp_exti_t* 基类指针
     */
    bsp_exti_t *bsp_stm32h723_port_get_exti(bsp_stm32h723_exti_index_t index);

    /**
     * @brief 获取 PWM 基类指针
     * @param index PWM 外设索引
     * @return bsp_pwm_t* 基类指针
     */
    bsp_pwm_t *bsp_stm32h723_port_get_pwm(bsp_stm32h723_pwm_index_t index);

    /**
     * @brief 获取 USB VCP 基类指针
     * @return bsp_usb_vcp_t* 基类指针
     */
    bsp_usb_vcp_t *bsp_stm32h723_port_get_usb_vcp(void);

    /**
     * @brief 获取 Timebase 基类指针
     * @return bsp_timebase_t* 基类指针
     */
    bsp_timebase_t *bsp_stm32h723_port_get_timebase(void);

    /**
     * @brief 获取 Watchdog 基类指针
     * @return bsp_watchdog_t* 基类指针，若未初始化则返回 NULL
     */
    bsp_watchdog_t *bsp_stm32h723_port_get_watchdog(void);

    /**
     * @brief 查询端口是否已初始化
     * @return true 已初始化
     */
    bool bsp_stm32h723_port_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STM32H723_PORT_H */