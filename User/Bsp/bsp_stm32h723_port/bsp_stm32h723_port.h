#ifndef BSP_STM32H723_PORT_H
#define BSP_STM32H723_PORT_H

#include "bsp_can.h"
#include "bsp_exti.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_timebase.h"
#include "bsp_usart.h"
#include "bsp_usb_vcp.h"
#include "bsp_watchdog.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BSP_STM32H723_CAN_1 = 0,
        BSP_STM32H723_CAN_2,
        BSP_STM32H723_CAN_3,
        BSP_STM32H723_CAN_COUNT
    } bsp_stm32h723_can_index_t;

    typedef enum
    {
        BSP_STM32H723_USART_2 = 0,
        BSP_STM32H723_USART_6,
        BSP_STM32H723_UART_7,
        BSP_STM32H723_UART_8,
        BSP_STM32H723_UART_DR16,
        BSP_STM32H723_USART_COUNT
    } bsp_stm32h723_usart_index_t;

    typedef enum
    {
        BSP_STM32H723_EXTI_BMI088_GYROSCOPE = 0,
        BSP_STM32H723_EXTI_BMI088_ACCELEROMETER,
        BSP_STM32H723_EXTI_COUNT
    } bsp_stm32h723_exti_index_t;

    typedef enum
    {
        BSP_STM32H723_PWM_AUXILIARY_1 = 0,
        BSP_STM32H723_PWM_AUXILIARY_2,
        BSP_STM32H723_PWM_AUXILIARY_3,
        BSP_STM32H723_PWM_AUXILIARY_4,
        BSP_STM32H723_PWM_BUZZER,
        BSP_STM32H723_PWM_COUNT
    } bsp_stm32h723_pwm_index_t;

    typedef struct
    {
        bool initialize_watchdog;
    } bsp_stm32h723_port_config_t;

    bsp_status_t bsp_stm32h723_port_init(const bsp_stm32h723_port_config_t *config);
    bsp_can_t *bsp_stm32h723_port_get_can(bsp_stm32h723_can_index_t index);
    bsp_usart_t *bsp_stm32h723_port_get_usart(bsp_stm32h723_usart_index_t index);
    bsp_spi_t *bsp_stm32h723_port_get_bmi088_spi(void);
    bsp_exti_t *bsp_stm32h723_port_get_exti(bsp_stm32h723_exti_index_t index);
    bsp_pwm_t *bsp_stm32h723_port_get_pwm(bsp_stm32h723_pwm_index_t index);
    bsp_usb_vcp_t *bsp_stm32h723_port_get_usb_vcp(void);
    bsp_timebase_t *bsp_stm32h723_port_get_timebase(void);
    bsp_watchdog_t *bsp_stm32h723_port_get_watchdog(void);
    bool bsp_stm32h723_port_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif
