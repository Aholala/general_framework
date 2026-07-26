#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_pwm bsp_pwm_t;
    typedef struct bsp_pwm_device bsp_pwm_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_pwm_t *const me);
        bsp_status_t (*stop)(bsp_pwm_t *const me);
        bsp_status_t (*set_frequency)(bsp_pwm_t *const me, uint32_t frequency_hz);
        bsp_status_t (*get_frequency)(const bsp_pwm_t *const me, uint32_t *frequency_hz);
        bsp_status_t (*set_pulse)(bsp_pwm_t *const me, uint32_t pulse_ticks);
        bsp_status_t (*get_pulse)(const bsp_pwm_t *const me, uint32_t *pulse_ticks);
        bsp_status_t (*get_period)(const bsp_pwm_t *const me, uint32_t *period_ticks);
    } bsp_pwm_ops_t;

    struct bsp_pwm
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle, uint32_t channel);
        bsp_status_t (*deinit)(void *device_handle, uint32_t channel);
        bsp_status_t (*start)(void *device_handle, uint32_t channel);
        bsp_status_t (*stop)(void *device_handle, uint32_t channel);
        bsp_status_t (*set_frequency)(void *device_handle, uint32_t channel, uint32_t frequency_hz);
        bsp_status_t (*get_frequency)(const void *device_handle, uint32_t channel,
                                      uint32_t *frequency_hz);
        bsp_status_t (*set_pulse)(void *device_handle, uint32_t channel, uint32_t pulse_ticks);
        bsp_status_t (*get_pulse)(const void *device_handle, uint32_t channel,
                                  uint32_t *pulse_ticks);
        bsp_status_t (*get_period)(const void *device_handle, uint32_t channel,
                                   uint32_t *period_ticks);
    } bsp_pwm_driver_ops_t;

    struct bsp_pwm_device
    {
        bsp_pwm_t super;
        const bsp_pwm_driver_ops_t *driver_ops;
        uint32_t channel;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_pwm_driver_ops_t *driver_ops;
        uint32_t channel;
    } bsp_pwm_config_t;

    bsp_status_t bsp_pwm_init(bsp_pwm_device_t *const me, const bsp_pwm_config_t *const config);
    bsp_pwm_t *bsp_pwm_as_base(bsp_pwm_device_t *const me);
    bsp_status_t bsp_pwm_start(bsp_pwm_t *const me);
    bsp_status_t bsp_pwm_stop(bsp_pwm_t *const me);
    bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *const me, uint32_t frequency_hz);
    bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *const me, uint32_t *frequency_hz);
    bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *const me, uint32_t pulse_ticks);
    bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *const me, uint32_t *pulse_ticks);
    bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *const me, float duty_cycle);
    bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *const me, float *duty_cycle);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
