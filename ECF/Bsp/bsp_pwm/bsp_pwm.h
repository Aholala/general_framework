#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bsp_status_t (*init)(void *handle, uint32_t channel);
    bsp_status_t (*deinit)(void *handle, uint32_t channel);
    bsp_status_t (*start)(void *handle, uint32_t channel);
    bsp_status_t (*stop)(void *handle, uint32_t channel);
    bsp_status_t (*set_frequency)(void *handle, uint32_t channel, uint32_t frequency_hz);
    bsp_status_t (*get_frequency)(const void *handle, uint32_t channel, uint32_t *frequency_hz);
    bsp_status_t (*set_pulse)(void *handle, uint32_t channel, uint32_t pulse_ticks);
    bsp_status_t (*get_pulse)(const void *handle, uint32_t channel, uint32_t *pulse_ticks);
    bsp_status_t (*get_period)(const void *handle, uint32_t channel, uint32_t *period_ticks);
} bsp_pwm_driver_ops_t;

typedef struct bsp_pwm
{
    void *device_handle;
    uint32_t channel;
    bool is_initialized;
} bsp_pwm_t;

typedef struct
{
    void *device_handle;
    const bsp_pwm_driver_ops_t *driver_ops;
    uint32_t channel;
} bsp_pwm_config_t;

bsp_status_t bsp_pwm_bind_platform(const bsp_pwm_driver_ops_t *driver_ops);
bsp_status_t bsp_pwm_init(bsp_pwm_t *me, const bsp_pwm_config_t *config);
bsp_status_t bsp_pwm_deinit(bsp_pwm_t *me);
bool bsp_pwm_is_initialized(const bsp_pwm_t *me);
bsp_status_t bsp_pwm_start(bsp_pwm_t *me);
bsp_status_t bsp_pwm_stop(bsp_pwm_t *me);
bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *me, uint32_t frequency_hz);
bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *me, uint32_t *frequency_hz);
bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *me, uint32_t pulse_ticks);
bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *me, uint32_t *pulse_ticks);
bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *me, float duty_cycle);
bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *me, float *duty_cycle);

#ifdef __cplusplus
}
#endif
#endif
