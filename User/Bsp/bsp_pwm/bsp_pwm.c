#include "bsp_pwm.h"

#include <math.h>

static const bsp_pwm_driver_ops_t *bsp_pwm_platform_ops;

static bool bsp_pwm_ops_valid(const bsp_pwm_driver_ops_t *ops)
{
    return (ops != NULL) && (ops->start != NULL) && (ops->stop != NULL) &&
           (ops->set_frequency != NULL) && (ops->get_frequency != NULL) &&
           (ops->set_pulse != NULL) && (ops->get_pulse != NULL) && (ops->get_period != NULL);
}

bsp_status_t bsp_pwm_bind_platform(const bsp_pwm_driver_ops_t *driver_ops)
{
    if (!bsp_pwm_ops_valid(driver_ops)) return BSP_STATUS_INVALID_ARGUMENT;
    if ((bsp_pwm_platform_ops != NULL) && (bsp_pwm_platform_ops != driver_ops)) return BSP_STATUS_BUSY;
    bsp_pwm_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_pwm_init(bsp_pwm_t *me, const bsp_pwm_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    status = bsp_pwm_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK) return status;
    if (bsp_pwm_platform_ops->init != NULL)
    {
        status = bsp_pwm_platform_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle; me->channel = config->channel; me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_pwm_deinit(bsp_pwm_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_pwm_platform_ops == NULL))
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    (void)bsp_pwm_platform_ops->stop(me->device_handle, me->channel);
    if (bsp_pwm_platform_ops->deinit != NULL) status = bsp_pwm_platform_ops->deinit(me->device_handle, me->channel);
    if (status == BSP_STATUS_OK) { me->device_handle = NULL; me->channel = 0U; me->is_initialized = false; }
    return status;
}

bool bsp_pwm_is_initialized(const bsp_pwm_t *me)
{ return (me != NULL) && me->is_initialized && (bsp_pwm_platform_ops != NULL); }

#define BSP_PWM_CALL(me, member) \
    (((me) == NULL) ? BSP_STATUS_INVALID_ARGUMENT : \
     (!bsp_pwm_is_initialized(me) ? BSP_STATUS_NOT_INITIALIZED : \
      bsp_pwm_platform_ops->member((me)->device_handle, (me)->channel)))

bsp_status_t bsp_pwm_start(bsp_pwm_t *me) { return BSP_PWM_CALL(me, start); }
bsp_status_t bsp_pwm_stop(bsp_pwm_t *me) { return BSP_PWM_CALL(me, stop); }

bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *me, uint32_t frequency_hz)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    if (frequency_hz == 0U) return BSP_STATUS_OUT_OF_RANGE;
    return bsp_pwm_platform_ops->set_frequency(me->device_handle, me->channel, frequency_hz);
}
bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *me, uint32_t *frequency_hz)
{
    if ((me == NULL) || (frequency_hz == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_pwm_platform_ops->get_frequency(me->device_handle, me->channel, frequency_hz);
}
bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *me, uint32_t pulse_ticks)
{
    uint32_t period_ticks; bsp_status_t status;
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if (status != BSP_STATUS_OK) return status;
    if (pulse_ticks > period_ticks) return BSP_STATUS_OUT_OF_RANGE;
    return bsp_pwm_platform_ops->set_pulse(me->device_handle, me->channel, pulse_ticks);
}
bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *me, uint32_t *pulse_ticks)
{
    if ((me == NULL) || (pulse_ticks == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_pwm_platform_ops->get_pulse(me->device_handle, me->channel, pulse_ticks);
}
bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *me, float duty_cycle)
{
    uint32_t period_ticks; bsp_status_t status;
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    if (!isfinite(duty_cycle) || (duty_cycle < 0.0F) || (duty_cycle > 1.0F)) return BSP_STATUS_OUT_OF_RANGE;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if (status != BSP_STATUS_OK) return status;
    return bsp_pwm_platform_ops->set_pulse(me->device_handle, me->channel,
                                            (uint32_t)((float)period_ticks * duty_cycle + 0.5F));
}
bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *me, float *duty_cycle)
{
    uint32_t period_ticks, pulse_ticks; bsp_status_t status;
    if ((me == NULL) || (duty_cycle == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_pwm_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    status = bsp_pwm_platform_ops->get_period(me->device_handle, me->channel, &period_ticks);
    if ((status != BSP_STATUS_OK) || (period_ticks == 0U)) return (status != BSP_STATUS_OK) ? status : BSP_STATUS_IO_ERROR;
    status = bsp_pwm_platform_ops->get_pulse(me->device_handle, me->channel, &pulse_ticks);
    if (status == BSP_STATUS_OK) *duty_cycle = (float)pulse_ticks / (float)period_ticks;
    return status;
}
