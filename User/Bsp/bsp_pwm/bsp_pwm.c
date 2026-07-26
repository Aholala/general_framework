#include "bsp_pwm.h"

#include <math.h>
#include <stddef.h>

static bsp_pwm_device_t *bsp_pwm_get_device(bsp_pwm_t *const pwm_base)
{
    return BSP_CONTAINER_OF(pwm_base, bsp_pwm_device_t, super);
}

static const bsp_pwm_device_t *bsp_pwm_get_device_const(const bsp_pwm_t *const pwm_base)
{
    return BSP_CONTAINER_OF_CONST(pwm_base, bsp_pwm_device_t, super);
}

static const bsp_pwm_ops_t *bsp_pwm_get_ops(const bsp_pwm_t *const pwm_base)
{
    return BSP_CONTAINER_OF_CONST(pwm_base->super.vptr, bsp_pwm_ops_t, super);
}

static bsp_status_t bsp_pwm_device_deinit(bsp_device_t *const device_base)
{
    bsp_pwm_t *const pwm_base = BSP_CONTAINER_OF(device_base, bsp_pwm_t, super);
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

static bsp_status_t bsp_pwm_device_start(bsp_pwm_t *const pwm_base)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->start(pwm_base->super.device_handle, me->channel);
}

static bsp_status_t bsp_pwm_device_stop(bsp_pwm_t *const pwm_base)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->stop(pwm_base->super.device_handle, me->channel);
}

static bsp_status_t bsp_pwm_device_set_frequency(bsp_pwm_t *const pwm_base, uint32_t frequency_hz)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->set_frequency(pwm_base->super.device_handle, me->channel, frequency_hz);
}

static bsp_status_t bsp_pwm_device_get_frequency(const bsp_pwm_t *const pwm_base,
                                                 uint32_t *frequency_hz)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_frequency(pwm_base->super.device_handle, me->channel, frequency_hz);
}

static bsp_status_t bsp_pwm_device_set_pulse(bsp_pwm_t *const pwm_base, uint32_t pulse_ticks)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->set_pulse(pwm_base->super.device_handle, me->channel, pulse_ticks);
}

static bsp_status_t bsp_pwm_device_get_pulse(const bsp_pwm_t *const pwm_base, uint32_t *pulse_ticks)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_pulse(pwm_base->super.device_handle, me->channel, pulse_ticks);
}

static bsp_status_t bsp_pwm_device_get_period(const bsp_pwm_t *const pwm_base,
                                              uint32_t *period_ticks)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_period(pwm_base->super.device_handle, me->channel, period_ticks);
}

static const bsp_pwm_ops_t s_bsp_pwm_device_ops = {.super = {.deinit = bsp_pwm_device_deinit},
                                                   .start = bsp_pwm_device_start,
                                                   .stop = bsp_pwm_device_stop,
                                                   .set_frequency = bsp_pwm_device_set_frequency,
                                                   .get_frequency = bsp_pwm_device_get_frequency,
                                                   .set_pulse = bsp_pwm_device_set_pulse,
                                                   .get_pulse = bsp_pwm_device_get_pulse,
                                                   .get_period = bsp_pwm_device_get_period};

static bsp_status_t bsp_pwm_validate(const bsp_pwm_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_pwm_init(bsp_pwm_device_t *const me, const bsp_pwm_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_frequency == NULL) ||
        (config->driver_ops->get_frequency == NULL) || (config->driver_ops->set_pulse == NULL) ||
        (config->driver_ops->get_pulse == NULL) || (config->driver_ops->get_period == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->super.super.is_initialized = false;
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    return bsp_device_init(&me->super.super, &s_bsp_pwm_device_ops.super, config->device_handle);
}

bsp_pwm_t *bsp_pwm_as_base(bsp_pwm_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

#define BSP_PWM_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_pwm_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_pwm_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->member(me) : status;               \
    }

BSP_PWM_PUBLIC_ACTION(bsp_pwm_start, start)
BSP_PWM_PUBLIC_ACTION(bsp_pwm_stop, stop)

bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *const me, uint32_t frequency_hz)
{
    bsp_status_t status = bsp_pwm_validate(me);
    if ((status == BSP_STATUS_OK) && (frequency_hz == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->set_frequency(me, frequency_hz)
                                     : status;
}

bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_pwm_validate(me);
    if ((status == BSP_STATUS_OK) && (frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->get_frequency(me, frequency_hz)
                                     : status;
}

bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *const me, uint32_t pulse_ticks)
{
    uint32_t period_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (pulse_ticks > period_ticks)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_pwm_get_ops(me)->set_pulse(me, pulse_ticks);
}

bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *const me, uint32_t *pulse_ticks)
{
    bsp_status_t status = bsp_pwm_validate(me);
    uint32_t period_ticks;
    if ((status == BSP_STATUS_OK) && (pulse_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_pwm_get_ops(me)->get_pulse(me, pulse_ticks);
    if ((status == BSP_STATUS_OK) && (*pulse_ticks > period_ticks))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *const me, float duty_cycle)
{
    uint32_t period_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!isfinite(duty_cycle) || (duty_cycle < 0.0F) ||
        (duty_cycle > 1.0F))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    return bsp_pwm_get_ops(me)->set_pulse(me, (uint32_t)((float)period_ticks * duty_cycle + 0.5F));
}

bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *const me, float *duty_cycle)
{
    uint32_t period_ticks;
    uint32_t pulse_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    if ((status == BSP_STATUS_OK) && (duty_cycle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if ((status != BSP_STATUS_OK) || (period_ticks == 0U))
    {
        return (status != BSP_STATUS_OK) ? status : BSP_STATUS_IO_ERROR;
    }
    status = bsp_pwm_get_ops(me)->get_pulse(me, &pulse_ticks);
    if (status == BSP_STATUS_OK)
    {
        *duty_cycle = (float)pulse_ticks / (float)period_ticks;
    }
    return status;
}
