#include "bsp_timer.h"

#include <stddef.h>

static bsp_timer_device_t *bsp_timer_get_device(bsp_timer_t *const timer_base)
{
    return BSP_CONTAINER_OF(timer_base, bsp_timer_device_t, super);
}

static const bsp_timer_device_t *bsp_timer_get_device_const(const bsp_timer_t *const timer_base)
{
    return BSP_CONTAINER_OF_CONST(timer_base, bsp_timer_device_t, super);
}

static const bsp_timer_ops_t *bsp_timer_get_ops(const bsp_timer_t *const timer_base)
{
    return BSP_CONTAINER_OF_CONST(timer_base->super.vptr, bsp_timer_ops_t, super);
}

static bsp_status_t bsp_timer_device_deinit(bsp_device_t *const device_base)
{
    bsp_timer_t *const timer_base = BSP_CONTAINER_OF(device_base, bsp_timer_t, super);
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle);
}

#define BSP_TIMER_FORWARD(name, member)                                                            \
    static bsp_status_t name(bsp_timer_t *const timer_base)                                        \
    {                                                                                              \
        bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);                           \
        return me->driver_ops->member(timer_base->super.device_handle);                            \
    }

BSP_TIMER_FORWARD(bsp_timer_device_start, start)
BSP_TIMER_FORWARD(bsp_timer_device_stop, stop)

static bsp_status_t bsp_timer_device_set_counter(bsp_timer_t *const timer_base,
                                                 uint32_t counter_ticks)
{
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    return me->driver_ops->set_counter(timer_base->super.device_handle, counter_ticks);
}

static bsp_status_t bsp_timer_device_get_counter(const bsp_timer_t *const timer_base,
                                                 uint32_t *counter_ticks)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_counter(timer_base->super.device_handle, counter_ticks);
}

static bsp_status_t bsp_timer_device_set_period(bsp_timer_t *const timer_base,
                                                uint32_t period_ticks)
{
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    return me->driver_ops->set_period(timer_base->super.device_handle, period_ticks);
}

static bsp_status_t bsp_timer_device_get_period(const bsp_timer_t *const timer_base,
                                                uint32_t *period_ticks)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_period(timer_base->super.device_handle, period_ticks);
}

static bsp_status_t bsp_timer_device_get_frequency(const bsp_timer_t *const timer_base,
                                                   uint32_t *frequency_hz)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_frequency(timer_base->super.device_handle, frequency_hz);
}

static const bsp_timer_ops_t s_bsp_timer_device_ops = {.super = {.deinit = bsp_timer_device_deinit},
                                                       .start = bsp_timer_device_start,
                                                       .stop = bsp_timer_device_stop,
                                                       .set_counter = bsp_timer_device_set_counter,
                                                       .get_counter = bsp_timer_device_get_counter,
                                                       .set_period = bsp_timer_device_set_period,
                                                       .get_period = bsp_timer_device_get_period,
                                                       .get_frequency =
                                                           bsp_timer_device_get_frequency};

static bsp_status_t bsp_timer_validate(const bsp_timer_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_timer_init(bsp_timer_device_t *const me, const bsp_timer_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_counter == NULL) ||
        (config->driver_ops->get_counter == NULL) || (config->driver_ops->set_period == NULL) ||
        (config->driver_ops->get_period == NULL) || (config->driver_ops->get_frequency == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->super.super.is_initialized = false;
    me->driver_ops = config->driver_ops;
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    return bsp_device_init(&me->super.super, &s_bsp_timer_device_ops.super, config->device_handle);
}

bsp_timer_t *bsp_timer_as_base(bsp_timer_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_timer_set_callback(bsp_timer_t *const me, bsp_timer_callback_t callback,
                                    void *user_context)
{
    bsp_status_t status = bsp_timer_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

bsp_status_t bsp_timer_start(bsp_timer_t *const me)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->start(me) : status;
}

bsp_status_t bsp_timer_stop(bsp_timer_t *const me)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->stop(me) : status;
}

bsp_status_t bsp_timer_reset(bsp_timer_t *const me)
{
    return bsp_timer_set_counter(me, 0U);
}

bsp_status_t bsp_timer_set_counter(bsp_timer_t *const me, uint32_t counter_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->set_counter(me, counter_ticks)
                                     : status;
}

bsp_status_t bsp_timer_get_counter(const bsp_timer_t *const me, uint32_t *counter_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    if ((status == BSP_STATUS_OK) && (counter_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_counter(me, counter_ticks)
                                     : status;
}

bsp_status_t bsp_timer_set_period(bsp_timer_t *const me, uint32_t period_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    if ((status == BSP_STATUS_OK) && (period_ticks == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->set_period(me, period_ticks) : status;
}

bsp_status_t bsp_timer_get_period(const bsp_timer_t *const me, uint32_t *period_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    if ((status == BSP_STATUS_OK) && (period_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_period(me, period_ticks) : status;
}

bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_timer_validate(me);
    if ((status == BSP_STATUS_OK) && (frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_frequency(me, frequency_hz)
                                     : status;
}

void bsp_timer_notify_elapsed(bsp_timer_t *const me)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(me, me->user_context);
    }
}
