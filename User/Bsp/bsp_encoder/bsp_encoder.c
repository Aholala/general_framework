#include "bsp_encoder.h"

#include <limits.h>
#include <stddef.h>

static bsp_encoder_device_t *bsp_encoder_get_device(bsp_encoder_t *const encoder_base)
{
    return BSP_CONTAINER_OF(encoder_base, bsp_encoder_device_t, super);
}

static const bsp_encoder_device_t *
bsp_encoder_get_device_const(const bsp_encoder_t *const encoder_base)
{
    return BSP_CONTAINER_OF_CONST(encoder_base, bsp_encoder_device_t, super);
}

static const bsp_encoder_ops_t *bsp_encoder_get_ops(const bsp_encoder_t *const encoder_base)
{
    return BSP_CONTAINER_OF_CONST(encoder_base->super.vptr, bsp_encoder_ops_t, super);
}

static bsp_status_t bsp_encoder_device_deinit(bsp_device_t *const device_base)
{
    bsp_encoder_t *const encoder_base = BSP_CONTAINER_OF(device_base, bsp_encoder_t, super);
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle);
}

static bsp_status_t bsp_encoder_device_start(bsp_encoder_t *const encoder_base)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    return me->driver_ops->start(encoder_base->super.device_handle);
}

static bsp_status_t bsp_encoder_device_stop(bsp_encoder_t *const encoder_base)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    return me->driver_ops->stop(encoder_base->super.device_handle);
}

static bsp_status_t bsp_encoder_device_set_count(bsp_encoder_t *const encoder_base, int32_t count)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    return me->driver_ops->set_count(encoder_base->super.device_handle, count);
}

static bsp_status_t bsp_encoder_device_get_count(const bsp_encoder_t *const encoder_base,
                                                 int32_t *count)
{
    const bsp_encoder_device_t *const me = bsp_encoder_get_device_const(encoder_base);
    return me->driver_ops->get_count(encoder_base->super.device_handle, count);
}

static bsp_status_t bsp_encoder_device_get_direction(const bsp_encoder_t *const encoder_base,
                                                     bsp_encoder_direction_t *direction)
{
    const bsp_encoder_device_t *const me = bsp_encoder_get_device_const(encoder_base);
    return me->driver_ops->get_direction(encoder_base->super.device_handle, direction);
}

static const bsp_encoder_ops_t s_bsp_encoder_device_ops = {
    .super = {.deinit = bsp_encoder_device_deinit},
    .start = bsp_encoder_device_start,
    .stop = bsp_encoder_device_stop,
    .set_count = bsp_encoder_device_set_count,
    .get_count = bsp_encoder_device_get_count,
    .get_direction = bsp_encoder_device_get_direction};

static bsp_status_t bsp_encoder_validate(const bsp_encoder_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_encoder_init(bsp_encoder_device_t *const me,
                              const bsp_encoder_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_count == NULL) ||
        (config->driver_ops->get_count == NULL) ||
        (config->driver_ops->get_direction == NULL) ||
        ((config->counter_modulus != 0U) &&
         (config->counter_modulus < 2U)))
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
    me->super.previous_count = 0;
    me->super.counter_modulus = config->counter_modulus;
    return bsp_device_init(&me->super.super, &s_bsp_encoder_device_ops.super,
                           config->device_handle);
}

bsp_encoder_t *bsp_encoder_as_base(bsp_encoder_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_encoder_start(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->start(me) : status;
}

bsp_status_t bsp_encoder_stop(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->stop(me) : status;
}

bsp_status_t bsp_encoder_reset(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_set_count(me, 0);
    if (status == BSP_STATUS_OK)
    {
        me->previous_count = 0;
    }
    return status;
}

bsp_status_t bsp_encoder_set_count(bsp_encoder_t *const me, int32_t count)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->set_count(me, count) : status;
}

bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *const me, int32_t *count)
{
    bsp_status_t status = bsp_encoder_validate(me);
    if ((status == BSP_STATUS_OK) && (count == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->get_count(me, count) : status;
}

bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *const me, int32_t *count_delta)
{
    int32_t current_count;
    int64_t count_difference;
    bsp_status_t status;
    if (count_delta == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_encoder_get_count(me, &current_count);
    if (status == BSP_STATUS_OK)
    {
        count_difference =
            (int64_t)current_count - (int64_t)me->previous_count;
        if (me->counter_modulus > 0U)
        {
            const int64_t modulus = (int64_t)me->counter_modulus;
            const int64_t half_modulus = modulus / 2;
            if (count_difference > half_modulus)
            {
                count_difference -= modulus;
            }
            else if (count_difference < -half_modulus)
            {
                count_difference += modulus;
            }
        }
        if ((count_difference > INT32_MAX) ||
            (count_difference < INT32_MIN))
        {
            return BSP_STATUS_OUT_OF_RANGE;
        }
        *count_delta = (int32_t)count_difference;
        me->previous_count = current_count;
    }
    return status;
}

bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *const me,
                                       bsp_encoder_direction_t *direction)
{
    bsp_status_t status = bsp_encoder_validate(me);
    if ((status == BSP_STATUS_OK) && (direction == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->get_direction(me, direction)
                                     : status;
}
