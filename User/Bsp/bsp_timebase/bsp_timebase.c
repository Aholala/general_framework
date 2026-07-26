#include "bsp_timebase.h"

#include <stddef.h>

#define BSP_TIMEBASE_MICROSECONDS_PER_SECOND (1000000ULL)
#define BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES (UINT32_MAX / 2U)

static bsp_timebase_device_t *bsp_timebase_get_device(bsp_timebase_t *const me)
{
    return BSP_CONTAINER_OF(me, bsp_timebase_device_t, super);
}

static const bsp_timebase_device_t *bsp_timebase_get_device_const(const bsp_timebase_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_timebase_device_t, super);
}

static const bsp_timebase_ops_t *bsp_timebase_get_ops(const bsp_timebase_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_timebase_ops_t, super);
}

static bsp_status_t bsp_timebase_device_deinit(bsp_device_t *const device_base)
{
    bsp_timebase_t *const timebase_base = BSP_CONTAINER_OF(device_base, bsp_timebase_t, super);
    bsp_timebase_device_t *const me = bsp_timebase_get_device(timebase_base);

    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_timebase_device_reset(bsp_timebase_t *const timebase_base)
{
    bsp_timebase_device_t *const me = bsp_timebase_get_device(timebase_base);
    return (me->driver_ops->reset != NULL)
               ? me->driver_ops->reset(timebase_base->super.device_handle)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_timebase_device_get_cycle_count(const bsp_timebase_t *const timebase_base,
                                                        uint32_t *cycle_count)
{
    const bsp_timebase_device_t *const me = bsp_timebase_get_device_const(timebase_base);
    return me->driver_ops->get_cycle_count(timebase_base->super.device_handle, cycle_count);
}

static bsp_status_t bsp_timebase_device_get_frequency(const bsp_timebase_t *const timebase_base,
                                                      uint32_t *frequency_hz)
{
    const bsp_timebase_device_t *const me = bsp_timebase_get_device_const(timebase_base);
    return me->driver_ops->get_frequency(timebase_base->super.device_handle, frequency_hz);
}

static const bsp_timebase_ops_t s_bsp_timebase_device_ops = {
    .super = {.deinit = bsp_timebase_device_deinit},
    .reset = bsp_timebase_device_reset,
    .get_cycle_count = bsp_timebase_device_get_cycle_count,
    .get_frequency = bsp_timebase_device_get_frequency,
};

static bsp_status_t bsp_timebase_validate(const bsp_timebase_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_timebase_init(bsp_timebase_device_t *const me,
                               const bsp_timebase_config_t *const config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->get_cycle_count == NULL) ||
        (config->driver_ops->get_frequency == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->super.super.is_initialized = false;
    me->driver_ops = config->driver_ops;
    if (config->driver_ops->init != NULL)
    {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    return bsp_device_init(&me->super.super, &s_bsp_timebase_device_ops.super,
                           config->device_handle);
}

bsp_timebase_t *bsp_timebase_as_base(bsp_timebase_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_timebase_reset(bsp_timebase_t *const me)
{
    const bsp_status_t status = bsp_timebase_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timebase_get_ops(me)->reset(me) : status;
}

bsp_status_t bsp_timebase_get_cycle_count(const bsp_timebase_t *const me, uint32_t *cycle_count)
{
    const bsp_status_t status = bsp_timebase_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_timebase_get_ops(me)->get_cycle_count(me, cycle_count);
}

bsp_status_t bsp_timebase_get_frequency(const bsp_timebase_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_timebase_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_timebase_get_ops(me)->get_frequency(me, frequency_hz);
    if ((status == BSP_STATUS_OK) && (*frequency_hz == 0U))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

bsp_status_t bsp_timebase_now(const bsp_timebase_t *const me, bsp_timebase_time_point_t *time_point)
{
    if (time_point == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_timebase_get_cycle_count(me, &time_point->cycle_count);
}

bsp_status_t bsp_timebase_elapsed_cycles(const bsp_timebase_t *const me,
                                         bsp_timebase_time_point_t start_time,
                                         uint32_t *elapsed_cycles)
{
    uint32_t current_cycle_count;
    bsp_status_t status;

    if (elapsed_cycles == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_timebase_get_cycle_count(me, &current_cycle_count);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    *elapsed_cycles = current_cycle_count - start_time.cycle_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_timebase_cycles_to_us(const bsp_timebase_t *const me, uint32_t cycle_count,
                                       uint32_t *time_us)
{
    uint32_t frequency_hz;
    uint64_t calculated_time_us;
    bsp_status_t status;

    if (time_us == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_timebase_get_frequency(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    calculated_time_us =
        ((uint64_t)cycle_count * BSP_TIMEBASE_MICROSECONDS_PER_SECOND) / frequency_hz;
    if (calculated_time_us > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *time_us = (uint32_t)calculated_time_us;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_timebase_us_to_cycles(const bsp_timebase_t *const me, uint32_t time_us,
                                       uint32_t *cycle_count)
{
    uint32_t frequency_hz;
    uint64_t calculated_cycle_count;
    bsp_status_t status;

    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_timebase_get_frequency(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    calculated_cycle_count =
        ((uint64_t)time_us * frequency_hz + BSP_TIMEBASE_MICROSECONDS_PER_SECOND - 1ULL) /
        BSP_TIMEBASE_MICROSECONDS_PER_SECOND;
    if (calculated_cycle_count > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *cycle_count = (uint32_t)calculated_cycle_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_timebase_delay_us(const bsp_timebase_t *const me, uint32_t delay_us)
{
    bsp_timebase_time_point_t start_time;
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    status = bsp_timebase_us_to_cycles(me, delay_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    status = bsp_timebase_now(me, &start_time);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    do
    {
        status = bsp_timebase_elapsed_cycles(me, start_time, &elapsed_cycles);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    } while (elapsed_cycles < required_cycles);
    return BSP_STATUS_OK;
}

bsp_status_t bsp_timebase_has_elapsed_us(const bsp_timebase_t *const me,
                                         bsp_timebase_time_point_t start_time, uint32_t duration_us,
                                         bool *has_elapsed)
{
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    if (has_elapsed == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_timebase_us_to_cycles(me, duration_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    status = bsp_timebase_elapsed_cycles(me, start_time, &elapsed_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    *has_elapsed = elapsed_cycles >= required_cycles;
    return BSP_STATUS_OK;
}
