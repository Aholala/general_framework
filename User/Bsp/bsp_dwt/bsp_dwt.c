/**
 * @file bsp_dwt.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief DWT 周期计数的硬件无关换算与时间差实现。
 * @version 1.0
 * @date 2026-06-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_dwt.h"

#include <limits.h>
#include <stddef.h>

#define BSP_DWT_MICROSECONDS_PER_SECOND (1000000ULL)
#define BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES (UINT32_MAX / 2U)

static bsp_status_t bsp_dwt_validate(const bsp_dwt_t *me)
{
    return bsp_dwt_is_initialized(me) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_dwt_init(bsp_dwt_t *me, const bsp_dwt_config_t *config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->driver_ops == NULL) ||
        (config->driver_ops->init == NULL) || (config->driver_ops->reset == NULL) ||
        (config->driver_ops->get_cycle_count == NULL) ||
        (config->driver_ops->get_frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    *me = (bsp_dwt_t){
        .device_handle = config->device_handle,
        .driver_ops = config->driver_ops,
        .is_initialized = false,
    };
    status = me->driver_ops->init(me->device_handle);
    if (status != BSP_STATUS_OK)
    {
        *me = (bsp_dwt_t){0};
        return status;
    }
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bool bsp_dwt_is_initialized(const bsp_dwt_t *me)
{
    return (me != NULL) && me->is_initialized && (me->driver_ops != NULL);
}

bsp_status_t bsp_dwt_reset(bsp_dwt_t *me)
{
    const bsp_status_t status = bsp_dwt_validate(me);

    return (status == BSP_STATUS_OK) ? me->driver_ops->reset(me->device_handle) : status;
}

bsp_status_t bsp_dwt_get_cycle_count(const bsp_dwt_t *me, uint32_t *cycle_count)
{
    const bsp_status_t status = bsp_dwt_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return me->driver_ops->get_cycle_count(me->device_handle, cycle_count);
}

bsp_status_t bsp_dwt_get_frequency_hz(const bsp_dwt_t *me, uint32_t *frequency_hz)
{
    const bsp_status_t status = bsp_dwt_validate(me);
    bsp_status_t driver_status;

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    driver_status = me->driver_ops->get_frequency_hz(me->device_handle, frequency_hz);
    if ((driver_status == BSP_STATUS_OK) && (*frequency_hz == 0U))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return driver_status;
}

bsp_status_t bsp_dwt_now(const bsp_dwt_t *me, bsp_dwt_time_point_t *time_point)
{
    if (time_point == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_dwt_get_cycle_count(me, &time_point->cycle_count);
}

bsp_status_t bsp_dwt_elapsed_cycles(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                    uint32_t *elapsed_cycles)
{
    uint32_t current_cycle_count;
    bsp_status_t status;

    if (elapsed_cycles == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_dwt_get_cycle_count(me, &current_cycle_count);
    if (status == BSP_STATUS_OK)
    {
        *elapsed_cycles = current_cycle_count - start_time.cycle_count;
    }
    return status;
}

bsp_status_t bsp_dwt_cycles_to_us(const bsp_dwt_t *me, uint32_t cycle_count, uint32_t *time_us)
{
    uint32_t frequency_hz;
    uint64_t converted_time_us;
    bsp_status_t status;

    if (time_us == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_dwt_get_frequency_hz(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    converted_time_us =
        ((uint64_t)cycle_count * BSP_DWT_MICROSECONDS_PER_SECOND) / (uint64_t)frequency_hz;
    if (converted_time_us > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *time_us = (uint32_t)converted_time_us;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_us_to_cycles(const bsp_dwt_t *me, uint32_t time_us, uint32_t *cycle_count)
{
    uint32_t frequency_hz;
    uint64_t converted_cycle_count;
    bsp_status_t status;

    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_dwt_get_frequency_hz(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    converted_cycle_count =
        ((uint64_t)time_us * (uint64_t)frequency_hz + BSP_DWT_MICROSECONDS_PER_SECOND - 1ULL) /
        BSP_DWT_MICROSECONDS_PER_SECOND;
    if (converted_cycle_count > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *cycle_count = (uint32_t)converted_cycle_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_delay_us(const bsp_dwt_t *me, uint32_t delay_us)
{
    bsp_dwt_time_point_t start_time;
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    status = bsp_dwt_us_to_cycles(me, delay_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    status = bsp_dwt_now(me, &start_time);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    do
    {
        status = bsp_dwt_elapsed_cycles(me, start_time, &elapsed_cycles);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    } while (elapsed_cycles < required_cycles);
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_has_elapsed_us(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                    uint32_t duration_us, bool *has_elapsed)
{
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    if (has_elapsed == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_dwt_us_to_cycles(me, duration_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    status = bsp_dwt_elapsed_cycles(me, start_time, &elapsed_cycles);
    if (status == BSP_STATUS_OK)
    {
        *has_elapsed = elapsed_cycles >= required_cycles;
    }
    return status;
}
