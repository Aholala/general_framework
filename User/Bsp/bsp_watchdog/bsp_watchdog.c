/**
 * @file bsp_watchdog.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_watchdog.h"

#include <stddef.h>

static bsp_watchdog_device_t *bsp_watchdog_get_device(bsp_watchdog_t *const me)
{
    return BSP_CONTAINER_OF(me, bsp_watchdog_device_t, super);
}

static const bsp_watchdog_device_t *bsp_watchdog_get_device_const(const bsp_watchdog_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_watchdog_device_t, super);
}

static const bsp_watchdog_ops_t *bsp_watchdog_get_ops(const bsp_watchdog_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_watchdog_ops_t, super);
}

static bsp_status_t bsp_watchdog_device_deinit(bsp_device_t *const device_base)
{
    bsp_watchdog_t *const watchdog_base = BSP_CONTAINER_OF(device_base, bsp_watchdog_t, super);
    bsp_watchdog_device_t *const me = bsp_watchdog_get_device(watchdog_base);

    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_watchdog_device_refresh(bsp_watchdog_t *const watchdog_base)
{
    bsp_watchdog_device_t *const me = bsp_watchdog_get_device(watchdog_base);
    return me->driver_ops->refresh(watchdog_base->super.device_handle);
}

static bsp_status_t bsp_watchdog_device_get_timeout_ms(const bsp_watchdog_t *const watchdog_base,
                                                       uint32_t *timeout_ms)
{
    const bsp_watchdog_device_t *const me = bsp_watchdog_get_device_const(watchdog_base);
    return (me->driver_ops->get_timeout_ms != NULL)
               ? me->driver_ops->get_timeout_ms(watchdog_base->super.device_handle, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t
bsp_watchdog_device_get_reset_detected(const bsp_watchdog_t *const watchdog_base,
                                       bool *reset_detected)
{
    const bsp_watchdog_device_t *const me = bsp_watchdog_get_device_const(watchdog_base);
    return (me->driver_ops->get_reset_detected != NULL)
               ? me->driver_ops->get_reset_detected(watchdog_base->super.device_handle,
                                                    reset_detected)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_watchdog_ops_t s_bsp_watchdog_device_ops = {
    .super = {.deinit = bsp_watchdog_device_deinit},
    .refresh = bsp_watchdog_device_refresh,
    .get_timeout_ms = bsp_watchdog_device_get_timeout_ms,
    .get_reset_detected = bsp_watchdog_device_get_reset_detected,
};

static bsp_status_t bsp_watchdog_validate(const bsp_watchdog_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_watchdog_init(bsp_watchdog_device_t *const me,
                               const bsp_watchdog_config_t *const config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->refresh == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_watchdog_device_ops.super,
                           config->device_handle);
}

bsp_watchdog_t *bsp_watchdog_as_base(bsp_watchdog_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *const me)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_watchdog_get_ops(me)->refresh(me) : status;
}

bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *const me, uint32_t *timeout_ms)
{
    const bsp_status_t status = bsp_watchdog_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (timeout_ms == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_watchdog_get_ops(me)->get_timeout_ms(me, timeout_ms);
}

bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *const me, bool *reset_detected)
{
    const bsp_status_t status = bsp_watchdog_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (reset_detected == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_watchdog_get_ops(me)->get_reset_detected(me, reset_detected);
}
