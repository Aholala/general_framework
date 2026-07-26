#include "bsp_exti.h"

#include <stddef.h>

static bsp_exti_device_t *bsp_exti_get_device(bsp_exti_t *const me)
{
    return BSP_CONTAINER_OF(me, bsp_exti_device_t, super);
}

static const bsp_exti_ops_t *bsp_exti_get_ops(const bsp_exti_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_exti_ops_t, super);
}

static bsp_status_t bsp_exti_device_deinit(bsp_device_t *const device_base)
{
    bsp_exti_t *const exti_base = BSP_CONTAINER_OF(device_base, bsp_exti_t, super);
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);

    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_exti_device_enable(bsp_exti_t *const exti_base)
{
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);
    return me->driver_ops->enable(exti_base->super.device_handle);
}

static bsp_status_t bsp_exti_device_disable(bsp_exti_t *const exti_base)
{
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);
    return me->driver_ops->disable(exti_base->super.device_handle);
}

static const bsp_exti_ops_t s_bsp_exti_device_ops = {.super = {.deinit = bsp_exti_device_deinit},
                                                     .enable = bsp_exti_device_enable,
                                                     .disable = bsp_exti_device_disable};

bsp_status_t bsp_exti_init(bsp_exti_device_t *const me, const bsp_exti_config_t *const config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->enable == NULL) ||
        (config->driver_ops->disable == NULL))
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
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    return bsp_device_init(&me->super.super, &s_bsp_exti_device_ops.super, config->device_handle);
}

bsp_exti_t *bsp_exti_as_base(bsp_exti_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_exti_set_callback(bsp_exti_t *const me, bsp_exti_callback_t callback,
                                   void *user_context)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_exti_enable(bsp_exti_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    return bsp_exti_get_ops(me)->enable(me);
}

bsp_status_t bsp_exti_disable(bsp_exti_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    return bsp_exti_get_ops(me)->disable(me);
}

void bsp_exti_notify(bsp_exti_t *const me)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(me, me->user_context);
    }
}
