#include "bsp_exti.h"

static const bsp_exti_driver_ops_t *bsp_exti_platform_ops;

bsp_status_t bsp_exti_bind_platform(const bsp_exti_driver_ops_t *driver_ops)
{
    if ((driver_ops == NULL) || (driver_ops->enable == NULL) || (driver_ops->disable == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    if ((bsp_exti_platform_ops != NULL) && (bsp_exti_platform_ops != driver_ops))
        return BSP_STATUS_BUSY;
    bsp_exti_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_exti_init(bsp_exti_t *me, const bsp_exti_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    status = bsp_exti_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK) return status;
    if (bsp_exti_platform_ops->init != NULL)
    {
        status = bsp_exti_platform_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle;
    me->callback = config->callback;
    me->user_context = config->user_context;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_exti_deinit(bsp_exti_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_exti_platform_ops == NULL))
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    (void)bsp_exti_platform_ops->disable(me->device_handle);
    if (bsp_exti_platform_ops->deinit != NULL) status = bsp_exti_platform_ops->deinit(me->device_handle);
    if (status == BSP_STATUS_OK)
    {
        me->device_handle = NULL; me->callback = NULL; me->user_context = NULL;
        me->is_initialized = false;
    }
    return status;
}

bool bsp_exti_is_initialized(const bsp_exti_t *me)
{ return (me != NULL) && me->is_initialized && (bsp_exti_platform_ops != NULL); }

bsp_status_t bsp_exti_set_callback(bsp_exti_t *me, bsp_exti_callback_t callback, void *user_context)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    me->callback = callback; me->user_context = user_context; return BSP_STATUS_OK;
}
bsp_status_t bsp_exti_enable(bsp_exti_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_exti_platform_ops->enable(me->device_handle);
}
bsp_status_t bsp_exti_disable(bsp_exti_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_exti_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_exti_platform_ops->disable(me->device_handle);
}
void bsp_exti_notify(bsp_exti_t *me)
{
    if (bsp_exti_is_initialized(me) && (me->callback != NULL)) me->callback(me, me->user_context);
}
