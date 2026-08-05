#include "bsp_gpio.h"

static const bsp_gpio_driver_ops_t *bsp_gpio_platform_ops;

static bool bsp_gpio_ops_valid(const bsp_gpio_driver_ops_t *ops)
{
    return (ops != NULL) && (ops->read != NULL) && (ops->write != NULL) &&
           (ops->toggle != NULL);
}

bsp_status_t bsp_gpio_bind_platform(const bsp_gpio_driver_ops_t *driver_ops)
{
    if (!bsp_gpio_ops_valid(driver_ops))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if ((bsp_gpio_platform_ops != NULL) && (bsp_gpio_platform_ops != driver_ops))
    {
        return BSP_STATUS_BUSY;
    }
    bsp_gpio_platform_ops = driver_ops;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_init(bsp_gpio_t *me, const bsp_gpio_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_gpio_bind_platform(config->driver_ops);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (bsp_gpio_platform_ops->init != NULL)
    {
        status = bsp_gpio_platform_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->device_handle = config->device_handle;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_gpio_deinit(bsp_gpio_t *me)
{
    bsp_status_t status = BSP_STATUS_OK;
    if ((me == NULL) || !me->is_initialized || (bsp_gpio_platform_ops == NULL))
    {
        return (me == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_NOT_INITIALIZED;
    }
    if (bsp_gpio_platform_ops->deinit != NULL)
    {
        status = bsp_gpio_platform_ops->deinit(me->device_handle);
    }
    if (status == BSP_STATUS_OK)
    {
        me->device_handle = NULL;
        me->is_initialized = false;
    }
    return status;
}

bool bsp_gpio_is_initialized(const bsp_gpio_t *me)
{
    return (me != NULL) && me->is_initialized && (bsp_gpio_platform_ops != NULL);
}

bsp_status_t bsp_gpio_read(const bsp_gpio_t *me, bool *level)
{
    if ((me == NULL) || (level == NULL)) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->read(me->device_handle, level);
}

bsp_status_t bsp_gpio_write(bsp_gpio_t *me, bool level)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->write(me->device_handle, level);
}

bsp_status_t bsp_gpio_toggle(bsp_gpio_t *me)
{
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    if (!bsp_gpio_is_initialized(me)) return BSP_STATUS_NOT_INITIALIZED;
    return bsp_gpio_platform_ops->toggle(me->device_handle);
}
