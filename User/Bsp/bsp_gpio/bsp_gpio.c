#include "bsp_gpio.h"

#include <stddef.h>

static bsp_gpio_device_t *bsp_gpio_get_device(bsp_gpio_t *const me)
{
    return BSP_CONTAINER_OF(me, bsp_gpio_device_t, super);
}

static const bsp_gpio_device_t *bsp_gpio_get_device_const(const bsp_gpio_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_gpio_device_t, super);
}

static const bsp_gpio_ops_t *bsp_gpio_get_ops(const bsp_gpio_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_gpio_ops_t, super);
}

static bsp_status_t bsp_gpio_device_deinit(bsp_device_t *const device_base)
{
    bsp_gpio_t *const gpio_base = BSP_CONTAINER_OF(device_base, bsp_gpio_t, super);
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);

    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_gpio_device_read(const bsp_gpio_t *const gpio_base, bool *is_high)
{
    const bsp_gpio_device_t *const me = bsp_gpio_get_device_const(gpio_base);
    return me->driver_ops->read(gpio_base->super.device_handle, is_high);
}

static bsp_status_t bsp_gpio_device_write(bsp_gpio_t *const gpio_base, bool is_high)
{
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);
    return (me->driver_ops->write != NULL)
               ? me->driver_ops->write(gpio_base->super.device_handle, is_high)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_gpio_device_toggle(bsp_gpio_t *const gpio_base)
{
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);
    return (me->driver_ops->toggle != NULL) ? me->driver_ops->toggle(gpio_base->super.device_handle)
                                            : BSP_STATUS_UNSUPPORTED;
}

static const bsp_gpio_ops_t s_bsp_gpio_device_ops = {
    .super = {.deinit = bsp_gpio_device_deinit},
    .read = bsp_gpio_device_read,
    .write = bsp_gpio_device_write,
    .toggle = bsp_gpio_device_toggle,
};

static bsp_status_t bsp_gpio_validate(const bsp_gpio_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_gpio_init(bsp_gpio_device_t *const me, const bsp_gpio_config_t *const config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->read == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_gpio_device_ops.super, config->device_handle);
}

bsp_gpio_t *bsp_gpio_as_base(bsp_gpio_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_gpio_read(const bsp_gpio_t *const me, bool *is_high)
{
    const bsp_status_t status = bsp_gpio_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (is_high == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_gpio_get_ops(me)->read(me, is_high);
}

bsp_status_t bsp_gpio_write(bsp_gpio_t *const me, bool is_high)
{
    const bsp_status_t status = bsp_gpio_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_gpio_get_ops(me)->write(me, is_high) : status;
}

bsp_status_t bsp_gpio_toggle(bsp_gpio_t *const me)
{
    const bsp_status_t status = bsp_gpio_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_gpio_get_ops(me)->toggle(me) : status;
}
