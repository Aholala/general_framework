#include "bsp_rng.h"

static bsp_status_t bsp_rng_deinit_virtual(bsp_device_t *device)
{
    bsp_rng_device_t *const me = BSP_CONTAINER_OF(device, bsp_rng_device_t, super.super);
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

static bsp_status_t bsp_rng_get_uint32_virtual(bsp_rng_t *me, uint32_t *value)
{
    bsp_rng_device_t *const device = BSP_CONTAINER_OF(me, bsp_rng_device_t, super);
    return device->driver_ops->get_uint32(me->super.device_handle, value);
}

static bsp_status_t bsp_rng_fill_virtual(bsp_rng_t *me, void *data, size_t size)
{
    bsp_rng_device_t *const device = BSP_CONTAINER_OF(me, bsp_rng_device_t, super);
    return device->driver_ops->fill(me->super.device_handle, data, size);
}

static const bsp_rng_ops_t bsp_rng_ops = {
    .super = {.deinit = bsp_rng_deinit_virtual},
    .get_uint32 = bsp_rng_get_uint32_virtual,
    .fill = bsp_rng_fill_virtual,
};

bsp_status_t bsp_rng_init(bsp_rng_device_t *me, const bsp_rng_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->get_uint32 == NULL) ||
        (config->driver_ops->fill == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->driver_ops = config->driver_ops;
    status = bsp_device_init(&me->super.super, &bsp_rng_ops.super, config->device_handle);
    if (status == BSP_STATUS_OK)
    {
        status = config->driver_ops->init(config->device_handle);
    }
    if (status != BSP_STATUS_OK)
    {
        me->super.super.is_initialized = false;
    }
    return status;
}

bsp_rng_t *bsp_rng_as_base(bsp_rng_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_rng_get_uint32(bsp_rng_t *me, uint32_t *value)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (value == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_rng_ops_t *)me->super.vptr)->get_uint32(me, value);
}

bsp_status_t bsp_rng_fill(bsp_rng_t *me, void *data, size_t size)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_rng_ops_t *)me->super.vptr)->fill(me, data, size);
}
