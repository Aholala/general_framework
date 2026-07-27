#include "bsp_crc.h"

static bsp_status_t bsp_crc_deinit_virtual(bsp_device_t *device)
{
    bsp_crc_device_t *const me = BSP_CONTAINER_OF(device, bsp_crc_device_t, super.super);
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

static bsp_status_t bsp_crc_calculate_virtual(bsp_crc_t *me, const void *data, size_t size,
                                              uint32_t initial_value, uint32_t *result)
{
    bsp_crc_device_t *const device = BSP_CONTAINER_OF(me, bsp_crc_device_t, super);
    return device->driver_ops->calculate(me->super.device_handle, data, size, initial_value,
                                         result);
}

static const bsp_crc_ops_t bsp_crc_ops = {
    .super = {.deinit = bsp_crc_deinit_virtual},
    .calculate = bsp_crc_calculate_virtual,
};

bsp_status_t bsp_crc_init(bsp_crc_device_t *me, const bsp_crc_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->calculate == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->driver_ops = config->driver_ops;
    status = bsp_device_init(&me->super.super, &bsp_crc_ops.super, config->device_handle);
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

bsp_crc_t *bsp_crc_as_base(bsp_crc_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_crc_calculate(bsp_crc_t *me, const void *data, size_t size, uint32_t initial_value,
                               uint32_t *result)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U) ||
        (result == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_crc_ops_t *)me->super.vptr)
        ->calculate(me, data, size, initial_value, result);
}
