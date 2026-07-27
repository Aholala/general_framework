#include "bsp_storage.h"

static bsp_storage_device_t *bsp_storage_get_device(bsp_storage_t *me)
{
    return BSP_CONTAINER_OF(me, bsp_storage_device_t, super);
}

static const bsp_storage_device_t *bsp_storage_get_const_device(const bsp_storage_t *me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_storage_device_t, super);
}

static bsp_status_t bsp_storage_deinit_virtual(bsp_device_t *device)
{
    bsp_storage_device_t *const me = BSP_CONTAINER_OF(device, bsp_storage_device_t, super.super);
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

static bsp_status_t bsp_storage_read_virtual(bsp_storage_t *me, uint64_t address, void *data,
                                             size_t size)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->read(me->super.device_handle, address, data, size);
}

static bsp_status_t bsp_storage_program_virtual(bsp_storage_t *me, uint64_t address,
                                                const void *data, size_t size)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->program(me->super.device_handle, address, data, size);
}

static bsp_status_t bsp_storage_erase_virtual(bsp_storage_t *me, uint64_t address, size_t size)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->erase(me->super.device_handle, address, size);
}

static bsp_status_t bsp_storage_sync_virtual(bsp_storage_t *me)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->sync(me->super.device_handle);
}

static bsp_status_t bsp_storage_geometry_virtual(const bsp_storage_t *me,
                                                 bsp_storage_geometry_t *geometry)
{
    const bsp_storage_device_t *const device = bsp_storage_get_const_device(me);
    return device->driver_ops->get_geometry(me->super.device_handle, geometry);
}

static const bsp_storage_ops_t bsp_storage_ops = {
    .super = {.deinit = bsp_storage_deinit_virtual},
    .read = bsp_storage_read_virtual,
    .program = bsp_storage_program_virtual,
    .erase = bsp_storage_erase_virtual,
    .sync = bsp_storage_sync_virtual,
    .get_geometry = bsp_storage_geometry_virtual,
};

bsp_status_t bsp_storage_init(bsp_storage_device_t *me, const bsp_storage_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->read == NULL) ||
        (config->driver_ops->program == NULL) || (config->driver_ops->erase == NULL) ||
        (config->driver_ops->sync == NULL) || (config->driver_ops->get_geometry == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->driver_ops = config->driver_ops;
    status = bsp_device_init(&me->super.super, &bsp_storage_ops.super, config->device_handle);
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

bsp_storage_t *bsp_storage_as_base(bsp_storage_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_storage_read(bsp_storage_t *me, uint64_t address, void *data, size_t size)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_storage_ops_t *)me->super.vptr)->read(me, address, data, size);
}

bsp_status_t bsp_storage_program(bsp_storage_t *me, uint64_t address, const void *data, size_t size)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_storage_ops_t *)me->super.vptr)->program(me, address, data, size);
}

bsp_status_t bsp_storage_erase(bsp_storage_t *me, uint64_t address, size_t size)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_storage_ops_t *)me->super.vptr)->erase(me, address, size);
}

bsp_status_t bsp_storage_sync(bsp_storage_t *me)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    return ((const bsp_storage_ops_t *)me->super.vptr)->sync(me);
}

bsp_status_t bsp_storage_get_geometry(const bsp_storage_t *me, bsp_storage_geometry_t *geometry)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (geometry == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_storage_ops_t *)me->super.vptr)->get_geometry(me, geometry);
}
