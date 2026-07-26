#include "bsp_common.h"

#include <stddef.h>

#define BSP_DEVICE_OBJECT_MAGIC (0x4253504FU)

bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                             void *const device_handle)
{
    if ((me == NULL) || (vptr == NULL) || (device_handle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->vptr = vptr;
    me->device_handle = device_handle;
    me->object_magic = BSP_DEVICE_OBJECT_MAGIC;
    me->is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_device_deinit(bsp_device_t *const me)
{
    bsp_status_t status;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_device_is_initialized(me))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    status = (me->vptr->deinit != NULL) ? me->vptr->deinit(me) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK)
    {
        me->vptr = NULL;
        me->device_handle = NULL;
        me->object_magic = 0U;
        me->is_initialized = false;
    }
    return status;
}

bool bsp_device_is_initialized(const bsp_device_t *const me)
{
    return (me != NULL) && (me->object_magic == BSP_DEVICE_OBJECT_MAGIC) && me->is_initialized &&
           (me->vptr != NULL) && (me->device_handle != NULL);
}

bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode)
{
    return (transfer_mode == BSP_TRANSFER_MODE_BLOCKING) ||
           (transfer_mode == BSP_TRANSFER_MODE_INTERRUPT) ||
           (transfer_mode == BSP_TRANSFER_MODE_DMA);
}

void *bsp_device_get_handle(const bsp_device_t *const me)
{
    return bsp_device_is_initialized(me) ? me->device_handle : NULL;
}
