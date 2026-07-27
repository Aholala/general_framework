#include "bsp_rtc.h"

static bsp_status_t bsp_rtc_deinit_virtual(bsp_device_t *device)
{
    bsp_rtc_device_t *const me = BSP_CONTAINER_OF(device, bsp_rtc_device_t, super.super);
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

static bsp_status_t bsp_rtc_get_time_virtual(bsp_rtc_t *me, bsp_rtc_time_t *time)
{
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    return device->driver_ops->get_time(me->super.device_handle, time);
}

static bsp_status_t bsp_rtc_set_time_virtual(bsp_rtc_t *me, const bsp_rtc_time_t *time)
{
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    return device->driver_ops->set_time(me->super.device_handle, time);
}

static bsp_status_t bsp_rtc_get_unix_virtual(bsp_rtc_t *me, uint64_t *unix_time_s)
{
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    return device->driver_ops->get_unix_time(me->super.device_handle, unix_time_s);
}

static const bsp_rtc_ops_t bsp_rtc_ops = {
    .super = {.deinit = bsp_rtc_deinit_virtual},
    .get_time = bsp_rtc_get_time_virtual,
    .set_time = bsp_rtc_set_time_virtual,
    .get_unix_time = bsp_rtc_get_unix_virtual,
};

bsp_status_t bsp_rtc_init(bsp_rtc_device_t *me, const bsp_rtc_config_t *config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->get_time == NULL) ||
        (config->driver_ops->set_time == NULL) || (config->driver_ops->get_unix_time == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->driver_ops = config->driver_ops;
    status = bsp_device_init(&me->super.super, &bsp_rtc_ops.super, config->device_handle);
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

bsp_rtc_t *bsp_rtc_as_base(bsp_rtc_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_rtc_get_time(bsp_rtc_t *me, bsp_rtc_time_t *time)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (time == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_rtc_ops_t *)me->super.vptr)->get_time(me, time);
}

bsp_status_t bsp_rtc_set_time(bsp_rtc_t *me, const bsp_rtc_time_t *time)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (time == NULL) ||
        (time->month < 1U) || (time->month > 12U) || (time->day < 1U) || (time->day > 31U) ||
        (time->hour > 23U) || (time->minute > 59U) || (time->second > 59U) ||
        (time->millisecond > 999U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_rtc_ops_t *)me->super.vptr)->set_time(me, time);
}

bsp_status_t bsp_rtc_get_unix_time(bsp_rtc_t *me, uint64_t *unix_time_s)
{
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (unix_time_s == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return ((const bsp_rtc_ops_t *)me->super.vptr)->get_unix_time(me, unix_time_s);
}
