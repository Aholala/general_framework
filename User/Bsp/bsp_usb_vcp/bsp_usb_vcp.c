#include "bsp_usb_vcp.h"

#include <stddef.h>

static bsp_usb_vcp_device_t *bsp_usb_vcp_get_device(bsp_usb_vcp_t *const usb_vcp_base)
{
    return BSP_CONTAINER_OF(usb_vcp_base, bsp_usb_vcp_device_t, super);
}

static const bsp_usb_vcp_device_t *
bsp_usb_vcp_get_device_const(const bsp_usb_vcp_t *const usb_vcp_base)
{
    return BSP_CONTAINER_OF_CONST(usb_vcp_base, bsp_usb_vcp_device_t, super);
}

static const bsp_usb_vcp_ops_t *bsp_usb_vcp_get_ops(const bsp_usb_vcp_t *const usb_vcp_base)
{
    return BSP_CONTAINER_OF_CONST(usb_vcp_base->super.vptr, bsp_usb_vcp_ops_t, super);
}

static bsp_status_t bsp_usb_vcp_validate(const bsp_usb_vcp_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

static bsp_status_t bsp_usb_vcp_device_deinit(bsp_device_t *const device_base)
{
    bsp_usb_vcp_t *const usb_vcp_base = BSP_CONTAINER_OF(device_base, bsp_usb_vcp_t, super);
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle);
}

static bsp_status_t bsp_usb_vcp_device_transmit(bsp_usb_vcp_t *const usb_vcp_base,
                                                const uint8_t *transmit_data, size_t data_size,
                                                uint32_t timeout_ms)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    return me->driver_ops->transmit(usb_vcp_base->super.device_handle, transmit_data, data_size,
                                    timeout_ms);
}

static bsp_status_t bsp_usb_vcp_device_receive(bsp_usb_vcp_t *const usb_vcp_base,
                                               uint8_t *receive_data, size_t data_capacity)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    return me->driver_ops->receive(usb_vcp_base->super.device_handle, receive_data, data_capacity);
}

static bsp_status_t bsp_usb_vcp_device_abort(bsp_usb_vcp_t *const usb_vcp_base)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    return (me->driver_ops->abort != NULL)
               ? me->driver_ops->abort(usb_vcp_base->super.device_handle)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_usb_vcp_device_get_connected(const bsp_usb_vcp_t *const usb_vcp_base,
                                                     bool *is_connected)
{
    const bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device_const(usb_vcp_base);
    return (me->driver_ops->get_connected != NULL)
               ? me->driver_ops->get_connected(
                     usb_vcp_base->super.device_handle, is_connected)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_usb_vcp_device_get_busy(const bsp_usb_vcp_t *const usb_vcp_base,
                                                bool *is_busy)
{
    const bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device_const(usb_vcp_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(
                     usb_vcp_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_usb_vcp_ops_t s_bsp_usb_vcp_device_ops = {
    .super = {.deinit = bsp_usb_vcp_device_deinit},
    .transmit = bsp_usb_vcp_device_transmit,
    .receive = bsp_usb_vcp_device_receive,
    .abort = bsp_usb_vcp_device_abort,
    .get_connected = bsp_usb_vcp_device_get_connected,
    .get_busy = bsp_usb_vcp_device_get_busy};

bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_device_t *const me,
                              const bsp_usb_vcp_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) ||
        (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->super.super.is_initialized = false;
    me->driver_ops = config->driver_ops;
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    return bsp_device_init(&me->super.super, &s_bsp_usb_vcp_device_ops.super,
                           config->device_handle);
}

bsp_usb_vcp_t *bsp_usb_vcp_as_base(bsp_usb_vcp_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *const me, bsp_event_callback_t callback,
                                      void *user_context)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                  size_t data_size, uint32_t timeout_ms)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((transmit_data == NULL) || (data_size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usb_vcp_get_ops(me)->transmit(me, transmit_data, data_size, timeout_ms);
}

bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                 size_t data_capacity)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((receive_data == NULL) || (data_capacity == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usb_vcp_get_ops(me)->receive(me, receive_data, data_capacity);
}

bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *const me)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->abort(me) : status;
}

bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *const me, bool *is_connected)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if ((status == BSP_STATUS_OK) && (is_connected == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->get_connected(me, is_connected)
                                     : status;
}

bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *const me, bool *is_busy)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if ((status == BSP_STATUS_OK) && (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->get_busy(me, is_busy) : status;
}

void bsp_usb_vcp_notify(bsp_usb_vcp_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
