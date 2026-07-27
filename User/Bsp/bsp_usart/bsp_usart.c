#include "bsp_usart.h"

#include <stddef.h>

static bsp_usart_device_t *bsp_usart_get_device(bsp_usart_t *const me)
{
    return BSP_CONTAINER_OF(me, bsp_usart_device_t, super);
}

static const bsp_usart_device_t *bsp_usart_get_device_const(const bsp_usart_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_usart_device_t, super);
}

static const bsp_usart_ops_t *bsp_usart_get_ops(const bsp_usart_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_usart_ops_t, super);
}

static bsp_status_t bsp_usart_device_deinit(bsp_device_t *const device_base)
{
    bsp_usart_t *const usart_base = BSP_CONTAINER_OF(device_base, bsp_usart_t, super);
    bsp_usart_device_t *const me = bsp_usart_get_device(usart_base);
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_usart_device_transmit(bsp_usart_t *const base, const uint8_t *data,
                                              size_t size, bsp_transfer_mode_t mode,
                                              uint32_t timeout_ms)
{
    bsp_usart_device_t *const me = bsp_usart_get_device(base);
    return me->driver_ops->transmit(base->super.device_handle, data, size, mode, timeout_ms);
}

static bsp_status_t bsp_usart_device_receive(bsp_usart_t *const base, uint8_t *data, size_t size,
                                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_usart_device_t *const me = bsp_usart_get_device(base);
    return me->driver_ops->receive(base->super.device_handle, data, size, mode, timeout_ms);
}

static bsp_status_t bsp_usart_device_receive_to_idle(bsp_usart_t *const base, uint8_t *data,
                                                     size_t capacity, bsp_transfer_mode_t mode,
                                                     uint32_t timeout_ms)
{
    bsp_usart_device_t *const me = bsp_usart_get_device(base);
    return (me->driver_ops->receive_to_idle != NULL)
               ? me->driver_ops->receive_to_idle(base->super.device_handle, data, capacity, mode,
                                                 timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_usart_device_abort(bsp_usart_t *const base)
{
    bsp_usart_device_t *const me = bsp_usart_get_device(base);
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(base->super.device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_usart_device_get_busy(const bsp_usart_t *const base, bool *is_busy)
{
    const bsp_usart_device_t *const me = bsp_usart_get_device_const(base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_usart_ops_t s_bsp_usart_device_ops = {.super = {.deinit = bsp_usart_device_deinit},
                                                       .transmit = bsp_usart_device_transmit,
                                                       .receive = bsp_usart_device_receive,
                                                       .receive_to_idle =
                                                           bsp_usart_device_receive_to_idle,
                                                       .abort = bsp_usart_device_abort,
                                                       .get_busy = bsp_usart_device_get_busy};

bsp_status_t bsp_usart_init(bsp_usart_device_t *const me, const bsp_usart_config_t *const config)
{
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_usart_device_ops.super, config->device_handle);
}

bsp_usart_t *bsp_usart_as_base(bsp_usart_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

static bsp_status_t bsp_usart_validate(const bsp_usart_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_usart_set_callback(bsp_usart_t *const me, bsp_event_callback_t callback,
                                    void *user_context)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_usart_transmit(bsp_usart_t *const me, const uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usart_get_ops(me)->transmit(me, data, size, mode, timeout_ms);
}

bsp_status_t bsp_usart_receive(bsp_usart_t *const me, uint8_t *data, size_t size,
                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usart_get_ops(me)->receive(me, data, size, mode, timeout_ms);
}

bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *const me, uint8_t *data, size_t capacity,
                                       bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (capacity == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usart_get_ops(me)->receive_to_idle(me, data, capacity, mode, timeout_ms);
}

bsp_status_t bsp_usart_abort(bsp_usart_t *const me)
{
    const bsp_status_t status = bsp_usart_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_usart_get_ops(me)->abort(me) : status;
}

bsp_status_t bsp_usart_get_busy(const bsp_usart_t *const me, bool *is_busy)
{
    const bsp_status_t status = bsp_usart_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_usart_get_ops(me)->get_busy(me, is_busy);
}

void bsp_usart_notify(bsp_usart_t *const me, bsp_event_t event, bsp_status_t status,
                      size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
