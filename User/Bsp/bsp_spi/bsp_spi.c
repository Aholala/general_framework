#include "bsp_spi.h"

#include <stddef.h>

static bsp_spi_device_t *bsp_spi_get_device(bsp_spi_t *const spi_base)
{
    return BSP_CONTAINER_OF(spi_base, bsp_spi_device_t, super);
}

static const bsp_spi_device_t *bsp_spi_get_device_const(const bsp_spi_t *const spi_base)
{
    return BSP_CONTAINER_OF_CONST(spi_base, bsp_spi_device_t, super);
}

static const bsp_spi_ops_t *bsp_spi_get_ops(const bsp_spi_t *const spi_base)
{
    return BSP_CONTAINER_OF_CONST(spi_base->super.vptr, bsp_spi_ops_t, super);
}

static bsp_status_t bsp_spi_device_deinit(bsp_device_t *const device_base)
{
    bsp_spi_t *const spi_base = BSP_CONTAINER_OF(device_base, bsp_spi_t, super);
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_spi_device_transmit(bsp_spi_t *const spi_base, const uint8_t *transmit_data,
                                            size_t data_size, bsp_transfer_mode_t mode,
                                            uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return me->driver_ops->transmit(spi_base->super.device_handle, transmit_data, data_size, mode,
                                    timeout_ms);
}

static bsp_status_t bsp_spi_device_receive(bsp_spi_t *const spi_base, uint8_t *receive_data,
                                           size_t data_size, bsp_transfer_mode_t mode,
                                           uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return me->driver_ops->receive(spi_base->super.device_handle, receive_data, data_size, mode,
                                   timeout_ms);
}

static bsp_status_t bsp_spi_device_exchange(bsp_spi_t *const spi_base, const uint8_t *transmit_data,
                                            uint8_t *receive_data, size_t data_size,
                                            bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return (me->driver_ops->exchange != NULL)
               ? me->driver_ops->exchange(spi_base->super.device_handle, transmit_data,
                                          receive_data, data_size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_spi_device_abort(bsp_spi_t *const spi_base)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(spi_base->super.device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_spi_device_get_busy(const bsp_spi_t *const spi_base, bool *is_busy)
{
    const bsp_spi_device_t *const me = bsp_spi_get_device_const(spi_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(spi_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_spi_ops_t s_bsp_spi_device_ops = {.super = {.deinit = bsp_spi_device_deinit},
                                                   .transmit = bsp_spi_device_transmit,
                                                   .receive = bsp_spi_device_receive,
                                                   .exchange = bsp_spi_device_exchange,
                                                   .abort = bsp_spi_device_abort,
                                                   .get_busy = bsp_spi_device_get_busy};

bsp_status_t bsp_spi_init(bsp_spi_device_t *const me, const bsp_spi_config_t *const config)
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
    return bsp_device_init(&me->super.super, &s_bsp_spi_device_ops.super, config->device_handle);
}

bsp_spi_t *bsp_spi_as_base(bsp_spi_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

static bsp_status_t bsp_spi_validate(const bsp_spi_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_spi_set_callback(bsp_spi_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_spi_transmit(bsp_spi_t *const me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_spi_get_ops(me)->transmit(me, data, size, mode, timeout_ms);
}

bsp_status_t bsp_spi_receive(bsp_spi_t *const me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_spi_get_ops(me)->receive(me, data, size, mode, timeout_ms);
}

bsp_status_t bsp_spi_exchange(bsp_spi_t *const me, const uint8_t *transmit_data,
                              uint8_t *receive_data, size_t size, bsp_transfer_mode_t mode,
                              uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((transmit_data == NULL) || (receive_data == NULL) || (size == 0U) ||
        !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_spi_get_ops(me)->exchange(me, transmit_data, receive_data, size, mode, timeout_ms);
}

bsp_status_t bsp_spi_abort(bsp_spi_t *const me)
{
    const bsp_status_t status = bsp_spi_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_spi_get_ops(me)->abort(me) : status;
}

bsp_status_t bsp_spi_get_busy(const bsp_spi_t *const me, bool *is_busy)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_spi_get_ops(me)->get_busy(me, is_busy);
}

void bsp_spi_notify(bsp_spi_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
