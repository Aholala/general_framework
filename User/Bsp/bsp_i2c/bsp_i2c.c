#include "bsp_i2c.h"

#include <stddef.h>

static bsp_i2c_device_t *bsp_i2c_get_device(bsp_i2c_t *const i2c_base)
{
    return BSP_CONTAINER_OF(i2c_base, bsp_i2c_device_t, super);
}

static const bsp_i2c_device_t *bsp_i2c_get_device_const(const bsp_i2c_t *const i2c_base)
{
    return BSP_CONTAINER_OF_CONST(i2c_base, bsp_i2c_device_t, super);
}

static const bsp_i2c_ops_t *bsp_i2c_get_ops(const bsp_i2c_t *const i2c_base)
{
    return BSP_CONTAINER_OF_CONST(i2c_base->super.vptr, bsp_i2c_ops_t, super);
}

static bsp_status_t bsp_i2c_device_deinit(bsp_device_t *const device_base)
{
    bsp_i2c_t *const i2c_base = BSP_CONTAINER_OF(device_base, bsp_i2c_t, super);
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

static bsp_status_t bsp_i2c_device_transmit(bsp_i2c_t *const i2c_base, uint16_t address_7bit,
                                            const uint8_t *transmit_data, size_t data_size,
                                            bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return me->driver_ops->transmit(i2c_base->super.device_handle, address_7bit, transmit_data,
                                    data_size, mode, timeout_ms);
}

static bsp_status_t bsp_i2c_device_receive(bsp_i2c_t *const i2c_base, uint16_t address_7bit,
                                           uint8_t *receive_data, size_t data_size,
                                           bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return me->driver_ops->receive(i2c_base->super.device_handle, address_7bit, receive_data,
                                   data_size, mode, timeout_ms);
}

static bsp_status_t bsp_i2c_device_memory_write(bsp_i2c_t *const i2c_base, uint16_t address,
                                                uint16_t memory,
                                                bsp_i2c_memory_address_size_t address_size,
                                                const uint8_t *data, size_t size,
                                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->memory_write != NULL)
               ? me->driver_ops->memory_write(i2c_base->super.device_handle, address, memory,
                                              address_size, data, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_i2c_device_memory_read(bsp_i2c_t *const i2c_base, uint16_t address,
                                               uint16_t memory,
                                               bsp_i2c_memory_address_size_t address_size,
                                               uint8_t *data, size_t size, bsp_transfer_mode_t mode,
                                               uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->memory_read != NULL)
               ? me->driver_ops->memory_read(i2c_base->super.device_handle, address, memory,
                                             address_size, data, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_i2c_device_is_ready(bsp_i2c_t *const i2c_base, uint16_t address,
                                            uint32_t trials, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->is_device_ready != NULL)
               ? me->driver_ops->is_device_ready(i2c_base->super.device_handle, address, trials,
                                                 timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_i2c_device_abort(bsp_i2c_t *const i2c_base, uint16_t address_7bit)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->abort != NULL)
               ? me->driver_ops->abort(i2c_base->super.device_handle, address_7bit)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_i2c_device_get_busy(const bsp_i2c_t *const i2c_base, bool *is_busy)
{
    const bsp_i2c_device_t *const me = bsp_i2c_get_device_const(i2c_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(i2c_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_i2c_ops_t s_bsp_i2c_device_ops = {.super = {.deinit = bsp_i2c_device_deinit},
                                                   .transmit = bsp_i2c_device_transmit,
                                                   .receive = bsp_i2c_device_receive,
                                                   .memory_write = bsp_i2c_device_memory_write,
                                                   .memory_read = bsp_i2c_device_memory_read,
                                                   .is_device_ready = bsp_i2c_device_is_ready,
                                                   .abort = bsp_i2c_device_abort,
                                                   .get_busy = bsp_i2c_device_get_busy};

bsp_status_t bsp_i2c_init(bsp_i2c_device_t *const me, const bsp_i2c_config_t *const config)
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
    return bsp_device_init(&me->super.super, &s_bsp_i2c_device_ops.super, config->device_handle);
}

bsp_i2c_t *bsp_i2c_as_base(bsp_i2c_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

static bsp_status_t bsp_i2c_validate(const bsp_i2c_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

static bsp_status_t bsp_i2c_validate_transfer(const bsp_i2c_t *const me, uint16_t address,
                                              const void *data, size_t size)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (address <= 0x7FU) ? BSP_STATUS_OK : BSP_STATUS_OUT_OF_RANGE;
}

bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_i2c_transmit(bsp_i2c_t *const me, uint16_t address, const uint8_t *data,
                              size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if ((status == BSP_STATUS_OK) && !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK)
               ? bsp_i2c_get_ops(me)->transmit(me, address, data, size, mode, timeout_ms)
               : status;
}

bsp_status_t bsp_i2c_receive(bsp_i2c_t *const me, uint16_t address, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if ((status == BSP_STATUS_OK) && !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK)
               ? bsp_i2c_get_ops(me)->receive(me, address, data, size, mode, timeout_ms)
               : status;
}

static bool bsp_i2c_is_address_size_valid(bsp_i2c_memory_address_size_t size)
{
    return (size == BSP_I2C_MEMORY_ADDRESS_8_BIT) || (size == BSP_I2C_MEMORY_ADDRESS_16_BIT);
}

bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *const me, uint16_t address, uint16_t memory,
                                  bsp_i2c_memory_address_size_t address_size, const uint8_t *data,
                                  size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_i2c_is_address_size_valid(address_size))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_i2c_get_ops(me)->memory_write(me, address, memory, address_size, data, size, mode,
                                             timeout_ms);
}

bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *const me, uint16_t address, uint16_t memory,
                                 bsp_i2c_memory_address_size_t address_size, uint8_t *data,
                                 size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!bsp_i2c_is_address_size_valid(address_size))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_i2c_get_ops(me)->memory_read(me, address, memory, address_size, data, size, mode,
                                            timeout_ms);
}

bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *const me, uint16_t address, uint32_t trials,
                                     uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((address > 0x7FU) || (trials == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_i2c_get_ops(me)->is_device_ready(me, address, trials, timeout_ms);
}

bsp_status_t bsp_i2c_abort(bsp_i2c_t *const me, uint16_t address)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (address > 0x7FU)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_i2c_get_ops(me)->abort(me, address);
}

bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *const me, bool *is_busy)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_i2c_get_ops(me)->get_busy(me, is_busy);
}

void bsp_i2c_notify(bsp_i2c_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
