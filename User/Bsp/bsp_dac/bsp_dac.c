#include "bsp_dac.h"

#include <math.h>
#include <stddef.h>

static bsp_dac_device_t *bsp_dac_get_device(bsp_dac_t *const dac_base)
{
    return BSP_CONTAINER_OF(dac_base, bsp_dac_device_t, super);
}

static const bsp_dac_device_t *bsp_dac_get_device_const(const bsp_dac_t *const dac_base)
{
    return BSP_CONTAINER_OF_CONST(dac_base, bsp_dac_device_t, super);
}

static const bsp_dac_ops_t *bsp_dac_get_ops(const bsp_dac_t *const dac_base)
{
    return BSP_CONTAINER_OF_CONST(dac_base->super.vptr, bsp_dac_ops_t, super);
}

static bsp_status_t bsp_dac_device_deinit(bsp_device_t *const device_base)
{
    bsp_dac_t *const dac_base = BSP_CONTAINER_OF(device_base, bsp_dac_t, super);
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

#define BSP_DAC_FORWARD(name, member)                                                              \
    static bsp_status_t name(bsp_dac_t *const dac_base)                                            \
    {                                                                                              \
        bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);                                 \
        return me->driver_ops->member(dac_base->super.device_handle, me->channel);                 \
    }

BSP_DAC_FORWARD(bsp_dac_device_start, start)
BSP_DAC_FORWARD(bsp_dac_device_stop, stop)
static bsp_status_t bsp_dac_device_stop_dma(bsp_dac_t *const dac_base)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    return (me->driver_ops->stop_dma != NULL)
               ? me->driver_ops->stop_dma(dac_base->super.device_handle,
                                          me->channel)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_dac_device_set_raw(bsp_dac_t *const dac_base, uint32_t raw_value)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    return me->driver_ops->set_raw(dac_base->super.device_handle, me->channel, raw_value);
}

static bsp_status_t bsp_dac_device_get_raw(const bsp_dac_t *const dac_base, uint32_t *raw_value)
{
    const bsp_dac_device_t *const me = bsp_dac_get_device_const(dac_base);
    return me->driver_ops->get_raw(dac_base->super.device_handle, me->channel, raw_value);
}

static bsp_status_t bsp_dac_device_start_dma(bsp_dac_t *const dac_base,
                                             const uint32_t *sample_buffer, size_t sample_count)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    return (me->driver_ops->start_dma != NULL)
               ? me->driver_ops->start_dma(dac_base->super.device_handle,
                                           me->channel, sample_buffer,
                                           sample_count)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_dac_ops_t s_bsp_dac_device_ops = {.super = {.deinit = bsp_dac_device_deinit},
                                                   .start = bsp_dac_device_start,
                                                   .stop = bsp_dac_device_stop,
                                                   .set_raw = bsp_dac_device_set_raw,
                                                   .get_raw = bsp_dac_device_get_raw,
                                                   .start_dma = bsp_dac_device_start_dma,
                                                   .stop_dma = bsp_dac_device_stop_dma};

static bsp_status_t bsp_dac_validate(const bsp_dac_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_dac_init(bsp_dac_device_t *const me, const bsp_dac_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) ||
        (config->driver_ops->start == NULL) || (config->driver_ops->stop == NULL) ||
        (config->driver_ops->set_raw == NULL) ||
        (config->driver_ops->get_raw == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->super.super.is_initialized = false;
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    me->super.reference_voltage_v = config->reference_voltage_v;
    me->super.maximum_raw_value = (1UL << config->resolution_bits) - 1UL;
    return bsp_device_init(&me->super.super, &s_bsp_dac_device_ops.super, config->device_handle);
}

bsp_dac_t *bsp_dac_as_base(bsp_dac_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_dac_set_callback(bsp_dac_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    bsp_status_t status = bsp_dac_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

#define BSP_DAC_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_dac_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_dac_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_dac_get_ops(me)->member(me) : status;               \
    }

BSP_DAC_PUBLIC_ACTION(bsp_dac_start, start)
BSP_DAC_PUBLIC_ACTION(bsp_dac_stop, stop)
BSP_DAC_PUBLIC_ACTION(bsp_dac_stop_dma, stop_dma)

bsp_status_t bsp_dac_set_raw(bsp_dac_t *const me, uint32_t raw_value)
{
    bsp_status_t status = bsp_dac_validate(me);
    if ((status == BSP_STATUS_OK) && (raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return (status == BSP_STATUS_OK) ? bsp_dac_get_ops(me)->set_raw(me, raw_value) : status;
}

bsp_status_t bsp_dac_get_raw(const bsp_dac_t *const me, uint32_t *raw_value)
{
    bsp_status_t status = bsp_dac_validate(me);
    if ((status == BSP_STATUS_OK) && (raw_value == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_dac_get_ops(me)->get_raw(me, raw_value);
    if ((status == BSP_STATUS_OK) && (*raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

bsp_status_t bsp_dac_set_normalized(bsp_dac_t *const me, float normalized_value)
{
    const bsp_status_t status = bsp_dac_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!isfinite(normalized_value) || (normalized_value < 0.0F) ||
        (normalized_value > 1.0F))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_dac_set_raw(me, (uint32_t)((float)me->maximum_raw_value * normalized_value + 0.5F));
}

bsp_status_t bsp_dac_set_voltage(bsp_dac_t *const me, float voltage_v)
{
    const bsp_status_t status = bsp_dac_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!isfinite(voltage_v) || (voltage_v < 0.0F) ||
        (voltage_v > me->reference_voltage_v))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_dac_set_normalized(me, voltage_v / me->reference_voltage_v);
}

bsp_status_t bsp_dac_start_dma(bsp_dac_t *const me, const uint32_t *sample_buffer,
                               size_t sample_count)
{
    bsp_status_t status = bsp_dac_validate(me);
    size_t sample_index;
    if ((status == BSP_STATUS_OK) && ((sample_buffer == NULL) || (sample_count == 0U)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status == BSP_STATUS_OK)
    {
        for (sample_index = 0U; sample_index < sample_count;
             ++sample_index)
        {
            if (sample_buffer[sample_index] > me->maximum_raw_value)
            {
                return BSP_STATUS_OUT_OF_RANGE;
            }
        }
    }
    return (status == BSP_STATUS_OK)
               ? bsp_dac_get_ops(me)->start_dma(me, sample_buffer, sample_count)
               : status;
}

void bsp_dac_notify(bsp_dac_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
