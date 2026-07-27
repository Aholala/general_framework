#include "bsp_adc.h"

#include <math.h>
#include <stddef.h>

static bsp_adc_device_t *bsp_adc_get_device(bsp_adc_t *const adc_base)
{
    return BSP_CONTAINER_OF(adc_base, bsp_adc_device_t, super);
}

static const bsp_adc_ops_t *bsp_adc_get_ops(const bsp_adc_t *const adc_base)
{
    return BSP_CONTAINER_OF_CONST(adc_base->super.vptr, bsp_adc_ops_t, super);
}

static bsp_status_t bsp_adc_device_deinit(bsp_device_t *const device_base)
{
    bsp_adc_t *const adc_base = BSP_CONTAINER_OF(device_base, bsp_adc_t, super);
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

#define BSP_ADC_FORWARD(name, member)                                                              \
    static bsp_status_t name(bsp_adc_t *const adc_base)                                            \
    {                                                                                              \
        bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);                                 \
        return me->driver_ops->member(adc_base->super.device_handle, me->channel);                 \
    }

BSP_ADC_FORWARD(bsp_adc_device_start, start)
BSP_ADC_FORWARD(bsp_adc_device_stop, stop)
static bsp_status_t bsp_adc_device_stop_dma(bsp_adc_t *const adc_base)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return (me->driver_ops->stop_dma != NULL)
               ? me->driver_ops->stop_dma(adc_base->super.device_handle, me->channel)
               : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_adc_device_calibrate(bsp_adc_t *const adc_base)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return me->driver_ops->calibrate(adc_base->super.device_handle);
}

static bsp_status_t bsp_adc_device_read_raw(bsp_adc_t *const adc_base, uint32_t *raw_value,
                                            uint32_t timeout_ms)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return me->driver_ops->read_raw(adc_base->super.device_handle, me->channel, raw_value,
                                    timeout_ms);
}

static bsp_status_t bsp_adc_device_start_dma(bsp_adc_t *const adc_base, uint32_t *sample_buffer,
                                             size_t sample_count)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return (me->driver_ops->start_dma != NULL)
               ? me->driver_ops->start_dma(adc_base->super.device_handle, me->channel,
                                           sample_buffer, sample_count)
               : BSP_STATUS_UNSUPPORTED;
}

static const bsp_adc_ops_t s_bsp_adc_device_ops = {.super = {.deinit = bsp_adc_device_deinit},
                                                   .start = bsp_adc_device_start,
                                                   .stop = bsp_adc_device_stop,
                                                   .calibrate = bsp_adc_device_calibrate,
                                                   .read_raw = bsp_adc_device_read_raw,
                                                   .start_dma = bsp_adc_device_start_dma,
                                                   .stop_dma = bsp_adc_device_stop_dma};

static bsp_status_t bsp_adc_validate(const bsp_adc_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_adc_init(bsp_adc_device_t *const me, const bsp_adc_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->calibrate == NULL) ||
        (config->driver_ops->read_raw == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_adc_device_ops.super, config->device_handle);
}

bsp_adc_t *bsp_adc_as_base(bsp_adc_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

bsp_status_t bsp_adc_set_callback(bsp_adc_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    bsp_status_t status = bsp_adc_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

#define BSP_ADC_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_adc_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_adc_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_adc_get_ops(me)->member(me) : status;               \
    }

BSP_ADC_PUBLIC_ACTION(bsp_adc_start, start)
BSP_ADC_PUBLIC_ACTION(bsp_adc_stop, stop)
BSP_ADC_PUBLIC_ACTION(bsp_adc_calibrate, calibrate)
BSP_ADC_PUBLIC_ACTION(bsp_adc_stop_dma, stop_dma)

bsp_status_t bsp_adc_read_raw(bsp_adc_t *const me, uint32_t *raw_value, uint32_t timeout_ms)
{
    bsp_status_t status = bsp_adc_validate(me);
    if ((status == BSP_STATUS_OK) && (raw_value == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_adc_get_ops(me)->read_raw(me, raw_value, timeout_ms);
    if ((status == BSP_STATUS_OK) && (*raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

bsp_status_t bsp_adc_read_normalized(bsp_adc_t *const me, float *normalized_value,
                                     uint32_t timeout_ms)
{
    uint32_t raw_value;
    bsp_status_t status;
    if (normalized_value == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_adc_read_raw(me, &raw_value, timeout_ms);
    if (status == BSP_STATUS_OK)
    {
        *normalized_value = (float)raw_value / (float)me->maximum_raw_value;
    }
    return status;
}

bsp_status_t bsp_adc_read_voltage(bsp_adc_t *const me, float *voltage_v, uint32_t timeout_ms)
{
    float normalized_value;
    bsp_status_t status;
    if (voltage_v == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_adc_read_normalized(me, &normalized_value, timeout_ms);
    if (status == BSP_STATUS_OK)
    {
        *voltage_v = normalized_value * me->reference_voltage_v;
    }
    return status;
}

bsp_status_t bsp_adc_start_dma(bsp_adc_t *const me, uint32_t *sample_buffer, size_t sample_count)
{
    bsp_status_t status = bsp_adc_validate(me);
    if ((status == BSP_STATUS_OK) && ((sample_buffer == NULL) || (sample_count == 0U)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK)
               ? bsp_adc_get_ops(me)->start_dma(me, sample_buffer, sample_count)
               : status;
}

void bsp_adc_notify(bsp_adc_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
