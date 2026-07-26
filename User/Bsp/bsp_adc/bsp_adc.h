#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_adc bsp_adc_t;
    typedef struct bsp_adc_device bsp_adc_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_adc_t *const me);
        bsp_status_t (*stop)(bsp_adc_t *const me);
        bsp_status_t (*calibrate)(bsp_adc_t *const me);
        bsp_status_t (*read_raw)(bsp_adc_t *const me, uint32_t *raw_value, uint32_t timeout_ms);
        bsp_status_t (*start_dma)(bsp_adc_t *const me, uint32_t *sample_buffer,
                                  size_t sample_count);
        bsp_status_t (*stop_dma)(bsp_adc_t *const me);
    } bsp_adc_ops_t;

    struct bsp_adc
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
        float reference_voltage_v;
        uint32_t maximum_raw_value;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle, uint32_t channel);
        bsp_status_t (*deinit)(void *device_handle, uint32_t channel);
        bsp_status_t (*start)(void *device_handle, uint32_t channel);
        bsp_status_t (*stop)(void *device_handle, uint32_t channel);
        bsp_status_t (*calibrate)(void *device_handle);
        bsp_status_t (*read_raw)(void *device_handle, uint32_t channel, uint32_t *raw_value,
                                 uint32_t timeout_ms);
        bsp_status_t (*start_dma)(void *device_handle, uint32_t channel, uint32_t *sample_buffer,
                                  size_t sample_count);
        bsp_status_t (*stop_dma)(void *device_handle, uint32_t channel);
    } bsp_adc_driver_ops_t;

    struct bsp_adc_device
    {
        bsp_adc_t super;
        const bsp_adc_driver_ops_t *driver_ops;
        uint32_t channel;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_adc_driver_ops_t *driver_ops;
        uint32_t channel;
        uint8_t resolution_bits;
        float reference_voltage_v;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_adc_config_t;

    bsp_status_t bsp_adc_init(bsp_adc_device_t *const me, const bsp_adc_config_t *const config);
    bsp_adc_t *bsp_adc_as_base(bsp_adc_device_t *const me);
    bsp_status_t bsp_adc_set_callback(bsp_adc_t *const me, bsp_event_callback_t callback,
                                      void *user_context);
    bsp_status_t bsp_adc_start(bsp_adc_t *const me);
    bsp_status_t bsp_adc_stop(bsp_adc_t *const me);
    bsp_status_t bsp_adc_calibrate(bsp_adc_t *const me);
    bsp_status_t bsp_adc_read_raw(bsp_adc_t *const me, uint32_t *raw_value, uint32_t timeout_ms);
    bsp_status_t bsp_adc_read_normalized(bsp_adc_t *const me, float *normalized_value,
                                         uint32_t timeout_ms);
    bsp_status_t bsp_adc_read_voltage(bsp_adc_t *const me, float *voltage_v, uint32_t timeout_ms);
    bsp_status_t bsp_adc_start_dma(bsp_adc_t *const me, uint32_t *sample_buffer,
                                   size_t sample_count);
    bsp_status_t bsp_adc_stop_dma(bsp_adc_t *const me);
    void bsp_adc_notify(bsp_adc_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
