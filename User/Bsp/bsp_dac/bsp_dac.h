#ifndef BSP_DAC_H
#define BSP_DAC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_dac bsp_dac_t;
    typedef struct bsp_dac_device bsp_dac_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_dac_t *const me);
        bsp_status_t (*stop)(bsp_dac_t *const me);
        bsp_status_t (*set_raw)(bsp_dac_t *const me, uint32_t raw_value);
        bsp_status_t (*get_raw)(const bsp_dac_t *const me, uint32_t *raw_value);
        bsp_status_t (*start_dma)(bsp_dac_t *const me, const uint32_t *sample_buffer,
                                  size_t sample_count);
        bsp_status_t (*stop_dma)(bsp_dac_t *const me);
    } bsp_dac_ops_t;

    struct bsp_dac
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
        bsp_status_t (*set_raw)(void *device_handle, uint32_t channel, uint32_t raw_value);
        bsp_status_t (*get_raw)(const void *device_handle, uint32_t channel, uint32_t *raw_value);
        bsp_status_t (*start_dma)(void *device_handle, uint32_t channel,
                                  const uint32_t *sample_buffer, size_t sample_count);
        bsp_status_t (*stop_dma)(void *device_handle, uint32_t channel);
    } bsp_dac_driver_ops_t;

    struct bsp_dac_device
    {
        bsp_dac_t super;
        const bsp_dac_driver_ops_t *driver_ops;
        uint32_t channel;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_dac_driver_ops_t *driver_ops;
        uint32_t channel;
        uint8_t resolution_bits;
        float reference_voltage_v;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_dac_config_t;

    bsp_status_t bsp_dac_init(bsp_dac_device_t *const me, const bsp_dac_config_t *const config);
    bsp_dac_t *bsp_dac_as_base(bsp_dac_device_t *const me);
    bsp_status_t bsp_dac_set_callback(bsp_dac_t *const me, bsp_event_callback_t callback,
                                      void *user_context);
    bsp_status_t bsp_dac_start(bsp_dac_t *const me);
    bsp_status_t bsp_dac_stop(bsp_dac_t *const me);
    bsp_status_t bsp_dac_set_raw(bsp_dac_t *const me, uint32_t raw_value);
    bsp_status_t bsp_dac_get_raw(const bsp_dac_t *const me, uint32_t *raw_value);
    bsp_status_t bsp_dac_set_normalized(bsp_dac_t *const me, float normalized_value);
    bsp_status_t bsp_dac_set_voltage(bsp_dac_t *const me, float voltage_v);
    bsp_status_t bsp_dac_start_dma(bsp_dac_t *const me, const uint32_t *sample_buffer,
                                   size_t sample_count);
    bsp_status_t bsp_dac_stop_dma(bsp_dac_t *const me);
    void bsp_dac_notify(bsp_dac_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DAC_H */
