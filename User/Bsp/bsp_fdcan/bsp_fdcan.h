#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "bsp_can.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_fdcan bsp_fdcan_t;
    typedef struct bsp_fdcan_device bsp_fdcan_device_t;

    typedef enum
    {
        BSP_FDCAN_FORMAT_CLASSIC = 0,
        BSP_FDCAN_FORMAT_FD_NO_BRS,
        BSP_FDCAN_FORMAT_FD_BRS
    } bsp_fdcan_format_t;

    typedef struct
    {
        uint32_t identifier;
        bsp_can_id_type_t id_type;
        bsp_can_frame_type_t frame_type;
        bsp_fdcan_format_t format;
        uint8_t data_length;
        uint8_t data[64];
    } bsp_fdcan_frame_t;

    typedef struct
    {
        bool is_bus_off;
        bool is_error_passive;
        bool has_warning;
        uint8_t transmit_error_count;
        uint8_t receive_error_count;
        uint32_t last_error_code;
    } bsp_fdcan_protocol_status_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_fdcan_t *const me);
        bsp_status_t (*stop)(bsp_fdcan_t *const me);
        bsp_status_t (*configure_filter)(bsp_fdcan_t *const me,
                                         const bsp_can_filter_t *filter_config);
        bsp_status_t (*transmit)(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                 uint32_t timeout_ms);
        bsp_status_t (*receive)(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                bsp_fdcan_frame_t *frame);
        bsp_status_t (*get_protocol_status)(const bsp_fdcan_t *const me,
                                            bsp_fdcan_protocol_status_t *protocol_status);
        bsp_status_t (*get_transmit_free_level)(const bsp_fdcan_t *const me, uint32_t *free_level);
    } bsp_fdcan_ops_t;

    struct bsp_fdcan
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*start)(void *device_handle);
        bsp_status_t (*stop)(void *device_handle);
        bsp_status_t (*configure_filter)(void *device_handle,
                                         const bsp_can_filter_t *filter_config);
        bsp_status_t (*transmit)(void *device_handle, const bsp_fdcan_frame_t *frame,
                                 uint32_t timeout_ms);
        bsp_status_t (*receive)(void *device_handle, bsp_can_receive_fifo_t receive_fifo,
                                bsp_fdcan_frame_t *frame);
        bsp_status_t (*get_protocol_status)(const void *device_handle,
                                            bsp_fdcan_protocol_status_t *protocol_status);
        bsp_status_t (*get_transmit_free_level)(const void *device_handle, uint32_t *free_level);
    } bsp_fdcan_driver_ops_t;

    struct bsp_fdcan_device
    {
        bsp_fdcan_t super;
        const bsp_fdcan_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_fdcan_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_fdcan_config_t;

    bsp_status_t bsp_fdcan_init(bsp_fdcan_device_t *const me,
                                const bsp_fdcan_config_t *const config);
    bsp_fdcan_t *bsp_fdcan_as_base(bsp_fdcan_device_t *const me);
    bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t *const me, bsp_event_callback_t callback,
                                        void *user_context);
    bsp_status_t bsp_fdcan_start(bsp_fdcan_t *const me);
    bsp_status_t bsp_fdcan_stop(bsp_fdcan_t *const me);
    bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t *const me,
                                            const bsp_can_filter_t *filter_config);
    bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                    uint32_t timeout_ms);
    bsp_status_t bsp_fdcan_receive(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                   bsp_fdcan_frame_t *frame);
    bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t *const me,
                                               bsp_fdcan_protocol_status_t *protocol_status);
    bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t *const me,
                                                   uint32_t *free_level);
    void bsp_fdcan_notify(bsp_fdcan_t *const me, bsp_event_t event, bsp_status_t status,
                          size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_H */
