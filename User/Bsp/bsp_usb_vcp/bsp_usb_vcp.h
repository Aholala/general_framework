#ifndef BSP_USB_VCP_H
#define BSP_USB_VCP_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_usb_vcp bsp_usb_vcp_t;
    typedef struct bsp_usb_vcp_device bsp_usb_vcp_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*transmit)(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                 size_t data_size, uint32_t timeout_ms);
        bsp_status_t (*receive)(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                size_t data_capacity);
        bsp_status_t (*abort)(bsp_usb_vcp_t *const me);
        bsp_status_t (*get_connected)(const bsp_usb_vcp_t *const me, bool *is_connected);
        bsp_status_t (*get_busy)(const bsp_usb_vcp_t *const me, bool *is_busy);
    } bsp_usb_vcp_ops_t;

    struct bsp_usb_vcp
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*transmit)(void *device_handle, const uint8_t *transmit_data,
                                 size_t data_size, uint32_t timeout_ms);
        bsp_status_t (*receive)(void *device_handle, uint8_t *receive_data, size_t data_capacity);
        bsp_status_t (*abort)(void *device_handle);
        bsp_status_t (*get_connected)(const void *device_handle, bool *is_connected);
        bsp_status_t (*get_busy)(const void *device_handle, bool *is_busy);
    } bsp_usb_vcp_driver_ops_t;

    struct bsp_usb_vcp_device
    {
        bsp_usb_vcp_t super;
        const bsp_usb_vcp_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_usb_vcp_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_usb_vcp_config_t;

    bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_device_t *const me,
                                  const bsp_usb_vcp_config_t *const config);
    bsp_usb_vcp_t *bsp_usb_vcp_as_base(bsp_usb_vcp_device_t *const me);
    bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *const me, bsp_event_callback_t callback,
                                          void *user_context);
    bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                      size_t data_size, uint32_t timeout_ms);
    bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                     size_t data_capacity);
    bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *const me);
    bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *const me, bool *is_connected);
    bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *const me, bool *is_busy);
    void bsp_usb_vcp_notify(bsp_usb_vcp_t *const me, bsp_event_t event, bsp_status_t status,
                            size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USB_VCP_H */
