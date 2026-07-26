#ifndef BSP_USART_H
#define BSP_USART_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_usart bsp_usart_t;
    typedef struct bsp_usart_device bsp_usart_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*transmit)(bsp_usart_t *const me, const uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*receive)(bsp_usart_t *const me, uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*receive_to_idle)(bsp_usart_t *const me, uint8_t *data, size_t capacity,
                                        bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*abort)(bsp_usart_t *const me);
        bsp_status_t (*get_busy)(const bsp_usart_t *const me, bool *is_busy);
    } bsp_usart_ops_t;

    struct bsp_usart
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*transmit)(void *device_handle, const uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*receive)(void *device_handle, uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*receive_to_idle)(void *device_handle, uint8_t *data, size_t capacity,
                                        bsp_transfer_mode_t mode, uint32_t timeout_ms);
        bsp_status_t (*abort)(void *device_handle);
        bsp_status_t (*get_busy)(const void *device_handle, bool *is_busy);
    } bsp_usart_driver_ops_t;

    struct bsp_usart_device
    {
        bsp_usart_t super;
        const bsp_usart_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_usart_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_usart_config_t;

    bsp_status_t bsp_usart_init(bsp_usart_device_t *const me,
                                const bsp_usart_config_t *const config);
    bsp_usart_t *bsp_usart_as_base(bsp_usart_device_t *const me);
    bsp_status_t bsp_usart_set_callback(bsp_usart_t *const me, bsp_event_callback_t callback,
                                        void *user_context);
    bsp_status_t bsp_usart_transmit(bsp_usart_t *const me, const uint8_t *data, size_t size,
                                    bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_usart_receive(bsp_usart_t *const me, uint8_t *data, size_t size,
                                   bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *const me, uint8_t *data, size_t capacity,
                                           bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_usart_abort(bsp_usart_t *const me);
    bsp_status_t bsp_usart_get_busy(const bsp_usart_t *const me, bool *is_busy);
    void bsp_usart_notify(bsp_usart_t *const me, bsp_event_t event, bsp_status_t status,
                          size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */
