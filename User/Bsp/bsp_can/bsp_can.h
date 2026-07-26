#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_can bsp_can_t;
    typedef struct bsp_can_device bsp_can_device_t;

    typedef enum
    {
        BSP_CAN_ID_STANDARD = 0,
        BSP_CAN_ID_EXTENDED
    } bsp_can_id_type_t;
    typedef enum
    {
        BSP_CAN_FRAME_DATA = 0,
        BSP_CAN_FRAME_REMOTE
    } bsp_can_frame_type_t;
    typedef enum
    {
        BSP_CAN_RX_FIFO_0 = 0,
        BSP_CAN_RX_FIFO_1
    } bsp_can_receive_fifo_t;

    typedef struct
    {
        uint32_t identifier;
        bsp_can_id_type_t id_type;
        bsp_can_frame_type_t frame_type;
        uint8_t data_length;
        uint8_t data[8];
    } bsp_can_frame_t;

    typedef struct
    {
        uint32_t identifier;
        uint32_t mask;
        bsp_can_id_type_t id_type;
        bsp_can_receive_fifo_t receive_fifo;
        uint32_t filter_index;
    } bsp_can_filter_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_can_t *const);
        bsp_status_t (*stop)(bsp_can_t *const);
        bsp_status_t (*configure_filter)(bsp_can_t *const, const bsp_can_filter_t *);
        bsp_status_t (*transmit)(bsp_can_t *const, const bsp_can_frame_t *, uint32_t);
        bsp_status_t (*receive)(bsp_can_t *const, bsp_can_receive_fifo_t, bsp_can_frame_t *);
        bsp_status_t (*get_tx_free_level)(const bsp_can_t *const, uint32_t *);
    } bsp_can_ops_t;

    struct bsp_can
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *);
        bsp_status_t (*deinit)(void *);
        bsp_status_t (*start)(void *);
        bsp_status_t (*stop)(void *);
        bsp_status_t (*configure_filter)(void *, const bsp_can_filter_t *);
        bsp_status_t (*transmit)(void *, const bsp_can_frame_t *, uint32_t);
        bsp_status_t (*receive)(void *, bsp_can_receive_fifo_t, bsp_can_frame_t *);
        bsp_status_t (*get_tx_free_level)(const void *, uint32_t *);
    } bsp_can_driver_ops_t;

    struct bsp_can_device
    {
        bsp_can_t super;
        const bsp_can_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_can_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_can_config_t;

    bsp_status_t bsp_can_init(bsp_can_device_t *const me, const bsp_can_config_t *const config);
    bsp_can_t *bsp_can_as_base(bsp_can_device_t *const me);
    bsp_status_t bsp_can_set_callback(bsp_can_t *const me, bsp_event_callback_t callback,
                                      void *user_context);
    bsp_status_t bsp_can_start(bsp_can_t *const me);
    bsp_status_t bsp_can_stop(bsp_can_t *const me);
    bsp_status_t bsp_can_configure_filter(bsp_can_t *const me, const bsp_can_filter_t *filter);
    bsp_status_t bsp_can_transmit(bsp_can_t *const me, const bsp_can_frame_t *frame,
                                  uint32_t timeout_ms);
    bsp_status_t bsp_can_receive(bsp_can_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                 bsp_can_frame_t *frame);
    bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *const me, uint32_t *free_level);
    void bsp_can_notify(bsp_can_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
