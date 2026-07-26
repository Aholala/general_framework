#include "bsp_fdcan_classic_adapter.h"

#include <stddef.h>

static bsp_fdcan_classic_adapter_t *bsp_fdcan_classic_adapter_from_handle(void *const device_handle)
{
    return (bsp_fdcan_classic_adapter_t *)device_handle;
}

static bsp_status_t bsp_fdcan_classic_adapter_start(void *const device_handle)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    return bsp_fdcan_start(me->fdcan);
}

static bsp_status_t bsp_fdcan_classic_adapter_stop(void *const device_handle)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    return bsp_fdcan_stop(me->fdcan);
}

static bsp_status_t bsp_fdcan_classic_adapter_deinit(void *const device_handle)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    return bsp_fdcan_set_callback(me->fdcan, NULL, NULL);
}

static bsp_status_t
bsp_fdcan_classic_adapter_configure_filter(void *const device_handle,
                                           const bsp_can_filter_t *const filter_config)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    return bsp_fdcan_configure_filter(me->fdcan, filter_config);
}

static bsp_status_t bsp_fdcan_classic_adapter_transmit(void *const device_handle,
                                                       const bsp_can_frame_t *const can_frame,
                                                       uint32_t timeout_ms)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    bsp_fdcan_frame_t fdcan_frame = {
        .identifier = can_frame->identifier,
        .id_type = can_frame->id_type,
        .frame_type = can_frame->frame_type,
        .format = BSP_FDCAN_FORMAT_CLASSIC,
        .data_length = can_frame->data_length,
        .data = {0U},
    };
    size_t data_index;

    for (data_index = 0U; data_index < can_frame->data_length; ++data_index)
    {
        fdcan_frame.data[data_index] = can_frame->data[data_index];
    }
    return bsp_fdcan_transmit(me->fdcan, &fdcan_frame, timeout_ms);
}

static bsp_status_t bsp_fdcan_classic_adapter_receive(void *const device_handle,
                                                      bsp_can_receive_fifo_t receive_fifo,
                                                      bsp_can_frame_t *const can_frame)
{
    bsp_fdcan_classic_adapter_t *const me = bsp_fdcan_classic_adapter_from_handle(device_handle);
    bsp_fdcan_frame_t fdcan_frame;
    bsp_status_t status;
    size_t data_index;

    status = bsp_fdcan_receive(me->fdcan, receive_fifo, &fdcan_frame);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((fdcan_frame.format != BSP_FDCAN_FORMAT_CLASSIC) ||
        (fdcan_frame.data_length > sizeof(can_frame->data)))
    {
        return BSP_STATUS_UNSUPPORTED;
    }

    can_frame->identifier = fdcan_frame.identifier;
    can_frame->id_type = fdcan_frame.id_type;
    can_frame->frame_type = fdcan_frame.frame_type;
    can_frame->data_length = fdcan_frame.data_length;
    for (data_index = 0U; data_index < fdcan_frame.data_length; ++data_index)
    {
        can_frame->data[data_index] = fdcan_frame.data[data_index];
    }
    return BSP_STATUS_OK;
}

static bsp_status_t
bsp_fdcan_classic_adapter_get_transmit_free_level(const void *const device_handle,
                                                  uint32_t *const free_level)
{
    const bsp_fdcan_classic_adapter_t *const me =
        (const bsp_fdcan_classic_adapter_t *)device_handle;
    return bsp_fdcan_get_transmit_free_level(me->fdcan, free_level);
}

static void bsp_fdcan_classic_adapter_event_callback(bsp_event_t event, bsp_status_t status,
                                                     size_t transferred_size, void *user_context)
{
    bsp_fdcan_classic_adapter_t *const me = (bsp_fdcan_classic_adapter_t *)user_context;
    if (me != NULL)
    {
        bsp_can_notify(&me->super.super, event, status, transferred_size);
    }
}

static const bsp_can_driver_ops_t s_bsp_fdcan_classic_adapter_driver_ops = {
    .init = NULL,
    .deinit = bsp_fdcan_classic_adapter_deinit,
    .start = bsp_fdcan_classic_adapter_start,
    .stop = bsp_fdcan_classic_adapter_stop,
    .configure_filter = bsp_fdcan_classic_adapter_configure_filter,
    .transmit = bsp_fdcan_classic_adapter_transmit,
    .receive = bsp_fdcan_classic_adapter_receive,
    .get_tx_free_level = bsp_fdcan_classic_adapter_get_transmit_free_level,
};

bsp_status_t bsp_fdcan_classic_adapter_init(bsp_fdcan_classic_adapter_t *const me,
                                            const bsp_fdcan_classic_adapter_config_t *const config)
{
    bsp_can_config_t can_config;
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->fdcan == NULL) ||
        !bsp_device_is_initialized(&config->fdcan->super))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    me->fdcan = config->fdcan;
    can_config = (bsp_can_config_t){
        .device_handle = me,
        .driver_ops = &s_bsp_fdcan_classic_adapter_driver_ops,
        .callback = config->callback,
        .user_context = config->user_context,
    };
    status = bsp_can_init(&me->super, &can_config);
    if (status != BSP_STATUS_OK)
    {
        me->fdcan = NULL;
        return status;
    }

    status = bsp_fdcan_set_callback(config->fdcan, bsp_fdcan_classic_adapter_event_callback, me);
    if (status != BSP_STATUS_OK)
    {
        (void)bsp_device_deinit(&me->super.super.super);
        me->fdcan = NULL;
    }
    return status;
}

bsp_can_t *bsp_fdcan_classic_adapter_as_can(bsp_fdcan_classic_adapter_t *const me)
{
    return (me != NULL) ? bsp_can_as_base(&me->super) : NULL;
}
