#include "bsp_can.h"

#include <stddef.h>

static bsp_can_device_t *bsp_can_get_device(bsp_can_t *const can_base)
{
    return BSP_CONTAINER_OF(can_base, bsp_can_device_t, super);
}
static const bsp_can_device_t *bsp_can_get_device_const(const bsp_can_t *const can_base)
{
    return BSP_CONTAINER_OF_CONST(can_base, bsp_can_device_t, super);
}
static const bsp_can_ops_t *bsp_can_get_ops(const bsp_can_t *const can_base)
{
    return BSP_CONTAINER_OF_CONST(can_base->super.vptr, bsp_can_ops_t, super);
}

static bsp_status_t bsp_can_device_deinit(bsp_device_t *const device_base)
{
    bsp_can_t *const can_base = BSP_CONTAINER_OF(device_base, bsp_can_t, super);
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}
static bsp_status_t bsp_can_device_start(bsp_can_t *const can_base)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->start(can_base->super.device_handle);
}
static bsp_status_t bsp_can_device_stop(bsp_can_t *const can_base)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->stop(can_base->super.device_handle);
}
static bsp_status_t bsp_can_device_configure_filter(bsp_can_t *const can_base,
                                                    const bsp_can_filter_t *filter_config)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->configure_filter(can_base->super.device_handle, filter_config);
}
static bsp_status_t bsp_can_device_transmit(bsp_can_t *const can_base, const bsp_can_frame_t *frame,
                                            uint32_t timeout_ms)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->transmit(can_base->super.device_handle, frame, timeout_ms);
}
static bsp_status_t bsp_can_device_receive(bsp_can_t *const can_base,
                                           bsp_can_receive_fifo_t receive_fifo,
                                           bsp_can_frame_t *frame)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->receive(can_base->super.device_handle, receive_fifo, frame);
}
static bsp_status_t bsp_can_device_get_transmit_free_level(const bsp_can_t *const can_base,
                                                           uint32_t *free_level)
{
    const bsp_can_device_t *const me = bsp_can_get_device_const(can_base);
    return me->driver_ops->get_tx_free_level(can_base->super.device_handle, free_level);
}

static const bsp_can_ops_t s_bsp_can_device_ops = {
    .super = {.deinit = bsp_can_device_deinit},
    .start = bsp_can_device_start,
    .stop = bsp_can_device_stop,
    .configure_filter = bsp_can_device_configure_filter,
    .transmit = bsp_can_device_transmit,
    .receive = bsp_can_device_receive,
    .get_tx_free_level = bsp_can_device_get_transmit_free_level};

bsp_status_t bsp_can_init(bsp_can_device_t *const me, const bsp_can_config_t *const config)
{
    bsp_status_t status;
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->configure_filter == NULL) ||
        (config->driver_ops->transmit == NULL) || (config->driver_ops->receive == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_can_device_ops.super, config->device_handle);
}

bsp_can_t *bsp_can_as_base(bsp_can_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}
static bsp_status_t bsp_can_validate(const bsp_can_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}
static bool bsp_can_is_id_valid(uint32_t id, bsp_can_id_type_t type)
{
    return (type == BSP_CAN_ID_STANDARD) ? (id <= 0x7FFU)
                                         : ((type == BSP_CAN_ID_EXTENDED) && (id <= 0x1FFFFFFFU));
}

static bool bsp_can_is_frame_valid(const bsp_can_frame_t *frame)
{
    return (frame != NULL) && (frame->data_length <= 8U) &&
           ((frame->frame_type == BSP_CAN_FRAME_DATA) ||
            (frame->frame_type == BSP_CAN_FRAME_REMOTE)) &&
           bsp_can_is_id_valid(frame->identifier, frame->id_type);
}

bsp_status_t bsp_can_set_callback(bsp_can_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}
bsp_status_t bsp_can_start(bsp_can_t *const me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_can_get_ops(me)->start(me) : status;
}
bsp_status_t bsp_can_stop(bsp_can_t *const me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_can_get_ops(me)->stop(me) : status;
}
bsp_status_t bsp_can_configure_filter(bsp_can_t *const me, const bsp_can_filter_t *filter_config)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((filter_config == NULL) ||
        !bsp_can_is_id_valid(filter_config->identifier, filter_config->id_type) ||
        !bsp_can_is_id_valid(filter_config->mask, filter_config->id_type) ||
        ((filter_config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter_config->receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_can_get_ops(me)->configure_filter(me, filter_config);
}
bsp_status_t bsp_can_transmit(bsp_can_t *const me, const bsp_can_frame_t *frame,
                              uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!bsp_can_is_frame_valid(frame))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_can_get_ops(me)->transmit(me, frame, timeout_ms);
}
bsp_status_t bsp_can_receive(bsp_can_t *const me, bsp_can_receive_fifo_t receive_fifo,
                             bsp_can_frame_t *frame)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((frame == NULL) ||
        ((receive_fifo != BSP_CAN_RX_FIFO_0) && (receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    {
        const bsp_status_t receive_status =
            bsp_can_get_ops(me)->receive(me, receive_fifo, frame);
        if (receive_status != BSP_STATUS_OK)
        {
            return receive_status;
        }
    }
    return bsp_can_is_frame_valid(frame) ? BSP_STATUS_OK
                                         : BSP_STATUS_IO_ERROR;
}
bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *const me, uint32_t *free_level)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (free_level == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_can_get_device_const(me)->driver_ops->get_tx_free_level == NULL)
    {
        return BSP_STATUS_UNSUPPORTED;
    }
    return bsp_can_get_ops(me)->get_tx_free_level(me, free_level);
}
void bsp_can_notify(bsp_can_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
