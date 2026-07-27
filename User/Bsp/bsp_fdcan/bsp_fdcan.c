#include "bsp_fdcan.h"

#include <stddef.h>

static bsp_fdcan_device_t *bsp_fdcan_get_device(bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF(fdcan_base, bsp_fdcan_device_t, super);
}

static const bsp_fdcan_device_t *bsp_fdcan_get_device_const(const bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF_CONST(fdcan_base, bsp_fdcan_device_t, super);
}

static const bsp_fdcan_ops_t *bsp_fdcan_get_ops(const bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF_CONST(fdcan_base->super.vptr, bsp_fdcan_ops_t, super);
}

static bool bsp_fdcan_is_data_length_valid(uint8_t data_length)
{
    return (data_length <= 8U) || (data_length == 12U) || (data_length == 16U) ||
           (data_length == 20U) || (data_length == 24U) || (data_length == 32U) ||
           (data_length == 48U) || (data_length == 64U);
}

static bool bsp_fdcan_is_frame_valid(const bsp_fdcan_frame_t *const frame)
{
    if ((frame == NULL) || !bsp_fdcan_is_data_length_valid(frame->data_length))
    {
        return false;
    }
    if (((frame->id_type != BSP_CAN_ID_STANDARD) && (frame->id_type != BSP_CAN_ID_EXTENDED)) ||
        ((frame->frame_type != BSP_CAN_FRAME_DATA) &&
         (frame->frame_type != BSP_CAN_FRAME_REMOTE)) ||
        ((frame->format != BSP_FDCAN_FORMAT_CLASSIC) &&
         (frame->format != BSP_FDCAN_FORMAT_FD_NO_BRS) &&
         (frame->format != BSP_FDCAN_FORMAT_FD_BRS)))
    {
        return false;
    }
    if ((frame->format == BSP_FDCAN_FORMAT_CLASSIC) && (frame->data_length > 8U))
    {
        return false;
    }
    if ((frame->id_type == BSP_CAN_ID_STANDARD) && (frame->identifier > 0x7FFU))
    {
        return false;
    }
    return !((frame->id_type == BSP_CAN_ID_EXTENDED) && (frame->identifier > 0x1FFFFFFFU));
}

static bsp_status_t bsp_fdcan_device_deinit(bsp_device_t *const device_base)
{
    bsp_fdcan_t *const fdcan_base = BSP_CONTAINER_OF(device_base, bsp_fdcan_t, super);
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);

    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    return me->driver_ops->deinit(device_base->device_handle);
}

#define BSP_FDCAN_FORWARD_MUTABLE(name, member)                                                    \
    static bsp_status_t name(bsp_fdcan_t *const fdcan_base)                                        \
    {                                                                                              \
        bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);                           \
        return me->driver_ops->member(fdcan_base->super.device_handle);                            \
    }

BSP_FDCAN_FORWARD_MUTABLE(bsp_fdcan_device_start, start)
BSP_FDCAN_FORWARD_MUTABLE(bsp_fdcan_device_stop, stop)

static bsp_status_t bsp_fdcan_device_configure_filter(bsp_fdcan_t *const fdcan_base,
                                                      const bsp_can_filter_t *filter_config)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->configure_filter(fdcan_base->super.device_handle, filter_config);
}

static bsp_status_t bsp_fdcan_device_transmit(bsp_fdcan_t *const fdcan_base,
                                              const bsp_fdcan_frame_t *frame, uint32_t timeout_ms)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->transmit(fdcan_base->super.device_handle, frame, timeout_ms);
}

static bsp_status_t bsp_fdcan_device_receive(bsp_fdcan_t *const fdcan_base,
                                             bsp_can_receive_fifo_t receive_fifo,
                                             bsp_fdcan_frame_t *frame)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->receive(fdcan_base->super.device_handle, receive_fifo, frame);
}

static bsp_status_t
bsp_fdcan_device_get_protocol_status(const bsp_fdcan_t *const fdcan_base,
                                     bsp_fdcan_protocol_status_t *protocol_status)
{
    const bsp_fdcan_device_t *const me = bsp_fdcan_get_device_const(fdcan_base);
    return me->driver_ops->get_protocol_status(fdcan_base->super.device_handle, protocol_status);
}

static bsp_status_t bsp_fdcan_device_get_transmit_free_level(const bsp_fdcan_t *const fdcan_base,
                                                             uint32_t *free_level)
{
    const bsp_fdcan_device_t *const me = bsp_fdcan_get_device_const(fdcan_base);
    return me->driver_ops->get_transmit_free_level(fdcan_base->super.device_handle, free_level);
}

static const bsp_fdcan_ops_t s_bsp_fdcan_device_ops = {
    .super = {.deinit = bsp_fdcan_device_deinit},
    .start = bsp_fdcan_device_start,
    .stop = bsp_fdcan_device_stop,
    .configure_filter = bsp_fdcan_device_configure_filter,
    .transmit = bsp_fdcan_device_transmit,
    .receive = bsp_fdcan_device_receive,
    .get_protocol_status = bsp_fdcan_device_get_protocol_status,
    .get_transmit_free_level = bsp_fdcan_device_get_transmit_free_level};

bsp_status_t bsp_fdcan_init(bsp_fdcan_device_t *const me, const bsp_fdcan_config_t *const config)
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
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    }
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    return bsp_device_init(&me->super.super, &s_bsp_fdcan_device_ops.super, config->device_handle);
}

bsp_fdcan_t *bsp_fdcan_as_base(bsp_fdcan_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

static bsp_status_t bsp_fdcan_validate(const bsp_fdcan_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t *const me, bsp_event_callback_t callback,
                                    void *user_context)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

bsp_status_t bsp_fdcan_start(bsp_fdcan_t *const me)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_fdcan_get_ops(me)->start(me) : status;
}

bsp_status_t bsp_fdcan_stop(bsp_fdcan_t *const me)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_fdcan_get_ops(me)->stop(me) : status;
}

bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t *const me,
                                        const bsp_can_filter_t *filter_config)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((filter_config == NULL) ||
        ((filter_config->id_type != BSP_CAN_ID_STANDARD) &&
         (filter_config->id_type != BSP_CAN_ID_EXTENDED)) ||
        ((filter_config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter_config->receive_fifo != BSP_CAN_RX_FIFO_1)) ||
        ((filter_config->id_type == BSP_CAN_ID_STANDARD) &&
         ((filter_config->identifier > 0x7FFU) || (filter_config->mask > 0x7FFU))) ||
        ((filter_config->id_type == BSP_CAN_ID_EXTENDED) &&
         ((filter_config->identifier > 0x1FFFFFFFU) || (filter_config->mask > 0x1FFFFFFFU))))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_fdcan_get_ops(me)->configure_filter(me, filter_config);
}

bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                uint32_t timeout_ms)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (!bsp_fdcan_is_frame_valid(frame))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_fdcan_get_ops(me)->transmit(me, frame, timeout_ms);
}

bsp_status_t bsp_fdcan_receive(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                               bsp_fdcan_frame_t *frame)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if ((frame == NULL) ||
        ((receive_fifo != BSP_CAN_RX_FIFO_0) && (receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    status = bsp_fdcan_get_ops(me)->receive(me, receive_fifo, frame);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    return bsp_fdcan_is_frame_valid(frame) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t *const me,
                                           bsp_fdcan_protocol_status_t *protocol_status)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (protocol_status == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_fdcan_get_device_const(me)->driver_ops->get_protocol_status == NULL)
    {
        return BSP_STATUS_UNSUPPORTED;
    }
    return bsp_fdcan_get_ops(me)->get_protocol_status(me, protocol_status);
}

bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t *const me, uint32_t *free_level)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (free_level == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_fdcan_get_device_const(me)->driver_ops->get_transmit_free_level == NULL)
    {
        return BSP_STATUS_UNSUPPORTED;
    }
    return bsp_fdcan_get_ops(me)->get_transmit_free_level(me, free_level);
}

void bsp_fdcan_notify(bsp_fdcan_t *const me, bsp_event_t event, bsp_status_t status,
                      size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
