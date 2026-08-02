#include "module_usb_comm.h"
#include "alg_crc.h"

#include <string.h>

#define MODULE_USB_COMM_MODE_INDEX (2U)
#define MODULE_USB_COMM_ID_INDEX (3U)
#define MODULE_USB_COMM_EXTRA_DATA_INDEX (4U)
#define MODULE_USB_COMM_CRC_INDEX (MODULE_USB_COMM_FRAME_SIZE - 1U)

static void module_usb_comm_increment_saturated(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

uint8_t module_usb_comm_crc8(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;
    return alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result)
               ? (uint8_t)result
               : 0U;
}

static void module_usb_comm_encode(uint8_t *frame, const module_usb_comm_data_t *data)
{
    frame[0] = MODULE_USB_COMM_HEADER_FIRST;
    frame[1] = MODULE_USB_COMM_HEADER_SECOND;
    frame[MODULE_USB_COMM_MODE_INDEX] = data->mode;
    frame[MODULE_USB_COMM_ID_INDEX] = data->id;
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
    (void)memcpy(&frame[MODULE_USB_COMM_EXTRA_DATA_INDEX], data->extra_data,
                 MODULE_USB_COMM_EXTRA_DATA_SIZE);
#endif
    frame[MODULE_USB_COMM_CRC_INDEX] = module_usb_comm_crc8(frame, MODULE_USB_COMM_CRC_INDEX);
}

static void module_usb_comm_decode(const uint8_t *frame, module_usb_comm_data_t *data)
{
    data->mode = frame[MODULE_USB_COMM_MODE_INDEX];
    data->id = frame[MODULE_USB_COMM_ID_INDEX];
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
    (void)memcpy(data->extra_data, &frame[MODULE_USB_COMM_EXTRA_DATA_INDEX],
                 MODULE_USB_COMM_EXTRA_DATA_SIZE);
#endif
}

module_usb_comm_status_t module_usb_comm_init(
    module_usb_comm_t *me, const module_usb_comm_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usb_vcp == NULL) ||
        !bsp_device_is_initialized(&config->usb_vcp->super))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->usb_vcp = config->usb_vcp;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_USB_COMM_STATUS_OK;
}

module_usb_comm_status_t module_usb_comm_send(
    module_usb_comm_t *me, const module_usb_comm_data_t *data)
{
    bool is_busy;
    if ((me == NULL) || (data == NULL) ||
        (data->id < MODULE_USB_COMM_MINIMUM_ID) ||
        (data->id > MODULE_USB_COMM_MAXIMUM_ID))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    if (bsp_usb_vcp_get_busy(me->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_USB_COMM_STATUS_TRANSPORT_ERROR;
    }
    if (is_busy)
    {
        return MODULE_USB_COMM_STATUS_BUSY;
    }
    module_usb_comm_encode(me->transmit_frame, data);
    return (bsp_usb_vcp_transmit(me->usb_vcp, me->transmit_frame,
                                 sizeof(me->transmit_frame),
                                 me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_USB_COMM_STATUS_OK
               : MODULE_USB_COMM_STATUS_TRANSPORT_ERROR;
}

module_usb_comm_status_t module_usb_comm_feed_data(
    module_usb_comm_t *me, const uint8_t *received_bytes, size_t received_size)
{
    bool received_valid = false;
    bool checksum_error = false;
    bool invalid_frame = false;
    size_t index;
    if ((me == NULL) || ((received_bytes == NULL) && (received_size > 0U)))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    for (index = 0U; index < received_size; ++index)
    {
        const uint8_t byte = received_bytes[index];
        if (me->stream_size == 0U)
        {
            if (byte == MODULE_USB_COMM_HEADER_FIRST)
            {
                me->stream_frame[0] = byte;
                me->stream_size = 1U;
            }
            continue;
        }
        if (me->stream_size == 1U)
        {
            if (byte == MODULE_USB_COMM_HEADER_SECOND)
            {
                me->stream_frame[1] = byte;
                me->stream_size = 2U;
            }
            else if (byte != MODULE_USB_COMM_HEADER_FIRST)
            {
                me->stream_size = 0U;
            }
            continue;
        }
        me->stream_frame[me->stream_size++] = byte;
        if (me->stream_size == MODULE_USB_COMM_FRAME_SIZE)
        {
            const bool id_is_valid =
                (me->stream_frame[MODULE_USB_COMM_ID_INDEX] >= MODULE_USB_COMM_MINIMUM_ID) &&
                (me->stream_frame[MODULE_USB_COMM_ID_INDEX] <= MODULE_USB_COMM_MAXIMUM_ID);
            const bool crc_is_valid =
                me->stream_frame[MODULE_USB_COMM_CRC_INDEX] ==
                module_usb_comm_crc8(me->stream_frame, MODULE_USB_COMM_CRC_INDEX);
            if (id_is_valid && crc_is_valid)
            {
                module_usb_comm_decode(me->stream_frame, &me->received_data.data);
                module_usb_comm_increment_saturated(&me->received_data.update_count);
                module_usb_comm_increment_saturated(&me->valid_frame_count);
                me->received_data.is_valid = true;
                received_valid = true;
            }
            else if (!crc_is_valid)
            {
                module_usb_comm_increment_saturated(&me->checksum_error_count);
                checksum_error = true;
            }
            else
            {
                module_usb_comm_increment_saturated(&me->invalid_frame_count);
                invalid_frame = true;
            }
            me->stream_size = 0U;
        }
    }
    if (received_valid)
    {
        return MODULE_USB_COMM_STATUS_OK;
    }
    if (checksum_error)
    {
        return MODULE_USB_COMM_STATUS_CHECKSUM_ERROR;
    }
    return invalid_frame ? MODULE_USB_COMM_STATUS_INVALID_FRAME
                         : MODULE_USB_COMM_STATUS_OK;
}

module_usb_comm_status_t module_usb_comm_get_data(
    const module_usb_comm_t *me, module_usb_comm_process_data_t *process_data)
{
    if ((me == NULL) || (process_data == NULL))
    {
        return MODULE_USB_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_USB_COMM_STATUS_NOT_INITIALIZED;
    }
    if (!me->received_data.is_valid)
    {
        return MODULE_USB_COMM_STATUS_NO_DATA;
    }
    *process_data = me->received_data;
    return MODULE_USB_COMM_STATUS_OK;
}
