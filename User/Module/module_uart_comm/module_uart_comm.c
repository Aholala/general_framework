#include "module_uart_comm.h"
#include "alg_crc.h"

#include <string.h>

#define MODULE_UART_COMM_DATA_INDEX (2U)
#define MODULE_UART_COMM_CRC_INDEX (MODULE_UART_COMM_FRAME_SIZE - 1U)

static void module_uart_comm_increment_saturated(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

uint8_t module_uart_comm_crc8(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;
    return alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result)
               ? (uint8_t)result
               : 0U;
}

module_uart_comm_status_t module_uart_comm_init(
    module_uart_comm_t *me, const module_uart_comm_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) ||
        !bsp_transfer_mode_is_valid(config->transmit_mode))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->usart = config->usart;
    me->transmit_mode = config->transmit_mode;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_UART_COMM_STATUS_OK;
}

module_uart_comm_status_t module_uart_comm_send(
    module_uart_comm_t *me, const uint8_t *data, size_t data_size)
{
    bsp_status_t status;
    if ((me == NULL) || (data == NULL) || (data_size != MODULE_UART_COMM_DATA_SIZE))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    me->transmit_frame[0] = MODULE_UART_COMM_HEADER_FIRST;
    me->transmit_frame[1] = MODULE_UART_COMM_HEADER_SECOND;
    (void)memcpy(&me->transmit_frame[MODULE_UART_COMM_DATA_INDEX], data, data_size);
    me->transmit_frame[MODULE_UART_COMM_CRC_INDEX] =
        module_uart_comm_crc8(me->transmit_frame, MODULE_UART_COMM_CRC_INDEX);
    status = bsp_usart_transmit(me->usart, me->transmit_frame,
                                sizeof(me->transmit_frame), me->transmit_mode,
                                me->transmit_timeout_ms);
    if (status == BSP_STATUS_BUSY)
    {
        return MODULE_UART_COMM_STATUS_BUSY;
    }
    return (status == BSP_STATUS_OK) ? MODULE_UART_COMM_STATUS_OK
                                     : MODULE_UART_COMM_STATUS_TRANSPORT_ERROR;
}

module_uart_comm_status_t module_uart_comm_feed_data(
    module_uart_comm_t *me, const uint8_t *received_bytes, size_t received_size)
{
    bool received_valid = false;
    bool checksum_error = false;
    size_t index;
    if ((me == NULL) || ((received_bytes == NULL) && (received_size > 0U)))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    for (index = 0U; index < received_size; ++index)
    {
        const uint8_t byte = received_bytes[index];
        if (me->stream_size == 0U)
        {
            if (byte == MODULE_UART_COMM_HEADER_FIRST)
            {
                me->stream_frame[0] = byte;
                me->stream_size = 1U;
            }
            continue;
        }
        if (me->stream_size == 1U)
        {
            if (byte == MODULE_UART_COMM_HEADER_SECOND)
            {
                me->stream_frame[1] = byte;
                me->stream_size = 2U;
            }
            else if (byte != MODULE_UART_COMM_HEADER_FIRST)
            {
                me->stream_size = 0U;
            }
            continue;
        }
        me->stream_frame[me->stream_size++] = byte;
        if (me->stream_size == MODULE_UART_COMM_FRAME_SIZE)
        {
            if (me->stream_frame[MODULE_UART_COMM_CRC_INDEX] ==
                module_uart_comm_crc8(me->stream_frame, MODULE_UART_COMM_CRC_INDEX))
            {
                (void)memcpy(me->received_data.data,
                             &me->stream_frame[MODULE_UART_COMM_DATA_INDEX],
                             MODULE_UART_COMM_DATA_SIZE);
                module_uart_comm_increment_saturated(&me->received_data.update_count);
                module_uart_comm_increment_saturated(&me->valid_frame_count);
                me->received_data.is_valid = true;
                received_valid = true;
            }
            else
            {
                module_uart_comm_increment_saturated(&me->checksum_error_count);
                checksum_error = true;
            }
            me->stream_size = 0U;
        }
    }
    if (received_valid)
    {
        return MODULE_UART_COMM_STATUS_OK;
    }
    return checksum_error ? MODULE_UART_COMM_STATUS_CHECKSUM_ERROR
                          : MODULE_UART_COMM_STATUS_OK;
}

module_uart_comm_status_t module_uart_comm_get_data(
    const module_uart_comm_t *me, module_uart_comm_process_data_t *process_data)
{
    if ((me == NULL) || (process_data == NULL))
    {
        return MODULE_UART_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_UART_COMM_STATUS_NOT_INITIALIZED;
    }
    if (!me->received_data.is_valid)
    {
        return MODULE_UART_COMM_STATUS_NO_DATA;
    }
    *process_data = me->received_data;
    return MODULE_UART_COMM_STATUS_OK;
}
