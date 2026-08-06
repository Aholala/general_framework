/**
 * @file module_nrf24l01_link.c
 * @brief ACE 点对点协议的封包、序号和 CRC 校验实现。
 */

#include "module_nrf24l01_link.h"

#include "alg_crc.h"

#include <stddef.h>
#include <string.h>

const uint8_t module_nrf24l01_link_address[MODULE_NRF24L01_LINK_ADDRESS_SIZE] = {'A', 'C',
                                                                                         'E'};

static module_nrf24l01_link_status_t
module_nrf24l01_link_map_radio_status(module_nrf24l01_status_t status)
{
    switch (status)
    {
    case MODULE_NRF24L01_STATUS_OK:
        return MODULE_NRF24L01_LINK_STATUS_OK;
    case MODULE_NRF24L01_STATUS_NO_DATA:
        return MODULE_NRF24L01_LINK_STATUS_NO_DATA;
    case MODULE_NRF24L01_STATUS_BUSY:
        return MODULE_NRF24L01_LINK_STATUS_BUSY;
    case MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT:
        return MODULE_NRF24L01_LINK_STATUS_MAXIMUM_RETRANSMIT;
    case MODULE_NRF24L01_STATUS_INVALID_ARGUMENT:
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    case MODULE_NRF24L01_STATUS_NOT_INITIALIZED:
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    case MODULE_NRF24L01_STATUS_NOT_STARTED:
        return MODULE_NRF24L01_LINK_STATUS_NOT_STARTED;
    case MODULE_NRF24L01_STATUS_TRANSPORT_ERROR:
    case MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND:
    default:
        return MODULE_NRF24L01_LINK_STATUS_TRANSPORT_ERROR;
    }
}

static uint16_t module_nrf24l01_link_crc16(const uint8_t *data, size_t data_size)
{
    uint32_t result = 0U;

    return alg_crc_calculate(&alg_crc16_ccitt_false_config, data, data_size, &result)
               ? (uint16_t)result
               : 0U;
}

module_nrf24l01_link_status_t module_nrf24l01_link_init(module_nrf24l01_link_t *me,
                                                                module_nrf24l01_t *radio)
{
    if ((me == NULL) || (radio == NULL) || !module_device_is_initialized(&radio->super) ||
        (radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    *me = (module_nrf24l01_link_t){
        .radio = radio,
        .next_transmit_sequence = 0U,
        .is_initialized = true,
    };
    return MODULE_NRF24L01_LINK_STATUS_OK;
}

module_nrf24l01_link_status_t module_nrf24l01_link_send(module_nrf24l01_link_t *me,
                                                                uint8_t message_type,
                                                                const uint8_t *packet_data,
                                                                size_t data_size)
{
    uint8_t radio_payload[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE] = {0U};
    uint16_t checksum;
    module_nrf24l01_status_t radio_status;

    if ((me == NULL) || ((packet_data == NULL) && (data_size != 0U)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->radio == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    }
    if ((me->radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE) ||
        (data_size > (size_t)(me->radio->payload_size - MODULE_NRF24L01_LINK_OVERHEAD_SIZE)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    radio_payload[0] = MODULE_NRF24L01_LINK_HEADER_FIRST;
    radio_payload[1] = MODULE_NRF24L01_LINK_HEADER_SECOND;
    radio_payload[2] = message_type;
    radio_payload[3] = me->next_transmit_sequence;
    radio_payload[4] = (uint8_t)data_size;
    if (data_size != 0U)
    {
        memcpy(&radio_payload[5], packet_data, data_size);
    }
    checksum = module_nrf24l01_link_crc16(radio_payload, 5U + data_size);
    radio_payload[5U + data_size] = (uint8_t)checksum;
    radio_payload[6U + data_size] = (uint8_t)(checksum >> 8U);

    radio_status = module_nrf24l01_transmit(me->radio, radio_payload, me->radio->payload_size);
    if (radio_status == MODULE_NRF24L01_STATUS_OK)
    {
        ++me->next_transmit_sequence;
    }
    return module_nrf24l01_link_map_radio_status(radio_status);
}

module_nrf24l01_link_status_t
module_nrf24l01_link_receive(module_nrf24l01_link_t *me,
                                 module_nrf24l01_link_packet_t *packet, uint8_t *pipe_index)
{
    uint8_t radio_payload[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE];
    size_t checksum_offset;
    uint16_t received_checksum;
    module_nrf24l01_status_t radio_status;

    if ((me == NULL) || (packet == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->radio == NULL))
    {
        return MODULE_NRF24L01_LINK_STATUS_NOT_INITIALIZED;
    }
    if (me->radio->payload_size < MODULE_NRF24L01_LINK_OVERHEAD_SIZE)
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_ARGUMENT;
    }

    radio_status =
        module_nrf24l01_receive(me->radio, radio_payload, sizeof(radio_payload), pipe_index);
    if (radio_status != MODULE_NRF24L01_STATUS_OK)
    {
        return module_nrf24l01_link_map_radio_status(radio_status);
    }
    if ((radio_payload[0] != MODULE_NRF24L01_LINK_HEADER_FIRST) ||
        (radio_payload[1] != MODULE_NRF24L01_LINK_HEADER_SECOND) ||
        (radio_payload[4] >
         (uint8_t)(me->radio->payload_size - MODULE_NRF24L01_LINK_OVERHEAD_SIZE)))
    {
        return MODULE_NRF24L01_LINK_STATUS_INVALID_FRAME;
    }

    checksum_offset = 5U + radio_payload[4];
    received_checksum = (uint16_t)radio_payload[checksum_offset] |
                        (uint16_t)((uint16_t)radio_payload[checksum_offset + 1U] << 8U);
    if (received_checksum != module_nrf24l01_link_crc16(radio_payload, checksum_offset))
    {
        return MODULE_NRF24L01_LINK_STATUS_CHECKSUM_ERROR;
    }

    packet->message_type = radio_payload[2];
    packet->sequence = radio_payload[3];
    packet->data_size = radio_payload[4];
    memset(packet->data, 0, sizeof(packet->data));
    if (packet->data_size != 0U)
    {
        memcpy(packet->data, &radio_payload[5], packet->data_size);
    }
    return MODULE_NRF24L01_LINK_STATUS_OK;
}

uint8_t module_nrf24l01_link_get_next_sequence(const module_nrf24l01_link_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->next_transmit_sequence : 0U;
}
