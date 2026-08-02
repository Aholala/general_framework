/**
 * @file module_nrf24l01_ace_link.h
 * @brief 基于 nRF24L01 固定载荷的 ACE 点对点协议。
 */

#ifndef MODULE_NRF24L01_ACE_LINK_H
#define MODULE_NRF24L01_ACE_LINK_H

#include "module_nrf24l01.h"

#define MODULE_NRF24L01_ACE_LINK_ADDRESS_SIZE (3U)
#define MODULE_NRF24L01_ACE_LINK_HEADER_FIRST (0xA5U)
#define MODULE_NRF24L01_ACE_LINK_HEADER_SECOND (0x5AU)
#define MODULE_NRF24L01_ACE_LINK_OVERHEAD_SIZE (7U)
#define MODULE_NRF24L01_ACE_LINK_MAXIMUM_DATA_SIZE                                                 \
    (MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE - MODULE_NRF24L01_ACE_LINK_OVERHEAD_SIZE)

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_NRF24L01_ACE_LINK_STATUS_OK = 0,
        MODULE_NRF24L01_ACE_LINK_STATUS_NO_DATA,
        MODULE_NRF24L01_ACE_LINK_STATUS_BUSY,
        MODULE_NRF24L01_ACE_LINK_STATUS_MAXIMUM_RETRANSMIT,
        MODULE_NRF24L01_ACE_LINK_STATUS_INVALID_ARGUMENT,
        MODULE_NRF24L01_ACE_LINK_STATUS_NOT_INITIALIZED,
        MODULE_NRF24L01_ACE_LINK_STATUS_NOT_STARTED,
        MODULE_NRF24L01_ACE_LINK_STATUS_TRANSPORT_ERROR,
        MODULE_NRF24L01_ACE_LINK_STATUS_INVALID_FRAME,
        MODULE_NRF24L01_ACE_LINK_STATUS_CHECKSUM_ERROR
    } module_nrf24l01_ace_link_status_t;

    typedef struct
    {
        uint8_t message_type;
        uint8_t sequence;
        uint8_t data_size;
        uint8_t data[MODULE_NRF24L01_ACE_LINK_MAXIMUM_DATA_SIZE];
    } module_nrf24l01_ace_link_packet_t;

    typedef struct
    {
        module_nrf24l01_t *radio;
        uint8_t next_transmit_sequence;
        bool is_initialized;
    } module_nrf24l01_ace_link_t;

    /** @brief 默认链路地址，ASCII {'A', 'C', 'E'}。 */
    extern const uint8_t module_nrf24l01_ace_link_address[MODULE_NRF24L01_ACE_LINK_ADDRESS_SIZE];

    module_nrf24l01_ace_link_status_t module_nrf24l01_ace_link_init(module_nrf24l01_ace_link_t *me,
                                                                    module_nrf24l01_t *radio);

    module_nrf24l01_ace_link_status_t module_nrf24l01_ace_link_send(module_nrf24l01_ace_link_t *me,
                                                                    uint8_t message_type,
                                                                    const uint8_t *packet_data,
                                                                    size_t data_size);

    module_nrf24l01_ace_link_status_t
    module_nrf24l01_ace_link_receive(module_nrf24l01_ace_link_t *me,
                                     module_nrf24l01_ace_link_packet_t *packet,
                                     uint8_t *pipe_index);

    uint8_t module_nrf24l01_ace_link_get_next_sequence(const module_nrf24l01_ace_link_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NRF24L01_ACE_LINK_H */
