/** @file module_uart_comm.h
 * @brief 独立的普通 UART 固定长度通信协议。
 */
#ifndef MODULE_UART_COMM_H
#define MODULE_UART_COMM_H

#include "bsp_usart.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MODULE_UART_COMM_DATA_SIZE
#define MODULE_UART_COMM_DATA_SIZE (8U)
#endif

#if (MODULE_UART_COMM_DATA_SIZE < 1U) || (MODULE_UART_COMM_DATA_SIZE > 252U)
#error "MODULE_UART_COMM_DATA_SIZE must be in range 1..252"
#endif

#define MODULE_UART_COMM_HEADER_FIRST (0xA5U)
#define MODULE_UART_COMM_HEADER_SECOND (0x5AU)
#define MODULE_UART_COMM_FRAME_SIZE (MODULE_UART_COMM_DATA_SIZE + 3U)

    typedef enum
    {
        MODULE_UART_COMM_STATUS_OK = 0,
        MODULE_UART_COMM_STATUS_INVALID_ARGUMENT,
        MODULE_UART_COMM_STATUS_NOT_INITIALIZED,
        MODULE_UART_COMM_STATUS_TRANSPORT_ERROR,
        MODULE_UART_COMM_STATUS_BUSY,
        MODULE_UART_COMM_STATUS_INVALID_FRAME,
        MODULE_UART_COMM_STATUS_CHECKSUM_ERROR,
        MODULE_UART_COMM_STATUS_NO_DATA
    } module_uart_comm_status_t;

    typedef struct
    {
        uint8_t data[MODULE_UART_COMM_DATA_SIZE];
        uint32_t update_count;
        bool is_valid;
    } module_uart_comm_process_data_t;

    typedef struct
    {
        bsp_usart_t *usart;
        bsp_transfer_mode_t transmit_mode;
        uint32_t transmit_timeout_ms;
    } module_uart_comm_config_t;

    typedef struct
    {
        bsp_usart_t *usart;
        bsp_transfer_mode_t transmit_mode;
        uint32_t transmit_timeout_ms;
        uint8_t transmit_frame[MODULE_UART_COMM_FRAME_SIZE];
        uint8_t stream_frame[MODULE_UART_COMM_FRAME_SIZE];
        size_t stream_size;
        module_uart_comm_process_data_t received_data;
        uint32_t valid_frame_count;
        uint32_t checksum_error_count;
        bool is_initialized;
    } module_uart_comm_t;

    module_uart_comm_status_t module_uart_comm_init(
        module_uart_comm_t *me, const module_uart_comm_config_t *config);
    module_uart_comm_status_t module_uart_comm_send(
        module_uart_comm_t *me, const uint8_t *data, size_t data_size);
    module_uart_comm_status_t module_uart_comm_feed_data(
        module_uart_comm_t *me, const uint8_t *received_bytes, size_t received_size);
    module_uart_comm_status_t module_uart_comm_get_data(
        const module_uart_comm_t *me, module_uart_comm_process_data_t *process_data);
    uint8_t module_uart_comm_crc8(const uint8_t *data, size_t data_size);

#ifdef __cplusplus
}
#endif
#endif /* MODULE_UART_COMM_H */
