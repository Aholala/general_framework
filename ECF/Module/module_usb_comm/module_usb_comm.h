/**
 * @file module_usb_comm.h
 * @brief USB CDC 视觉双向固定帧协议。
 * @note 当前只定义 mode 和 ID；其余数据通过宏定义扩展区预留。
 */
#ifndef MODULE_USB_COMM_H
#define MODULE_USB_COMM_H

#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 尚未确定的扩展数据字节数；默认不占用任何空间。 */
#ifndef MODULE_USB_COMM_EXTRA_DATA_SIZE
#define MODULE_USB_COMM_EXTRA_DATA_SIZE (0U)
#endif

#if (MODULE_USB_COMM_EXTRA_DATA_SIZE > 250U)
#error "MODULE_USB_COMM_EXTRA_DATA_SIZE must be in range 0..250"
#endif

#define MODULE_USB_COMM_HEADER_FIRST (0xA5U)
#define MODULE_USB_COMM_HEADER_SECOND (0x5AU)
#define MODULE_USB_COMM_MINIMUM_ID (1U)
#define MODULE_USB_COMM_MAXIMUM_ID (7U)
#define MODULE_USB_COMM_FRAME_SIZE (MODULE_USB_COMM_EXTRA_DATA_SIZE + 5U)

    typedef enum
    {
        MODULE_USB_COMM_STATUS_OK = 0,
        MODULE_USB_COMM_STATUS_INVALID_ARGUMENT,
        MODULE_USB_COMM_STATUS_NOT_INITIALIZED,
        MODULE_USB_COMM_STATUS_TRANSPORT_ERROR,
        MODULE_USB_COMM_STATUS_BUSY,
        MODULE_USB_COMM_STATUS_INVALID_FRAME,
        MODULE_USB_COMM_STATUS_CHECKSUM_ERROR,
        MODULE_USB_COMM_STATUS_NO_DATA
    } module_usb_comm_status_t;

    /** @brief 当前 USB 协议数据；收发共用。 */
    typedef struct
    {
        uint8_t mode;
        uint8_t id; // 范围 1~7
#if MODULE_USB_COMM_EXTRA_DATA_SIZE > 0U
        uint8_t extra_data[MODULE_USB_COMM_EXTRA_DATA_SIZE];
#endif
    } module_usb_comm_data_t;

    typedef struct
    {
        module_usb_comm_data_t data;
        uint32_t update_count;
        bool is_valid;
    } module_usb_comm_process_data_t;

    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;
        uint32_t transmit_timeout_ms;
    } module_usb_comm_config_t;

    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;
        uint32_t transmit_timeout_ms;
        uint8_t transmit_frame[MODULE_USB_COMM_FRAME_SIZE];
        uint8_t stream_frame[MODULE_USB_COMM_FRAME_SIZE];
        size_t stream_size;
        module_usb_comm_process_data_t received_data;
        uint32_t valid_frame_count;
        uint32_t invalid_frame_count;
        uint32_t checksum_error_count;
        bool is_initialized;
    } module_usb_comm_t;

    module_usb_comm_status_t module_usb_comm_init(
        module_usb_comm_t *me, const module_usb_comm_config_t *config);
    module_usb_comm_status_t module_usb_comm_send(
        module_usb_comm_t *me, const module_usb_comm_data_t *data);
    module_usb_comm_status_t module_usb_comm_feed_data(
        module_usb_comm_t *me, const uint8_t *received_bytes, size_t received_size);
    module_usb_comm_status_t module_usb_comm_get_data(
        const module_usb_comm_t *me, module_usb_comm_process_data_t *process_data);
    uint8_t module_usb_comm_crc8(const uint8_t *data, size_t data_size);

#ifdef __cplusplus
}
#endif
#endif /* MODULE_USB_COMM_H */
