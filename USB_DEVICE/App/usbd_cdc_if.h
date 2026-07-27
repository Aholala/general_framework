#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE (512U)
#define APP_TX_DATA_SIZE (512U)

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t usb_cdc_transmit(uint8_t *transmit_data, uint16_t transmit_size);
void usb_cdc_receive_callback(const uint8_t *receive_data,
                              uint32_t receive_size);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CDC_IF_H */
