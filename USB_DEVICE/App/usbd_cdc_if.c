#include "usbd_cdc_if.h"

static uint8_t usb_cdc_receive_buffer[APP_RX_DATA_SIZE];
static uint8_t usb_cdc_transmit_buffer[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t usb_cdc_init(void);
static int8_t usb_cdc_deinit(void);
static int8_t usb_cdc_control(uint8_t command, uint8_t *command_data,
                              uint16_t command_data_size);
static int8_t usb_cdc_receive(uint8_t *receive_data, uint32_t *receive_size);
static int8_t usb_cdc_transmit_complete(uint8_t *transmit_data,
                                        uint32_t *transmit_size,
                                        uint8_t endpoint_number);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    usb_cdc_init,
    usb_cdc_deinit,
    usb_cdc_control,
    usb_cdc_receive,
    usb_cdc_transmit_complete,
};

__weak void usb_cdc_receive_callback(const uint8_t *receive_data,
                                     uint32_t receive_size)
{
  (void)receive_data;
  (void)receive_size;
}

static int8_t usb_cdc_init(void)
{
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_cdc_transmit_buffer, 0U);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_cdc_receive_buffer);
  return (int8_t)USBD_OK;
}

static int8_t usb_cdc_deinit(void)
{
  return (int8_t)USBD_OK;
}

static int8_t usb_cdc_control(uint8_t command, uint8_t *command_data,
                              uint16_t command_data_size)
{
  (void)command;
  (void)command_data;
  (void)command_data_size;
  return (int8_t)USBD_OK;
}

static int8_t usb_cdc_receive(uint8_t *receive_data, uint32_t *receive_size)
{
  if ((receive_data == NULL) || (receive_size == NULL))
  {
    return (int8_t)USBD_FAIL;
  }

  usb_cdc_receive_callback(receive_data, *receive_size);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_cdc_receive_buffer);
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

uint8_t usb_cdc_transmit(uint8_t *transmit_data, uint16_t transmit_size)
{
  USBD_CDC_HandleTypeDef *class_data;

  if ((transmit_data == NULL) || (transmit_size == 0U))
  {
    return USBD_FAIL;
  }

  class_data = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if ((class_data == NULL) || (class_data->TxState != 0U))
  {
    return USBD_BUSY;
  }

  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, transmit_data, transmit_size);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

static int8_t usb_cdc_transmit_complete(uint8_t *transmit_data,
                                        uint32_t *transmit_size,
                                        uint8_t endpoint_number)
{
  (void)transmit_data;
  (void)transmit_size;
  (void)endpoint_number;
  return (int8_t)USBD_OK;
}
