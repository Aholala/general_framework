# module_nrf24l01 — nRF24L01 无线收发

SPI 驱动 + ACE 链路协议。固定载荷、自动应答、自动重发。

## 用法

```c
module_nrf24l01_t radio;
module_nrf24l01_config_t cfg = {
    .spi = spi_ptr,
    .chip_enable_gpio  = &ce_pin,
    .chip_select_gpio  = &csn_pin,
    .channel = 100, .address_size = 3, .payload_size = 16,
    .link_address = ace_link_address,
    .data_rate = MODULE_NRF24L01_DATA_RATE_2_MBPS,
    .automatic_acknowledge_enabled = true,
    .delay_us = dwt_delay_us,
};
module_nrf24l01_init(&radio, &cfg);
module_nrf24l01_start(&radio);

// 发送
module_nrf24l01_transmit(&radio, payload, 16);
module_nrf24l01_status_t rc;
do { rc = module_nrf24l01_poll_transmit(&radio); }
while (rc == MODULE_NRF24L01_STATUS_BUSY);

// 接收
uint8_t rx[32]; uint8_t pipe;
rc = module_nrf24l01_receive(&radio, rx, sizeof(rx), &pipe);

module_nrf24l01_stop(&radio);
```

## ACE 链路

`module_nrf24l01_link_t` 封装地址、序号、CRC16 校验。提供 `link_send/link_receive` 高层接口。默认使用 3 字节公共链路地址 `module_nrf24l01_link_address`。
