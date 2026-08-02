# UART 独立固定长度协议

该协议只用于普通 UART/蓝牙串口透传，不与 USB 或 nRF24 共用结构。

```text
[A5] [5A] [data[0] ... data[N-1]] [CRC8]
```

- `N = MODULE_UART_COMM_DATA_SIZE`，默认 8 字节，允许 1~252。
- 完整帧长为 `N + 3`。
- CRC8 初值 `0xFF`、多项式 `0x8C`、LSB first，覆盖帧头和全部数据位；计算统一调用 [`alg_crc`](../../Algorithm/alg_crc/README.md)。

```powershell
cmake --preset Debug -B .build/Debug -DMODULE_UART_COMM_DATA_SIZE=12
```

## 使用顺序

```c
/* 1. USART BSP 已初始化；引脚、波特率、校验位仍由 IOC 决定。 */
module_uart_comm_config_t config = {
    .usart = uart,
    .transmit_mode = BSP_TRANSFER_MODE_DMA,
    .transmit_timeout_ms = 10U,
};
module_uart_comm_init(&uart_comm, &config);

/* 2. 发送的数据长度必须正好等于宏定义数据区长度。 */
uint8_t tx[MODULE_UART_COMM_DATA_SIZE] = {0};
module_uart_comm_send(&uart_comm, tx, sizeof(tx));

/* 3. 空闲线/DMA 回调中投递收到的字节流。 */
module_uart_comm_feed_data(&uart_comm, received_bytes, received_size);

/* 4. 读取最近有效数据。 */
module_uart_comm_process_data_t rx;
if (module_uart_comm_get_data(&uart_comm, &rx) == MODULE_UART_COMM_STATUS_OK)
{
    uint8_t first = rx.data[0];
}
```

可读取 `received_data.data[]`、`update_count`、`is_valid`、`valid_frame_count` 和 `checksum_error_count`。蓝牙透明传输模块直接视为普通 UART。
