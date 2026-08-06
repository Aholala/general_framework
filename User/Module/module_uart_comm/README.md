# module_uart_comm — UART 固定帧协议

独立 UART 协议：`[0xA5][0x5A][data × N][CRC8]`。固定长度数据区。

## 用法

```c
module_uart_comm_t comm;
module_uart_comm_config_t cfg = {
    .usart = usart_ptr,
    .data_size = MODULE_UART_COMM_DATA_SIZE,  // 默认 8
    .receive_timeout_ms = 10,
};
module_uart_comm_init(&comm, &cfg);

// 接收（在任务中轮询）
module_uart_comm_process(&comm);

// 读数据
const module_uart_comm_process_data_t *data = module_uart_comm_get_data(&comm);
if (data->is_valid) {
    memcpy(buf, data->data, data->data_size);
}

// 发送
uint8_t payload[8] = {...};
module_uart_comm_send(&comm, payload, sizeof(payload));
```

## 协议帧

| 字节 | 内容 |
|------|------|
| 0 | 0xA5 帧头 |
| 1 | 0x5A |
| 2~N+1 | 数据（N = `MODULE_UART_COMM_DATA_SIZE`） |
| N+2 | CRC8（多项式 0x8C, LSB first, 初值 0xFF） |
