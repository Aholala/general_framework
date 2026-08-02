# USB CDC 视觉通信协议

USB 协议只服务视觉链路，不与 UART 或 nRF24 共用帧结构。当前只固定 `mode` 和 `id`，后续视觉字段暂不定义。

```text
[A5] [5A] [mode] [id] [extra_data × N] [CRC8]
```

- `id` 范围为 1~7。
- `N = MODULE_USB_COMM_EXTRA_DATA_SIZE`，默认 0，允许 0~250。
- 默认帧只有 5 字节：帧头2 + mode1 + id1 + CRC8。
- CRC8 初值 `0xFF`、多项式 `0x8C`、LSB first，覆盖 CRC 前全部字节；计算统一调用 [`alg_crc`](../../Algorithm/alg_crc/README.md)。
- 后续确定 pitch、yaw 和前馈数据后，再为 `extra_data` 定义正式字段和单位。

修改预留长度：

```powershell
cmake --preset Debug -B .build/Debug -DMODULE_USB_COMM_EXTRA_DATA_SIZE=16
```

## 使用顺序

```c
/* 1. USB VCP BSP 初始化完成后初始化协议对象。 */
module_usb_comm_config_t config = {
    .usb_vcp = usb_vcp,
    .transmit_timeout_ms = 10U,
};
module_usb_comm_init(&usb_comm, &config);

/* 2. 收发使用同一结构；当前只填写 mode 和 ID。 */
module_usb_comm_data_t tx = {
    .mode = 1U,
    .id = 3U,
};
module_usb_comm_send(&usb_comm, &tx);

/* 3. USB 接收回调中投递实际收到的字节。 */
module_usb_comm_feed_data(&usb_comm, received_bytes, received_size);

/* 4. 任务中读取最近有效帧。 */
module_usb_comm_process_data_t rx;
if (module_usb_comm_get_data(&usb_comm, &rx) == MODULE_USB_COMM_STATUS_OK)
{
    uint8_t mode = rx.data.mode;
    uint8_t id = rx.data.id;
}
```

调试器中可读取 `received_data.data`、`update_count`、`is_valid`、`valid_frame_count`、`invalid_frame_count` 和 `checksum_error_count`。异步发送缓冲区保存在对象内部。
