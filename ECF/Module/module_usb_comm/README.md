# module_usb_comm — USB CDC 视觉通信

视觉上位机协议：`[0xA5][0x5A][mode][id][extra_data × N][CRC8]`。ID 范围 1~7。

## 用法

```c
module_usb_comm_t comm;
module_usb_comm_config_t cfg = {
    .usb_vcp = board_config_get_usb_vcp(),
};
module_usb_comm_init(&comm, &cfg);

// 接收（在任务中轮询）
module_usb_comm_process(&comm);

// 读数据
const module_usb_comm_data_t *data = module_usb_comm_get_data(&comm);
if (data->is_valid) {
    uint8_t mode = data->mode;  // 当前模式
    uint8_t id   = data->id;    // 目标 ID (1~7)
}
```

## 协议帧

| 字节 | 内容 |
|------|------|
| 0 | 0xA5 |
| 1 | 0x5A |
| 2 | mode |
| 3 | id |
| 4~N | extra_data（`MODULE_USB_COMM_EXTRA_DATA_SIZE`，默认 0） |
| N+1 | CRC8（0x8C, LSB first, 初值 0xFF） |
