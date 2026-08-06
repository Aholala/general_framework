# bsp_usb_vcp — USB CDC 虚拟串口（完整 OOP）

USB CDC 抽象。环形队列接收，忙等重试发送。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `bsp_usb_vcp_t` | 基类：`super`(bsp_device_t), `callback`, `user_context` |
| `bsp_usb_vcp_device_t` | 派生设备 |
| `bsp_usb_vcp_config_t` | 配置：`device_handle`, `driver_ops` |
| `bsp_usb_vcp_driver_ops_t` | 平台实现：`transmit/receive/abort/get_connected/get_busy` |

## 用法

```c
bsp_usb_vcp_t *usb = board_config_get_usb_vcp();

// 发送（忙等重试，超时返回 BUSY/TIMEOUT）
bsp_status_t rc = bsp_usb_vcp_transmit(usb, data, 64, 50);

// 接收（从 4 槽环形队列取一帧）
uint8_t buf[512];
bsp_status_t rc = bsp_usb_vcp_receive(usb, buf, sizeof(buf));

// 检查连接状态
bool connected;
bsp_usb_vcp_get_connected(usb, &connected);

// 中止 + 清空队列
bsp_usb_vcp_abort(usb);
```

## 环形队列

接收队列 4 槽 × 512 字节。ISR 回调 (`usb_cdc_receive_callback`) 写入，`bsp_usb_vcp_receive` 读出。队列满时报 `NO_RESOURCE` 并递增 `overrun_count`。

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_usb_vcp_init(me, cfg)` | 初始化 |
| `bsp_usb_vcp_transmit(me, data, n, ms)` | 发送（忙等重试） |
| `bsp_usb_vcp_receive(me, buf, cap)` | 接收一帧 |
| `bsp_usb_vcp_abort(me)` | 清空队列 |
| `bsp_usb_vcp_get_connected(me, &connected)` | USB 是否枚举成功 |
| `bsp_usb_vcp_get_busy(me, &busy)` | 发送是否忙 |
| `bsp_usb_vcp_as_base(me)` | 向上转型 |
