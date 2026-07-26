# bsp_usb_vcp

USB CDC 虚拟串口抽象，向 Module 提供异步字节流，而不暴露 USB Device 中间件类型。

## 接口

- `bsp_usb_vcp_transmit`：发送一块数据；
- `bsp_usb_vcp_receive`：登记接收缓冲区；
- `bsp_usb_vcp_abort`：中止当前事务；
- `bsp_usb_vcp_get_connected`：查询主机枚举/连接状态；
- `bsp_usb_vcp_get_busy`：查询发送是否占用；
- `bsp_usb_vcp_notify`：平台回调入口。

## 初始化

```c
static bsp_usb_vcp_device_t vision_usb;
static const bsp_usb_vcp_config_t config = {
    .device_handle = &platform_usb_cdc,
    .driver_ops = &platform_usb_vcp_driver_ops,
    .callback = usb_event_callback,
    .user_context = &vision,
};

bsp_usb_vcp_init(&vision_usb, &config);
```

USB 枚举可能在 MCU 初始化后才完成，发送前应检查 `get_connected` 和 `get_busy`。

## 异步发送

许多 USB CDC 实现不会立即复制发送数组。调用者必须让发送缓冲区保持有效直到发送完成
通知。`module_vision` 因此使用对象内部持久缓冲区，而不是栈数组。

忙时返回 `BSP_STATUS_BUSY`，上层应丢弃可丢遥测或进入有界队列，不能阻塞控制任务等待
主机。

## 接收

`receive` 登记调用者持有的缓冲区。平台收到 USB 数据后完成缓存处理，再通知实际长度。
协议解析必须检查长度、帧头、序号和校验。

## 连接与容错

拔线、主机休眠或重新枚举都是正常状态。断开时视觉/调试 Module 应超时离线，并回退到
安全控制源；不能让 USB 状态阻塞云台和底盘周期。

## 平台边界

端点号、描述符、USB Device 类对象和中间件回调均留在平台 Port。通用头文件不包含
`usbd_cdc_if.h`。

## 建议验证

- 枚举前后连接状态；
- 连续发送、忙返回和完成通知；
- 主机拔插与重新枚举；
- 0 字节和最大包；
- 接收分包/粘包；
- 发送缓冲区生命周期；
- 两个逻辑协议共享 USB 时的外部仲裁。
