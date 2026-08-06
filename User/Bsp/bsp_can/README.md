# bsp_can — Classic CAN 抽象层（完整 OOP）

CAN 使用完整 OOP 模型（`super + vptr + container_of`），因为存在 FDCAN（H723）和 bxCAN（F405）两种平台实现。

## 调用路径

```text
bsp_can_transmit(can, frame, timeout)
  → validate(me)                     // NULL + is_initialized 检查
  → container_of(vptr) → ops 表      // 高层虚表
  → ops->transmit(me, frame, ms)     // 转发函数
    → driver_ops->transmit(device_handle, frame, ms)  // 底层平台表
      → HAL_FDCAN_AddMessageToTxFifoQ / HAL_CAN_AddTxMessage
```

## 关键结构体

| 结构体 | 角色 | 关键字段 |
|--------|------|---------|
| `bsp_can_frame_t` | CAN 帧 | `identifier`, `id_type`, `frame_type`, `data_length`(0-8), `data[8]` |
| `bsp_can_filter_t` | 硬件过滤器 | `identifier`, `mask`, `id_type`, `receive_fifo`, `filter_index` |
| `bsp_can_t` | 基类（App 持有） | `super`(bsp_device_t), `callback`, `user_context` |
| `bsp_can_device_t` | 派生设备 | `super`(bsp_can_t), `driver_ops` |
| `bsp_can_config_t` | 初始化配置 | `device_handle`, `driver_ops`, `callback`, `user_context` |
| `bsp_can_ops_t` | 高层虚表 | `start/stop/configure_filter/transmit/receive/get_tx_free_level` |
| `bsp_can_driver_ops_t` | 平台驱动表 | 同上 + `init/deinit` |

## 用法

### 初始化（由 board_config 完成）

```c
// board_config.c 中：每个 CAN 实例绑定 FDCAN 句柄
FDCAN_HandleTypeDef *handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
bsp_can_config_t cfg = {
    .device_handle = handles[i],
    .driver_ops    = &board_config_can_driver_ops,
};
bsp_can_init(&board_config_can_devices[i], &cfg);
```

### App 层获取

```c
bsp_can_t *can = board_config_get_can(BOARD_CONFIG_CAN_2);
```

### 启动 / 停止

```c
bsp_can_start(can);   // 配置全局过滤器 + 激活中断通知 + HAL_FDCAN_Start
bsp_can_stop(can);    // HAL_FDCAN_Stop
```

### 发送（阻塞，带超时）

```c
bsp_can_frame_t frame = {
    .identifier  = 0x200,
    .id_type     = BSP_CAN_ID_STANDARD,
    .frame_type  = BSP_CAN_FRAME_DATA,
    .data_length = 8,
    .data        = {0x01, 0x02, ...},
};
bsp_status_t rc = bsp_can_transmit(can, &frame, 10);  // 10ms 超时
if (rc == BSP_STATUS_TIMEOUT) { /* 总线忙 */ }
```

### 接收（轮询，在任务中调用）

```c
bsp_can_frame_t frame;
while (bsp_can_receive(can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK) {
    // 处理 frame
}
while (bsp_can_receive(can, BSP_CAN_RX_FIFO_1, &frame) == BSP_STATUS_OK) {
    // 处理 frame
}
```

### 中断回调（可选，当前项目用轮询）

```c
void on_can_event(bsp_event_t event, bsp_status_t status, size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_PENDING) {
        // ISR 上下文：只置标志，不处理数据
    }
}
bsp_can_set_callback(can, on_can_event, NULL);
```

## API 速查

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `bsp_can_init(me, cfg)` | 初始化设备 | OK / INVALID_ARGUMENT |
| `bsp_can_start(me)` | 启动 CAN 通信 | OK / IO_ERROR |
| `bsp_can_stop(me)` | 停止 CAN | OK |
| `bsp_can_transmit(me, frame, ms)` | 发送（阻塞） | OK / TIMEOUT / IO_ERROR |
| `bsp_can_receive(me, fifo, frame)` | 接收一帧 | OK / IO_ERROR |
| `bsp_can_configure_filter(me, filter)` | 配置硬件过滤器 | OK / INVALID_ARGUMENT |
| `bsp_can_get_transmit_free_level(me, &n)` | 查询发送邮箱余量 | OK |
| `bsp_can_set_callback(me, cb, ctx)` | 设置事件回调 | OK / NOT_INITIALIZED |
| `bsp_can_notify(me, event, status, size)` | ISR 通知入口 | void |
| `bsp_can_as_base(me)` | 向上转型 | `bsp_can_t *` |
| `bsp_device_is_initialized(&me->super)` | 检查初始化状态 | bool |
| `bsp_device_deinit(&me->super.super)` | 反初始化（停止 + 清空） | OK |

## 板间通信的 CAN 帧处理模式

当前 App 层在 FreeRTOS 任务中轮询 CAN FIFO：

```c
void app_robot_communication_update(uint32_t elapsed_ms) {
    bsp_can_frame_t frame;
    module_board_comm_update_time(&board_comm, elapsed_ms);
    while (bsp_can_receive(can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK) {
        module_board_comm_handle_frame(&board_comm, &frame);
    }
}
```

轮询而非中断的原因：FreeRTOS 任务上下文可以调任何 API，不需要 ISR→任务的消息队列。
