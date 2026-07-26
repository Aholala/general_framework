# module_vision

云台与视觉计算机之间的轻量二进制协议，基于 USB CDC 虚拟串口发送 IMU/心跳并接收目标
yaw、pitch、角速度、置信度和跟踪状态。

## 消息

- `MODULE_VISION_MESSAGE_IMU`：四元数、三轴角速度和时间戳；
- `MODULE_VISION_MESSAGE_TARGET`：目标角度、角速度、置信度和状态；
- `MODULE_VISION_MESSAGE_HEARTBEAT`：MCU 运行时间。

帧包含同步头、类型、序号、长度、payload 和 CRC16，最大 payload 为 48 字节。

## 初始化

```c
module_vision_init(&vision, &config);
```

配置需要 USB VCP 基类、发送超时和目标离线超时。BSP 必须实现 `get_busy`，因为异步发送
前必须确认对象内部发送缓冲区没有仍被 USB 使用。

## 发送缓冲区安全

发送帧构造在 `module_vision_t::transmit_buffer`，不是栈数组。USB 完成前该缓冲区保持
有效；忙时返回 `MODULE_VISION_STATUS_BUSY`，调用方可丢弃一帧高频 IMU 遥测。

## 流式接收

USB 数据通过 `module_vision_feed_data` 输入。模块把分包拼入内部 stream buffer，查找
完整帧并校验 CRC。非法帧返回错误并重新同步，不直接修改有效目标。

接收接口应在任务上下文调用；USB ISR 只转交数据块和长度。

## 目标生命周期

有效 TARGET 帧更新内部快照和计数。周期调用：

```c
module_vision_update_time(&vision, elapsed_time_ms);
```

超时后 `get_target` 返回 NO_TARGET。App 应平滑退出视觉跟随，回到遥控或保持模式，不能
继续锁定最后坐标。

## 坐标约定

协议值使用弧度、弧度每秒和毫秒。四元数顺序必须与视觉端共同固定。视觉补偿的时间同步、
弹道预测和相机到云台外参属于 App/算法层。

## 建议验证

- IMU/心跳帧字节和 CRC；
- TARGET 分包、粘包、噪声与错误 CRC；
- USB BUSY 时不覆盖发送缓冲；
- 序号回绕；
- 置信度和跟踪状态；
- 目标超时；
- 拔插 USB 后安全回退。
