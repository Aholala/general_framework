# module_bluetooth

基于 `bsp_usart_t` 的通用蓝牙串口模块，负责异步接收、双缓冲转交、在线超时、原始数据发送
和 AT 命令发送。它不绑定 HC-05、HC-06 或某个厂商协议。

## 依赖与内存

配置必须提供：

- 已初始化的 `bsp_usart_t *`；
- DMA 接收缓冲区 `receive_buffer`；
- 任务处理缓冲区 `processing_buffer`；
- 两块缓冲区各自容量；
- 接收模式、收发超时、离线超时；
- 逻辑名称和注册键；
- 可选接收回调和上下文。

处理缓冲区容量必须不小于接收缓冲区。两块内存均由调用者持有并覆盖对象生命周期。

## 为什么使用双缓冲

USART 完成/空闲中断中，模块把有效数据复制到处理缓冲区并立即重启 DMA。任务调用
`module_bluetooth_update` 后才执行用户回调。这样不会在 ISR 中解析命令，也不会因为任务
处理而长时间关闭接收。

当上一帧尚未处理时到达新帧，模块增加 `receive_overrun_count`，而不是覆盖正在使用的
数据。DMA 重启失败增加 `receive_restart_error_count`。

## 使用流程

```c
module_bluetooth_init(&bluetooth, &config);
module_bluetooth_start(&bluetooth);

for (;;)
{
    module_bluetooth_update(&bluetooth, elapsed_time_ms);
}
```

发送二进制数据使用 `module_bluetooth_transmit`；发送零结尾 AT 命令使用
`module_bluetooth_send_command`。调用方负责命令结束符和模块所需波特率。

## 在线状态

成功接收数据后设备标记在线。超过 `offline_timeout_ms` 未收到数据时离线。
`module_bluetooth_is_online` 只表示链路最近有活动，不代表远端协议或控制权限有效。

## 生命周期和并发

停止时中止接收并清除待处理状态。发送与更新最好由同一通信任务管理；多任务发送需要外部
队列。用户回调运行在任务上下文，但仍应保持有界执行时间。

## 建议验证

- DMA 空闲接收和立即重启；
- 连续帧、粘包和分包由上层协议处理；
- 处理未完成时的 overrun；
- 接收重启失败；
- 在线/离线超时；
- AT 命令和二进制发送；
- 停止后拒绝操作。
