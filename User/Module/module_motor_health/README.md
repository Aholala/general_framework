# module_motor_health

多电机健康聚合器，将注册、在线、故障、使能和温度状态转换为稳定的可用性数组，供底盘
降级运动学和 App 安全状态机使用。

## 原因位

`reason_mask` 可同时包含：

- `NOT_REGISTERED`；
- `OFFLINE`；
- `MOTOR_FAULT`；
- `NOT_ENABLED`；
- `OVER_TEMPERATURE`。

原因位比单一布尔值更适合诊断和赛场遥测。

## 配置

调用者提供电机指针数组、状态存储数组、可选的逐电机最高温度数组、故障/恢复确认时间，
以及是否要求使能、是否由本模块推进反馈离线计时。

```c
module_motor_health_init(&health, &config);
module_motor_health_update(&health, elapsed_time_ms);
module_motor_health_get_availability(
    &health, motor_is_available, motor_count);
```

所有数组容量至少为 `motor_count`，并覆盖对象生命周期。

## 确认与恢复

异常原因持续达到 `fault_confirmation_time_ms` 后标记不可用；所有要求恢复并持续达到
`recovery_confirmation_time_ms` 后重新可用。通信明确离线或电机硬故障是否需要立即停机，
仍由电机基类和 App 安全策略决定。

## 反馈时间所有权

`manage_feedback_time = true` 时，本模块调用电机反馈时间更新。系统只能有一个时间推进
所有者，不能 App 和健康模块同时累加，否则离线时间会翻倍。

## 与运动学连接

取得的 `motor_is_available` 可直接传给麦轮、全向轮、差速和舵轮算法。返回降级不等于
允许继续高速运行，App 仍需根据可控自由度降低速度或停机。

## 边界

本模块不发送电机命令、不清除故障、不自动重新使能，也不拥有电机对象。单个健康对象应由
一个周期任务更新。

## 建议验证

- 每个原因位单独及组合触发；
- 故障确认、恢复确认和阈值边界；
- 温度数组为空；
- 要求/不要求使能；
- 反馈时间单一所有者；
- 多电机不同状态；
- 计数溢出和无效容量。
# 扩展诊断

除离线、故障、未使能和过温外，健康监视器还可检查过流、编码器突跳、持续跟踪误差、堵转、长时间饱和以及 CAN 错误计数变化。命令值和控制误差通过只读 `observer` 回调取得，健康模块不依赖任何具体电机派生类。

所有阈值数组按电机索引提供；不需要的检查传 `NULL` 即可关闭。堵转与饱和具有独立确认时间，避免启动瞬态误报。`output_saturation_ratio` 应设置为 0 到 1；不使用观察器时建议设为 1。

周期编码器应同时提供 `encoder_modulus`（例如 DJI 反馈为 8192），突跳检测会选择跨零点的最短差值；绝对累计编码器可把该指针设为 `NULL`。
