# alg_chassis_fault

基于轮速约束残差的车轮故障确认和恢复管理。模块把瞬时打滑与持续异常区分开，为底盘正解
和逆解生成稳定的 `wheel_is_available` 数组。

## 工作原理

每个车轮保存：

- 当前残差绝对值；
- 连续故障确认计数；
- 连续恢复确认计数；
- 故障锁存状态。

残差超过 `fault_residual_threshold_m_per_s` 达到指定样本数后标记故障；故障状态下，残差
低于更严格的 `recovery_residual_threshold_m_per_s` 达到指定样本数后恢复。两阈值形成
滞回，避免状态抖动。

## 配置

`alg_chassis_fault_config_t` 指定轮数、两级阈值、确认样本数以及调用者提供的
`alg_chassis_fault_wheel_state_t` 数组。存储容量至少等于 `wheel_count`。

阈值和样本数是框架参数，不绑定具体车型。建议根据控制周期、编码器噪声和赛场打滑数据
标定。

## 使用

```c
static alg_chassis_fault_wheel_state_t wheel_states[4];
static alg_chassis_fault_t fault_monitor;

alg_chassis_fault_init(&fault_monitor, &config);
alg_chassis_fault_update(
    &fault_monitor,
    wheel_residuals_m_per_s,
    sensor_is_available,
    wheel_is_available,
    4U);
```

传感器本身离线时，该轮应立即输出不可用，不依赖残差确认。随后将可用性数组传给
`alg_mecanum`、`alg_omni`、`alg_swerve` 或 `alg_differential`。

## 手动恢复

`alg_chassis_fault_reset_wheel` 用于维修、重新标定或整车状态机明确确认后的复位。
`assume_available` 决定复位后初始状态。比赛运行中不应周期性强制清除故障，否则会绕过
确认机制。

## 边界

- 本模块不计算轮速残差，残差来自各运动学模块；
- 不判断 CAN 离线，通信健康状态通过 `sensor_is_available` 注入；
- 不直接停电机，输出只是一组可用性决策；
- 对象与状态数组由调用者持有；
- 单实例不支持并发更新。

## 建议验证

- 阈值上下波动不误触发；
- 连续超限达到确认次数；
- 故障后恢复滞回；
- 传感器立即离线；
- 单轮、相邻双轮和多轮故障；
- 饱和计数和手动复位；
- 输出容量不足及轮数为零。
