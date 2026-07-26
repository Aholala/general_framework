# 通用电机基类

`module_motor_t` 为电机对象提供统一的注册、使能、失能、目标设置、周期更新和
反馈接口。派生类第一成员必须命名为 `super`，虚表使用只读
`module_motor_ops_t`。

## 注册和生命周期

```text
派生类init
    -> module_motor_registry_register
    -> module_motor_enable
    -> 周期set_target/update
    -> module_motor_disable
    -> module_motor_registry_unregister
```

注册表存储由调用者提供，不分配动态内存。未注册对象不能执行控制操作。

## 反馈时效框架

反馈解析成功后，派生模块调用：

```c
(void)module_motor_notify_feedback(motor);
```

项目可以选择设置超时；`0U` 表示禁用超时判断：

```c
(void)module_motor_set_feedback_timeout(motor, feedback_timeout_ms);
(void)module_motor_update_feedback_time(motor, elapsed_time_ms);
```

达到超时时间后 `feedback.is_online` 变为 `false`。若电机正在输出，
基类会调用派生类 `disable` 清零命令并进入
`MODULE_MOTOR_STATE_FAULT`。离线电机不能使能，避免使用旧反馈继续闭环。

反馈恢复后故障不会自动解除。调用者应完成机械状态和控制目标检查，再显式执行：

```c
if (module_motor_clear_fault(motor) == MODULE_MOTOR_STATUS_OK)
{
    (void)module_motor_enable(motor);
}
```

`module_motor_update_feedback_time` 必须由任务周期或 `module_motor_health` 调用，否则
超时保护无法累计时间。

## 反馈单位

- 位置：`rad`
- 速度：`rad/s`
- 力矩：`N*m`
- 电流：`A`
- 温度：摄氏度

协议原始电流保存在 `current_raw`。只有派生模块配置了可靠换算比例时，
`is_current_a_valid` 才为真，不能把原始计数直接当作安培使用。
