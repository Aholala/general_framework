# M2006 / C610

M2006 使用 36:1 减速输出轴反馈，电机 ID 为 1~8，协议原始命令范围为 `[-10000, 10000]`。

## 模式和接口

| 模式 | 设置接口 |
| --- | --- |
| `MODULE_M2006_CONTROL_DIRECT` | `module_m2006_set_direct_command_raw()` |
| `MODULE_M2006_CONTROL_CURRENT` | `module_m2006_set_current_a()` |
| `MODULE_M2006_CONTROL_VELOCITY` | `module_m2006_set_velocity_rad_per_s()` |
| `MODULE_M2006_CONTROL_ANGLE` | `module_m2006_set_angle_rad()` |

配置中始终提供 `current_pid_config`；速度模式再提供 `velocity_pid_config`，角度模式再提供 `angle_pid_config`。每个配置的 `form` 可独立选择 `MODULE_MOTOR_PID_POSITIONAL` 或 `MODULE_MOTOR_PID_INCREMENTAL`。

接入顺序：DJI 总线初始化 → 填写 `module_m2006_config_t` → `module_m2006_init/register` → 路由反馈 → `enable` → 设置目标 → 周期 `update` → 一组电机统一 `bus_flush`。

调试时展开 `module_m2006_t.super` 可查看三级 PID、目标、ID、CAN 槽位和命令；再展开 `super.super` 可查看 `motor_name`、`delta_time_s`、`total_runtime_us`、`enabled_runtime_us`、`control_update_count` 与完整反馈。
