# M3508 / C620

M3508 使用 19:1 减速输出轴反馈，电机 ID 为 1~8，协议原始命令范围为 `[-16000, 16000]`。

## 模式和接口

| 模式 | 设置接口 |
| --- | --- |
| `MODULE_M3508_CONTROL_DIRECT` | `module_m3508_set_direct_command_raw()` |
| `MODULE_M3508_CONTROL_CURRENT` | `module_m3508_set_current_a()` |
| `MODULE_M3508_CONTROL_VELOCITY` | `module_m3508_set_velocity_rad_per_s()` |
| `MODULE_M3508_CONTROL_ANGLE` | `module_m3508_set_angle_rad()` |

电流、速度、角度三个环分别使用 `module_motor_pid_config_t`，可以分别选择位置式或增量式 PID。非直通模式必须填写真实的电流换算系数，不能把 raw command 当作安培值。

接入顺序：DJI 总线初始化 → 填写 `module_m3508_config_t` → `module_m3508_init/register` → 路由反馈 → `enable` → 设置目标 → 周期 `update` → `module_dji_motor_bus_flush()`。

运行信息位于 `module_m3508_t.super.super`：电机名称、协议 ID、最近 dt、总运行时间、累计使能时间、更新次数、位置/速度/电流/温度和在线状态；三级 PID 与各级目标位于 `module_m3508_t.super`。
