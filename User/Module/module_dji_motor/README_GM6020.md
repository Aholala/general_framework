# GM6020

GM6020 为直驱云台电机，电机 ID 为 1~7，直通模式发送原始电压命令，范围为 `[-30000, 30000]`。

## 模式和接口

| 模式 | 设置接口 |
| --- | --- |
| `MODULE_GM6020_CONTROL_VOLTAGE` | `module_gm6020_set_voltage_command_raw()` |
| `MODULE_GM6020_CONTROL_CURRENT` | `module_gm6020_set_current_a()` |
| `MODULE_GM6020_CONTROL_VELOCITY` | `module_gm6020_set_velocity_rad_per_s()` |
| `MODULE_GM6020_CONTROL_ANGLE` | `module_gm6020_set_angle_rad()` |

除电压直通外，控制链依次为电流环、速度→电流、角度→速度→电流。三个环均用 `module_motor_pid_config_t`，可分别选择位置式或增量式算法。

接入顺序：DJI 总线初始化 → 填写 `module_gm6020_config_t` → `module_gm6020_init/register` → 路由反馈 → `enable` → 设置目标 → 周期 `update` → `module_dji_motor_bus_flush()`。

展开 `module_gm6020_t` 可读取电机名称、三个 PID、各级目标、ID、CAN 映射、最终电压命令、多圈角度、速度、电流、温度、最近 dt、总运行时间、累计使能时间和更新次数。
