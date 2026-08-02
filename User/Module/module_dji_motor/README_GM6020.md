# GM6020

GM6020 为直驱云台电机，电机 ID 为 1~7。电压模式原始命令范围为
`[-25000, 25000]`；电流模式及速度/角度级联控制的最终电流命令范围为
`[-16384, 16384]`，对应 `[-3 A, 3 A]`。

电压模式发送 ID 为 `0x1FF`（ID 1~4）或 `0x2FF`（ID 5~7）；电流模式以及速度、
角度三级闭环的最终输出必须使用 `0x1FE`（ID 1~4）或 `0x2FE`（ID 5~7）。总线对象
会根据初始化时的 `control_mode` 自动选择，不能在运行中直接改枚举字段切换模式。

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
