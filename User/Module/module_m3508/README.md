# M3508 + C620 电机模块

## 职责

`module_m3508` 继承 `module_dji_motor_t`，固定型号为 M3508，使用 C620 电调命令范围和
19:1 默认减速比。

```text
module_motor_t
    └── module_dji_motor_t
            └── module_m3508_t
```

## CAN 映射

| 电机 ID | 控制帧 ID | 反馈帧 ID |
|---|---:|---:|
| 1～4 | `0x200` | `0x201`～`0x204` |
| 5～8 | `0x1FF` | `0x205`～`0x208` |

原始电流命令范围为 `[-16000, 16000]`。反馈包含编码器、转速、电流原始值和温度。

## 初始化

```c
const module_m3508_config_t config = {
    .logical_name = "left_wheel_motor",
    .registration_key = 2U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M3508_CONTROL_VELOCITY,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .velocity_pid_config = wheel_velocity_pid_config,
};

(void)module_m3508_init(&left_wheel_motor, &config);
(void)module_m3508_register(&left_wheel_motor, &motor_registry);
(void)module_m3508_enable(&left_wheel_motor);
```

## 控制接口

- `module_m3508_set_current_command_raw`；
- `module_m3508_set_velocity_rad_per_s`；
- `module_m3508_set_position_rad`；
- `module_m3508_update`；
- `module_m3508_get_feedback`。

接口与初始化模式不匹配时返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`。所有同总线 DJI 电机更新后
只调用一次 `module_dji_motor_bus_flush`。

## 注意事项

当前默认减速比为 `19.0F`。如果项目需要使用更精确的实际减速比，应把减速比改为型号配置项，
不要在 App 层对位置和速度重复换算。反馈电流暂为协议原始值。
