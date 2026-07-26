# M2006 + C610 电机模块

## 职责

`module_m2006` 继承 `module_dji_motor_t`，固定型号为 M2006，使用 C610 电调的命令范围和
36:1 默认减速比，避免调用者误配成 M3508 或 GM6020。

```text
module_motor_t
    └── module_dji_motor_t
            └── module_m2006_t
```

## CAN 映射

| 电机 ID | 控制帧 ID | 反馈帧 ID |
|---|---:|---:|
| 1～4 | `0x200` | `0x201`～`0x204` |
| 5～8 | `0x1FF` | `0x205`～`0x208` |

原始电流命令范围为 `[-10000, 10000]`。反馈包含 0～8191 编码器、转速、电流原始值和温度。

## 初始化

```c
const module_m2006_config_t config = {
    .logical_name = "feeder_motor",
    .registration_key = 1U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M2006_CONTROL_VELOCITY,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .velocity_pid_config = feeder_velocity_pid_config,
};

(void)module_m2006_init(&feeder_motor, &config);
(void)module_m2006_register(&feeder_motor, &motor_registry);
(void)module_m2006_enable(&feeder_motor);
```

## 控制接口

- `module_m2006_set_current_command_raw`：C610 原始电流命令；
- `module_m2006_set_velocity_rad_per_s`：输出轴速度；
- `module_m2006_set_position_rad`：输出轴累计位置；
- `module_m2006_update`：执行当前模式控制；
- `module_m2006_get_feedback`：读取统一反馈。

目标接口必须与初始化 `control_mode` 一致，否则返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`。所有
DJI 电机更新后，由共享 `module_dji_motor_bus_flush` 统一发送分组帧。

## 注意事项

默认减速比使用 `36.0F`。特殊减速箱需要扩展配置，不能在 App 中事后修正反馈。反馈电流目前
保留协议原始值，在加入可靠的 C610 电流比例前，不应当作安培用于保护。
