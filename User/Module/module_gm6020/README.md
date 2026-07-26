# GM6020 云台电机模块

## 职责

`module_gm6020` 是 `module_dji_motor_t` 的专用派生封装，固定电机型号为 GM6020，防止调用者
误填 M2006/M3508 型号、错误减速比或错误 CAN 分组。对象仍通过 `module_motor_t` 基类参加统一
注册和多态调度。

```text
module_motor_t
    └── module_dji_motor_t
            └── module_gm6020_t
```

## 协议映射

| 电机 ID | 控制帧 ID | 命令槽位 | 反馈帧 ID |
|---|---:|---:|---:|
| 1～4 | `0x1FF` | 0～3 | `0x205`～`0x208` |
| 5～7 | `0x2FF` | 0～2 | `0x209`～`0x20B` |

控制命令为有符号 16 位原始电压命令，范围限制为 `[-30000, 30000]`。反馈包含 0～8191
编码器值、转速、转矩电流原始值和温度。GM6020 采用直驱减速比 `1.0F`。

## 初始化和注册

```c
static module_gm6020_t yaw_motor;

const module_gm6020_config_t config = {
    .logical_name = "yaw_motor",
    .registration_key = 10U,
    .motor_bus = &gimbal_motor_bus,
    .control_mode = MODULE_GM6020_CONTROL_POSITION,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .position_pid_config = yaw_cascade_pid_config,
};

(void)module_gm6020_init(&yaw_motor, &config);
(void)module_gm6020_register(&yaw_motor, &motor_registry);
(void)module_gm6020_enable(&yaw_motor);
```

必须先初始化共享 `module_dji_motor_bus_t` 和 `module_motor_registry_t`。对象初始化后仍必须注册，
未注册对象不能使能、设置目标或更新。

## 控制接口

| 配置模式 | 专用目标接口 | 单位 |
|---|---|---|
| `MODULE_GM6020_CONTROL_VOLTAGE` | `module_gm6020_set_voltage_command_raw` | 协议原始值 |
| `MODULE_GM6020_CONTROL_VELOCITY` | `module_gm6020_set_velocity_rad_per_s` | `rad/s` |
| `MODULE_GM6020_CONTROL_POSITION` | `module_gm6020_set_position_rad` | `rad` |

调用与初始化模式不匹配的目标接口会返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`，避免把角度误当速度
或把物理电压误当协议原始命令。

## 周期调度

```c
(void)module_gm6020_set_position_rad(&yaw_motor, yaw_target_rad);
(void)module_gm6020_update(&yaw_motor, delta_time_s);

/* 所有 DJI 电机更新完成后统一发送。 */
(void)module_dji_motor_bus_flush(&gimbal_motor_bus);
```

CAN 接收帧仍交给共享的 `module_dji_motor_bus_handle_feedback`。反馈可通过
`module_gm6020_get_feedback` 获取；反馈位置已完成跨零累计，单位为输出轴弧度。

## 安全和移植

- `motor_identifier` 只能为 1～7；
- `direction_sign` 只能为 `1.0F` 或 `-1.0F`；
- 过温后对象进入 `MODULE_MOTOR_STATE_FAULT` 并把命令清零；
- `current_a` 当前保存转矩电流原始值，不应直接按安培使用；
- App 应实现反馈超时、机械限位、软限位和失控急停；
- CAN 位时序和具体控制器由 BSP 平台端配置。
