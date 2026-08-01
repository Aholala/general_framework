# DJI 电机模块

本模块统一实现 M2006/C610、M3508/C620 和 GM6020 的 CAN 反馈、多圈角度、总线分组发送及三级 PID 控制。型号入口见 [M2006](README_M2006.md)、[M3508](README_M3508.md) 和 [GM6020](README_GM6020.md)。

## 控制链

四种模式在初始化时固定：

| 模式 | 控制链 | 目标单位 |
| --- | --- | --- |
| `MODULE_DJI_CONTROL_DIRECT` | 原始协议命令直接发送 | raw count |
| `MODULE_DJI_CONTROL_CURRENT` | 电流 PID → 协议命令 | A |
| `MODULE_DJI_CONTROL_VELOCITY` | 速度 PID → 电流 PID → 协议命令 | rad/s |
| `MODULE_DJI_CONTROL_ANGLE` | 角度 PID → 速度 PID → 电流 PID → 协议命令 | rad |

`current_*` 始终表示安培值或电流环。跳过 PID 的接口统一叫 `*_direct_command_raw`，不会再把“原始命令”误称为电流 PID。

每一级使用独立的 `module_motor_pid_config_t`，通过 `form` 选择：

```c
module_motor_pid_config_t current_pid = {
    .form = MODULE_MOTOR_PID_POSITIONAL,
    .positional_config = current_positional_config,
};

module_motor_pid_config_t velocity_pid = {
    .form = MODULE_MOTOR_PID_INCREMENTAL,
    .incremental_config = velocity_incremental_config,
};
```

位置式/增量式指 PID 算法形式，不是角度环/速度环名称。三个环可分别选择，不要求相同。

## 接入顺序

```c
/* 1. 初始化 CAN BSP、DJI 总线和 module_motor_registry。 */
module_dji_motor_bus_init(&motor_bus, can, 2U);

/* 2. 填写型号、ID、方向、电流换算和三个 PID 配置。 */
module_dji_motor_config_t config = {
    .motor_name = "drive_left",
    .registration_key = 1U,
    .motor_bus = &motor_bus,
    .motor_model = MODULE_DJI_MOTOR_M3508,
    .control_mode = MODULE_DJI_CONTROL_ANGLE,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = current_scale_a_per_count,
    .current_pid_config = current_pid,
    .velocity_pid_config = velocity_pid,
    .angle_pid_config = angle_pid,
};
module_dji_motor_init(&motor, &config);
module_dji_motor_register(&motor, &registry);

/* 3. CAN 接收任务先路由反馈；收到有效反馈后再 enable。 */
module_dji_motor_bus_handle_feedback(&motor_bus, &receive_frame);
module_motor_enable(module_dji_motor_as_base(&motor));

/* 4. 设置与固定模式对应的目标，周期调用 update。 */
module_motor_set_target(module_dji_motor_as_base(&motor), target_angle_rad);
module_motor_update(module_dji_motor_as_base(&motor), delta_time_s);

/* 5. 一组电机全部 update 后统一发送。 */
module_dji_motor_bus_flush(&motor_bus);
```

## 结构体中可查看的信息

| 位置 | 信息 |
| --- | --- |
| `motor.super` (`module_motor_t`) | 电机名称、注册键、协议 ID、状态、反馈、最近 `delta_time_s`、总运行时间、累计使能时间、更新次数、最近状态 |
| `motor` (`module_dji_motor_t`) | 型号、控制模式、DJI ID、接收 CAN ID、发送组/槽位、方向、减速比、温度上限、最终 raw 命令 |
| `current_pid` / `velocity_pid` / `angle_pid` | PID 形式、配置、历史状态和 `alg_pid_terms_t` |
| `target_current_a` / `target_velocity_rad_per_s` / `target_angle_rad` | 三级控制链各级目标 |
| `module_motor_feedback_t` | 多圈角度、速度、电流、温度、原始值、在线状态和反馈计数 |

也可用 `module_dji_motor_get_current_pid()`、`get_velocity_pid()`、`get_angle_pid()` 获取已启用控制环；未启用的环返回 `NULL`。

## 运行约束

- 非直通模式必须配置有效的 `current_scale_a_per_count`，并在反馈在线后运行。
- `direction_sign` 同时规范反馈和命令方向，App 只使用逻辑正方向。
- `module_motor_update()` 只计算命令；必须调用 `module_dji_motor_bus_flush()` 才会发 CAN 帧。
- 禁用或反馈超时会清零命令。重新使能时三级 PID 按当前反馈复位，减少突跳。
- 总运行时间读取 `total_runtime_us`，实际使能时间读取 `enabled_runtime_us`；二者都使用整数微秒，避免长期 float 累加失真。
