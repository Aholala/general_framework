# module_dm_motor — 达妙电机（DM4310）

继承 `module_motor_t`。MIT 模式（位置+速度+扭矩+KP+KD）、位置-速度模式和力位混合控制。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_dm_motor_t` | DM 电机基类（继承 `module_motor_t`） |
| `module_dm_motor_bus_t` | CAN 总线（管理多电机命令+反馈） |
| `module_dm4310_t/config_t` | DM-J4310-2EC 派生 |
| `module_dm_mit_command_t` | MIT 命令：`position_rad`, `velocity_rad_per_s`, `torque_nm`, `kp`, `kd` |
| `module_dm_force_position_command_t` | 力位控制命令 |

## 用法

```c
module_dm_motor_bus_t bus;
module_dm_motor_bus_init(&bus, can, timeout);

module_dm4310_t motor;
module_dm4310_config_t cfg = {
    .super = {
        .bus = &bus, .motor_name = "joint1", .registration_key = 1,
        .motor_identifier = 1,  // CAN ID
        .feedback_timeout_ms = 100,
    },
};
module_dm4310_init(&motor, &cfg);
module_dm_motor_register(&motor.super, &reg);

// MIT 模式命令
module_dm_mit_command_t cmd = {
    .position_rad = 1.0f, .velocity_rad_per_s = 0, .torque_nm = 0,
    .kp = 10.0f, .kd = 1.0f,
};
module_dm_motor_set_mit_command(&motor.super, &cmd);

// 总线发送
module_dm_motor_bus_flush(&bus);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_dm_motor_bus_init(bus, can, timeout)` | 初始化总线 |
| `module_dm_motor_bus_handle_feedback(bus, frame)` | 分发反馈帧 |
| `module_dm_motor_bus_flush(bus)` | 打包发送 |
| `module_dm4310_init(me, cfg)` | 初始化电机 |
| `module_dm_motor_register(me, reg)` | 注册 |
| `module_dm_motor_set_mit_command(me, cmd)` | MIT 命令 |
| `module_dm_motor_set_force_position_command(me, cmd)` | 力位命令 |
| `module_dm_motor_as_base(me)` | → `module_motor_t *` |
