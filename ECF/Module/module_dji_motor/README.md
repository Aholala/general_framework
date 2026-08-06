# module_dji_motor — DJI 电机（M2006/M3508/GM6020）

继承 `module_motor_t`。CAN 总线分组 + 三级串级 PID（角度→速度→电流）。

## 架构

```
module_dji_motor_bus_t（一条 CAN 总线, 最多 4 个槽位）
  ├── slot[0] → module_dji_motor_t（如 M3508 云台偏航）
  ├── slot[1] → module_dji_motor_t（如 M3508 云台俯仰）
  ├── slot[2] → module_dji_motor_t（如 M2006 拨弹）
  └── slot[3] → module_dji_motor_t（如 GM6020 底盘）

bus_handle_feedback(frame) → 按 ID 分发到槽位电机
bus_flush() → 打包所有槽位命令 → CAN 发送
```

## 三种电机对比

| 型号 | 电调 | 反馈帧 ID | 命令帧 ID | 控制方式 |
|------|------|-----------|-----------|---------|
| M2006 | C610 | 0x201~0x204 | 0x200 | 电流 |
| M3508 | C620 | 0x201~0x204 | 0x200(1-4) / 0x1FF(5-8) | 电流 |
| GM6020 | 内置 | 0x205~0x208 | 0x1FF(1-4) / 0x2FF(5-8) | 电压 |

## 用法

```c
// 1. 初始化总线
module_dji_motor_bus_t bus;
module_dji_motor_bus_init(&bus, board_config_get_can(BOARD_CONFIG_CAN_1), 5);

// 2. 初始化电机（以 M3508 为例）
module_motor_registry_t reg;
module_motor_t *storage[10];
module_motor_registry_init(&reg, storage, 10);

module_m3508_t pitch_motor;
module_m3508_config_t cfg = {
    .super = {
        .bus = &bus,
        .motor_name = "pitch",
        .registration_key = 1,
        .motor_identifier = 0x202,     // CAN ID
        .direction = 1,                 // 1=正向, -1=反向
        .current_limit_a = 8.0f,
        .feedback_timeout_ms = 100,
        .enable_current_pid = true,
        .enable_velocity_pid = true,
        .enable_angle_pid = true,
        .current_pid  = { .form = POSITIONAL, .positional_config = {...} },
        .velocity_pid = { .form = POSITIONAL, .positional_config = {...} },
        .angle_pid    = { .form = POSITIONAL, .positional_config = {...} },
    },
    .position_offset_rad = 0.0f,
};
module_m3508_init(&pitch_motor, &cfg);
module_dji_motor_register(&pitch_motor.super, &reg);

// 3. CAN 接收循环（在 FreeRTOS 任务中）
bsp_can_frame_t frame;
while (bsp_can_receive(can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK) {
    module_dji_motor_bus_handle_feedback(&bus, &frame);
}

// 4. 设目标 + 更新
module_motor_set_target(&pitch_motor.super, 0.5f);  // 0.5 rad
module_motor_update(&pitch_motor.super, 0.001f);     // PID 更新

// 5. 发送命令
module_dji_motor_bus_flush(&bus);  // 打包所有槽位 → CAN 发送

// 6. 读取反馈
const module_motor_feedback_t *fb = module_motor_get_feedback(&pitch_motor.super);

// 7. PID 分量调试
const module_motor_pid_t *pid = module_dji_motor_get_current_pid(&pitch_motor.super);
const alg_pid_terms_t *terms = module_motor_pid_get_terms(pid);
```

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_dji_motor_bus_t` | CAN 总线：4 槽位 + 命令打包 |
| `module_dji_motor_t` | DJI 电机基类（继承 `module_motor_t`） |
| `module_dji_motor_config_t` | 通用配置：PID 使能、方向、CAN 映射、限幅 |
| `module_m2006_t/config_t` | M2006 派生 |
| `module_m3508_t/config_t` | M3508 派生 |
| `module_gm6020_t/config_t` | GM6020 派生 |

## 三级 PID 控制链

```
目标角度 → [角度 PID] → 目标速度 → [速度 PID] → 目标电流 → [电流 PID] → CAN 命令
                                              ↑ 可旁路（set_target 直接设速度）
                          ↑ 可旁路（set_target 直接设电流）
```

通过 `enable_angle_pid` / `enable_velocity_pid` / `enable_current_pid` 选择启用的级数。

## API 速查

| 函数 | 功能 |
|------|------|
| `module_dji_motor_bus_init(bus, can, timeout)` | 初始化总线 |
| `module_dji_motor_bus_handle_feedback(bus, frame)` | 分发 CAN 反馈帧 |
| `module_dji_motor_bus_flush(bus)` | 打包发送所有槽位命令 |
| `module_m2006_init/m3508_init/gm6020_init(me, cfg)` | 初始化电机 |
| `module_dji_motor_register/unregister(me, reg)` | 注册/注销到总线+注册表 |
| `module_dji_motor_as_base(me)` | 向上转型 → `module_motor_t *` |
| `module_dji_motor_reset_position(me, rad)` | 重定义当前位置 |
| `module_dji_motor_get_command(me)` | 获取当前 CAN 命令值 |
| `module_dji_motor_get_current/velocity/angle_pid(me)` | 获取各级 PID 对象 |
