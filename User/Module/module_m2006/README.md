# M2006 + C610 电机模块 —— 完整使用指南

## 1. 模块概述

`module_m2006` 是大疆 M2006 电机（搭配 C610 电调）的专用派生模块，继承自 `module_dji_motor_t`（后者继承自 `module_motor_t`）。它固定电机型号为 M2006，使用 C610 电调的命令范围 `[-10000, 10000]` 和 36:1 默认减速比，防止调用者误配成 M3508 或 GM6020。

**继承关系**：

```text
module_motor_t
    └── module_dji_motor_t
            └── module_m2006_t
```

**核心功能**：

- 电流模式（直接输出原始电流命令，-10000~10000）
- 速度模式（速度 PID，单位 rad/s）
- 位置模式（位置串级 PID，单位 rad）
- 专用目标接口自动校验控制模式匹配性
- 反馈数据读取（位置、速度、电流、温度）

## 2. 设计边界

| **模块负责**                         | **模块不负责**                                       |
| :----------------------------------- | :--------------------------------------------------- |
| M2006 型号固定（默认减速比 36:1）    | 电机 PID 参数调优                                    |
| 三种控制模式的专用目标接口           | 电机散热和物理安装                                   |
| 控制模式匹配校验（防止误用）         | 运动学计算和轨迹规划                                 |
| 转发到 `module_dji_motor` 的通用功能 | CAN 总线位时序和硬件配置                             |
| C610 电调原始电流命令封装            | 电流安培值换算（需配置 `current_scale_a_per_count`） |

## 3. 协议映射

M2006（C610 电调）的 CAN 协议映射规则：

| 电机 ID | 控制帧 ID |    反馈帧 ID     |
| :------ | :-------: | :--------------: |
| 1～4    |  `0x200`  | `0x201`～`0x204` |
| 5～8    |  `0x1FF`  | `0x205`～`0x208` |

**关键约束**：

- M2006 支持电机 ID 1~8（与 GM6020 的 1~7 不同）
- 控制命令为有符号 16 位原始电流命令，范围 `[-10000, 10000]`
- 反馈包含 13 位编码器值（0~8191）、转速、电流原始值和温度
- 默认减速比 `36.0F`（由 `module_dji_motor` 内部自动设置）

## 4. API 参考

| 函数                                   | 说明                        | 返回值                        |
| :------------------------------------- | :-------------------------- | :---------------------------- |
| `module_m2006_init`                    | 初始化 M2006 电机           | `OK` / `INVALID_ARGUMENT`     |
| `module_m2006_register`                | 注册到电机注册表            | `OK` / `DUPLICATE_KEY`        |
| `module_m2006_unregister`              | 从注册表注销                | `OK` / `NOT_REGISTERED`       |
| `module_m2006_as_motor`                | 转为 `module_motor_t *`     | 基类指针                      |
| `module_m2006_as_dji_motor`            | 转为 `module_dji_motor_t *` | 大疆基类指针                  |
| `module_m2006_enable`                  | 使能电机                    | `OK` / `FEEDBACK_UNAVAILABLE` |
| `module_m2006_disable`                 | 禁用电机                    | `OK`                          |
| `module_m2006_set_current_command_raw` | 电流模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m2006_set_velocity_rad_per_s`  | 速度模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m2006_set_position_rad`        | 位置模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m2006_update`                  | 周期更新                    | `OK` / `MOTOR_ERROR`          |
| `module_m2006_get_feedback`            | 获取反馈数据                | 反馈指针 / `NULL`             |
| `module_m2006_get_current_command_raw` | 获取当前电流命令            | int16 值                      |

## 5. 使用示例

### 5.1 初始化（速度模式）

```c
static module_m2006_t s_feeder_motor;

// 速度 PID 配置
alg_pid_config_t velocity_pid = {
    .proportional_gain = 0.5F,
    .integral_gain = 0.01F,
    .derivative_gain = 0.001F,
    .integral_limit = 100.0F,
    .output_limit = 10000.0F,      // C610 最大命令值
};

const module_m2006_config_t config = {
    .logical_name = "feeder_motor",
    .registration_key = 1U,
    .motor_bus = &chassis_motor_bus,     // 共享的 DJI 电机总线
    .control_mode = MODULE_M2006_CONTROL_VELOCITY,
    .motor_identifier = 1U,              // ID 1（使用 0x200 组）
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,   // 不换算电流
    .velocity_pid_config = velocity_pid,
};

module_m2006_init(&s_feeder_motor, &config);
module_m2006_register(&s_feeder_motor, &motor_registry);
module_m2006_enable(&s_feeder_motor);
```

### 5.2 电流模式配置

```c
// 电流模式不需要 PID 配置
const module_m2006_config_t config_current = {
    .logical_name = "feeder_motor",
    .registration_key = 1U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M2006_CONTROL_CURRENT,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,
};

module_m2006_init(&s_feeder_motor, &config_current);

// 控制时设置原始电流命令
module_m2006_set_current_command_raw(&s_feeder_motor, 3000);
```

### 5.3 位置模式配置

```c
// 速度 PID 配置
alg_pid_config_t velocity_pid = {
    .proportional_gain = 0.5F,
    .integral_gain = 0.01F,
    .derivative_gain = 0.001F,
    .integral_limit = 100.0F,
    .output_limit = 10000.0F,
};

// 位置串级 PID 配置
alg_pid_cascade_config_t position_pid = {
    .position_pid = {.proportional_gain = 5.0F, .integral_gain = 0.0F, .derivative_gain = 0.0F,
                     .integral_limit = 0.0F, .output_limit = 30.0F},
    .velocity_pid = velocity_pid,
};

const module_m2006_config_t config_pos = {
    .logical_name = "feeder_motor",
    .registration_key = 1U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M2006_CONTROL_POSITION,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,
    .position_pid_config = position_pid,
};

module_m2006_init(&s_feeder_motor, &config_pos);
```

### 5.4 周期调度

```c
void control_loop(float delta_time_s) {
    // 1. 设置目标
    module_m2006_set_velocity_rad_per_s(&s_feeder_motor, target_velocity_rad_per_s);

    // 2. 更新电机
    module_m2006_update(&s_feeder_motor, delta_time_s);

    // 3. 统一发送 CAN 命令
    module_dji_motor_bus_flush(&chassis_motor_bus);
}
```

### 5.5 反馈读取

```c
const module_motor_feedback_t *fb = module_m2006_get_feedback(&s_feeder_motor);
if (fb != NULL && fb->is_online) {
    float current_position = fb->position_rad;       // 输出轴弧度（含减速比）
    float current_velocity = fb->velocity_rad_per_s; // 输出轴速度（含减速比）
    float temperature = fb->motor_temperature_c;     // ℃
    int16_t current_raw = fb->current_raw;           // 原始电流值
}
```

## 6. 控制模式匹配校验

模块提供控制模式匹配校验，防止误用：

| 配置模式   | 可用的目标接口            |
| :--------- | :------------------------ |
| `CURRENT`  | `set_current_command_raw` |
| `VELOCITY` | `set_velocity_rad_per_s`  |
| `POSITION` | `set_position_rad`        |

调用与初始化模式不匹配的接口会返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`，避免：

- 将位置误当电流
- 将速度误当位置

## 7. 反馈与状态

| 数据                  | 单位  | 说明                             |
| :-------------------- | :---- | :------------------------------- |
| `position_rad`        | rad   | 输出轴位置（多圈累计，含减速比） |
| `velocity_rad_per_s`  | rad/s | 输出轴速度（含减速比）           |
| `current_raw`         | —     | 电流原始值（C610 协议值）        |
| `motor_temperature_c` | ℃     | 电机温度                         |
| `is_online`           | —     | 反馈是否在线                     |

**注意**：

- `current_a` 需正确配置 `current_scale_a_per_count` 才能转换为安培
- 位置已完成跨零累计，单位已转换为输出轴弧度（含 36:1 减速比）
- 默认减速比 36.0F，由 `module_dji_motor` 内部自动设置

## 8. M2006 与 M3508 的区别

| 特性         | M2006                          | M3508    |
| :----------- | :----------------------------- | :------- |
| 减速比       | 36:1                           | 19:1     |
| 最大命令值   | 10000                          | 16000    |
| 控制帧 ID 组 | ID 1~4 → 0x200，ID 5~8 → 0x1FF | 同 M2006 |
| 反馈帧 ID    | ID 1~8 → 0x201~0x208           | 同 M2006 |

## 9. 安全要求

| 要求           | 说明                                                   |
| :------------- | :----------------------------------------------------- |
| **使能前检查** | 确保机械机构不会因当前位置误差突然运动                 |
| **过温保护**   | 过温后对象进入 `MODULE_MOTOR_STATE_FAULT` 并把命令清零 |
| **故障处理**   | App 应实现反馈超时、机械限位、软限位和失控急停         |
| **CAN 配置**   | CAN 位时序和具体控制器由 BSP 平台端配置                |

## 10. 错误码速查

| 错误码                 | 触发场景                   |
| :--------------------- | :------------------------- |
| `INVALID_ARGUMENT`     | 参数为空、控制模式非法     |
| `NOT_INITIALIZED`      | 对象未初始化               |
| `NOT_REGISTERED`       | 未注册到注册表             |
| `DUPLICATE_KEY`        | 注册键值重复               |
| `UNSUPPORTED`          | 调用与控制模式不匹配的命令 |
| `FEEDBACK_UNAVAILABLE` | 反馈离线或故障             |
| `TRANSPORT_ERROR`      | CAN 发送失败               |

## 11. 建议验证测试项

- [ ] `motor_identifier` 在 1~8 范围内
- [ ] 三种控制模式均能正常发送命令
- [ ] 电流模式命令范围 `[-10000, 10000]` 边界
- [ ] 控制模式不匹配时返回 `UNSUPPORTED`
- [ ] 与 `module_dji_motor_bus` 配合工作正常
- [ ] 反馈位置包含 36:1 减速比换算
- [ ] 过温后进入 `FAULT` 状态
- [ ] 方向符号 `direction_sign` 为 +1 或 -1

---

**总结**：`module_m2006` 是大疆 M2006 电机（C610 电调）的专用封装，通过固定型号（M2006）和减速比（36:1）防止误用。其三种控制模式（电流/速度/位置）通过专用目标接口和模式匹配校验确保类型安全。继承链 `module_motor_t → module_dji_motor_t → module_m2006_t` 使得该模块可以无缝接入电机注册表和统一调度框架。
