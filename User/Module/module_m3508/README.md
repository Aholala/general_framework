# M3508 + C620 电机模块 —— 完整使用指南

## 1. 模块概述

`module_m3508` 是大疆 M3508 电机（搭配 C620 电调）的专用派生模块，继承自 `module_dji_motor_t`（后者继承自 `module_motor_t`）。它固定电机型号为 M3508，使用 C620 电调的命令范围 `[-16000, 16000]` 和 19:1 默认减速比，防止调用者误配成 M2006 或 GM6020。

**继承关系**：

```text
module_motor_t
    └── module_dji_motor_t
            └── module_m3508_t
```

**核心功能**：

- 电流模式（直接输出原始电流命令，-16000~16000）
- 速度模式（速度 PID，单位 rad/s）
- 位置模式（位置串级 PID，单位 rad）
- 专用目标接口自动校验控制模式匹配性
- 反馈数据读取（位置、速度、电流、温度）

## 2. 设计边界

| **模块负责**                         | **模块不负责**                                       |
| :----------------------------------- | :--------------------------------------------------- |
| M3508 型号固定（默认减速比 19:1）    | 电机 PID 参数调优                                    |
| 三种控制模式的专用目标接口           | 电机散热和物理安装                                   |
| 控制模式匹配校验（防止误用）         | 运动学计算和轨迹规划                                 |
| 转发到 `module_dji_motor` 的通用功能 | CAN 总线位时序和硬件配置                             |
| C620 电调原始电流命令封装            | 电流安培值换算（需配置 `current_scale_a_per_count`） |

## 3. 协议映射

M3508（C620 电调）的 CAN 协议映射规则：

| 电机 ID | 控制帧 ID |    反馈帧 ID     |
| :------ | :-------: | :--------------: |
| 1～4    |  `0x200`  | `0x201`～`0x204` |
| 5～8    |  `0x1FF`  | `0x205`～`0x208` |

**关键约束**：

- M3508 支持电机 ID 1~8
- 控制命令为有符号 16 位原始电流命令，范围 `[-16000, 16000]`
- 反馈包含 13 位编码器值（0~8191）、转速、电流原始值和温度
- 默认减速比 `19.0F`（由 `module_dji_motor` 内部自动设置）

## 4. 电机型号对比

| 特性         | M3508              | M2006          | GM6020         |
| :----------- | :----------------- | :------------- | :------------- |
| 配套电调     | C620               | C610           | 内置驱动       |
| 减速比       | 19:1               | 36:1           | 1:1（直驱）    |
| 最大命令值   | 16000              | 10000          | 30000          |
| 命令单位     | 电流（原始值）     | 电流（原始值） | 电压（原始值） |
| 支持 ID 范围 | 1~8                | 1~8            | 1~7            |
| 典型应用     | 底盘轮毂、发射机构 | 拨弹盘、小负载 | 云台、大负载   |

## 5. API 参考

| 函数                                   | 说明                        | 返回值                        |
| :------------------------------------- | :-------------------------- | :---------------------------- |
| `module_m3508_init`                    | 初始化 M3508 电机           | `OK` / `INVALID_ARGUMENT`     |
| `module_m3508_register`                | 注册到电机注册表            | `OK` / `DUPLICATE_KEY`        |
| `module_m3508_unregister`              | 从注册表注销                | `OK` / `NOT_REGISTERED`       |
| `module_m3508_as_motor`                | 转为 `module_motor_t *`     | 基类指针                      |
| `module_m3508_as_dji_motor`            | 转为 `module_dji_motor_t *` | 大疆基类指针                  |
| `module_m3508_enable`                  | 使能电机                    | `OK` / `FEEDBACK_UNAVAILABLE` |
| `module_m3508_disable`                 | 禁用电机                    | `OK`                          |
| `module_m3508_set_current_command_raw` | 电流模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m3508_set_velocity_rad_per_s`  | 速度模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m3508_set_position_rad`        | 位置模式命令                | `OK` / `UNSUPPORTED`          |
| `module_m3508_update`                  | 周期更新                    | `OK` / `MOTOR_ERROR`          |
| `module_m3508_get_feedback`            | 获取反馈数据                | 反馈指针 / `NULL`             |
| `module_m3508_get_current_command_raw` | 获取当前电流命令            | int16 值                      |

## 6. 使用示例

### 6.1 初始化（速度模式，典型底盘应用）

```c
static module_m3508_t s_left_wheel_motor;

// 速度 PID 配置
alg_pid_config_t velocity_pid = {
    .proportional_gain = 0.5F,
    .integral_gain = 0.01F,
    .derivative_gain = 0.001F,
    .integral_limit = 100.0F,
    .output_limit = 16000.0F,      // C620 最大命令值
};

const module_m3508_config_t config = {
    .logical_name = "left_wheel_motor",
    .registration_key = 2U,
    .motor_bus = &chassis_motor_bus,     // 共享的 DJI 电机总线
    .control_mode = MODULE_M3508_CONTROL_VELOCITY,
    .motor_identifier = 1U,              // ID 1（使用 0x200 组）
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,   // 不换算电流
    .velocity_pid_config = velocity_pid,
};

module_m3508_init(&s_left_wheel_motor, &config);
module_m3508_register(&s_left_wheel_motor, &motor_registry);
module_m3508_enable(&s_left_wheel_motor);
```

### 6.2 电流模式配置

```c
// 电流模式不需要 PID 配置
const module_m3508_config_t config_current = {
    .logical_name = "left_wheel_motor",
    .registration_key = 2U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M3508_CONTROL_CURRENT,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,
};

module_m3508_init(&s_left_wheel_motor, &config_current);

// 控制时设置原始电流命令
module_m3508_set_current_command_raw(&s_left_wheel_motor, 5000);
```

### 6.3 位置模式配置

```c
// 速度 PID 配置
alg_pid_config_t velocity_pid = {
    .proportional_gain = 0.5F,
    .integral_gain = 0.01F,
    .derivative_gain = 0.001F,
    .integral_limit = 100.0F,
    .output_limit = 16000.0F,
};

// 位置串级 PID 配置
alg_pid_cascade_config_t position_pid = {
    .position_pid = {.proportional_gain = 5.0F, .integral_gain = 0.0F, .derivative_gain = 0.0F,
                     .integral_limit = 0.0F, .output_limit = 30.0F},
    .velocity_pid = velocity_pid,
};

const module_m3508_config_t config_pos = {
    .logical_name = "left_wheel_motor",
    .registration_key = 2U,
    .motor_bus = &chassis_motor_bus,
    .control_mode = MODULE_M3508_CONTROL_POSITION,
    .motor_identifier = 1U,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 80.0F,
    .current_scale_a_per_count = 0.0F,
    .position_pid_config = position_pid,
};

module_m3508_init(&s_left_wheel_motor, &config_pos);
```

### 6.4 周期调度

```c
void chassis_control_loop(float delta_time_s) {
    // 1. 设置目标（速度模式）
    module_m3508_set_velocity_rad_per_s(&s_left_wheel_motor, target_velocity_rad_per_s);
    module_m3508_set_velocity_rad_per_s(&s_right_wheel_motor, target_velocity_rad_per_s);

    // 2. 更新所有电机
    module_m3508_update(&s_left_wheel_motor, delta_time_s);
    module_m3508_update(&s_right_wheel_motor, delta_time_s);

    // 3. 统一发送 CAN 命令
    module_dji_motor_bus_flush(&chassis_motor_bus);
}
```

### 6.5 反馈读取

```c
const module_motor_feedback_t *fb = module_m3508_get_feedback(&s_left_wheel_motor);
if (fb != NULL && fb->is_online) {
    float current_position = fb->position_rad;       // 输出轴弧度（含19:1减速比）
    float current_velocity = fb->velocity_rad_per_s; // 输出轴速度（含19:1减速比）
    float temperature = fb->motor_temperature_c;     // ℃
    int16_t current_raw = fb->current_raw;           // 原始电流值（-16000~16000）
}
```

## 7. 控制模式匹配校验

模块提供控制模式匹配校验，防止误用：

| 配置模式   | 可用的目标接口            |
| :--------- | :------------------------ |
| `CURRENT`  | `set_current_command_raw` |
| `VELOCITY` | `set_velocity_rad_per_s`  |
| `POSITION` | `set_position_rad`        |

调用与初始化模式不匹配的接口会返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`。

## 8. 反馈与状态

| 数据                  | 单位  | 说明                                    |
| :-------------------- | :---- | :-------------------------------------- |
| `position_rad`        | rad   | 输出轴位置（多圈累计，含 19:1 减速比）  |
| `velocity_rad_per_s`  | rad/s | 输出轴速度（含 19:1 减速比）            |
| `current_raw`         | —     | 电流原始值（C620 协议值，-16000~16000） |
| `motor_temperature_c` | ℃     | 电机温度                                |
| `is_online`           | —     | 反馈是否在线                            |

**注意**：

- `current_a` 需正确配置 `current_scale_a_per_count` 才能转换为安培
- 位置已完成跨零累计，单位已转换为输出轴弧度（含 19:1 减速比）

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
- [ ] 电流模式命令范围 `[-16000, 16000]` 边界
- [ ] 控制模式不匹配时返回 `UNSUPPORTED`
- [ ] 与 `module_dji_motor_bus` 配合工作正常
- [ ] 反馈位置包含 19:1 减速比换算
- [ ] 过温后进入 `FAULT` 状态
- [ ] 方向符号 `direction_sign` 为 +1 或 -1

---

**总结**：`module_m3508` 是大疆 M3508 电机（C620 电调）的专用封装，通过固定型号（M3508）和减速比（19:1）防止误用。其三种控制模式（电流/速度/位置）通过专用目标接口和模式匹配校验确保类型安全。继承链 `module_motor_t → module_dji_motor_t → module_m3508_t` 使得该模块可以无缝接入电机注册表和统一调度框架。
