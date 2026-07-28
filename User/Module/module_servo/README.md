# 标准 PWM 舵机控制模块 (module_servo) —— 完整使用指南

## 1. 模块概述

`module_servo` 是一个标准 PWM 舵机控制模块，把弧度角度或归一化命令映射为微秒脉宽，并通过 `bsp_pwm_t` 输出。支持配置最小/中位/最大脉宽以及对应的角度范围，适配各种常见舵机（如 SG90、MG995、数字舵机等）。

**核心功能**：

- 角度到脉宽的线性映射（`set_angle`）
- 归一化输出到脉宽的映射（`set_normalized_output`）
- 直接设置微秒脉宽（`set_pulse_width`）
- 查询最后命令角度（`get_commanded_angle`）
- 启动/停止控制

## 2. 设计边界

| **模块负责**           | **模块不负责**                       |
| :--------------------- | :----------------------------------- |
| 角度→脉宽→占空比的转换 | 舵机位置反馈读取                     |
| PWM 频率和占空比设置   | 闭环控制（需配合编码器和控制器）     |
| 参数边界校验和钳位     | 机械限位保护（App 不应依赖堵转限位） |
| 开环命令输出           | 多个舵机通道的频率同步管理           |

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_servo_t                 (舵机对象：PWM、脉宽参数、角度参数)
```

## 4. 核心参数

### 4.1 典型舵机配置（50Hz，20ms 周期）

| 舵机类型      | 最小脉宽 | 中位脉宽 | 最大脉宽 | 角度范围    |
| :------------ | :------- | :------- | :------- | :---------- |
| SG90 标准舵机 | 500 us   | 1500 us  | 2500 us  | -90° ~ +90° |
| 某些数字舵机  | 600 us   | 1500 us  | 2400 us  | -90° ~ +90° |
| 非对称舵机    | 500 us   | 1450 us  | 2400 us  | -90° ~ +90° |

### 4.2 配置约束

- `minimum_pulse_width_us < neutral_pulse_width_us < maximum_pulse_width_us`
- `minimum_angle_rad < maximum_angle_rad`
- `maximum_pulse_width_us * frequency_hz < 1,000,000`（保证占空比 < 100%）

## 5. 控制接口

### 5.1 set_angle（角度控制）

```c
// 设置目标角度（弧度），自动钳位到配置范围
module_servo_set_angle(&servo, target_angle_rad);
```

映射关系：角度 → 位置比例 → 脉宽（线性插值）

### 5.2 set_normalized_output（归一化控制）

```c
// 输入 [-1.0, 1.0]，用于速度/位置控制器的输出
module_servo_set_normalized_output(&servo, controller_output);
```

映射关系：

- `> 0`：中位到最大脉宽
- `< 0`：中位到最小脉宽
- `0`：中位脉宽

### 5.3 set_pulse_width（直接脉宽控制）

```c
// 直接设置微秒脉宽（须在 min ~ max 范围内）
module_servo_set_pulse_width(&servo, 1500.0F);
```

## 6. 使用示例

### 6.1 初始化与启动

```c
static module_servo_t s_servo;

const module_servo_config_t cfg = {
    .pwm = board_servo_pwm,              // 已初始化的 PWM 基类
    .frequency_hz = 50,                   // 标准舵机 50Hz
    .minimum_pulse_width_us = 500.0F,
    .neutral_pulse_width_us = 1500.0F,
    .maximum_pulse_width_us = 2500.0F,
    .minimum_angle_rad = -1.5708F,        // -90°
    .maximum_angle_rad = 1.5708F,         // +90°
    .logical_name = "arm_servo",
    .registration_key = 0,
};

module_servo_init(&s_servo, &cfg);
module_servo_start(&s_servo);
```

### 6.2 控制舵机

```c
// 移动到 45°（0.7854 rad）
module_servo_set_angle(&s_servo, 0.7854F);

// 或使用归一化控制（控制器输出 0.5 → 中位到最大的一半）
module_servo_set_normalized_output(&s_servo, 0.5F);
```

### 6.3 查询命令角度

```c
float current_angle;
module_servo_get_commanded_angle(&s_servo, &current_angle);
// 注意：这是命令值，不是实测值
```

## 7. 注意事项

- **无反馈**：普通 PWM 舵机没有位置反馈，`commanded_angle_rad` 只是命令值，不代表实际角度。需要闭环控制时应额外接入编码器。
- **频率共享**：多个舵机通道可能共享同一定时器频率。平台配置必须保证所有通道频率兼容。
- **安全启动**：启动时输出中位脉宽，避免突发运动。
- **停止行为**：停止后 PWM 输出行为取决于 BSP 平台的配置（可能保持最后电平、高阻或固定电平）。

## 8. 错误码速查

| 状态码             | 触发场景                                                       |
| :----------------- | :------------------------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、脉宽参数非法、角度范围非法、脉宽超过周期、角度为 NaN |
| `NOT_INITIALIZED`  | 对象未初始化                                                   |
| `NOT_STARTED`      | 未调用 `start`                                                 |
| `TRANSPORT_ERROR`  | PWM 设置频率/占空比失败、启动/停止失败                         |

## 9. 建议验证测试项

- [ ] 最小、中位、最大角度输出正确脉宽
- [ ] 非对称脉宽映射（中位不在正中间）
- [ ] 归一化正负端点映射正确
- [ ] 直接微秒脉宽设置在范围内
- [ ] 超范围输入被钳位（不返回错误，而是限制到边界）
- [ ] 启动后输出中位脉宽
- [ ] 停止后 PWM 输出安全状态
- [ ] 多通道共享定时器时频率兼容

---

**总结**：`module_servo` 提供了简洁的舵机开环控制接口，通过配置脉宽和角度的映射关系，适配各种标准 PWM 舵机。三种控制接口（角度、归一化、直接脉宽）满足不同应用场景的需求。配合 `module_device` 基类，可统一接入系统调度。
