# module_servo — PWM 舵机

开环 PWM 控制。把角度/归一化值映射为微秒脉宽。无反馈，`get_commanded_angle()` 返回的是命令值。

## 关键结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `module_servo_t` | 舵机对象 | `pwm`, `frequency_hz`(常用 50), `minimum/maximum_pulse_width_us`, `neutral_pulse_width_us`, `commanded_angle_rad` |
| `module_servo_config_t` | 配置 | 同上 + `minimum/maximum_angle_rad`, `logical_name` |

## 用法

```c
module_servo_t servo;
module_servo_config_t cfg = {
    .pwm = board_config_get_pwm(BOARD_CONFIG_PWM_SERVO_1),
    .frequency_hz = 50,                     // 50Hz 标准舵机
    .minimum_pulse_width_us = 500,          // 0° = 500µs
    .neutral_pulse_width_us  = 1500,        // 中位
    .maximum_pulse_width_us = 2500,         // 180° = 2500µs
    .minimum_angle_rad = -1.57f,            // -90°
    .maximum_angle_rad = 1.57f,             // +90°
};
module_servo_init(&servo, &cfg);
module_servo_start(&servo);  // 输出中位脉宽

// 设角度
module_servo_set_angle(&servo, 0.5f);  // 0.5 rad（自动钳位）

// 设归一化输出 [-1, 1]（常用于控制器输出）
module_servo_set_normalized_output(&servo, 0.3f);

// 直接设脉宽
module_servo_set_pulse_width(&servo, 1800.0f);  // 1800µs

// 读取命令值（非实测）
float angle;
module_servo_get_commanded_angle(&servo, &angle);

// 停止
module_servo_stop(&servo);
```

## 脉宽映射

```
角度 → 位置比例 → 脉宽
-90° → 0.0 → 500µs
  0° → 0.5 → 1500µs (neutral)
+90° → 1.0 → 2500µs
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_servo_init(me, cfg)` | 初始化 |
| `module_servo_start(me)` | 设频率 + 输出中位脉宽 |
| `module_servo_stop(me)` | 关闭 PWM |
| `module_servo_set_angle(me, rad)` | 设角度（自动钳位） |
| `module_servo_set_normalized_output(me, v)` | 设归一化 [-1,1] |
| `module_servo_set_pulse_width(me, us)` | 设脉宽（微秒） |
| `module_servo_get_commanded_angle(me, &rad)` | 读命令角度 |
