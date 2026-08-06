# bsp_pwm

PWM 使用一份全局 platform dispatcher。每个 `bsp_pwm_t` 只保存定时器不透明句柄、通道和
初始化状态，不再继承 `bsp_device_t`。board_config 通过 `entries[]` 表驱动多通道初始化。

## 调用路径

```text
bsp_pwm_set_pulse(pwm, ticks)
  -> 边界检查（pulse ≤ period）
  -> platform_ops->set_pulse(handle, channel, ticks)
  -> HAL/寄存器
```

## 关键结构体

| 结构体 | 字段 | 说明 |
|--------|------|------|
| `bsp_pwm_t` | `device_handle`, `channel`, `is_initialized` | 轻量句柄 |
| `bsp_pwm_config_t` | `device_handle`, `driver_ops`, `channel` | 初始化配置 |
| `bsp_pwm_driver_ops_t` | `init`, `deinit`, `start`, `stop`, `set/get_frequency`, `set/get_pulse`, `get_period` | 9 个平台函数，全部必须 |

## 用法

```c
// 获取已初始化的 PWM（board_config 已调用 bsp_pwm_init）
bsp_pwm_t *buzzer = board_config_get_pwm(BOARD_CONFIG_PWM_BUZZER);

// 设置频率和占空比
bsp_pwm_set_frequency(buzzer, 4000);   // 4kHz
bsp_pwm_set_duty_cycle(buzzer, 0.5f);  // 50%

// 启动 / 停止
bsp_pwm_start(buzzer);
// ... 使用中 ...
bsp_pwm_stop(buzzer);
```

## 添加新 PWM 通道（舵机等）

**1.** `board_config.h` — 取消注释枚举槽位：
```c
typedef enum {
    BOARD_CONFIG_PWM_BUZZER = 0,
    BOARD_CONFIG_PWM_SERVO_1,    // ← 取消注释
    BOARD_CONFIG_PWM_COUNT        // 自动 = 2
} board_config_pwm_index_t;
```

**2.** `board_config.c` — 在 entries 表追加条目：
```c
[BOARD_CONFIG_PWM_SERVO_1] = {&htim2, 1U, BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL},
```

BSP/Module/App 代码不变。`board_config_get_pwm(BOARD_CONFIG_PWM_SERVO_1)` 返回可用句柄。

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_pwm_bind_platform(ops)` | 注册平台 ops |
| `bsp_pwm_init(me, cfg)` | 初始化 |
| `bsp_pwm_deinit(me)` | 反初始化（先 stop） |
| `bsp_pwm_start(me)` | 启动 PWM 输出 |
| `bsp_pwm_stop(me)` | 停止输出 |
| `bsp_pwm_set_frequency(me, hz)` | 设频率，0 返回 OUT_OF_RANGE |
| `bsp_pwm_get_frequency(me, &hz)` | 读频率 |
| `bsp_pwm_set_pulse(me, ticks)` | 设脉宽（定时器 tick），自动校验 ≤ period |
| `bsp_pwm_get_pulse(me, &ticks)` | 读脉宽 |
| `bsp_pwm_set_duty_cycle(me, 0~1)` | 设占空比（便利函数） |
| `bsp_pwm_get_duty_cycle(me, &duty)` | 读占空比 |
| `bsp_pwm_is_initialized(me)` | 状态检查 |
