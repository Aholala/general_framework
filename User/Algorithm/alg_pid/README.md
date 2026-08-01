# alg_pid —— 通用 PID 控制器库 —— 完整使用指南

## 1. 模块概述

`alg_pid` 是 Algorithm 层独立的高级 PID 控制算法库，提供位置式、增量式、串级、增益调度和模糊自适应等多种 PID 变体。所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有控制器实例状态独立，支持多路控制环并发。

**核心功能**：

- **位置式 PID（`alg_pid_t`）**：二自由度比例/微分、微分先行、积分分离、死区、前馈（速度/加速度/外部）、多种抗积分饱和（条件积分/反算）、输出/积分限幅、微分滤波。
- **增量式 PID（`alg_pid_incremental_t`）**：输出增量形式，适用于累加型执行器；支持增量限幅、总输出限幅、微分滤波、死区、外部前馈增量。
- **串级 PID（`alg_pid_cascade_t`）**：位置环 + 速度环串级，支持外环降频、速度环设定值限幅、双环独立前馈。
- **增益调度 PID（`alg_pid_gain_schedule_t`）**：根据工作点线性插值 Kp、Ki、Kd。
- **模糊自适应 PID（`alg_pid_fuzzy_t`）**：基于归一化误差和误差变化率，通过二维规则表在线调整 Kp、Ki、Kd。
- **无扰切换**：`alg_pid_track_output()` 实现手动/自动模式平滑切换。
- **调试支持**：`alg_pid_terms_t` 提供 P/I/D/FF 各项分量观测。

**设计哲学**：

- **零动态内存**：所有配置、状态、工作区均由调用者静态分配。
- **纯标准库依赖**：仅依赖 `<stdbool.h>`、`stddef.h>`、`<stdint.h>`、`<math.h>`。
- **连续时间增益**：`Ki`、`Kd` 为连续时间形式，内部根据实际 `delta_time_s` 离散化，采样周期变化无需重算增益。
- **多实例支持**：每个控制器对象独立状态，可创建任意多个控制环。
- **显式错误返回**：所有接口返回状态码，便于诊断。

---

## 2. 功能清单

| 功能特性                             | 支持情况                                         |
| :----------------------------------- | :----------------------------------------------- |
| 比例项（P）                          | ✅                                               |
| 积分项（I）                          | ✅（连续时间增益）                               |
| 微分项（D）                          | ✅（连续时间增益 + 一阶低通滤波）                |
| 二自由度比例（设定值权重 β）         | ✅（0~1）                                        |
| 二自由度微分（设定值权重 γ）         | ✅（0~1）                                        |
| 微分模式选择                         | 对误差微分 / 对测量值微分（微分先行）            |
| 积分分离（阈值）                     | ✅                                               |
| 误差死区                             | ✅                                               |
| 速度前馈（Kvff）                     | ✅                                               |
| 加速度前馈（Kaff）                   | ✅                                               |
| 外部前馈（`additional_feedforward`） | ✅                                               |
| 输出限幅（min/max）                  | ✅                                               |
| 积分限幅（min/max）                  | ✅                                               |
| 抗积分饱和                           | 无 / 条件积分(Clamping) / 反算(Back-calculation) |
| 微分滤波                             | ✅（一阶低通，截止频率可调）                     |
| 无扰切换（输出跟踪）                 | ✅                                               |
| 运行时参数更新                       | ✅（`alg_pid_set_config`）                       |
| 调试分量观测                         | ✅（`alg_pid_terms_t`）                          |
| **增量式 PID**                       | ✅（含增量限幅、总输出限幅、微分滤波）           |
| **串级 PID**                         | ✅（位置环 + 速度环，外环降频）                  |
| **角度串级 PID**                     | ✅（角度外环 + 角速度内环的专用封装）            |
| **增益调度 PID**                     | ✅（线性插值 Kp/Ki/Kd）                          |
| **模糊自适应 PID**                   | ✅（二维规则表双线性插值）                       |

---

## 3. 配置与初始化

### 3.1 位置式 PID 配置

推荐先调用 `alg_pid_config_init()` 获取默认配置，再修改需要的参数：

```c
#include "alg_pid.h"

static alg_pid_velocity_t s_speed_controller;

void init_speed_controller(void) {
    alg_pid_config_t cfg;
    alg_pid_config_init(&cfg);

    cfg.proportional_gain = 2.0f;           // Kp
    cfg.integral_gain = 10.0f;              // Ki (连续时间)
    cfg.derivative_gain = 0.01f;            // Kd (连续时间)
    cfg.setpoint_weight = 1.0f;             // β (默认1)
    cfg.derivative_setpoint_weight = 0.0f;  // γ (默认0，即微分先行)
    cfg.derivative_filter_cutoff_hz = 50.0f; // 微分滤波
    cfg.output_min = -20.0f;
    cfg.output_max = 20.0f;
    cfg.integral_min = -5.0f;
    cfg.integral_max = 5.0f;
    cfg.anti_windup_mode = ALG_PID_ANTI_WINDUP_CLAMPING;
    cfg.derivative_mode = ALG_PID_DERIVATIVE_ON_MEASUREMENT;

    alg_pid_velocity_init(&s_speed_controller, &cfg);
}
```

**默认配置**：

- 所有增益为 0
- `setpoint_weight` = 1.0
- `derivative_setpoint_weight` = 0.0（微分先行）
- 微分滤波关闭（0 Hz）
- 积分分离关闭（阈值 0）
- 输出/积分限幅：±INF
- 抗积分饱和：`CLAMPING`
- 微分模式：`ON_MEASUREMENT`（对测量值微分）

### 3.2 增量式 PID 配置

```c
static alg_pid_incremental_t s_inc_controller;

void init_incremental(void) {
    alg_pid_incremental_config_t cfg;
    alg_pid_incremental_config_init(&cfg);

    cfg.proportional_gain = 1.0f;
    cfg.integral_gain = 5.0f;
    cfg.derivative_gain = 0.0f;
    cfg.delta_output_min = -1.0f;
    cfg.delta_output_max = 1.0f;
    cfg.output_min = -100.0f;
    cfg.output_max = 100.0f;

    alg_pid_incremental_init(&s_inc_controller, &cfg);
}
```

---

## 4. 使用示例

### 4.1 位置式 PID（简单更新）

```c
float output;
alg_pid_update(&controller, setpoint, measurement, delta_time_s, &output);
```

### 4.2 位置式 PID（高级更新，含前馈）

```c
alg_pid_input_t input = {
    .setpoint = target_position,
    .measurement = measured_position,
    .setpoint_rate_per_s = target_velocity,
    .setpoint_acceleration_per_s2 = target_acceleration,
    .additional_feedforward = gravity_compensation,
    .delta_time_s = control_period_s
};
alg_pid_update_advanced(&controller, &input, &output);
```

最终前馈 = `Kvff * setpoint_rate + Kaff * setpoint_accel + additional_feedforward`。

### 4.3 无扰切换（手动→自动）

```c
// 在切换前，用当前手动输出反算积分项
alg_pid_track_output(&controller,
                     current_setpoint,
                     current_measurement,
                     current_ff,
                     manual_output);
// 然后切换到自动模式，输出将平滑过渡
alg_pid_update(&controller, setpoint, measurement, dt, &output);
```

### 4.4 增量式 PID

```c
float output;
alg_pid_incremental_reset(&inc_ctrl, current_output); // 初始化当前输出
// 每周期：
alg_pid_incremental_update(&inc_ctrl, setpoint, measurement,
                            feedforward_delta, dt, &output);
```

### 4.5 串级 PID

```c
alg_pid_cascade_config_t cascade_cfg = {
    .position_config = pos_cfg,
    .velocity_config = vel_cfg,
    .position_loop_divider = 5,    // 位置环 200Hz，速度环 1kHz
    .velocity_setpoint_min = -10.0f,
    .velocity_setpoint_max = 10.0f
};
alg_pid_cascade_t cascade;
alg_pid_cascade_init(&cascade, &cascade_cfg);

alg_pid_cascade_input_t input = {
    .position_setpoint = pos_target,
    .position_measurement = pos_measure,
    .velocity_measurement = vel_measure,
    .velocity_feedforward = vel_ff,      // 叠加到速度设定值
    .actuator_feedforward = act_ff,      // 叠加到最终输出
    .delta_time_s = 0.001f
};
float output;
alg_pid_cascade_update(&cascade, &input, &output);
```

#### 4.5.1 角度串级 PID 封装

角度控制封装直接属于 `alg_pid`：

```c
alg_pid_angle_config_t angle_config = {
    .cascade_config = cascade_cfg,
};
alg_pid_angle_t angle_controller;
alg_pid_angle_init(&angle_controller, &angle_config);
alg_pid_angle_reset(&angle_controller, current_angle_rad,
                    current_velocity_rad_per_s, current_output);

alg_pid_angle_input_t angle_input = {
    .target_position_rad = target_angle_rad,
    .target_velocity_rad_per_s = target_velocity_rad_per_s,
    .measured_position_rad = current_angle_rad,
    .measured_velocity_rad_per_s = current_velocity_rad_per_s,
    .actuator_feedforward = feedforward,
    .delta_time_s = 0.001F,
};
alg_pid_angle_update(&angle_controller, &angle_input, &output);
```

可通过 `alg_pid_angle_get_velocity_setpoint()` 观察角度外环生成的角速度目标。

### 4.6 增益调度 PID

```c
static const alg_pid_gain_point_t gains[] = {
    {0.0f,   1.0f, 0.5f, 0.01f},
    {50.0f,  2.0f, 0.8f, 0.02f},
    {100.0f, 3.0f, 1.0f, 0.03f}
};
alg_pid_gain_schedule_t gs;
alg_pid_gain_schedule_init(&gs, &base_config, gains, 3);

// 每周期根据当前工作点更新
alg_pid_gain_schedule_update(&gs, operating_point, &input, &output);
```

### 4.7 模糊自适应 PID

```c
// 规则表为 5x5（axis_point_count=5），行=误差，列=误差变化率
static const float kp_table[5*5] = { ... };
static const float ki_table[5*5] = { ... };
static const float kd_table[5*5] = { ... };

alg_pid_fuzzy_config_t fuzzy_cfg = {
    .base_config = base_cfg,
    .proportional_adjustment_table = kp_table,
    .integral_adjustment_table = ki_table,
    .derivative_adjustment_table = kd_table,
    .axis_point_count = 5,
    .error_normalization = 10.0f,
    .error_rate_normalization = 20.0f
};
alg_pid_fuzzy_t fuzzy;
alg_pid_fuzzy_init(&fuzzy, &fuzzy_cfg);
alg_pid_fuzzy_update(&fuzzy, &input, &output);
```

---

## 5. 参数调优建议

- **位置环**：通常以 P 或 PD 为主，输出速度目标；积分项视稳态精度需求。
- **速度环**：通常使用 PI，输出力矩/电流；D 项在速度反馈噪声大时慎用。
- **微分先行**：`derivative_setpoint_weight = 0` 避免设定值阶跃冲击。
- **设定值权重 β**：减小 β 可降低超调，但会减弱对扰动的快速响应。
- **抗积分饱和**：条件积分（`CLAMPING`）参数少，适合多数场景；反算（`BACK_CALCULATION`）需调 `back_calculation_gain`，通常设为 `sqrt(Kp)` 量级。
- **积分分离阈值**：设为期望误差范围，当误差较大时暂停积分，防止积分过冲。
- **死区**：仅用于消除测量噪声导致的微小抖动，过大会引入静态误差。

---

## 6. 数值实现细节

- **增益离散化**：积分项 = `Ki * error * dt`，微分项 = `Kd * (Δerror / dt)`，均为连续时间增益，适应变周期。
- **微分滤波**：一阶低通，截止频率 `fc`，时间常数 `τ = 1/(2π·fc)`，采用后向欧拉离散。
- **抗积分饱和**：
  - `CLAMPING`：输出饱和且误差同向时暂停积分累加。
  - `BACK_CALCULATION`：`integral += Kb * (saturated - unsaturated) * dt`，将饱和误差反馈回积分。
- **对称化**：积分限幅与输出限幅独立。
- **数值检查**：所有输入、中间变量和输出均检查 `isfinite()`，防止 NaN/Inf 传播。

---

## 7. 实时性建议

- **位置式/增量式 PID**：纯算术运算（+ - \* /），O(1)，适合高频率中断（kHz 级）。
- **串级 PID**：每个周期执行速度环，位置环按降频执行，计算量等效于两个 PID。
- **增益调度 PID**：每次需查找区间 + 线性插值，O(log N)（N 为增益点数），适合数 kHz 调用。
- **模糊自适应 PID**：每次需双线性插值查表，O(1)（表尺寸固定），计算量略高于普通 PID，但仍适合 kHz 级。

---

## 8. 并发约束

- 每个控制器对象（`alg_pid_t`、`alg_pid_incremental_t` 等）**不能**被多个执行上下文同时修改。
- 若需多任务共享，使用互斥锁或通过消息传递（如将更新操作放入同一任务）。

---

## 9. 建议验证测试项

- [ ] 位置环阶跃响应（超调、稳态误差、调节时间）
- [ ] 速度环正弦跟踪（幅频/相频特性）
- [ ] 积分限幅与输出限幅行为
- [ ] 条件积分与反算抗饱和（含饱和退出）
- [ ] 二自由度比例项（β 调参）
- [ ] 微分先行（设定值阶跃时微分冲击）
- [ ] 速度前馈与加速度前馈
- [ ] 积分分离与死区
- [ ] 无扰切换（跟踪输出）
- [ ] 增量式 PID（累积输出、增量限幅）
- [ ] 串级 PID（外环降频、速度限幅）
- [ ] 增益调度（插值连续性）
- [ ] 模糊自适应（规则表插值）
- [ ] 运行中动态改参
- [ ] 非法参数与未初始化对象防护

---

## 一页式使用顺序与可读信息

1. 先用 `alg_pid_config_init()` 取得完整默认值，再设置 Kp/Ki/Kd、限幅、死区、滤波和抗积分饱和。
2. 根据对象用途调用 `alg_pid_init()`、`alg_pid_velocity_init()` 或 `alg_pid_position_init()`。
3. 每周期准备 `alg_pid_input_t`，传入目标、反馈、前馈和真实 `delta_time_s`，调用 `alg_pid_update_advanced()`；简单场景可用 `alg_pid_update()`。
4. 把返回输出经过 App 安全限制后交给电机；执行器实际饱和时可用 `alg_pid_track_output()` 回算。
5. 模式切换、重新使能和反馈突变后调用 reset，避免旧积分继续作用。

| 可读取结构体                            | 主要信息                                                              |
| --------------------------------------- | --------------------------------------------------------------------- |
| `alg_pid_terms_t`                       | P/I/D/前馈分量、限幅前输出和最终输出，通过 `alg_pid_get_terms()` 获取 |
| `alg_pid_t`                             | 积分、上次误差/测量、微分滤波和配置；仅调试读取                       |
| `alg_pid_incremental_t`                 | 增量式 PID 的误差历史、输出和分项                                     |
| `alg_pid_cascade_t` / `alg_pid_angle_t` | 内外环状态、速度设定值和最终输出                                      |

调参时同时观察各分项和饱和状态，不能只看最终输出。
