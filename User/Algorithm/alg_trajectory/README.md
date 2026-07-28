# alg_trajectory —— 一维轨迹生成与多轴同步库 —— 完整使用指南

## 1. 模块概述

`alg_trajectory` 是 Algorithm 层独立的一维轨迹生成及多轴同步控制库。提供单轴梯形速度 / S 曲线规划，以及多轴同步五次多项式插值。所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，每个轨迹对象独立状态，支持多实例。

**核心功能**：

- **单轴轨迹生成（`alg_trajectory_t`）**：
  - 位置目标（到达指定位置，可指定终端速度）
  - 速度目标（逼近目标速度并保持）
  - 两种剖面：梯形速度（恒加/减速）和 S 曲线（加加速度限制）
  - 在线平滑切换目标（不中断当前运动）
  - 制动距离计算（用于限位预判）

- **多轴同步（`alg_trajectory_group_t`）**：
  - 任意数量轴同时起步、同时停止
  - 每轴独立配置速度、加速度、加加速度限制
  - 五次多项式插值，保证位置、速度、加速度连续且端值为零
  - 自动计算共同持续时间（最短时间规划）

**设计哲学**：

- **零动态内存**：所有配置、状态、工作区均由调用者静态分配
- **纯标准库依赖**：仅依赖 `<stdbool.h>`、`<stddef.h>`、`<math.h>`
- **多实例支持**：可同时运行多个独立轨迹
- **实时性**：纯算术运算，无迭代，适合高频控制环
- **显式错误返回**：所有接口返回状态码

---

## 2. 单轴轨迹生成器

### 2.1 配置参数

| 参数                          | 单位     | 说明                           |
| :---------------------------- | :------- | :----------------------------- |
| `maximum_velocity_per_s`      | 单位/秒  | 最大速度（>0）                 |
| `maximum_acceleration_per_s2` | 单位/秒² | 最大加速度（>0）               |
| `maximum_deceleration_per_s2` | 单位/秒² | 最大减速度（>0）               |
| `maximum_jerk_per_s3`         | 单位/秒³ | 最大加加速度（>0，S 曲线有效） |
| `position_tolerance`          | 单位     | 位置到达容差（>=0）            |
| `velocity_tolerance_per_s`    | 单位/秒  | 速度到达容差（>=0）            |

### 2.2 使用流程

```c
#include "alg_trajectory.h"

static alg_trajectory_config_t cfg = {
    .maximum_velocity_per_s = 1.0f,
    .maximum_acceleration_per_s2 = 2.0f,
    .maximum_deceleration_per_s2 = 2.0f,
    .maximum_jerk_per_s3 = 5.0f,
    .position_tolerance = 0.001f,
    .velocity_tolerance_per_s = 0.01f
};

static alg_trajectory_state_t init = {0.0f, 0.0f, 0.0f};
static alg_trajectory_t traj;

void init_trajectory(void) {
    alg_trajectory_init(&traj, &cfg, ALG_TRAJECTORY_PROFILE_S_CURVE, &init);
}

void move_to(float target) {
    alg_trajectory_set_position_target(&traj, target, 0.0f);
}

void control_loop(float dt) {
    alg_trajectory_state_t cmd;
    alg_trajectory_status_t status = alg_trajectory_update(&traj, dt, &cmd);
    // 输出 cmd.position 给执行器
    if (status == ALG_TRAJECTORY_STATUS_FINISHED) {
        // 到达目标
    }
}
```

### 2.3 在线目标切换

任何时候可调用 `set_position_target` 或 `set_velocity_target`，生成器会从当前状态继续规划（不重置）。切换后 `is_finished` 变为 `false`。

```c
// 在运动过程中改变目标
alg_trajectory_set_position_target(&traj, new_target, 0.0f);
```

### 2.4 速度模式

```c
alg_trajectory_set_velocity_target(&traj, 0.5f); // 持续以 0.5 单位/秒 运动
```

速度模式不会停止，当速度接近目标且加速度归零后，`is_finished` 变为 `true`，之后将持续保持该速度。

### 2.5 制动距离

```c
float stop_dist = alg_trajectory_calculate_stopping_distance(current_speed, decel_limit);
// 如果 stop_dist >= 剩余距离，需提前减速或触发限位
```

---

## 3. 多轴同步轨迹组

### 3.1 原理

同步组采用五次多项式时间缩放，使所有轴同时从起点运动到终点，且起点和终点的速度、加速度均为零。每轴独立限制，总持续时间取所有轴所需时间中的最大值。

```
s_i(t) = start_i + Δi * (10τ³ - 15τ⁴ + 6τ⁵)
v_i(t) = Δi * (30τ² - 60τ³ + 30τ⁴) / T
a_i(t) = Δi * (60τ - 180τ² + 120τ³) / T²
```

其中 τ = t / T，T 为共同持续时间。

### 3.2 使用流程

```c
#include "alg_trajectory_group.h"

#define AXIS_COUNT 2

static alg_trajectory_config_t axis_cfgs[AXIS_COUNT] = {
    {1.0f, 2.0f, 2.0f, 5.0f, 0.001f, 0.01f},
    {1.5f, 3.0f, 3.0f, 6.0f, 0.001f, 0.01f}
};
static alg_trajectory_state_t init_states[AXIS_COUNT] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}
};
static alg_trajectory_config_t config_storage[AXIS_COUNT];
static alg_trajectory_state_t state_storage[AXIS_COUNT];
static float start_pos[AXIS_COUNT];
static float target_pos[AXIS_COUNT];
static alg_trajectory_group_t group;

void init_group(void) {
    alg_trajectory_group_init(&group,
                              config_storage,
                              state_storage,
                              start_pos,
                              target_pos,
                              AXIS_COUNT,
                              init_states,
                              axis_cfgs);
}

void move_to_sync(float target1, float target2) {
    float targets[AXIS_COUNT] = {target1, target2};
    alg_trajectory_group_set_target(&group, targets);
}

void control_loop(float dt) {
    alg_trajectory_status_t status = alg_trajectory_group_update(&group, dt);
    // 读取每轴状态
    const alg_trajectory_state_t *s1 = alg_trajectory_group_get_state(&group, 0);
    const alg_trajectory_state_t *s2 = alg_trajectory_group_get_state(&group, 1);
    // 输出 s1->position, s2->position ...
    if (alg_trajectory_group_is_finished(&group)) {
        // 同步完成
    }
}
```

### 3.3 在线重新规划

可随时调用 `alg_trajectory_group_set_target()` 重新设置目标，生成器会以当前位置作为新起点重新计算同步时间。

---

## 4. 数值实现细节

### 4.1 单轴 S 曲线

- 采用加加速度限制的“制动距离”策略，实时判断应加速、减速还是匀速。
- 加速度变化率受 `maximum_jerk_per_s3` 限制。
- 微分滤波：无，直接使用加速度指令积分，保证响应快速。

### 4.2 同步组时间估算

时间估算使用解析近似公式：

- 速度限制主导：`T_v = 1.875 * |Δ| / Vmax`
- 加速度限制主导：`T_a = sqrt(5.7735 * |Δ| / Amax)`
- 加加速度限制主导：`T_j = cbrt(60 * |Δ| / Jmax)`

取三者的最大值作为该轴所需时间。所有轴的 `T` 取最大，确保所有轴满足自身约束。

### 4.3 完成判定

- **单轴位置模式**：位置误差 ≤ `position_tolerance` 且速度误差 ≤ `velocity_tolerance_per_s`，且加速度已衰减至接近零。
- **单轴速度模式**：速度误差 ≤ `velocity_tolerance_per_s`，且加速度归零（S 曲线需等待 jerk 过渡）。
- **同步组**：时间到达 `duration_s` 后自动完成。

---

## 5. 实时性建议

- **单轴更新**：约 20 次浮点乘加，O(1)，适合数 kHz 控制环。
- **同步组更新**：每轴 3 次乘加，O(N)，N 通常≤6，适合 kHz 级。
- **时间估算**：`alg_trajectory_group_set_target` 中每轴调用一次 sqrt/cbrt，计算量稍大，但仅在目标切换时执行，不影响周期控制。

---

## 6. 并发约束

- 每个轨迹对象（`alg_trajectory_t` 或 `alg_trajectory_group_t`）**不能**被多个任务并发更新。
- 推荐在单一任务或中断中完成所有更新。

---

## 7. 验证建议

- [ ] 单轴正反向位置阶跃（梯形与 S 曲线对比）
- [ ] 非零初速度和终端速度（如连续轨迹衔接）
- [ ] 运行中反向切换目标
- [ ] 速度模式（匀速跟踪）
- [ ] 控制周期抖动（dt 变化）
- [ ] 极短距离、零距离命令
- [ ] 单轴制动距离计算与仿真对比
- [ ] 多轴同步：不同位移、不同限制的轴同时起停
- [ ] 同步组在线重新规划
- [ ] 非法参数（NaN、负值、空指针）的鲁棒性

---

**总结**：`alg_trajectory` 提供了轻量级、高性能的一维轨迹生成及多轴同步能力，适用于各种运动控制场景（云台、底盘、机械臂等）。其零动态内存、纯 C11 的实现和丰富的规划选项，使其成为嵌入式实时运动控制的理想基础模块。
