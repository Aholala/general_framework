# 底盘运动学模块 —— 完整使用指南

## 1. 模块概述

`alg_chassis_motion` 是底盘运动学的公共数学内核，各类底盘把轮子测量转换为线性约束，本模块负责速度求解、坐标变换、任意旋转中心转换、轮速统一缩放和里程计积分。`alg_chassis_wheel_monitor` 根据轮速残差判断车轮异常与恢复。

**核心功能**：

- **加权最小二乘速度求解**：使用 QR 分解求解约束方程组
- **坐标变换**：参考系 ↔ 车体坐标系
- **旋转中心转换**：任意点速度 → 原点速度
- **轮速缩放**：统一缩放所有轮速，保持比例
- **里程计积分**：Euler / Midpoint / Exact 三种方法
- **车轮监测**：基于残差的故障/恢复滞回检测

**设计哲学**：

- **通用约束模型**：每个轮子或传感器提供形如 `a*vx + b*vy + c*wz = measured` 的约束
- **鲁棒求解**：支持加权、禁用约束、已知分量锁定
- **数值稳定**：使用列缩放和 QR 分解
- **无动态内存**：所有数组由调用者分配

## 2. 坐标约定

| 符号                            | 含义           | 单位  |
| :------------------------------ | :------------- | :---- |
| `velocity_x_m_per_s`            | 车体 X 轴速度  | m/s   |
| `velocity_y_m_per_s`            | 车体 Y 轴速度  | m/s   |
| `angular_velocity_rad_per_s`    | 绕 Z 轴角速度  | rad/s |
| `position_x_m` / `position_y_m` | 参考坐标系位置 | m     |
| `heading_rad`                   | 车体航向角     | rad   |

旋转方向和车体轴方向必须由项目统一，所有轮组模型使用同一约定。

## 3. 约束求解

### 3.1 约束模型

单个轮子或传感器提供约束：

```
a * vx + b * vy + c * wz = measured_velocity
```

对应 `alg_chassis_constraint_t` 的三个系数、测量值、权重和可用状态。

### 3.2 已知分量锁定

`known_component_mask` 可锁定一个或多个已知分量：

- 差速底盘：`vy = 0`（固定横向速度为零）
- 全向底盘：全部未知（`mask = 0`）
- 已知航向速度：锁定 `wz`

### 3.3 求解结果

| 字段                                | 说明                         |
| :---------------------------------- | :--------------------------- |
| `velocity`                          | 求解出的速度                 |
| `residual_root_mean_square_m_per_s` | 残差均方根，衡量拟合质量     |
| `used_constraint_count`             | 实际使用的约束数             |
| `unknown_component_count`           | 未知分量数                   |
| `is_degraded`                       | 是否降级（约束数少于名义值） |

## 4. 坐标变换

### 4.1 参考系 → 车体系

```c
alg_chassis_velocity_t body_velocity;
alg_chassis_transform_reference_to_body(&ref_velocity, heading_rad, &body_velocity);
```

将世界坐标系或云台坐标系下的速度命令旋转到车体坐标系，角速度保持不变。

### 4.2 旋转中心转换

```c
alg_chassis_velocity_t origin_velocity;
alg_chassis_convert_center_velocity_to_origin(&center_velocity, cx, cy, &origin_velocity);
```

将旋转中心处的速度转换到车体原点。旋转中心可以位于：

- 底盘中心
- 某个轮子
- 云台投影点
- 车体外部

## 5. 车轮状态监测

`alg_chassis_wheel_monitor` 使用轮速残差和传感器在线状态生成稳定的 `wheel_is_available` 数组。残差连续超过故障阈值后标记异常，连续低于恢复阈值后恢复，两个阈值形成滞回，避免状态反复跳变。

### 5.1 状态机

```text
                    +-------------------+
                    |   NORMAL          |
                    | (is_faulted=false) |
                    +--------+----------+
                             |
                    残差 >= 故障阈值
                    累积故障计数
                    +--------+----------+
                    |   达到确认样本数   |
                    +--------+----------+
                             |
                             v
                    +--------+----------+
                    |   FAULT           |
                    | (is_faulted=true) |
                    +--------+----------+
                             |
                    残差 <= 恢复阈值
                    累积恢复计数
                    +--------+----------+
                    |   达到确认样本数   |
                    +--------+----------+
                             |
                             v
                    +-------------------+
                    |   NORMAL          |
                    +-------------------+
```

### 5.2 滞回设计

- 故障阈值 > 恢复阈值（形成滞回）
- 避免残差在阈值附近反复跳变
- 确认样本数用于防抖

### 5.3 使用示例

```c
static alg_chassis_wheel_monitor_state_t wheel_states[4];
static alg_chassis_wheel_monitor_t wheel_monitor;

alg_chassis_wheel_monitor_init(&wheel_monitor, &monitor_config);
alg_chassis_wheel_monitor_update(&wheel_monitor,
                                 wheel_residuals_m_per_s,
                                 sensor_is_available,
                                 wheel_is_available,
                                 4U);
```

## 6. 使用示例

### 6.1 四轮全向底盘速度求解

```c
alg_chassis_constraint_t constraints[4] = {
    // 左前轮
    {.velocity_x_coefficient = 1.0F, .velocity_y_coefficient = 1.0F,
     .angular_velocity_coefficient_m = 0.3F, .measured_velocity_m_per_s = wheel0_speed,
     .weight = 1.0F, .is_available = true},
    // 右前轮
    {.velocity_x_coefficient = 1.0F, .velocity_y_coefficient = -1.0F,
     .angular_velocity_coefficient_m = -0.3F, .measured_velocity_m_per_s = wheel1_speed,
     .weight = 1.0F, .is_available = true},
    // 左后轮
    {.velocity_x_coefficient = -1.0F, .velocity_y_coefficient = 1.0F,
     .angular_velocity_coefficient_m = 0.3F, .measured_velocity_m_per_s = wheel2_speed,
     .weight = 1.0F, .is_available = true},
    // 右后轮
    {.velocity_x_coefficient = -1.0F, .velocity_y_coefficient = -1.0F,
     .angular_velocity_coefficient_m = -0.3F, .measured_velocity_m_per_s = wheel3_speed,
     .weight = 1.0F, .is_available = true},
};

alg_chassis_solution_t solution;
alg_chassis_solve_velocity(constraints, 4, 0, NULL, 4, &solution);
// solution.velocity.vx, vy, wz 即为底盘速度
```

### 6.2 差速底盘（固定横向速度为零）

```c
const uint8_t known_mask = ALG_CHASSIS_COMPONENT_VELOCITY_Y;
alg_chassis_velocity_t known_vel = {.velocity_y_m_per_s = 0.0F};

alg_chassis_solve_velocity(constraints, 2, known_mask, &known_vel, 2, &solution);
```

### 6.3 轮速缩放

```c
float wheel_velocities[4] = {2.0F, 2.5F, 3.0F, 2.0F};
bool available[4] = {true, true, true, false};  // 右后轮不可用
float scale;

alg_chassis_scale_wheel_velocities(wheel_velocities, available, 4, 2.5F, &scale);
// wheel_velocities 被统一缩放到最大 2.5 m/s
// scale = 2.5 / 3.0 = 0.833
```

### 6.4 里程计积分

```c
alg_chassis_pose_t pose = {.position_x_m = 0.0F, .position_y_m = 0.0F, .heading_rad = 0.0F};
alg_chassis_velocity_t body_vel = {.velocity_x_m_per_s = 1.0F, .velocity_y_m_per_s = 0.0F,
                                   .angular_velocity_rad_per_s = 0.5F};

// 使用中点积分（推荐）
alg_chassis_integrate_odometry(&pose, &body_vel, 0.01F, ALG_CHASSIS_INTEGRATION_MIDPOINT);
```

## 7. 错误码速查

| 状态码             | 触发场景                           |
| :----------------- | :--------------------------------- |
| `OK`               | 操作成功                           |
| `DEGRADED`         | 可用约束数少于名义值（但仍可求解） |
| `INVALID_ARGUMENT` | 参数为空、非有限数、负权重等       |
| `UNDERDETERMINED`  | 约束数少于未知分量数               |
| `SINGULAR`         | 矩阵奇异（约束线性相关）           |
| `NUMERICAL_ERROR`  | 数值异常（溢出、除零等）           |

## 8. 文件职责

| 文件                          | 说明                                     |
| :---------------------------- | :--------------------------------------- |
| `alg_chassis_motion.h`        | 公共类型和 API 声明                      |
| `alg_chassis_motion.c`        | 速度求解、坐标变换、轮速缩放、里程计积分 |
| `alg_chassis_wheel_monitor.h` | 车轮监测类型和 API                       |
| `alg_chassis_wheel_monitor.c` | 车轮异常确认、恢复和可用性输出           |

## 9. 建议验证测试项

- [ ] 三个单独速度分量的求解
- [ ] 组合运动（vx + vy + wz）的求解
- [ ] 已知分量掩码的所有组合
- [ ] 不同权重、禁用约束
- [ ] 中心、轮边和车体外旋转中心转换
- [ ] 参考系旋转 0、±π/2 和 π
- [ ] 三种里程计积分方法
- [ ] 欠约束、共线约束和非有限输入
- [ ] 车轮监测的故障/恢复滞回
- [ ] 传感器不可用直接标记故障

---

## 一页式使用顺序与可读信息

1. 各运动学模块先把轮速转换成 `alg_chassis_constraint_t[]`，并给出可用轮标志。
2. 调用 `alg_chassis_solve_velocity()` 得到 `alg_chassis_solution_t`；约束不足时检查降级标志而不是盲用全部速度分量。
3. 用 `alg_chassis_calculate_constraint_residuals()` 计算逐轮残差。
4. 把残差交给 `alg_chassis_wheel_monitor_update()`，经过故障/恢复防抖后得到轮可用性。
5. 下一周期把新的可用性反馈给麦轮、全向轮或舵轮解算，形成降级闭环。

| 可读取结构体                              | 主要信息                                             |
| ----------------------------------------- | ---------------------------------------------------- |
| `alg_chassis_velocity_t`                  | X/Y 线速度和 Z 角速度                                |
| `alg_chassis_pose_t`                      | 里程计位置和偏航角                                   |
| `alg_chassis_solution_t`                  | 解算速度、RMS 残差、使用约束数、未知分量数和降级标志 |
| `alg_chassis_wheel_monitor_wheel_state_t` | 每轮故障/恢复计数和故障标志                          |

残差和轮故障只是估计结果，最终停机或降级策略由 App 决定。
