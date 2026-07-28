# alg_swerve —— 任意数量舵轮底盘运动学库（使用指南）

## 1. 模块概述

`alg_swerve` 是 Algorithm 层独立的舵轮底盘运动学算法库。它不限定模块数量或布局，支持任意几何配置的舵轮底盘（如四轮矩形、三轮三角形、非对称布局等）。提供完整的逆运动学、正运动学、参考坐标系命令、任意旋转中心、模块失效降级、舵角最短路径优化和静止自锁功能。

所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有数据由调用者管理。

**核心功能**：

- 任意数量、任意位置的舵轮模块运动学
- 逆运动学（车体速度 → 各模块轮速+舵角）
- 支持参考航向坐标系命令（车体/云台/世界坐标）
- 任意旋转中心（绕底盘内/外任意点）
- 模块失效降级（部分模块不可用时自动处理）
- 轮速统一缩放（超限时保持运动方向）
- 舵角最短路径优化（减少转向时间）
- 正运动学（各模块轮速+舵角 → 车体速度，加权最小二乘）
- 静止自锁（抵抗外力推动）

**设计哲学**：

- **零动态内存**：配置数组和工作区均由调用者提供
- **纯标准库依赖**：仅依赖 `<stdbool.h>`、`stddef.h>`、`<math.h>` 及 `alg_chassis_motion`
- **通用性**：不绑定特定模块数量或几何布局
- **数值鲁棒**：加权最小二乘正解、角度回绕、饱和比例保持

---

## 2. 依赖关系

本模块依赖 `alg_chassis_motion` 模块提供的公共类型和辅助函数：

- `alg_chassis_velocity_t`：速度向量
- `alg_chassis_solution_t`：正解结果（速度 + 残差）
- `alg_chassis_constraint_t`：速度约束
- `alg_chassis_solve_velocity()`：加权最小二乘求解器

使用前需确保 `alg_chassis_motion.h` 可用并正确链接。

---

## 3. 舵轮模型

每个舵轮模块由几何位置 `(x_i, y_i)` 定义。逆解时，根据刚体运动学公式计算模块在车体坐标系中的速度矢量：

```
vx_i = vx_body - ω * (y_i - y_center)
vy_i = vy_body + ω * (x_i - x_center)
speed_i = hypot(vx_i, vy_i)
steering_angle_i = atan2(vy_i, vx_i)
```

其中 `(x_center, y_center)` 为旋转中心坐标。

---

## 4. 使用示例

### 4.1 标准四轮矩形布局初始化

```c
#include "alg_swerve.h"

#define MODULE_COUNT 4

static alg_swerve_module_geometry_t geometry[MODULE_COUNT];
static alg_swerve_t chassis;

void init_chassis(void) {
    // 生成矩形布局：半轴距0.2m，半轮距0.15m
    alg_swerve_configure_rectangular_layout(geometry, 0.2f, 0.15f);
    alg_swerve_init(&chassis, geometry, MODULE_COUNT, 1.0f); // 最大轮速1m/s
}
```

### 4.2 自定义非对称布局

```c
alg_swerve_module_geometry_t custom_geometry[3] = {
    {.position_x_m = 0.2f, .position_y_m = 0.1f},
    {.position_x_m = -0.1f, .position_y_m = 0.2f},
    {.position_x_m = -0.1f, .position_y_m = -0.1f}
};
alg_swerve_init(&chassis, custom_geometry, 3, 1.0f);
```

### 4.3 逆运动学（车体系命令）

```c
alg_swerve_command_t cmd = {
    .velocity_x_m_per_s = 0.5f,
    .velocity_y_m_per_s = 0.2f,
    .angular_velocity_rad_per_s = 0.3f,
    .reference_heading_rad = 0.0f,
    .center_of_rotation_x_m = 0.0f,
    .center_of_rotation_y_m = 0.0f,
    .command_is_reference_relative = false   // 车体系
};
alg_swerve_module_target_t targets[MODULE_COUNT];

alg_swerve_status_t ret = alg_swerve_calculate(&chassis, &cmd, targets, MODULE_COUNT);
// ret == ALG_SWERVE_STATUS_OK 表示所有模块可用
```

### 4.4 逆运动学（参考航向坐标系）

```c
cmd.command_is_reference_relative = true;
cmd.reference_heading_rad = 0.5f;  // 云台方向
// 此时 velocity_x/y 被视为相对参考航向的速度
alg_swerve_calculate(&chassis, &cmd, targets, MODULE_COUNT);
```

### 4.5 逆运动学（任意旋转中心 + 模块失效）

```c
bool available[MODULE_COUNT] = {true, true, false, true}; // 左后模块失效
cmd.center_of_rotation_x_m = 0.1f;  // 绕前方0.1m处
cmd.center_of_rotation_y_m = 0.0f;

ret = alg_swerve_calculate_with_availability(
    &chassis, &cmd, available, targets, MODULE_COUNT);
// ret == ALG_SWERVE_STATUS_DEGRADED 表示部分模块不可用
// targets[2] 的轮速为0，舵角为0
```

### 4.6 舵角最短路径优化（发送前调用）

```c
float current_steering_angle = get_module_steering_angle(module_index);
alg_swerve_optimize_target(current_steering_angle, &targets[module_index]);
// 优化后，转向角度差不超过 ±90°，若反转则轮速取反
```

### 4.7 静止自锁

```c
alg_swerve_module_target_t lock_targets[MODULE_COUNT];
alg_swerve_calculate_self_lock(&chassis, lock_targets, MODULE_COUNT);
// 各模块舵角指向中心，轮速为0
```

### 4.8 正运动学（从实测状态估计车体速度）

```c
alg_swerve_module_target_t measured[MODULE_COUNT] = { ... }; // 来自编码器反馈
bool available[MODULE_COUNT] = {true, true, true, true};
float weights[MODULE_COUNT] = {1.0f, 1.0f, 0.8f, 0.8f}; // 不同权重
alg_chassis_constraint_t constraints[2 * MODULE_COUNT];
alg_chassis_solution_t solution;

alg_chassis_status_t status = alg_swerve_forward(
    &chassis,
    measured,
    available,
    weights,
    0,           // 无先验
    NULL,
    constraints,
    2 * MODULE_COUNT,
    &solution
);
// solution.velocity 为估计的车体速度
// solution.residual_rms 为残差均方根
```

---

## 5. 缺模块降级与正解鲁棒性

- **逆解**：不可用模块输出零，其余模块按原运动方向计算，超限时统一缩放。
- **正解**：每个模块提供两个约束（x/y方向速度），可用模块数不足 3 个（即约束数不足 3）时，需要提供已知速度分量（如 IMU 角速度）才能求解，否则返回 `SINGULAR`。

残差 RMS 可用于检测模块异常或地面打滑。

---

## 6. 坐标与单位

| 量                  | 单位  |
| :------------------ | :---- |
| 位置 (`x`, `y`)     | m     |
| 线速度 (`vx`, `vy`) | m/s   |
| 角速度 (`wz`)       | rad/s |
| 轮线速度            | m/s   |
| 舵角                | rad   |

---

## 7. 实时性与内存

- 对象只保存配置指针和基本参数，无动态内存。
- 逆解：O(N) 次乘加和 atan2，适合高频控制中断（N 为模块数）。
- 正解：调用加权最小二乘求解器（约束数 2N），复杂度 O(N×3²)，可满足数 kHz 更新率。
- 工作区（约束数组）由调用者提供，可复用。

---

## 8. 错误处理

| 状态码                               | 含义                                   |
| :----------------------------------- | :------------------------------------- |
| `ALG_SWERVE_STATUS_OK`               | 成功（所有模块可用）                   |
| `ALG_SWERVE_STATUS_DEGRADED`         | 部分模块不可用，已降级                 |
| `ALG_SWERVE_STATUS_INVALID_ARGUMENT` | 空指针、非法数值、容量不足             |
| `ALG_SWERVE_STATUS_NOT_INITIALIZED`  | 对象未初始化                           |
| `ALG_CHASSIS_STATUS_SINGULAR`        | 正解时约束奇异（可用约束不足且无先验） |

---

## 9. 建议验证测试项

- [ ] 矩形四轮布局初始化
- [ ] 三轮、非对称布局
- [ ] 车体系命令（x/y平移、旋转、复合）
- [ ] 参考航向坐标系命令
- [ ] 绕原点、绕单轮、绕车外点旋转
- [ ] 各模块单独失效及组合失效
- [ ] 舵角最短路径优化（跨 ±π 边界）
- [ ] 静止自锁角度计算
- [ ] 正解与逆解往返一致性
- [ ] 加权正解对噪声的抑制
- [ ] 先验速度约束（已知vx/wz）辅助降级

---

**总结**：`alg_swerve` 提供了高度灵活的舵轮底盘运动学解决方案，适用于各种舵轮布局（包括非标准布局），并集成了参考坐标系、任意旋转中心、模块失效处理和舵角优化等实用功能。其轻量级、零动态内存的实现使其成为嵌入式实时控制系统的理想选择。
