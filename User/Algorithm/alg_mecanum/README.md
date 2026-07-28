# alg_mecanum —— 四轮麦克纳姆底盘运动学库

## 1. 模块概述

`alg_mecanum` 是 Algorithm 层独立的四轮麦克纳姆轮底盘运动学算法库。它提供了完整的逆运动学（从期望机体速度到各轮角速度）、正运动学（从实测轮速估计机体速度）以及任意旋转中心控制功能，并支持 X/O 两种辊子布局、轮速饱和保护以及缺轮降级运行。

所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有数据由调用者管理。

**核心功能**：

- 四轮麦克纳姆底盘全向逆解（机体速度 → 轮角速度）
- 支持**任意旋转中心**（绕底盘内或外任意点旋转）
- X 型和 O 型辊子排列
- 轮速饱和缩放（保持运动方向）
- 基于加权最小二乘的正运动学（轮速 → 机体速度）
- 缺轮降级（任意单轮或双轮失效）
- 支持先验速度约束（已知部分自由度）

**设计哲学**：

- **零动态内存**：所有运算仅使用对象内部预计算系数和调用者提供的栈/静态数组
- **纯标准库依赖**：仅依赖 `<stdbool.h>`、`stddef.h>`、`<math.h>` 及 `alg_chassis_motion` 公共类型
- **多实例支持**：可同时创建多个不同配置的底盘模型
- **数值鲁棒**：加权最小二乘正解、饱和比例保持、缺轮检测

---

## 2. 依赖关系

本模块依赖 `alg_chassis_motion` 模块提供的公共类型和辅助函数：

- `alg_chassis_velocity_t`：速度向量（vx, vy, wz）
- `alg_chassis_solution_t`：正解结果（速度 + 残差）
- `alg_chassis_constraint_t`：轮速约束
- `alg_chassis_solve_velocity()`：加权最小二乘求解器
- `alg_chassis_convert_center_velocity_to_origin()`：旋转中心速度转换
- `alg_chassis_scale_wheel_velocities()`：轮速饱和缩放

使用前需确保 `alg_chassis_motion.h` 可用并已正确初始化相关辅助函数。

---

## 3. 轮序与几何定义

固定轮序为（按枚举顺序）：

| 索引 | 名称       |
| :--: | :--------- |
|  0   | 左前（FL） |
|  1   | 右前（FR） |
|  2   | 左后（RL） |
|  3   | 右后（RR） |

几何参数定义在 `alg_mecanum_config_t` 中：

- `wheel_radius_m`：轮半径（米），>0
- `half_wheelbase_m`：半轴距（纵向距离的一半），>0
- `half_track_width_m`：半轮距（横向距离的一半），>0
- `direction_sign[4]`：各轮安装方向（+1 或 -1），用于处理电机安装反向
- `odometry_weight[4]`：正解时各轮的权重（>0），可用来给更可靠的轮子更高权重
- `maximum_wheel_angular_velocity_rad_per_s`：轮角速度上限（>0）
- `roller_arrangement`：辊子排列类型（X 型或 O 型）

**辊子排列影响**：

- X 型：横向运动时对角轮同向，系数 `lateral_coefficient` 为 `[-1, 1, 1, -1]`
- O 型：横向运动时对角轮反向，系数为 `[1, -1, -1, 1]`

---

## 4. 模型方程

麦克纳姆轮运动学方程（以底盘原点速度 `vx, vy, wz` 表示）：

对每个轮子 i：

```
v_linear_i = vx + K_ly_i * vy + K_ang_i * wz
ω_i = v_linear_i / r * direction_sign_i
```

其中：

- `K_ly_i` 为横向速度系数（由辊子布局决定，±1）
- `K_ang_i` 为角速度系数（由几何和辊子布局共同决定，单位米）
- `r` 为轮半径

**任意旋转中心**时，先将旋转中心处的速度转换到原点，再应用上式。

---

## 5. 使用示例

### 5.1 初始化

```c
#include "alg_mecanum.h"

// 定义配置（以 X 型布局为例）
alg_mecanum_config_t config = {
    .wheel_radius_m = 0.05f,
    .half_wheelbase_m = 0.15f,
    .half_track_width_m = 0.13f,
    .direction_sign = {1.0f, 1.0f, 1.0f, 1.0f},  // 全部正装
    .odometry_weight = {1.0f, 1.0f, 1.0f, 1.0f}, // 等权重
    .maximum_wheel_angular_velocity_rad_per_s = 50.0f,
    .roller_arrangement = ALG_MECANUM_ROLLER_X
};

alg_mecanum_t chassis;
alg_chassis_status_t status = alg_mecanum_init(&chassis, &config);
if (status != ALG_CHASSIS_STATUS_OK) {
    // 处理错误
}
```

### 5.2 逆运动学（绕底盘中心）

```c
alg_chassis_velocity_t target = {
    .velocity_x_m_per_s = 0.5f,
    .velocity_y_m_per_s = 0.2f,
    .angular_velocity_rad_per_s = 0.3f
};

bool available[4] = {true, true, true, true};  // 四轮均正常
float wheel_omega[4];
float applied_scale;

alg_chassis_status_t ret = alg_mecanum_inverse(
    &chassis,
    &target,
    available,
    wheel_omega,
    &applied_scale
);

// 若 applied_scale < 1.0f，说明某轮超速，所有轮按比例缩放
// 不可用轮对应的 wheel_omega 输出为 0
```

### 5.3 逆运动学（绕任意旋转中心）

```c
// 假设要绕底盘前方 0.1m 处的点旋转
float center_x = 0.1f;
float center_y = 0.0f;
alg_chassis_velocity_t center_velocity = {
    .velocity_x_m_per_s = 0.0f,
    .velocity_y_m_per_s = 0.0f,
    .angular_velocity_rad_per_s = 0.5f
};

ret = alg_mecanum_inverse_with_center_of_rotation(
    &chassis,
    &center_velocity,
    center_x,
    center_y,
    available,
    wheel_omega,
    &applied_scale
);
// 此时底盘将绕 (0.1, 0) 点旋转
```

### 5.4 正运动学（从轮速估计机体速度）

```c
float measured_omega[4] = { ... };  // 实测轮角速度
bool available[4] = {true, true, false, true}; // 左后轮失效
uint8_t known_mask = 0;  // 不施加先验约束
alg_chassis_solution_t solution;

ret = alg_mecanum_forward(
    &chassis,
    measured_omega,
    available,
    known_mask,
    NULL,  // 无先验速度
    &solution
);

if (ret == ALG_CHASSIS_STATUS_OK) {
    // solution.velocity 为估计的车体速度
    // solution.residual_rms 为加权残差均方根
}
```

### 5.5 带先验约束的正解

若已知部分自由度（如垂直方向速度为零），可使用 `known_component_mask` 和 `known_velocity` 约束解空间。

```c
// 已知 vx 必须为 0.5 m/s
uint8_t mask = (1 << 0); // bit0 对应 vx
alg_chassis_velocity_t known = {.velocity_x_m_per_s = 0.5f};

ret = alg_mecanum_forward(&chassis, measured_omega, available,
                          mask, &known, &solution);
```

---

## 6. 缺轮降级与正解鲁棒性

- **逆解**：不可用轮输出零，其余轮按相同方向比例缩放。`applied_scale` 反映饱和情况。
- **正解**：基于加权最小二乘，自动处理缺轮。只要可用轮数 ≥ 3 且约束矩阵满秩，即可解出三自由度。若仅剩 2 个轮，则需借助先验约束（如已知某速度分量）才可求解。

应用层应结合轮速监测模块，对持续出现大残差的轮子降权或剔除，避免传感器故障污染估计。

---

## 7. 坐标与单位

| 量                   | 单位              |
| :------------------- | :---------------- |
| 线速度 (`vx`, `vy`)  | m/s               |
| 角速度 (`wz`)        | rad/s             |
| 轮角速度 (`ω`)       | rad/s             |
| 长度（半径、轴距等） | m                 |
| 旋转中心坐标         | m（相对底盘原点） |

方向符号由 `direction_sign` 和辊子布局共同决定，调用者需确保与电机接线及坐标系定义一致。

---

## 8. 实时性与内存

- 对象大小：仅存储配置和 8 个预计算系数（`lateral_coefficient` 和 `angular_coefficient_m`），无动态内存。
- 计算开销：
  - 逆解：每个轮子仅需一次乘加，以及一次轮速缩放，适合高频控制中断。
  - 正解：内部调用加权最小二乘求解器（复杂度 O(4×3²)），仍可满足数 kHz 更新率。
- 多实例：可同时运行多个底盘模型（如主/备或不同配置）。

---

## 9. 错误处理

所有函数返回 `alg_chassis_status_t`，常见错误包括：

| 状态码                                | 含义                                                |
| :------------------------------------ | :-------------------------------------------------- |
| `ALG_CHASSIS_STATUS_OK`               | 成功                                                |
| `ALG_CHASSIS_STATUS_INVALID_ARGUMENT` | 空指针、非法几何参数（≤0）、权重非正、方向符号非 ±1 |
| `ALG_CHASSIS_STATUS_NOT_INITIALIZED`  | 对象未初始化                                        |
| `ALG_CHASSIS_STATUS_SINGULAR`         | 正解时约束矩阵奇异（可用于检测缺轮过多）            |
| `ALG_CHASSIS_STATUS_OUT_OF_RANGE`     | 输入速度非有限值                                    |

调用者应检查返回值，尤其在正解时处理奇异情况。

---

## 10. 建议验证测试项

- [ ] 纯 x 方向直线运动（所有轮速相等）
- [ ] 纯 y 方向横向运动（对角轮同向/反向取决于辊子布局）
- [ ] 原地旋转（四轮速对称）
- [ ] 复合运动（斜向 + 旋转）
- [ ] X 型和 O 型布局对比
- [ ] 方向符号反向配置
- [ ] 绕不同旋转中心（中心、轮边、车外点）
- [ ] 单轮失效时逆解输出（该轮置零，其余缩放）
- [ ] 单轮、双轮失效时正解精度
- [ ] 轮速饱和时缩放比例保持
- [ ] 正解与逆解往返一致性（在无噪声时）
- [ ] 加权正解对噪声的抑制作用
- [ ] 先验速度约束的正确性
- [ ] 奇异输入（如所有轮不可用）的鲁棒性

---

**总结**：`alg_mecanum` 为麦克纳姆底盘提供了完整且高效的运动学解决方案，支持灵活的旋转中心控制、缺轮降级和加权正解。其轻量级、无动态内存的实现非常适合嵌入式实时控制，可轻松集成到机器人底盘控制系统中。
