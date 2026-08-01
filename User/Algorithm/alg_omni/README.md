# alg_omni —— 通用全向轮底盘运动学库（使用指南）

## 1. 模块概述

`alg_omni` 是 Algorithm 层独立的全向轮底盘运动学算法库。它不限定轮子数量、位置和驱动方向，支持任意配置的全向轮（如三轮、四轮或非对称布局）。提供完整的逆运动学、正运动学、任意旋转中心控制、轮速饱和保护及缺轮降级。

所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有数据由调用者管理。

**核心功能**：

- 任意数量、位置和驱动方向的全向轮运动学
- 逆运动学（车体速度 → 各轮角速度）
- 正运动学（各轮角速度 → 车体速度，加权最小二乘）
- 任意旋转中心（绕底盘内/外任意点）
- 轮速超限统一缩放（保持运动方向）
- 缺轮降级（可用轮数不足时可借助已知速度约束）
- 提供便捷的切向均匀布局生成函数（三轮/四轮标准底盘）

**设计哲学**：

- **零动态内存**：配置数组和工作区均由调用者提供
- **纯标准库依赖**：仅依赖 `<stdbool.h>`、`stddef.h>`、`<math.h>` 及 `alg_chassis_motion`
- **通用性**：不绑定特定轮数或几何布局
- **数值鲁棒**：加权最小二乘正解、饱和比例保持、奇异检测

---

## 2. 依赖关系

本模块依赖 `alg_chassis_motion` 模块提供的公共类型和辅助函数：

- `alg_chassis_velocity_t`：速度向量
- `alg_chassis_solution_t`：正解结果（速度 + 残差）
- `alg_chassis_constraint_t`：轮速约束
- `alg_chassis_solve_velocity()`：加权最小二乘求解器
- `alg_chassis_convert_center_velocity_to_origin()`：旋转中心速度转换
- `alg_chassis_scale_wheel_velocities()`：轮速饱和缩放

使用前需确保 `alg_chassis_motion.h` 可用并正确链接。

---

## 3. 轮子配置模型

每个轮子由 `alg_omni_wheel_config_t` 描述：

| 字段                  | 单位 | 说明                                              |
| :-------------------- | :--- | :------------------------------------------------ |
| `position_x_m`        | m    | 轮心相对车体原点的 X 坐标                         |
| `position_y_m`        | m    | 轮心相对车体原点的 Y 坐标                         |
| `drive_direction_rad` | rad  | 驱动方向（轮子产生牵引力的方向角，相对车体 x 轴） |
| `wheel_radius_m`      | m    | 轮半径（>0）                                      |
| `direction_sign`      | -    | 电机安装方向（+1 或 -1）                          |
| `odometry_weight`     | -    | 正解时的权重（>0，大值表示更信赖该轮）            |

约束方程为：

```
v_linear_i = cos(θ_i)*vx + sin(θ_i)*vy + (-sin(θ_i)*y_i + cos(θ_i)*x_i)*wz
ω_i = v_linear_i / r_i * sign_i
```

其中 `θ_i` 为驱动方向角，`(x_i, y_i)` 为轮心位置，`r_i` 为半径，`sign_i` 为方向符号。

---

## 4. 使用示例

### 4.1 标准三轮全向底盘（切向布局）

```c
#include "alg_omni.h"

#define WHEEL_COUNT 3

static alg_omni_wheel_config_t wheel_configs[WHEEL_COUNT];
static alg_omni_t chassis;

void init_chassis(void) {
    // 生成三轮切向布局：半径 0.2m，轮半径 0.05m，第一个轮在 0° 位置
    alg_omni_configure_tangential_layout(
        wheel_configs,
        WHEEL_COUNT,
        0.2f,           // 轮心到中心距离
        0.05f,          // 轮半径
        0.0f,           // 第一个轮的位置角
        1.0f,           // 切向方向（+1 为逆时针切向）
        NULL,           // 方向符号全为 +1
        1.0f            // 等权重
    );

    alg_omni_init(&chassis, wheel_configs, WHEEL_COUNT, 50.0f);
}
```

### 4.2 非对称布局（直接填写配置）

```c
alg_omni_wheel_config_t custom_configs[4] = {
    {.position_x_m = 0.1f, .position_y_m = 0.1f, .drive_direction_rad = 0.0f, .wheel_radius_m = 0.05f, .direction_sign = 1.0f, .odometry_weight = 1.0f},
    // ... 其他轮
};
```

### 4.3 逆运动学（绕原点）

```c
alg_chassis_velocity_t cmd = {0.5f, 0.2f, 0.3f};
float wheel_omega[WHEEL_COUNT];
float scale;

alg_chassis_status_t ret = alg_omni_inverse(
    &chassis,
    &cmd,
    NULL,          // 所有轮可用
    wheel_omega,
    WHEEL_COUNT,
    &scale
);
// 若 scale < 1.0，则轮速被限制
```

### 4.4 逆运动学（绕指定点）

```c
// 绕前方 0.1m 处点旋转
float center_x = 0.1f, center_y = 0.0f;
alg_chassis_velocity_t center_cmd = {0.0f, 0.0f, 0.5f};

ret = alg_omni_inverse_with_center_of_rotation(
    &chassis,
    &center_cmd,
    center_x,
    center_y,
    NULL,
    wheel_omega,
    WHEEL_COUNT,
    &scale
);
```

### 4.5 正运动学（从轮速估计速度）

```c
float measured_omega[WHEEL_COUNT] = { ... };
bool available[WHEEL_COUNT] = {true, true, false, true}; // 第三个轮失效
alg_chassis_constraint_t constraints[WHEEL_COUNT];
alg_chassis_solution_t solution;

ret = alg_omni_forward(
    &chassis,
    measured_omega,
    available,
    0,              // 无先验约束
    NULL,
    constraints,
    WHEEL_COUNT,
    &solution
);
// solution.velocity 为估计的车体速度
// solution.residual_rms 为残差均方根
```

### 4.6 带已知角速度约束的正解（缺轮时）

```c
// 当仅剩两个轮时，可用 IMU 提供角速度
uint8_t mask = (1 << 2); // bit2 = wz
alg_chassis_velocity_t known = {.angular_velocity_rad_per_s = 0.1f};

ret = alg_omni_forward(&chassis, measured_omega, available,
                       mask, &known, constraints, WHEEL_COUNT, &solution);
```

---

## 5. 缺轮降级与正解鲁棒性

- **逆解**：不可用轮输出零，可用轮仍按原方向运动，饱和时统一缩放。
- **正解**：当可用轮数 ≥ 3 且约束矩阵满秩时，可独立求解三自由度。若不足 3 个有效轮，必须提供至少一个已知速度分量（如 `wz` 来自 IMU）才能求解，否则返回 `SINGULAR`。

残差 RMS 可反映轮速传感器噪声或异常，应用层可据此动态调整 `odometry_weight` 或剔除故障轮。

---

## 6. 坐标与单位

| 量                  | 单位  |
| :------------------ | :---- |
| 位置 (`x`, `y`)     | m     |
| 线速度 (`vx`, `vy`) | m/s   |
| 角速度 (`wz`)       | rad/s |
| 轮角速度 (`ω`)      | rad/s |
| 驱动方向角          | rad   |

---

## 7. 实时性与内存

- 对象只保存配置指针和基本参数，无动态内存。
- 逆解：O(N) 次乘加（N 为轮数），适合高频控制中断。
- 正解：调用加权最小二乘求解器，复杂度 O(N×3²)，仍可满足数千 Hz 更新率。
- 工作区（约束数组）由调用者提供，可复用同一缓冲区。

---

## 8. 错误处理

所有函数返回 `alg_chassis_status_t`，常见错误：

| 状态码                                | 含义                                         |
| :------------------------------------ | :------------------------------------------- |
| `ALG_CHASSIS_STATUS_OK`               | 成功                                         |
| `ALG_CHASSIS_STATUS_INVALID_ARGUMENT` | 空指针、非法几何/权重/方向符号、输出容量不足 |
| `ALG_CHASSIS_STATUS_NOT_INITIALIZED`  | 对象未初始化                                 |
| `ALG_CHASSIS_STATUS_SINGULAR`         | 正解时约束矩阵奇异（可用轮数不足且无先验）   |
| `ALG_CHASSIS_STATUS_OUT_OF_RANGE`     | 输入速度非有限值                             |

---

## 9. 建议验证测试项

- [ ] 三轮和四轮切向布局生成与初始化
- [ ] 任意非对称位置和驱动方向
- [ ] 纯 x/y 平移、原地旋转、复合运动
- [ ] 绕不同旋转中心（原点、轮边、车外点）
- [ ] 每个单轮失效及任意双轮失效
- [ ] 轮速饱和时缩放比例保持
- [ ] 正解与逆解往返一致性（无噪声时）
- [ ] 加权正解对噪声的抑制作用
- [ ] 已知角速度辅助降级求解
- [ ] 奇异输入（如所有轮不可用）的鲁棒性

---

## 一页式使用顺序与可读信息

1. 为每个轮子填写 `alg_omni_wheel_config_t`：位置、驱动方向、轮半径和权重；规则布局可先用 tangential helper 生成。
2. 调用 `alg_omni_init()` 绑定轮配置、轮数和最大轮角速度。
3. 控制方向调用 `alg_omni_inverse()`；偏置旋转中心使用 `inverse_with_center_of_rotation()`。
4. 把目标交给电机，并采集实际轮速和轮可用性。
5. 调用 `alg_omni_forward()` 读取 `alg_chassis_solution_t`，用于里程计和轮故障残差。

| 可读取结构体                | 主要信息                               |
| --------------------------- | -------------------------------------- |
| `alg_omni_wheel_config_t[]` | 每轮几何、驱动方向、半径和权重         |
| `alg_omni_t`                | 轮配置引用、轮数、约束工作区和轮速上限 |
| 轮角速度数组                | 逆解输出或正解输入                     |
| `alg_chassis_solution_t`    | 车体速度、残差和降级状态               |

配置数组由对象长期引用，初始化后不能释放或在运行中无同步地改写。
