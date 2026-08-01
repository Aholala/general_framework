# 姿态估计算法 (alg_attitude) 模块

## 1. 模块概述

`alg_attitude` 是一个面向六轴 IMU（加速度计 + 陀螺仪）的轻量级姿态估计算法库，支持 **Mahony 互补滤波** 和 **Madgwick 梯度下降滤波** 两种算法。它用于 EKF 尚未完成初始化、算力降级或赛场故障恢复等场景，不替代 `alg_imu_ekf` 的协方差估计。

**核心功能**：

- 支持 Mahony 互补滤波和 Madgwick 梯度下降滤波
- 加速度模长异常时自动退化为纯陀螺积分
- 外部航向注入（磁航向、视觉航向或机构约束）
- 输出欧拉角（roll/pitch/yaw）和旋转矩阵
- 多实例支持（可同时运行多个独立姿态估计器）

**设计哲学**：

- **轻量级**：适合算力受限的嵌入式系统
- **鲁棒性**：加速度异常时自动降级，避免错误修正
- **可注入**：外部航向可通过 `alg_attitude_correct_yaw()` 注入
- **无动态内存**：所有状态由调用者分配

## 2. 设计边界

| **模块负责**                      | **模块不负责**                          |
| :-------------------------------- | :-------------------------------------- |
| 基于陀螺仪和加速度计的姿态估计    | 磁力计数据融合（由 `alg_imu_ekf` 负责） |
| Mahony / Madgwick 两种滤波算法    | 协方差估计和不确定性量化                |
| 外部航向信息注入                  | 传感器校准（零偏、标度因数）            |
| 四元数归一化和欧拉角/旋转矩阵输出 | IMU 硬件初始化或数据采集                |
| 加速度异常时的自动降级处理        | 传感器轴对齐（由调用者保证）            |

**重要限制**：六轴 IMU 无法仅靠重力长期观测 yaw，这是物理限制。外部航向（磁力计、视觉或机械约束）必须通过 `alg_attitude_correct_yaw()` 定期注入，否则 yaw 会随时间漂移。

## 3. 核心类型

### 3.1 配置结构 (`alg_attitude_config_t`)

```c
typedef struct {
    alg_attitude_method_t method;           // 算法选择：MAHONY 或 MADGWICK
    float proportional_gain;                // Mahony 比例增益（Kp）
    float integral_gain;                    // Mahony 积分增益（Ki）
    float madgwick_beta;                    // Madgwick 梯度下降步长（β）
    float acceleration_min_m_per_s2;        // 加速度模长有效下限（m/s²）
    float acceleration_max_m_per_s2;        // 加速度模长有效上限（m/s²）
} alg_attitude_config_t;
```

| 参数                        | 单位 | 说明                                       |
| :-------------------------- | :--- | :----------------------------------------- |
| `proportional_gain`         | —    | Mahony 比例增益，控制姿态误差的修正速度    |
| `integral_gain`             | —    | Mahony 积分增益，用于补偿陀螺零偏          |
| `madgwick_beta`             | —    | Madgwick 梯度下降步长，控制收敛速度        |
| `acceleration_min_m_per_s2` | m/s² | 加速度模长有效下限，通常设为 7.0（0.7G）   |
| `acceleration_max_m_per_s2` | m/s² | 加速度模长有效上限，通常设为 12.5（1.25G） |

**加速度窗口**：当加速度模长在 `[min, max]` 范围内时，认为 IMU 处于静止或匀速运动状态，使用加速度计修正姿态；超出范围时（如加速/减速/自由落体），退化为纯陀螺积分。

### 3.2 四元数结构 (`alg_attitude_quaternion_t`)

```c
typedef struct {
    float q0;  // 标量分量（w）
    float q1;  // X 轴分量
    float q2;  // Y 轴分量
    float q3;  // Z 轴分量
} alg_attitude_quaternion_t;
```

### 3.3 旋转矩阵结构 (`alg_attitude_rotation_matrix_t`)

```c
typedef struct {
    float element[3][3];
} alg_attitude_rotation_matrix_t;
```

## 4. API 参考

| 函数                               | 说明                         | 返回值                                                      |
| :--------------------------------- | :--------------------------- | :---------------------------------------------------------- |
| `alg_attitude_init`                | 初始化姿态估计器             | `OK` / `INVALID_ARGUMENT`                                   |
| `alg_attitude_reset`               | 重置姿态到指定四元数         | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED`               |
| `alg_attitude_update`              | 更新姿态估计                 | `OK` / `GYRO_ONLY` / `INVALID_ARGUMENT` / `NUMERICAL_ERROR` |
| `alg_attitude_correct_yaw`         | 外部航向修正                 | `OK` / `INVALID_ARGUMENT` / `NUMERICAL_ERROR`               |
| `alg_attitude_get_euler`           | 获取欧拉角（roll/pitch/yaw） | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED`               |
| `alg_attitude_get_rotation_matrix` | 获取旋转矩阵                 | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED`               |

## 5. 算法说明

### 5.1 Mahony 互补滤波 (`ALG_ATTITUDE_METHOD_MAHONY`)

1. 从当前四元数估计重力方向（在传感器坐标系中）
2. 计算加速度计测量方向与估计方向的**向量叉积**得到姿态误差
3. 使用 PI 控制器将误差反馈修正陀螺仪角速度
4. 修正后的角速度积分更新四元数
5. 四元数归一化

**参数整定建议**：

- `proportional_gain`（Kp）：从较小值（如 0.5）开始，逐步增大直到响应足够快
- `integral_gain`（Ki）：先设为 0，观察静态漂移，再缓慢增加（如 Kp 的 2%~5%）

### 5.2 Madgwick 梯度下降滤波 (`ALG_ATTITUDE_METHOD_MADGWICK`)

1. 构造重力对齐的误差函数
2. 计算误差函数的梯度
3. 用梯度下降方向修正陀螺仪角速度
4. 修正后的角速度积分更新四元数
5. 四元数归一化

**参数整定建议**：

- `madgwick_beta`（β）：控制梯度下降的收敛速度，典型值 0.1~0.5

### 5.3 加速度检测与降级

- 当 `acceleration_norm` 在 `[acceleration_min_m_per_s2, acceleration_max_m_per_s2]` 范围内：使用加速度计修正姿态，返回 `OK`
- 超出范围：跳过加速度修正，仅用陀螺仪积分，返回 `GYRO_ONLY`

### 5.4 航向注入

`alg_attitude_correct_yaw()` 将外部测量的航向（来自磁力计、视觉或机械约束）注入姿态估计：

1. 计算当前 yaw 与测量 yaw 的误差（处理 π 环绕）
2. 应用修正增益（`correction_gain`）作为虚拟角速度进行修正

**适用场景**：

- 磁力计航向（需注意磁干扰检测）
- 视觉 SLAM 航向
- 云台电机编码器航向约束

## 6. 使用示例

### 6.1 初始化（Mahony 滤波器）

```c
#include "alg_attitude.h"

static alg_attitude_t s_attitude;

const alg_attitude_config_t config = {
    .method = ALG_ATTITUDE_METHOD_MAHONY,
    .proportional_gain = 2.0F,
    .integral_gain = 0.05F,
    .madgwick_beta = 0.0F,      // Mahony 模式不使用
    .acceleration_min_m_per_s2 = 7.0F,
    .acceleration_max_m_per_s2 = 12.5F,
};

// 初始姿态：单位四元数（水平静止）
alg_attitude_quaternion_t init_q = {1.0F, 0.0F, 0.0F, 0.0F};
alg_attitude_init(&s_attitude, &config, &init_q);
```

### 6.2 周期更新

```c
void imu_task(void) {
    // 获取传感器数据（单位：rad/s 和 m/s²）
    float gyro_x = imu_data.gyro_rad_per_s[0];
    float gyro_y = imu_data.gyro_rad_per_s[1];
    float gyro_z = imu_data.gyro_rad_per_s[2];
    float accel_x = imu_data.accel_m_per_s2[0];
    float accel_y = imu_data.accel_m_per_s2[1];
    float accel_z = imu_data.accel_m_per_s2[2];
    float dt = 0.01F;  // 10ms 控制周期

    alg_attitude_status_t status = alg_attitude_update(
        &s_attitude,
        gyro_x, gyro_y, gyro_z,
        accel_x, accel_y, accel_z,
        dt
    );

    if (status == ALG_ATTITUDE_STATUS_GYRO_ONLY) {
        // 加速度异常，仅用陀螺仪积分
    } else if (status != ALG_ATTITUDE_STATUS_OK) {
        // 错误处理
    }
}
```

### 6.3 获取欧拉角

```c
float roll, pitch, yaw;
if (alg_attitude_get_euler(&s_attitude, &roll, &pitch, &yaw) == ALG_ATTITUDE_STATUS_OK) {
    // roll/pitch/yaw 单位为弧度
}
```

### 6.4 获取旋转矩阵

```c
alg_attitude_rotation_matrix_t rot;
if (alg_attitude_get_rotation_matrix(&s_attitude, &rot) == ALG_ATTITUDE_STATUS_OK) {
    // 使用 rot.element[i][j]
}
```

### 6.5 外部航向修正

```c
// 从磁力计或视觉获取航向（弧度）
float measured_yaw = magnetometer_heading();
float correction_gain = 0.5F;  // 0~1，值越大修正越快
alg_attitude_correct_yaw(&s_attitude, measured_yaw, correction_gain);
```

## 7. 四元数与欧拉角约定

### 7.1 四元数约定

- 四元数为 `q = [w, x, y, z]`（`q0, q1, q2, q3`）
- 旋转方向遵循右手定则
- 四元数始终归一化

### 7.2 欧拉角顺序

- 输出顺序：**roll → pitch → yaw**（ZYX 旋转顺序）
- 单位：弧度
- 范围：roll [-π, π]，pitch [-π/2, π/2]，yaw [-π, π]

### 7.3 旋转矩阵

- 从**机体坐标系**到**世界坐标系**的旋转
- 可用于向量变换：`world_vec = R * body_vec`

## 8. 注意事项

- **加速度窗口**：必须根据实际运动情况调整 `acceleration_min/max`，典型值 7.0~12.5（对应 0.7G~1.25G）
- **积分增益**：`integral_gain` 应从小值开始整定，避免在持续加速时积分累积导致姿态发散
- **航向修正**：`alg_attitude_correct_yaw()` 不是必需的，但六轴 IMU 长期运行后 yaw 会漂移，应定期注入外部航向
- **线程安全**：同一实例不支持多线程并发访问

## 9. 文件结构

| 文件                      | 说明                                         |
| :------------------------ | :------------------------------------------- |
| `alg_attitude.h`          | 公共类型和 API 声明                          |
| `alg_attitude_core.c`     | 初始化、复位、更新调度、四元数积分和航向修正 |
| `alg_attitude_mahony.c`   | Mahony PI 互补滤波实现                       |
| `alg_attitude_madgwick.c` | Madgwick 梯度下降滤波实现                    |
| `alg_attitude_output.c`   | 欧拉角和旋转矩阵输出                         |
| `alg_attitude_internal.h` | 内部函数声明（仅供本模块使用）               |

## 10. 建议验证测试项

- [ ] 静止状态下 roll/pitch 稳定在 0° 附近
- [ ] 绕 X/Y/Z 轴旋转后欧拉角正确收敛
- [ ] 加速度模长超出窗口时退化为纯陀螺积分（返回 `GYRO_ONLY`）
- [ ] Mahony 和 Madgwick 两种方法均正常工作
- [ ] 外部航向注入能正确修正 yaw
- [ ] 四元数始终保持归一化（`|q| ≈ 1.0`）
- [ ] 长时间运行（> 10 分钟）无数值发散
- [ ] 多实例相互独立

---

## 一页式使用顺序与可读信息

1. 填写 `alg_attitude_config_t`，明确 Mahony/Madgwick、增益、加速度有效范围和时间步限制。
2. 用单位四元数或已知初始姿态调用 `alg_attitude_init()`。
3. 每个 IMU 周期先确认 BMI088 数据有效，再调用 `alg_attitude_update()`；陀螺仪必须是 rad/s，加速度必须是 m/s²。
4. 需要磁力计/视觉航向修正时，再调用 `alg_attitude_correct_yaw()`，不要把航向修正混进原始陀螺输入。
5. 用 `alg_attitude_get_euler()` 读取欧拉角；需要连续控制时优先读取对象中的四元数或旋转矩阵，避免欧拉角奇异。

| 可读取结构体                     | 主要信息                                                                            |
| -------------------------------- | ----------------------------------------------------------------------------------- |
| `alg_attitude_quaternion_t`      | 当前姿态四元数                                                                      |
| `alg_attitude_rotation_matrix_t` | 机体系与参考系之间的旋转矩阵                                                        |
| `alg_attitude_t`                 | `quaternion`、`rotation_matrix`、roll/pitch/yaw、积分误差、更新计数和加速度拒绝计数 |

读取输出前必须检查 `alg_attitude_update()` 返回值；时间步异常或输入非有限值时不要继续使用该周期的新结果。
