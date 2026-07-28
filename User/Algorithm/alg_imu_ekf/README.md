# IMU EKF 姿态估计模块 (alg_imu_ekf) —— 完整使用指南

## 1. 模块概述

`alg_imu_ekf` 是面向六轴 IMU（三轴陀螺仪 + 三轴加速度计）的四元数扩展卡尔曼滤波（EKF）模块。状态为 6 维：四元数（4）+ X/Y 陀螺仪零偏（2）。模块只依赖 `alg_filter`、`alg_kalman` 和标准数学库，不依赖 MCU、HAL、RTOS，不使用动态内存。

**核心功能**：

- 六轴 IMU 姿态估计（Roll/Pitch/Yaw）
- 陀螺仪 X/Y 轴零偏在线估计
- 加速度计观测自适应（模长拒绝 + NIS 卡方检验 + 噪声自适应）
- 四元数归一化与协方差投影
- 输出四元数、欧拉角、校正陀螺仪、重力向量、线性加速度

## 2. 状态与观测模型

### 2.1 状态向量

```
x = [qw, qx, qy, qz, bias_x, bias_y]
```

- `q`：机体系到世界系的单位四元数
- `bias_x`、`bias_y`：陀螺仪 X/Y 轴零偏（rad/s）
- Z 轴零偏不估计（六轴 IMU 无法观测），读取时恒为 0

### 2.2 预测输入

- 三轴陀螺仪读数（rad/s）作为控制输入
- 去除 X/Y 零偏后用于四元数积分

### 2.3 观测

- 三轴加速度计读数（m/s²），单位化后作为重力方向观测
- 加速度计无法观测 Yaw，因此 Yaw 依赖 Z 轴陀螺仪积分

## 3. 数据处理流程

一次 `alg_imu_ekf_update()` 执行：

1. **零偏协方差渐消**：对 X/Y 零偏相关协方差应用渐消因子
2. **状态预测**：用去除零偏后的角速度传播四元数和协方差
3. **加速度模长检查**：明显偏离 1g 时直接拒绝观测
4. **低通滤波**：对三轴加速度分别执行一阶低通滤波
5. **单位化**：将滤波后加速度归一化为重力方向
6. **创新计算**：计算残差、创新协方差和归一化创新平方（NIS）
7. **卡方检验**：NIS 超过拒绝阈值时跳过校正
8. **自适应噪声**：NIS 在自适应区间时放大测量噪声
9. **EKF 校正**：执行卡尔曼更新，四元数归一化与协方差投影

## 4. 参数整定建议

| 参数                                  | 作用               | 整定建议                               |
| :------------------------------------ | :----------------- | :------------------------------------- |
| `gyro_noise_std_rad_s`                | 陀螺仪白噪声       | 从数据手册获取，或 Allan 方差标定      |
| `gyro_bias_random_walk_std_rad_s2`    | 零偏随机游走       | 从 Allan 方差标定，控制零偏收敛速度    |
| `accelerometer_direction_noise_std`   | 加速度方向噪声     | 约 0.02~0.05，影响 Roll/Pitch 收敛速度 |
| `accelerometer_lpf_cutoff_hz`         | 加速度低通截止频率 | 10~50Hz，平衡噪声抑制与响应速度        |
| `accelerometer_rejection_threshold_g` | 模长拒绝阈值       | 0.15~0.30G，过大可能引入运动加速度     |
| `chi_square_rejection_threshold`      | NIS 拒绝阈值       | 3 自由度 99% 分位点 ≈ 11.345           |
| `gyro_bias_fading_factor`             | 零偏渐消因子       | 1.0001~1.001，过大会使零偏噪声增加     |

## 5. 使用示例

### 5.1 初始化

```c
static alg_imu_ekf_t s_imu_ekf;

void app_imu_estimator_init(const float accelerometer_m_s2[3]) {
    alg_imu_ekf_config_t config;
    alg_imu_ekf_config_init(&config);

    // 根据传感器特性调整参数
    config.gyro_noise_std_rad_s = 0.015F;
    config.accelerometer_direction_noise_std = 0.03F;
    config.accelerometer_lpf_cutoff_hz = 30.0F;

    alg_imu_ekf_init(&s_imu_ekf, &config);
    alg_imu_ekf_reset_from_accelerometer(&s_imu_ekf, accelerometer_m_s2);
}
```

### 5.2 周期更新

```c
void app_imu_estimator_update(const float gyroscope_rad_s[3],
                              const float accelerometer_m_s2[3],
                              float delta_time_s) {
    bool accelerometer_was_used;
    alg_imu_ekf_update(&s_imu_ekf, gyroscope_rad_s, accelerometer_m_s2,
                       delta_time_s, &accelerometer_was_used);

    // 获取姿态
    alg_imu_ekf_quaternion_t quat;
    alg_imu_ekf_get_quaternion(&s_imu_ekf, &quat);

    // 获取欧拉角
    alg_imu_ekf_euler_t euler;
    alg_imu_ekf_get_euler(&s_imu_ekf, &euler);
}
```

### 5.3 获取校正后陀螺仪

```c
float corrected_gyro[3];
alg_imu_ekf_get_corrected_gyroscope(&s_imu_ekf, raw_gyro, corrected_gyro);
```

### 5.4 获取线性加速度

```c
float linear_accel_body[3];
alg_imu_ekf_get_linear_acceleration_body(&s_imu_ekf, accelerometer_m_s2, linear_accel_body);
```

## 6. 坐标约定

- 右手坐标系
- 四元数表示机体系到世界系的旋转
- 世界系 `+Z` 向上
- 水平静止时加速度输入为 `[0, 0, +g]`
- 陀螺仪单位：`rad/s`
- 加速度计单位：`m/s²`
- 传感器安装方向、轴交换和符号转换应在 Module/BSP 层完成

## 7. 注意事项

- **Yaw 漂移**：六轴 IMU 无法仅靠重力长期观测 Yaw，需要外部航向修正（磁力计、视觉等）
- **线加速度**：不应仅靠六轴 IMU 二次积分得到位置（误差会快速发散）
- **线程安全**：同一实例不能多线程并发访问
- **零动态内存**：所有内存由调用者分配

## 8. 建议验证测试项

- [ ] 静止姿态收敛（Roll/Pitch 收敛到 0）
- [ ] 从加速度计重置姿态正确
- [ ] Yaw 积分（绕 Z 轴旋转后 Yaw 正确累加）
- [ ] 倾斜收敛（倾斜后 Roll/Pitch 正确收敛）
- [ ] X 轴零偏收敛（静止时零偏收敛到真值）
- [ ] 模长拒绝（加速度模长偏离 1g 时拒绝观测）
- [ ] NIS 拒绝（加速度方向不一致时拒绝观测）
- [ ] 自适应噪声（NIS 在自适应区间时噪声增大）
- [ ] 低通滤波效果
- [ ] 线性加速度输出（运动时分离重力）

---

**总结**：`alg_imu_ekf` 提供了完整的六轴 IMU 姿态估计解决方案，通过扩展卡尔曼滤波实现四元数状态和陀螺仪零偏的联合估计。其自适应测量噪声和卡方检验机制使其对运动加速度和异常观测具有鲁棒性。配合 `module_bmi088` 等 IMU 模块，可构建完整的姿态估计系统。
