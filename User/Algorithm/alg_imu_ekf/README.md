# alg_imu_ekf

`alg_imu_ekf` 是面向六轴 IMU 的四元数扩展卡尔曼滤波模块。模块只依赖
`alg_filter`、`alg_kalman` 和标准数学库，不依赖 MCU、HAL、RTOS，不使用动态内存。

## 状态与观测

状态为 6 维：

```text
x = [qw, qx, qy, qz, bias_x, bias_y]
```

- `q` 是机体系到世界系的单位四元数。
- `bias_x`、`bias_y` 是陀螺仪 X/Y 轴零偏，单位为 `rad/s`。
- 三轴陀螺仪是预测输入，三轴单位化加速度是重力方向观测。
- 加速度计无法观测 Yaw，因此不估计 Z 轴零偏；对外读取的 Z 轴零偏恒为 0。

六轴系统只能长期校正 Roll/Pitch。Yaw 依赖 Z 轴陀螺仪积分，会随零偏和温漂逐渐漂移。
需要稳定绝对航向时，应增加磁力计、视觉、双天线 GNSS 或其他航向观测。

## 数据处理流程

一次 `alg_imu_ekf_update()` 执行以下流程：

1. 对 X/Y 零偏协方差应用渐消因子，保留慢漂移跟踪能力。
2. 用去除 X/Y 零偏后的角速度传播四元数和协方差。
3. 检查原始加速度模长，明显偏离 1g 时直接拒绝观测。
4. 对三轴加速度分别执行一阶低通滤波，然后单位化为重力方向。
5. 计算残差、创新协方差和归一化创新平方 NIS。
6. NIS 超过卡方拒绝阈值时跳过校正，仅保留陀螺仪预测。
7. NIS 位于自适应区间时放大测量噪声，平滑降低校正增益。
8. 执行 EKF 校正、四元数归一化和协方差投影。

NIS 定义为：

```text
NIS = innovation^T * S^-1 * innovation
S   = H * P * H^T + R
```

默认拒绝阈值 `11.345` 对应 3 自由度卡方分布的约 99% 分位点。

## 坐标与单位约定

- 右手坐标系。
- 四元数表示机体系到世界系的旋转。
- 世界系 `+Z` 向上。
- 水平静止时加速度输入为 `[0, 0, +g]`。
- 陀螺仪单位必须为 `rad/s`。
- 加速度计单位必须为 `m/s^2`。
- `delta_time_s` 单位为秒，必须大于 0。

传感器安装方向、轴交换和符号转换应在 Module/BSP 层完成，不放入算法层。

## 初始化与更新

```c
static alg_imu_ekf_t s_imu_ekf;

void app_imu_estimator_init(const float accelerometer_m_per_s2[3])
{
    alg_imu_ekf_config_t config;

    (void)alg_imu_ekf_config_init(&config);
    config.accelerometer_lpf_cutoff_hz = 30.0F;
    config.accelerometer_rejection_threshold_g = 0.20F;
    config.chi_square_adaptation_threshold = 3.0F;
    config.chi_square_rejection_threshold = 11.345F;
    config.maximum_measurement_noise_scale = 20.0F;
    config.gyro_bias_fading_factor = 1.0001F;

    (void)alg_imu_ekf_init(&s_imu_ekf, &config);
    (void)alg_imu_ekf_reset_from_accelerometer(&s_imu_ekf,
                                                accelerometer_m_per_s2);
}

void app_imu_estimator_update(const float gyroscope_rad_per_s[3],
                              const float accelerometer_m_per_s2[3],
                              float delta_time_s)
{
    bool accelerometer_was_used;

    (void)alg_imu_ekf_update(&s_imu_ekf,
                             gyroscope_rad_per_s,
                             accelerometer_m_per_s2,
                             delta_time_s,
                             &accelerometer_was_used);
}
```

`alg_imu_ekf_reset_from_accelerometer` 应在设备静止时调用，它初始化 Roll/Pitch、将 Yaw 设为 0，
并用当前加速度预置低通滤波器。也可使用 `alg_imu_ekf_reset()` 传入四元数和两轴零偏。

预测和校正可以分开调用：

```c
alg_imu_ekf_predict(&s_imu_ekf, gyroscope_rad_per_s, delta_time_s);
alg_imu_ekf_correct_accelerometer(&s_imu_ekf,
                                  accelerometer_m_per_s2,
                                  delta_time_s);
```

单独校正时，观测被拒绝会返回 `ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED`；组合更新时
仍返回 `ALG_IMU_EKF_STATUS_OK`，并通过 `accelerometer_was_used` 返回本次是否使用了观测。

## 参数说明

| 参数 | 作用 |
| --- | --- |
| `gravity_m_s2` | 标准重力模长 |
| `gyro_noise_std_rad_s` | 陀螺仪白噪声标准差 |
| `gyro_bias_random_walk_std_rad_s2` | X/Y 零偏随机游走标准差 |
| `accelerometer_direction_noise_std` | 单位重力方向观测噪声 |
| `accelerometer_lpf_cutoff_hz` | 内置三轴一阶低通截止频率 |
| `accelerometer_rejection_threshold_g` | 原始模长偏离 1g 的硬拒绝比例 |
| `chi_square_adaptation_threshold` | 开始自适应降低增益的 NIS 阈值 |
| `chi_square_rejection_threshold` | 完全拒绝加速度观测的 NIS 阈值 |
| `maximum_measurement_noise_scale` | 自适应测量噪声最大倍率 |
| `gyro_bias_fading_factor` | 每次预测对零偏协方差的渐消倍率，必须不小于 1 |
| `initial_attitude_variance` | 初始四元数状态方差 |
| `initial_gyro_bias_variance` | 初始 X/Y 零偏方差 |

参数建议按“静态标定 → 单位和轴向检查 → 噪声估计 → LPF → NIS 门限 → 零偏跟踪”
的顺序整定。渐消因子应接近 1，过大会使零偏估计噪声明显增加。

## 输出与诊断

模块提供四元数、欧拉角、X/Y 零偏、校正角速度、机体系重力以及机体/世界系线加速度。
`alg_imu_ekf_get_diagnostics()` 可一次读取：

- 低通后的三轴加速度；
- 三维创新残差；
- 原始加速度模长与相对 1g 偏差；
- 最近一次 NIS；
- 最近一次测量噪声倍率；
- 最近一次加速度观测是否被使用。

线加速度对姿态误差、加速度零偏、比例误差和机械振动敏感，不应仅靠六轴 IMU 长期
二次积分得到位置。

## 生命周期与并发

`alg_imu_ekf_t` 内部的通用 EKF 保存了指向对象自身数组的指针。初始化后不能按值复制、
按值返回或移动对象；新实例必须重新调用 `alg_imu_ekf_init()`。

同一个实例只应由一个执行上下文更新。不要同时在中断和任务中调用同一实例。算法模块
不分配内存、不访问硬件，也不调用操作系统接口。

## 验证建议

集成时至少验证静止姿态、加速度初始化、Yaw 积分、倾斜收敛、X 轴零偏收敛、模长拒绝、
等模长错误方向的卡方拒绝、自适应噪声倍率、低通滤波、渐消因子、Z 轴零偏约束、
重力/线加速度输出和非法参数。还应使用实机静止、缓慢转动、快速运动和撞击数据检查
`accelerometer_was_used` 与诊断计数是否符合预期。
