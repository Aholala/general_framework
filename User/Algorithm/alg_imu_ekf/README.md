# alg_imu_ekf

`alg_imu_ekf` 是专门面向六轴 IMU 的四元数扩展卡尔曼滤波器。它使用三轴陀螺仪进行姿态预测，使用三轴加速度计观测重力方向，估计姿态四元数和陀螺仪零偏。

模块依赖通用 `alg_kalman` 数学内核，但向应用提供固定、清晰的 IMU 接口，不需要应用自行编写 EKF 状态函数和雅可比。

## 状态定义

EKF 使用 7 维状态：

```text
x = [qw, qx, qy, qz, bias_x, bias_y, bias_z]
```

其中：

- `q` 是从机体坐标系旋转到世界坐标系的单位四元数。
- `bias_x/y/z` 是陀螺仪零偏，单位为 `rad/s`。
- 陀螺仪测量作为 EKF 控制输入。
- 归一化加速度计作为三维重力方向观测。

## 已实现功能

- 四元数姿态传播。
- 四元数单位化。
- 四元数归一化后的协方差投影。
- 三轴陀螺仪零偏状态。
- 完整 7×7 状态雅可比。
- 完整 3×7 重力观测雅可比。
- 姿态相关陀螺过程噪声映射。
- 加速度计方向归一化。
- 动态加速度硬拒绝。
- 接近拒绝阈值时自适应增大测量噪声。
- 从静止加速度计初始化 Roll/Pitch。
- 四元数输出。
- Roll/Pitch/Yaw 输出。
- 估计零偏输出。
- 去零偏角速度输出。
- 机体系重力向量输出。
- 机体系与世界系线加速度输出。

## 六轴系统的物理限制

六轴 IMU 没有磁力计或其他航向观测，因此：

- Roll 和 Pitch 可以通过重力长期校正。
- Yaw 只能通过陀螺仪积分获得。
- Yaw 会随 Z 轴陀螺零偏和温漂逐渐漂移。
- Z 轴零偏在只有重力观测时不可完全观测。
- EKF 不能突破这个可观测性限制。

如果需要长期稳定的绝对航向，必须增加磁力计、视觉、双天线 GNSS、轮式运动约束或其他外部航向观测。

## 坐标系和符号约定

输入 IMU 数据必须先由 BSP 或 Module 层映射到统一的右手坐标系。算法层不处理传感器安装方向。

本模块约定：

- 四元数表示机体系到世界系的主动旋转。
- 世界系 `+Z` 为向上方向。
- 静止且水平时，加速度计输入为 `[0, 0, +g]`。
- 陀螺仪使用右手定则。
- 陀螺仪单位必须为 `rad/s`。
- 加速度计单位必须为 `m/s²`。
- 时间单位必须为秒。

如果驱动在静止水平时输出 `[0, 0, -g]`，必须在 Module 或传感器适配层完成符号转换后再传入 EKF。

## 初始化

```c
static AlgImuEkf_t s_imu_ekf;

void ImuEstimator_Init(const float accelerometer_m_s2[3])
{
    AlgImuEkfConfig_t config;

    (void)AlgImuEkfConfig_Init(&config);

    config.gyro_noise_std_rad_s = 0.015F;
    config.gyro_bias_random_walk_std_rad_s2 = 0.0005F;
    config.accelerometer_direction_noise_std = 0.03F;
    config.accelerometer_rejection_threshold_g = 0.20F;

    (void)AlgImuEkf_Init(&s_imu_ekf, &config);
    (void)AlgImuEkf_ResetFromAccelerometer(&s_imu_ekf,
                                           accelerometer_m_s2);
}
```

调用 `ResetFromAccelerometer()` 时设备应尽量保持静止。该函数根据重力确定 Roll 和 Pitch，初始 Yaw 固定为零。

如果已经有合法的初始四元数和离线标定零偏，可以调用 `AlgImuEkf_Reset()`。

## 周期更新

```c
bool accelerometer_used;

AlgImuEkfStatus_t status = AlgImuEkf_Update(
    &s_imu_ekf,
    gyroscope_rad_s,
    accelerometer_m_s2,
    delta_time_s,
    &accelerometer_used);
```

每次调用都会：

1. 使用陀螺仪减去估计零偏。
2. 传播四元数。
3. 更新协方差。
4. 检查加速度模长。
5. 条件允许时使用重力方向进行 EKF 校正。
6. 重新归一化四元数并投影协方差。

动态加速度超出阈值时，预测仍然有效，但本次不使用加速度计。此时 `accelerometer_used` 返回 `false`，函数仍返回 `ALG_IMU_EKF_STATUS_OK`。

也可以分开调用：

```c
AlgImuEkf_Predict(&s_imu_ekf, gyroscope_rad_s, delta_time_s);
AlgImuEkf_CorrectAccelerometer(&s_imu_ekf, accelerometer_m_s2);
```

单独校正时，被拒绝的加速度计返回 `ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED`。

## 姿态输出

```c
AlgImuEkfQuaternion_t quaternion;
AlgImuEkfEuler_t euler;
float gyro_bias_rad_s[3];

AlgImuEkf_GetQuaternion(&s_imu_ekf, &quaternion);
AlgImuEkf_GetEuler(&s_imu_ekf, &euler);
AlgImuEkf_GetGyroBias(&s_imu_ekf, gyro_bias_rad_s);
```

欧拉角输出单位是弧度。控制系统内部推荐继续使用四元数或旋转矩阵，欧拉角更适合显示和低维控制接口。

Pitch 接近 ±90° 时，欧拉角仍然存在固有奇异性；四元数本身不存在万向节锁。

## 重力和线加速度

```c
float gravity_body_m_s2[3];
float linear_acceleration_body_m_s2[3];
float linear_acceleration_world_m_s2[3];

AlgImuEkf_GetGravityBody(&s_imu_ekf, gravity_body_m_s2);
AlgImuEkf_GetLinearAccelerationBody(&s_imu_ekf,
                                     accelerometer_m_s2,
                                     linear_acceleration_body_m_s2);
AlgImuEkf_GetLinearAccelerationWorld(&s_imu_ekf,
                                      accelerometer_m_s2,
                                      linear_acceleration_world_m_s2);
```

线加速度由加速度计测量减去估计重力得到。它对以下误差非常敏感：

- 姿态误差。
- 加速度计零偏。
- 传感器比例因子误差。
- 机械振动。
- 坐标轴安装误差。

六轴 IMU 的线加速度不能长期二次积分得到可靠位置，必须结合其他位置或速度观测。

## 参数说明

### `gyro_noise_std_rad_s`

陀螺仪单次角速度噪声标准差。值越大，预测协方差增长越快，加速度计校正权重相对越高。

### `gyro_bias_random_walk_std_rad_s2`

陀螺零偏变化率噪声标准差。值太小会导致零偏跟踪温漂过慢；值太大可能把真实低频运动错误吸收到零偏状态。

### `accelerometer_direction_noise_std`

归一化加速度方向的标准差。值越小，Roll/Pitch 越相信加速度计，但振动影响也越明显。

### `accelerometer_rejection_threshold_g`

加速度模长相对标准重力的最大允许偏差。例如 `0.20F` 表示偏离 1g 超过 20% 时拒绝本次加速度校正。

### `accelerometer_noise_multiplier`

加速度模长接近拒绝阈值时，测量噪声的放大强度。它使校正权重在硬拒绝前平滑下降。

## 参数整定建议

1. 先完成陀螺仪和加速度计静态标定。
2. 确认轴向、符号、单位和采样周期。
3. 静止采集数据，估算陀螺标准差。
4. 从较大的加速度方向噪声开始，避免振动造成姿态抖动。
5. 观察 `was_accelerometer_used` 和 `last_accelerometer_deviation_g`。
6. 最后调整零偏随机游走，使 Roll/Pitch 零偏既能收敛又不追踪真实运动。

## 对象生命周期

`AlgImuEkf_t` 内部的通用 EKF 保存了指向对象自身数组的指针。因此对象初始化后不能通过结构体赋值、按值返回或直接复制到另一个地址。

正确方式：

```c
static AlgImuEkf_t s_filter;
AlgImuEkf_Init(&s_filter, &config);
```

不要这样做：

```c
AlgImuEkf_t copied_filter = initialized_filter;
```

需要重新创建实例时，应对新对象重新调用 `AlgImuEkf_Init()`。

## 实时和并发

- 同一个对象只应由一个执行上下文更新。
- 不要同时在中断和任务中调用同一实例。
- 推荐在固定周期任务中更新。
- `delta_time_s` 应使用实际测量周期，而不是写死一个与真实周期不一致的常数。
- 算法不使用动态内存，也不调用任何操作系统接口。

## 测试覆盖

`Test/alg_imu_ekf_test.c` 覆盖：

- 静止水平姿态。
- 从加速度计初始化已知 Roll。
- 恒定 Z 轴角速度的 Yaw 积分。
- Roll 倾斜误差的重力校正。
- X 轴陀螺零偏收敛。
- 动态加速度拒绝。
- 四元数和欧拉角输出。
- 重力向量和去重力线加速度。
- 非法配置和未初始化对象。
