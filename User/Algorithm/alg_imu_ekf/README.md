# alg_imu_ekf — IMU 扩展卡尔曼滤波

6 轴 IMU（陀螺+加速度）的 EKF 姿态估计。输出四元数/欧拉角、陀螺零偏估计、自适应测量噪声。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `alg_imu_ekf_t` | EKF 对象 |
| `alg_imu_ekf_config_t` | 配置：`gravity_m_s2`, `gyro_noise`, `accel_noise`, `initial_bias`, `lpf_cutoff` |
| `alg_imu_ekf_quaternion_t` | 四元数输出 `{q0, q1, q2, q3}` |
| `alg_imu_ekf_euler_t` | 欧拉角输出 `{roll, pitch, yaw}` (rad) |
| `alg_imu_ekf_diagnostics_t` | 诊断：`innovation[3]`, `nis`, `measurement_noise_scale`, `was_accelerometer_used` |

## 用法

```c
alg_imu_ekf_t ekf;
alg_imu_ekf_config_t cfg = {
    .gravity_m_s2 = 9.81f,
    .gyro_noise_rad_per_s = 0.01f,
    .accel_noise_m_per_s2 = 0.5f,
    .accelerometer_lpf_cutoff_hz = 20.0f,
};
alg_imu_ekf_init(&ekf, &cfg);

// 周期更新
alg_imu_ekf_update(&ekf,
    gyro_x, gyro_y, gyro_z,          // rad/s
    accel_x, accel_y, accel_z,        // m/s²
    0.001f);                          // dt (s)

// 读取姿态
alg_imu_ekf_quaternion_t q;
alg_imu_ekf_get_quaternion(&ekf, &q);
// q = {q0, q1, q2, q3}

alg_imu_ekf_euler_t euler;
alg_imu_ekf_get_euler(&ekf, &euler);
// euler = {roll_rad, pitch_rad, yaw_rad}

// 读取诊断
const alg_imu_ekf_diagnostics_t *diag = alg_imu_ekf_get_diagnostics(&ekf);
float nis = diag->normalized_innovation_squared;  // > threshold → 加速度干扰大
bool used = diag->was_accelerometer_used;          // 本次是否接受了加速度观测
```

## 重置

```c
// 已知初始姿态和零偏时重置
alg_imu_ekf_quaternion_t init_q = {1, 0, 0, 0};  // 水平
float bias[2] = {0, 0};
alg_imu_ekf_reset(&ekf, &init_q, bias);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `alg_imu_ekf_init(me, cfg)` | 初始化 |
| `alg_imu_ekf_update(me, gx,gy,gz, ax,ay,az, dt)` | 周期更新 |
| `alg_imu_ekf_reset(me, q, bias)` | 重置状态 |
| `alg_imu_ekf_get_quaternion(me, &q)` | 读四元数 |
| `alg_imu_ekf_get_euler(me, &e)` | 读欧拉角 |
| `alg_imu_ekf_get_gyro_bias(me, bias[2])` | 读陀螺零偏估计 |
| `alg_imu_ekf_get_corrected_gyroscope(me, gyro[3])` | 读零偏校正后角速度 |
| `alg_imu_ekf_get_diagnostics(me)` | 读诊断数据 |
