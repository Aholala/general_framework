# app_imu — IMU 姿态应用

包装 BMI088 + EKF。提供姿态快照和坐标系变换。

## 用法

```c
app_imu_config_t cfg = {
    .sensor = &bmi088,
    .accelerometer_correction_gain = 0.02f,
};
app_imu_init(&cfg);

// 周期更新（ISR 触发读取 → 任务中 EKF 更新 → 发布快照）
app_imu_update(dt);

// 读快照
const app_imu_snapshot_t *imu = app_imu_get_snapshot();
float yaw = imu->yaw_rad;
float pitch = imu->pitch_rad;
bool valid = imu->valid;
```
