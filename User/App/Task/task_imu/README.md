# task_imu

IMU 高频采样调度适配器。按照 `APP_IMU_PERIOD_MS` 调用 `app_imu_update()`，不直接
访问 SPI、BMI088 寄存器或姿态解算内部状态。

