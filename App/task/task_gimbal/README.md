# task_gimbal

云台 FreeRTOS 调度适配器。按照 `APP_GIMBAL_PERIOD_MS` 调用 `app_gimbal_update()`；
PID/LQR 和 IMU/编码器目标选择均属于 `app_gimbal`。

