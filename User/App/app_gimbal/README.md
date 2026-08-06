# app_gimbal — 云台控制

偏航(Yaw)+俯仰(Pitch)双轴云台。PID/LQR 两种控制模式，IMU/编码器两种反馈模式。

## 用法

```c
app_gimbal_config_t cfg = {
    .yaw_motor   = &yaw_gm6020,
    .pitch_motor = &pitch_gm6020,
    .target_tolerance_rad = 0.01f,
};
app_gimbal_init(&cfg);

// 周期更新
app_gimbal_update(dt);
// 从 app_exchange 读取命令 → 控制电机 → 发布反馈
```

## 控制模式

| 模式 | 说明 |
|------|------|
| `APP_GIMBAL_CONTROL_PID` | 双环 PID（位置+速度） |
| `APP_GIMBAL_CONTROL_LQR` | LQR 最优控制 |
| `APP_GIMBAL_FEEDBACK_IMU` | IMU 姿态作反馈（绝对角度） |
| `APP_GIMBAL_FEEDBACK_ENCODER` | 电机编码器作反馈（相对角度） |
