# module_swerve — 舵轮执行模块

单个舵轮的转向+驱动控制。接收 `alg_swerve_module_target_t`，输出转向角+轮速。内嵌舵角 PID + 轮速 PID。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_swerve_t` | 舵轮模块对象 |
| `module_swerve_config_t` | 配置：转向电机、驱动电机、舵角环参数 |

## 用法

```c
module_swerve_t module;
module_swerve_config_t cfg = {
    .steering_motor = &gm6020,  // 转向电机
    .drive_motor    = &m3508,   // 驱动电机
    .steering_gear_ratio = 1.0f,
    .wheel_radius_m = 0.076f,
};
module_swerve_init(&module, &cfg);

// 接收运动学解算目标
alg_swerve_module_target_t target = { .wheel_velocity_m_per_s = 1.5f, .steering_angle_rad = 0.3f };
module_swerve_set_target(&module, &target);

// 周期更新（PID 控制 + 舵角优化）
module_swerve_update(&module, 0.001f);

// 读取当前舵角
float angle = module_swerve_get_steering_angle(&module);
```

## 舵角优化

`module_swerve_set_target` 自动优化目标舵角：保持当前多圈位置，不在 `[-π, π)` 内回绕。送入位置环前维持连续性。

## API 速查

| 函数 | 功能 |
|------|------|
| `module_swerve_init(me, cfg)` | 初始化 |
| `module_swerve_set_target(me, target)` | 设目标（带舵角优化） |
| `module_swerve_update(me, dt)` | 周期更新 |
| `module_swerve_get_steering_angle(me)` | 读当前舵角 (rad) |
