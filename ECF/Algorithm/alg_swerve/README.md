# alg_swerve — 舵轮底盘运动学

任意数量舵轮模块的逆解/正解/里程计。每个模块独立舵角+轮速。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `alg_swerve_t` | 舵轮运动学对象 |
| `alg_swerve_config_t` | 配置：模块坐标 `{x_m, y_m}`、轮半径、舵角方向 |
| `alg_swerve_command_t` | 车体命令 `{vx, vy, wz}` |
| `alg_swerve_module_target_t` | 单个模块目标 `{wheel_velocity_m_per_s, steering_angle_rad}` |

## 用法

```c
alg_swerve_t swerve;
alg_swerve_config_t cfg = {
    .wheel_radius_m = 0.076f,  // 76mm 轮径
    .module_count = 4,
    .modules = {
        { .x_m = 0.15f,  .y_m = 0.15f },   // 左前
        { .x_m = 0.15f,  .y_m = -0.15f },  // 右前
        { .x_m = -0.15f, .y_m = 0.15f },   // 左后
        { .x_m = -0.15f, .y_m = -0.15f },  // 右后
    },
};
alg_swerve_init(&swerve, &cfg);

// 逆解：车体命令 → 各模块目标
alg_swerve_command_t cmd = {1.0f, 0, 0.5f};  // 前进+旋转
alg_swerve_module_target_t targets[4];
alg_swerve_inverse(&swerve, &cmd, targets);
// targets[0].wheel_velocity_m_per_s = ...
// targets[0].steering_angle_rad = ...

// 正解：各模块状态 → 车体速度
alg_swerve_module_target_t states[4] = {...};
alg_swerve_forward(&swerve, states);

// 里程计
alg_swerve_update_odometry(&swerve, states, 0.001f);
const alg_chassis_pose_t *pose = alg_swerve_get_pose(&swerve);

// 解算诊断
const alg_chassis_solution_t *sol = alg_swerve_get_solution(&swerve);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `alg_swerve_init(me, cfg)` | 初始化 |
| `alg_swerve_inverse(me, cmd, targets[])` | 逆解 |
| `alg_swerve_forward(me, states[])` | 正解 |
| `alg_swerve_update_odometry(me, states, dt)` | 里程计 |
| `alg_swerve_get_pose(me)` | 读位姿 |
| `alg_swerve_get_solution(me)` | 读解算诊断 |
