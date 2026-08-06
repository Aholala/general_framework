# alg_trajectory — 轨迹规划

一维轨迹生成（梯形速度 / S 曲线）+ 多轴同步组。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `alg_trajectory_t` | 单轴轨迹 |
| `alg_trajectory_config_t` | 配置：`max_velocity`, `max_acceleration`, `max_jerk`(S曲线) |
| `alg_trajectory_state_t` | 轨迹状态 `{position, velocity_per_s, acceleration_per_s2}` |
| `alg_trajectory_group_t` | 多轴同步组 |

## 用法

```c
// 梯形速度轨迹
alg_trajectory_t traj;
alg_trajectory_config_t cfg = {
    .max_velocity = 2.0f,        // 2 m/s
    .max_acceleration = 5.0f,    // 5 m/s²
};
alg_trajectory_init(&traj, &cfg);

// 设目标
alg_trajectory_set_target(&traj, 1.0f);  // 目标位置 1m

// 周期更新
alg_trajectory_state_t state;
alg_trajectory_update(&traj, 0.001f, &state);
// state.position → 当前位置
// state.velocity_per_s → 规划速度
// state.acceleration_per_s2 → 规划加速度

// S 曲线（加 jerk 限制）
cfg.max_jerk = 50.0f;
alg_trajectory_set_s_curve(&traj, &cfg);

// 多轴同步组（所有轴同时到达）
alg_trajectory_group_t group;
alg_trajectory_group_init(&group);
alg_trajectory_group_add(&group, &traj_x);
alg_trajectory_group_add(&group, &traj_y);
alg_trajectory_group_set_targets(&group, targets);  // 同步出发
```

## API 速查

| 函数 | 功能 |
|------|------|
| `alg_trajectory_init(me, cfg)` | 初始化 |
| `alg_trajectory_set_target(me, pos)` | 设目标位置 |
| `alg_trajectory_set_target_velocity(me, vel)` | 设目标速度（速度控制模式） |
| `alg_trajectory_update(me, dt, &state)` | 周期更新 |
| `alg_trajectory_is_done(me)` | 是否到达目标 |
| `alg_trajectory_group_init/add/set_targets` | 多轴同步 |
