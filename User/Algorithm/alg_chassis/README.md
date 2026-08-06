# alg_chassis — 底盘运动学

通用底盘解算接口。不实现具体运动学，只定义速度/位姿/约束数据结构。

## 关键结构体

| 结构体 | 字段 | 说明 |
|--------|------|------|
| `alg_chassis_velocity_t` | `velocity_x_m_per_s`, `velocity_y_m_per_s`, `angular_velocity_rad_per_s` | 车体速度（`+x`前 `+y`左 `+z`上，逆时针为正） |
| `alg_chassis_pose_t` | `position_x_m`, `position_y_m`, `heading_rad` | 里程计位姿 |
| `alg_chassis_solution_t` | `velocity`(输出), `residual_root_mean_square_m_per_s`, `used_constraint_count`, `is_degraded` | 解算结果 + 残差诊断 |
| `alg_chassis_constraint_t` | 轮速约束 | 用于降级解算 |

## 使用

```c
// 目标车体速度
alg_chassis_velocity_t target = {
    .velocity_x_m_per_s = 1.0f,
    .velocity_y_m_per_s = 0.0f,
    .angular_velocity_rad_per_s = 0.5f,
};

// 喂入运动学模块（以舵轮为例）
alg_swerve_inverse(&swerve, &target, module_targets);

// 读解算结果
const alg_chassis_solution_t *sol = alg_swerve_get_solution(&swerve);
if (sol->is_degraded) {
    // 某个舵轮离线，解算降级
    float residual = sol->residual_root_mean_square_m_per_s;
}
```

## 坐标系约定

```
     +x (前)
      ↑
      |
+y ←--+  (左)
      |
     +z (上，逆时针角速度为正)
```
