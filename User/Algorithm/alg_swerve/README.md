# alg_swerve

任意数量舵轮模块的运动学库。支持参考坐标系命令、任意旋转中心、模块失效、正解、舵角
最短路径优化和静止自锁。

## 几何与命令

每个 `alg_swerve_module_geometry_t` 保存舵轮相对车体原点的 x/y 位置。
`alg_swerve_command_t` 包含平移速度、角速度、参考航向和旋转中心。

当 `command_is_reference_relative = true` 时，平移命令先根据
`reference_heading_rad` 转换到车体系，适用于以云台或世界方向控制底盘。

## 初始化

```c
static alg_swerve_module_geometry_t geometry[4];
alg_swerve_configure_rectangular_layout(
    geometry, half_wheelbase_m, half_track_width_m);
alg_swerve_init(&model, geometry, 4U, maximum_wheel_velocity_m_per_s);
```

也可以提供任意模块数量和非矩形布局。几何数组由调用者持有并覆盖对象生命周期。

## 逆解

`alg_swerve_calculate` 输出每个模块的轮线速度和舵角。
`alg_swerve_calculate_with_availability` 会忽略不可用模块，并对剩余模块统一限速。

`alg_swerve_optimize_target` 将目标舵角调整到距离当前角度不超过 π/2；必要时反转轮速，
减少舵向旋转时间。该函数应在获得当前连续舵角后调用。

## 任意旋转中心

命令中的 `center_of_rotation_x_m/y_m` 可以选择底盘中心、云台轴投影、任意车轮或车体外部
点。小陀螺和狭窄区域转向不需要修改几何配置。

## 正解

`alg_swerve_forward` 从测得的轮速和舵角构造约束，通过权重、可用性和已知速度分量求车体
速度及残差。调用者提供至少 `2 * module_count` 的约束工作区。

## 静止自锁

`alg_swerve_calculate_self_lock` 输出交叉指向的舵角和零轮速，抵抗外力推动。它只生成目标，
实际保持能力取决于舵向闭环、驱动器使能和机械摩擦。

## 返回与降级

`ALG_SWERVE_STATUS_DEGRADED` 表示部分模块不可用但仍生成输出。应用层应限制速度并结合
`alg_chassis_fault` 判断是否允许继续。有效模块不足时不可假设还能控制三自由度。

## 建议验证

- 三轮、四轮和非矩形布局；
- 车体系与参考系命令；
- 中心、单轮和车外旋转中心；
- 舵角跨 ±π 优化和轮速反转；
- 任意单模块、双模块失效；
- 自锁角度；
- 正逆解往返和残差。
