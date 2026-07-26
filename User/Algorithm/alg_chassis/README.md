# alg_chassis

底盘运动学的公共数学内核。各类底盘把轮子测量转换为线性约束，本模块负责速度求解、
坐标变换、任意旋转中心转换、轮速统一缩放和里程计积分。

## 坐标约定

- `velocity_x_m_per_s`：车体 x 轴速度；
- `velocity_y_m_per_s`：车体 y 轴速度；
- `angular_velocity_rad_per_s`：绕 z 轴角速度；
- `position_x_m`、`position_y_m`：参考坐标系位置；
- `heading_rad`：车体航向角。

旋转方向和车体轴方向必须由项目统一，所有轮组模型使用同一约定。

## 约束求解

单个轮子或传感器提供：

```text
a * vx + b * vy + c * wz = measured_velocity
```

对应 `alg_chassis_constraint_t` 的三个系数、测量值、权重和可用状态。
`alg_chassis_solve_velocity` 使用加权最小二乘求未知速度分量。

`known_component_mask` 可锁定一个或多个已知分量，例如差速底盘固定横向速度为零。解算结果
包含使用的约束数量、未知分量数量、均方根残差和降级标志。

## 任意旋转中心

`alg_chassis_convert_center_velocity_to_origin` 把给定旋转中心处的速度转换到车体原点：

```c
alg_chassis_velocity_t origin_velocity;
alg_chassis_convert_center_velocity_to_origin(
    &center_velocity,
    center_of_rotation_x_m,
    center_of_rotation_y_m,
    &origin_velocity);
```

旋转中心可以位于底盘中心、某个轮子、云台投影点或车体外部。

## 坐标变换

`alg_chassis_transform_reference_to_body` 将云台坐标系或世界坐标系下的平移命令旋转到车体
坐标系，角速度保持不变。小陀螺模式和云台正方向控制应在调用具体轮组逆解前完成此变换。

## 轮速缩放

`alg_chassis_scale_wheel_velocities` 找到最大绝对轮速并统一缩放所有可用轮子，保持速度向量
比例。不可用轮子输出归零。`applied_scale` 用于上层抗积分饱和或功率限制诊断。

## 里程计

`alg_chassis_integrate_odometry` 支持：

- Euler：计算量最低；
- Midpoint：常用折中；
- Exact：恒定速度模型下精确积分。

输入速度必须是车体系速度，`delta_time_s` 必须为有限正数。长时间定位仍需 IMU、视觉或
其他绝对观测修正漂移。

## 内存与错误

无动态内存，约束、残差和输出空间均由调用者提供。求解会区分参数错误、欠约束、奇异和
数值异常。上层不得忽略 `DEGRADED`、`UNDERDETERMINED` 或 `SINGULAR`。

## 建议验证

- 三个单独速度分量和组合运动；
- 已知分量掩码的所有组合；
- 不同权重、禁用约束和异常残差；
- 中心、轮边和车体外旋转中心；
- 参考系旋转 0、±π/2 和 π；
- 三种里程计积分方法；
- 欠约束、共线约束和非有限输入。
