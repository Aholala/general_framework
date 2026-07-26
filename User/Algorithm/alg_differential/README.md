# alg_differential

二轮或四轮差速底盘运动学，支持逆解、正解、轮子失效降级和横向偏置旋转中心。

## 布局模型

每个轮子配置所属左/右侧、半径、安装方向和里程计权重。二轮布局通常左右各一轮；四轮
布局通常左右各两轮。`track_width_m` 是左右轮接地点之间的距离。

差速结构不具备主动横移能力，正解时通常通过已知分量掩码固定：

```c
known_velocity.velocity_y_m_per_s = 0.0F;
known_component_mask = ALG_CHASSIS_COMPONENT_VELOCITY_Y;
```

## 初始化

```c
alg_differential_t model;
alg_chassis_status_t status = alg_differential_init(
    &model,
    wheel_configs,
    wheel_count,
    track_width_m,
    maximum_wheel_angular_velocity_rad_per_s);
```

轮组配置数组被对象长期引用，必须在对象使用期保持有效，建议定义为 `static const`。

## 逆解

`alg_differential_inverse` 把车体前向速度和角速度转换为各轮角速度，并按最大轮速统一缩放。
不可用轮子输出为零。

`alg_differential_inverse_with_lateral_center_of_rotation` 支持旋转中心在车体 y 方向偏移，
适用于希望绕车体侧方某一点转动的机构。差速底盘不能表达任意 x/y 平面旋转中心。

## 正解

`alg_differential_forward` 把有效轮速转换成线性约束，调用公共求解器得到车体速度、残差与
降级状态。工作区容量至少等于 `wheel_count`。

当仅剩同侧轮或有效约束不足时会返回欠约束，应用层必须停止或进入明确的安全模式。

## 单位与符号

- 轮速：`rad/s`；
- 车体速度：`m/s`；
- 角速度：`rad/s`；
- 轮距和半径：`m`；
- `direction_sign`：通常为 `1.0F` 或 `-1.0F`。

## 实时性与所有权

无动态内存。逆解为 O(n)，正解使用公共小规模最小二乘。对象不拥有配置数组，初始化后
不得移动或按值复制正在使用的配置存储。

## 建议验证

- 二轮和四轮直行、原地旋转、圆弧运动；
- 左右轮反向安装；
- 不同轮径和权重；
- 横向偏置旋转中心；
- 单轮离线、同侧双轮离线；
- 轮速限幅和缩放比例；
- 正逆解往返及残差。
