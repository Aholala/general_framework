# alg_omni

通用三轮或四轮全向轮运动学。每个轮子可独立配置位置、驱动方向、半径、安装符号和里程计
权重，因此不绑定固定底盘尺寸或轮子角度。

## 轮子模型

`alg_omni_wheel_config_t` 描述：

- 轮心相对车体原点的 x/y 位置；
- 轮子产生牵引速度的方向角；
- 轮半径；
- 电机安装方向符号；
- 正解权重。

`alg_omni_configure_tangential_layout` 可生成均匀圆周切向布局，适用于常见三全向轮和
四全向轮底盘；非对称布局直接填写每轮配置即可。

## 初始化

```c
static alg_omni_wheel_config_t wheel_configs[3];
alg_omni_configure_tangential_layout(
    wheel_configs, 3U, center_distance_m, wheel_radius_m,
    first_wheel_angle_rad, 1.0F, direction_signs, 1.0F);

alg_omni_init(&model, wheel_configs, 3U, maximum_wheel_speed_rad_per_s);
```

配置数组被对象长期引用，必须保持有效。

## 逆解和旋转中心

`alg_omni_inverse` 计算车体原点速度对应的各轮角速度。
`alg_omni_inverse_with_center_of_rotation` 接受任意 x/y 旋转中心，可绕云台轴、轮子或车体
外部点运动。轮速超限时统一缩放，并通过 `applied_scale` 返回比例。

## 正解和降级

`alg_omni_forward` 将每个有效轮的测量速度转为约束并调用公共最小二乘求解器。工作区容量
至少为轮数。三轮布局失去一轮后通常无法独立求三自由度；可通过 IMU 提供已知角速度等
约束进行降级求解。

## 所有权与实时性

无动态内存。对象不拥有轮组配置和工作区。初始化后几何配置应保持不变；若机械结构变化，
重新初始化对象。

## 建议验证

- 三轮和四轮切向布局；
- 任意非对称位置和驱动方向；
- x/y 平移、旋转和复合命令；
- 多种旋转中心；
- 每个单轮和双轮失效；
- 已知角速度辅助降级；
- 正逆解往返、权重和残差。
