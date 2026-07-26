# alg_ackermann

四轮阿克曼底盘的逆运动学和基于约束最小二乘的正运动学。算法只处理几何与速度，不访问电机、
编码器或底层总线。

## 支持能力

- 前轮转向、四轮驱动的标准阿克曼布局；
- 四个车轮独立半径、方向符号和里程计权重；
- 转向角和轮速限制；
- 轮子可用性掩码；
- 缺轮时的降级正解；
- 已知速度分量约束，例如固定 `velocity_y_m_per_s = 0`；
- 轮速饱和时保持整体运动方向的比例缩放。

## 配置

`alg_ackermann_config_t` 需要轴距、轮距、四轮半径、方向符号、里程计权重、横向约束
权重、最大转角和最大轮角速度。长度单位为米，角度为弧度，时间为秒。

`direction_sign` 用于统一不同电机安装方向，通常为 `1.0F` 或 `-1.0F`。
`odometry_weight` 应为非负数，数值越大表示越信任该轮反馈。

## 使用流程

```c
alg_ackermann_t chassis_model;
alg_chassis_status_t status = alg_ackermann_init(&chassis_model, &config);

status = alg_ackermann_inverse(
    &chassis_model, &command_velocity, wheel_is_available,
    wheel_targets, &applied_scale);
```

`wheel_targets` 同时返回轮角速度和转向角。`applied_scale < 1.0F` 表示受最大轮速限制。

正解需要调用者提供 `ALG_ACKERMANN_CONSTRAINT_COUNT` 个约束工作区：

```c
alg_chassis_constraint_t workspace[ALG_ACKERMANN_CONSTRAINT_COUNT];
alg_chassis_solution_t solution;

status = alg_ackermann_forward(
    &chassis_model, measured_states, wheel_is_available,
    known_component_mask, &known_velocity, workspace, &solution);
```

## 降级与返回值

- `ALG_CHASSIS_STATUS_OK`：完整求解；
- `ALG_CHASSIS_STATUS_DEGRADED`：部分车轮不可用但仍可求解；
- `ALG_CHASSIS_STATUS_UNDERDETERMINED`：有效约束不足；
- `ALG_CHASSIS_STATUS_SINGULAR`：几何或约束退化；
- `ALG_CHASSIS_STATUS_INVALID_ARGUMENT`：尺寸、范围或指针错误。

应用层应根据 `solution.is_degraded` 和残差限制最大输出，不能把降级状态当作正常满功率运行。

## 内存与实时性

无动态内存。对象和工作区由调用者持有。逆解为固定四轮计算；正解使用固定规模约束求解，
适合控制周期调用。初始化后的对象只读，可由单个控制任务重复使用。

## 建议验证

- 直行、倒车、左/右转和零速度；
- 内外轮转角关系；
- 最大转角、最大轮速及比例缩放；
- 四种单轮失效和相邻两轮失效；
- 不同轮径与反向安装；
- 正解和逆解往返误差；
- 轴距、轮距或半径为零的构造失败。
