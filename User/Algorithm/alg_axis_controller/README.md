# alg_axis_controller

位置轴控制器的多态接口。相同的 `alg_axis_controller_t *` 可以指向串级 PID 或二维 LQR
控制器，使云台、舵向等上层逻辑不依赖具体闭环算法。

## 对象关系

```text
alg_axis_controller_t
├── alg_axis_pid_t
└── alg_axis_lqr_t
```

基类保存 `vptr` 和初始化状态，派生类将 `super` 放在首成员。公共接口先校验对象，再调用
只读操作表中的 `reset` 或 `update`。

## 输入模型

`alg_axis_controller_input_t` 包含：

- 目标位置和目标速度；
- 测量位置和测量速度；
- 执行器前馈；
- 本次控制周期 `delta_time_s`。

全部角度使用弧度，角速度使用弧度每秒。调用方必须先处理编码器跨圈或角度连续化。

## PID 派生类

`alg_axis_pid_t` 包装 `alg_pid_cascade_t`。外环由位置误差生成速度目标，内环根据速度误差
生成执行器输出。限幅、抗积分饱和、微分滤波等参数由 `alg_pid_cascade_config_t` 提供。

```c
alg_axis_pid_t pitch_pid;
alg_axis_pid_init(&pitch_pid, &pid_config);
alg_axis_controller_t *pitch_controller =
    alg_axis_pid_as_controller(&pitch_pid);
```

## LQR 派生类

`alg_axis_lqr_t` 使用位置误差和速度误差两个状态，增益矩阵长度为 2，并提供平衡控制量与
输出限幅。构造时复制两个增益，配置对象无需长期保存。

```c
alg_axis_lqr_t yaw_lqr;
alg_axis_lqr_init(&yaw_lqr, &lqr_config);
alg_axis_controller_t *yaw_controller =
    alg_axis_lqr_as_controller(&yaw_lqr);
```

## 统一调用

```c
alg_axis_controller_reset(controller, measured_position, measured_velocity, 0.0F);
alg_axis_controller_update(controller, &input, &control_output);
```

切换控制器时应先复位新控制器，使内部状态与当前测量值、当前输出连续，避免突变。

## 边界与所有权

- 本模块不选择 IMU 或编码器反馈源；
- 不执行轨迹规划、单位换算或电机发送；
- 对象由调用者静态分配；
- `gain_matrix` 仅在 LQR 初始化期间读取；
- 初始化后不得按值复制对象；
- 同一对象应由一个控制任务调用，跨任务使用需外部互斥。

## 建议验证

- 未初始化对象和空虚表；
- PID/LQR 对同一阶跃输入的多态调用；
- 输出上下限和前馈叠加；
- 零周期、负周期和非有限输入；
- 控制模式无扰切换；
- 目标位置跨圈前后的连续性由上游保证。
