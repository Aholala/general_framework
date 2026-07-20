# alg_pid

`alg_pid` 是 Algorithm 层的通用 PID 控制算法库。源码与头文件直接位于模块目录，统一使用 `alg_` 文件前缀、`AlgPid` 类型和函数前缀以及 `ALG_PID` 枚举前缀。

## 已实现功能

| 功能 | 对象或配置 |
|---|---|
| 高级位置式 PID | `AlgPid_t` |
| 位置环语义类型 | `AlgPidPosition_t` |
| 速度环语义类型 | `AlgPidVelocity_t` |
| 增量式 PID | `AlgPidIncremental_t` |
| 位置—速度串级 PID | `AlgPidCascade_t` |
| 一维增益调度 PID | `AlgPidGainSchedule_t` |
| 二维模糊自适应 PID | `AlgPidFuzzy_t` |
| 二自由度比例项 | `setpoint_weight` |
| 二自由度微分项 | `derivative_setpoint_weight` |
| 对误差微分 | `ALG_PID_DERIVATIVE_ON_ERROR` |
| 对测量值微分、微分先行 | `ALG_PID_DERIVATIVE_ON_MEASUREMENT` |
| 微分一阶低通 | `derivative_filter_cutoff_hz` |
| 积分限幅 | `integral_min/max` |
| 输出限幅 | `output_min/max` |
| 条件积分抗饱和 | `ALG_PID_ANTI_WINDUP_CLAMPING` |
| 反算抗饱和 | `ALG_PID_ANTI_WINDUP_BACK_CALCULATION` |
| 积分分离 | `integral_separation_threshold` |
| 误差死区 | `error_deadband` |
| 速度前馈 | `velocity_feedforward_gain` |
| 加速度前馈 | `acceleration_feedforward_gain` |
| 任意外部前馈 | `additional_feedforward` |
| 无扰切换/输出跟踪 | `AlgPid_TrackOutput()` |
| 运行时参数更新 | `AlgPid_SetConfig()` |
| P/I/D/FF 分项观测 | `AlgPidTerms_t` |

## 可移植性

- 纯 C11。
- 不依赖 HAL、CMSIS、RTOS 或系统时钟。
- 不使用动态内存。
- 不使用可变全局状态。
- 采样周期由调用者使用秒显式传入。
- 每个对象拥有独立状态，支持多个控制环实例。
- 支持裸机循环、定时中断和 RTOS 周期任务。

## 安全初始化

建议先获得默认配置，再修改需要的参数：

```c
static AlgPidVelocity_t s_velocity_controller;

void VelocityController_Init(void)
{
    AlgPidConfig_t config;

    (void)AlgPidConfig_Init(&config);
    config.proportional_gain = 2.0F;
    config.integral_gain = 10.0F;
    config.derivative_gain = 0.01F;
    config.output_min = -20.0F;
    config.output_max = 20.0F;
    config.integral_min = -5.0F;
    config.integral_max = 5.0F;
    config.derivative_filter_cutoff_hz = 50.0F;

    (void)AlgPidVelocity_Init(&s_velocity_controller, &config);
}
```

默认配置采用：

- 零增益。
- 比例设定值权重为 1。
- 对测量值微分，避免设定值阶跃产生微分冲击。
- 条件积分抗饱和。
- 未启用死区、积分分离和微分滤波。

## 普通位置式 PID

```c
float actuator_command;

AlgPidStatus_t status = AlgPid_Update(&controller,
                                      target,
                                      feedback,
                                      delta_time_s,
                                      &actuator_command);
```

库使用连续时间形式的增益并根据实际 `delta_time_s` 离散计算积分和微分，因此周期发生小幅变化时不需要重新换算 `Ki` 和 `Kd`。

## 高级输入和前馈

```c
AlgPidInput_t input = {
    .setpoint = target_position,
    .measurement = measured_position,
    .setpoint_rate_per_s = target_velocity,
    .setpoint_acceleration_per_s2 = target_acceleration,
    .additional_feedforward = gravity_compensation,
    .delta_time_s = control_period_s
};

AlgPid_UpdateAdvanced(&controller, &input, &output);
```

最终前馈为：

```text
FF = Kvff × setpoint_rate + Kaff × setpoint_acceleration + additional_feedforward
```

## 二自由度 PID

比例项使用：

```text
P = Kp × (beta × setpoint - measurement)
```

其中 `beta = setpoint_weight`，范围为 0 到 1。减小 beta 可以减弱位置设定值阶跃造成的比例冲击，同时保留对扰动的响应。

对误差微分时使用 `derivative_setpoint_weight` 控制设定值进入微分项的比例。设为 0 等效于微分先行，设为 1 是传统误差微分。

## 抗积分饱和

条件积分：

```c
config.anti_windup_mode = ALG_PID_ANTI_WINDUP_CLAMPING;
```

输出已饱和并且误差继续把输出推向饱和方向时暂停积分。它参数少，适合作为默认方案。

反算抗饱和：

```c
config.anti_windup_mode = ALG_PID_ANTI_WINDUP_BACK_CALCULATION;
config.back_calculation_gain = 5.0F;
```

反算通过饱和输出与未饱和输出之间的差值回退积分，适合对退出饱和速度有明确要求的控制器。

## 积分分离和死区

```c
config.integral_separation_threshold = 2.0F;
config.error_deadband = 0.01F;
```

- 积分分离阈值为零表示始终允许积分。
- 误差绝对值大于积分分离阈值时停止新增积分。
- 误差绝对值小于等于死区时，控制误差按零处理。

死区应谨慎使用，过大的死区会导致稳态位置误差和低速爬行。

## 增量式 PID

增量式 PID 计算每周期输出变化量：

```text
Δu = Kp(e-e1) + Ki·e·dt + Kd(e-2e1+e2)/dt
```

它同时支持：

- 单周期增量限幅。
- 总输出限幅。
- 误差死区。
- 微分增量低通。
- 外部前馈增量。

增量式控制器适合输出本身需要连续累加的执行器，但发生状态切换时必须调用 `AlgPidIncremental_Reset()` 设置当前输出。

## 位置环与速度环

位置环和速度环采用相同数学内核，分别使用语义类型：

```c
AlgPidPosition_t position_controller;
AlgPidVelocity_t velocity_controller;
```

推荐配置倾向：

- 位置环通常以 P 或 PD 为主，输出速度目标。
- 速度环通常使用 PI，输出电流、转矩或占空比。
- 速度反馈噪声较大时启用微分滤波或直接不使用 D 项。

具体增益必须根据执行器、负载、采样周期和单位整定，库不会内置与某一电机相关的固定参数。

## 位置—速度串级

`AlgPidCascade_t` 内含两个独立 PID 对象：

```text
位置误差 -> 位置 PID -> 速度目标 -> 速度 PID -> 执行器输出
```

`position_loop_divider` 指定位置环相对速度环的降频倍数。例如速度环为 1 kHz、位置环希望为 200 Hz，则设为 5。

位置环实际使用累积后的真实时间，不会把降频后的周期错误地当作速度环周期。

`velocity_feedforward` 直接叠加到位置环产生的速度目标；`actuator_feedforward` 直接叠加到速度环输出。

## 增益调度

增益调度根据工作点在线线性插值 Kp、Ki、Kd：

```c
static const AlgPidGainPoint_t gain_points[] = {
    {0.0F,   1.0F, 0.5F, 0.01F},
    {100.0F, 2.0F, 0.8F, 0.02F},
    {300.0F, 3.0F, 1.0F, 0.03F}
};
```

工作点必须严格递增。超出表格范围时使用最近端点，不进行外推。表格由调用者持有，生命周期必须覆盖控制器使用期。

## 模糊自适应 PID

`AlgPidFuzzy_t` 根据归一化误差和误差变化率，在二维规则面上对 Kp、Ki、Kd 调整量进行双线性插值：

```text
Kp = base_Kp + fuzzy_delta_Kp(error, error_rate)
Ki = base_Ki + fuzzy_delta_Ki(error, error_rate)
Kd = base_Kd + fuzzy_delta_Kd(error, error_rate)
```

三个规则表均为行优先的正方形数组。行对应从负到正的归一化误差，列对应从负到正的归一化误差变化率。`axis_point_count` 可以使用 3、5、7 等任意不小于 2 的尺寸。

库只实现规则推理和插值，不内置某一电机的经验规则。规则表由具体应用根据对象响应和安全边界提供，这样算法仍然可测试、可审查、可移植。

继电反馈自整定会主动激励真实对象，专家规则也强依赖产品安全策略，因此不在通用运行时 PID 控制器中自动执行；需要时应作为独立的受控调试模块实现。

## 无扰切换

从手动输出切换到闭环前调用：

```c
AlgPid_TrackOutput(&controller,
                   current_setpoint,
                   current_measurement,
                   current_feedforward,
                   current_actuator_output);
```

函数会反算积分项，使切换后的 PID 输出尽量贴合当前执行器输出，降低模式切换冲击。

## 调试信息

```c
const AlgPidTerms_t *terms = AlgPid_GetTerms(&controller);
```

可以分别观察：

- `proportional`
- `integral`
- `derivative`
- `feedforward`
- `unsaturated_output`
- `output`

这些数据适合通过日志或上位机绘图，但算法库本身不负责通信。

## 测试覆盖

`Test/alg_pid_test.c` 覆盖：

- 位置环和速度环。
- 积分累积与积分限幅。
- 条件积分与反算抗饱和。
- 二自由度比例项。
- 对测量值微分和设定值阶跃。
- 速度、加速度及外部前馈。
- 积分分离和误差死区。
- 无扰输出跟踪。
- 增量式 PID。
- 增益调度插值。
- 二维模糊规则面插值。
- 位置—速度串级和不同环路频率。
- 非法参数与未初始化对象。
