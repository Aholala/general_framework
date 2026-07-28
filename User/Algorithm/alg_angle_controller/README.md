# 角度控制器多态接口 (alg_angle_controller) —— 完整使用指南

## 1. 模块概述

`alg_angle_controller` 提供了角度控制器的多态接口，相同的 `alg_angle_controller_t *` 可以指向串级 PID 或二维 LQR 控制器，使云台、舵向等上层逻辑不依赖具体闭环算法。

**核心功能**：

- 串级 PID 控制器封装（`alg_angle_pid_t`）
- 二维 LQR 控制器封装（`alg_angle_lqr_t`）
- 统一的多态接口（`reset` / `update`）
- 支持前馈叠加

- **多态接口**：上层逻辑通过基类指针调用，不依赖具体控制器类型
- **无切换冲击**：切换控制器时应先 `reset`，使内部状态与当前测量值连续
- **无动态内存**：所有对象由调用者静态分配

## 2. 对象关系

```text
alg_angle_controller_t                    (基类：vptr、is_initialized)
    ├── alg_angle_pid_t                   (派生：封装 alg_pid_cascade_t)
    └── alg_angle_lqr_t                   (派生：封装 alg_lqr_controller_t)
```

## 3. 设计边界

| **模块负责**                         | **模块不负责**           |
| :----------------------------------- | :----------------------- |
| 角度控制器的多态接口（reset/update） | 轨迹规划                 |
| PID / LQR 两种控制算法的封装         | 单位换算                 |
| 控制输出限幅和前馈叠加               | 电机发送                 |
| 输入校验（有限数、正时间步长）       | 反馈源选择（IMU/编码器） |

## 4. 核心类型

### 4.1 控制器输入 (`alg_angle_controller_input_t`)

```c
typedef struct {
    float target_position_rad;          // 目标位置（弧度）
    float target_velocity_rad_per_s;    // 目标速度（rad/s）
    float measured_position_rad;        // 测量位置（弧度）
    float measured_velocity_rad_per_s;  // 测量速度（rad/s）
    float actuator_feedforward;         // 执行器前馈
    float delta_time_s;                 // 控制周期（秒），必须 > 0
} alg_angle_controller_input_t;
```

全部角度使用弧度，角速度使用弧度/秒。调用方必须先处理编码器跨圈或角度连续化。

### 4.2 PID 控制器配置 (`alg_angle_pid_config_t`)

```c
typedef struct {
    alg_pid_cascade_config_t cascade_config;
} alg_angle_pid_config_t;
```

串级 PID 结构：外环（位置 → 速度目标）→ 内环（速度 → 输出）。

### 4.3 LQR 控制器配置 (`alg_angle_lqr_config_t`)

```c
typedef struct {
    const float *gain_matrix;       // 增益矩阵（2 个元素：[Kp, Kd]）
    float control_min;              // 输出下限
    float control_max;              // 输出上限
    float equilibrium_control;      // 平衡控制量（稳态输出）
} alg_angle_lqr_config_t;
```

## 5. API 参考

| 函数                          | 说明                   | 返回值                    |
| :---------------------------- | :--------------------- | :------------------------ |
| `alg_angle_pid_init`          | 初始化 PID 角度控制器  | `OK` / `ALGORITHM_ERROR`  |
| `alg_angle_lqr_init`          | 初始化 LQR 角度控制器  | `OK` / `ALGORITHM_ERROR`  |
| `alg_angle_pid_as_controller` | 转为基类指针           | 基类指针 / `NULL`         |
| `alg_angle_lqr_as_controller` | 转为基类指针           | 基类指针 / `NULL`         |
| `alg_angle_controller_reset`  | 重置控制器状态（多态） | `OK` / `INVALID_ARGUMENT` |
| `alg_angle_controller_update` | 更新控制器输出（多态） | `OK` / `INVALID_ARGUMENT` |

## 6. 使用示例

### 6.1 PID 控制器初始化

```c
static alg_angle_pid_t s_pitch_pid;

// 配置串级 PID
alg_pid_cascade_config_t cascade_config = {
    .position_pid = {
        .proportional_gain = 5.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .integral_limit = 0.0F,
        .output_limit = 30.0F,
    },
    .velocity_pid = {
        .proportional_gain = 0.5F,
        .integral_gain = 0.01F,
        .derivative_gain = 0.001F,
        .integral_limit = 100.0F,
        .output_limit = 30000.0F,
    },
};

alg_angle_pid_config_t config = {.cascade_config = cascade_config};
alg_angle_pid_init(&s_pitch_pid, &config);

// 获取基类指针
alg_angle_controller_t *controller = alg_angle_pid_as_controller(&s_pitch_pid);
```

### 6.2 LQR 控制器初始化

```c
static alg_angle_lqr_t s_yaw_lqr;

const float gain_matrix[2] = {10.0F, 2.0F};  // [Kp, Kd]

alg_angle_lqr_config_t config = {
    .gain_matrix = gain_matrix,
    .control_min = -30000.0F,
    .control_max = 30000.0F,
    .equilibrium_control = 0.0F,
};

alg_angle_lqr_init(&s_yaw_lqr, &config);

// 获取基类指针
alg_angle_controller_t *controller = alg_angle_lqr_as_controller(&s_yaw_lqr);
```

### 6.3 统一调用

```c
// 1. 重置控制器（切换控制器时调用，使状态连续）
alg_angle_controller_reset(controller, current_position, current_velocity, 0.0F);

// 2. 周期控制
alg_angle_controller_input_t input = {
    .target_position_rad = target_position,
    .target_velocity_rad_per_s = target_velocity,
    .measured_position_rad = current_position,
    .measured_velocity_rad_per_s = current_velocity,
    .actuator_feedforward = feedforward,
    .delta_time_s = 0.01F,
};

float output;
alg_angle_controller_update(controller, &input, &output);
```

### 6.4 无扰切换示例

```c
// 从 PID 切换到 LQR
alg_angle_controller_t *old_controller = alg_angle_pid_as_controller(&pid);
alg_angle_controller_t *new_controller = alg_angle_lqr_as_controller(&lqr);

// 1. 获取当前状态
float pos = current_position;
float vel = current_velocity;

// 2. 重置新控制器（使用当前状态）
alg_angle_controller_reset(new_controller, pos, vel, 0.0F);

// 3. 后续使用新控制器
controller = new_controller;
```

## 7. 控制器对比

| 特性     | PID 串级              | LQR                       |
| :------- | :-------------------- | :------------------------ |
| 状态维度 | 2（位置 + 速度）      | 2（位置 + 速度）          |
| 调参方式 | 手动整定 Kp/Ki/Kd     | 增益矩阵（LQR 理论）      |
| 前馈支持 | 速度前馈 + 执行器前馈 | 执行器前馈                |
| 输出限幅 | 内环 output_limit     | control_min / control_max |
| 内部状态 | 积分项                | 无（静态增益）            |
| 适用场景 | 工程经验整定          | 已知系统模型              |

## 8. 注意事项

- **角度连续化**：调用方必须先处理编码器跨圈或角度连续化，控制器不处理角度包裹
- **切换冲击**：切换控制器时应先 `reset`，使内部状态与当前测量值连续
- **线程安全**：同一对象应由一个控制任务调用，跨任务需外部互斥
- **无动态内存**：对象由调用者静态分配，`gain_matrix` 仅在 LQR 初始化期间读取
- **初始化后不可复制**：对象按值复制会破坏虚表引用

## 9. 错误码速查

| 错误码             | 触发场景                                                 |
| :----------------- | :------------------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、输入包含 NaN/Inf、delta_time ≤ 0、LQR 配置非法 |
| `NOT_INITIALIZED`  | 对象未初始化或虚表为空                                   |
| `ALGORITHM_ERROR`  | PID/LQR 底层算法初始化或更新失败                         |

## 10. 建议验证测试项

- [ ] 未初始化对象调用返回 `NOT_INITIALIZED`
- [ ] PID 和 LQR 对相同阶跃输入的响应对比
- [ ] 输出上下限生效
- [ ] 前馈正确叠加
- [ ] 零周期、负周期和非有限输入返回 `INVALID_ARGUMENT`
- [ ] 控制器切换无冲击（reset 后输出连续）
- [ ] 长时间运行（> 10 分钟）无发散

---

**总结**：`alg_angle_controller` 提供了角度控制器的多态接口，通过虚表将 PID 和 LQR 两种算法统一到相同的 API 下。上层逻辑只需要持有 `alg_angle_controller_t *`，即可在不感知具体控制器类型的情况下完成控制。这种设计使云台、舵向等系统的控制器切换变得简单，且无切换冲击。配合 `module_swerve` 和 `module_gm6020` 等模块，可快速构建完整的伺服控制系统。
