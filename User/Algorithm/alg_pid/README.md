# alg_pid — PID 控制器库

纯算法，不依赖硬件。所有时间参数用秒（不是毫秒）。

## 包含的控制器

| 类型 | 结构体 | 用途 |
|------|--------|------|
| 标准 PID | `alg_pid_t` | 位置式 PID + 前馈 |
| 增量 PID | `alg_pid_incremental_t` | 输出增量 |
| 增益调度 | `alg_pid_gain_schedule_t` | 根据调度变量切换增益 |
| 模糊 PID | `alg_pid_fuzzy_t` | 模糊规则在线调参 |
| 串级 PID | `alg_pid_cascade_t` | 外环→内环 |
| 角度 PID | `alg_pid_angle_t` | 处理 -π/+π 环绕 |

## 标准 PID 用法

```c
alg_pid_t pid;
alg_pid_config_t cfg = {
    .kp = 1.5f, .ki = 0.1f, .kd = 0.05f,
    .integral_limit = 10.0f,          // 积分限幅
    .output_limit_min = -12.0f,       // 输出下限
    .output_limit_max = 12.0f,        // 输出上限
};
alg_pid_init(&pid, &cfg);

// 周期更新
alg_pid_input_t in = {
    .setpoint = 100.0f,
    .measurement = 98.5f,
    .feedforward = 0.0f,
};
float output;
alg_pid_update(&pid, &in, 0.001f, &output);  // dt = 1ms

// 读取各分量
const alg_pid_terms_t *terms = alg_pid_get_terms(&pid);
float p = terms->proportional;
float i = terms->integral;
float d = terms->derivative;
```

## 角度 PID（处理 -π/+π 环绕）

```c
alg_pid_angle_t angle_pid;
alg_pid_angle_config_t cfg = { ... };
alg_pid_angle_init(&angle_pid, &cfg);

// setpoint=π, measurement=-3.1 → 误差自动 wrap 为 0.0416（不是 -6.24）
float output;
alg_pid_angle_update(&angle_pid, 3.1415f, -3.1f, 0.001f, &output);
```

## 串级 PID 用法

```c
alg_pid_cascade_t cascade;
alg_pid_cascade_init(&cascade, &outer_cfg, &inner_cfg);

// 外环输出作为内环 setpoint
float inner_output;
alg_pid_cascade_update(&cascade, outer_sp, outer_mv, inner_mv, dt, &inner_output);
```

## API 速查（标准 PID）

| 函数 | 功能 |
|------|------|
| `alg_pid_init(me, cfg)` | 初始化 |
| `alg_pid_update(me, input, dt, &output)` | 更新（输入/输出通过指针） |
| `alg_pid_reset(me)` | 重置积分和微分历史 |
| `alg_pid_get_terms(me)` | 获取 P/I/D/FF 分量（只读） |
| `alg_pid_set_gains(me, kp, ki, kd)` | 运行时改增益 |
| `alg_pid_set_limits(me, min, max)` | 运行时改限幅 |
