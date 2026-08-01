# 通用电机基类（module_motor）

`module_motor` 统一电机注册、状态、反馈、运行统计和控制更新。具体协议由 DJI、DM 等派生模块实现。

## 使用顺序

```c
/* 1. 准备静态注册表。 */
static module_motor_t *motor_storage[MOTOR_COUNT];
static module_motor_registry_t registry;
module_motor_registry_init(&registry, motor_storage, MOTOR_COUNT);

/* 2. 初始化具体型号并注册，不能直接实例化基类。 */
module_m3508_init(&drive_motor, &drive_motor_config);
module_m3508_register(&drive_motor, &registry);

/* 3. 路由一次有效反馈后使能。 */
module_motor_t *motor = module_m3508_as_motor(&drive_motor);
module_motor_enable(motor);

/* 4. 每个控制周期设置目标并传入真实 dt。 */
module_motor_set_target(motor, target_value);
module_motor_update(motor, delta_time_s);

/* 5. 单独更新反馈年龄；超时后基类自动进入 FAULT。 */
module_motor_update_feedback_time(motor, elapsed_time_ms);

/* 6. 停机先 disable，再让协议总线发送零命令，最后注销。 */
```

## PID 形式选择

`module_motor_pid_t` 是电机控制环的统一容器：

```c
module_motor_pid_config_t loop_config = {
    .form = MODULE_MOTOR_PID_POSITIONAL, /* 位置式 PID 算法 */
    .positional_config = positional_pid_config,
};

/* 或 */
loop_config.form = MODULE_MOTOR_PID_INCREMENTAL;
loop_config.incremental_config = incremental_pid_config;
```

“位置式 PID”是数学形式，不等于电机角度环。DJI 电机的电流、速度和角度三个环各有一份配置，可以独立选择位置式或增量式。

## 可读取结构体

### `module_motor_t`

| 字段 | 信息 |
| --- | --- |
| `motor_name` | 调试可见的电机名称；字符串存储必须长期有效 |
| `registration_key` | 软件注册键 |
| `motor_identifier` | 派生协议电机 ID/主机 ID |
| `state` | `DISABLED`、`ENABLED` 或 `FAULT` |
| `feedback` | 完整反馈结构体 |
| `delta_time_s` | 最近一次成功控制更新使用的 dt |
| `total_runtime_us` | 累计成功更新时间，包含失能状态 |
| `enabled_runtime_us` | 累计使能运行时间；使用整数微秒避免长期 float 精度下降 |
| `control_update_count` | 成功控制更新次数 |
| `last_update_status` | 最近一次 update 的返回状态 |
| `feedback_timeout_ms` | 反馈超时阈值 |

### `module_motor_feedback_t`

可读取 `position_rad`、`velocity_rad_per_s`、`torque_nm`、`current_a`、`motor_temperature_c`、协议原始位置/电流、反馈次数、反馈年龄和在线状态。读取 `current_a` 前检查 `is_current_a_valid`。

### `module_motor_pid_t`

可读取 PID `form`、位置式或增量式控制器状态，以及其中的 `alg_pid_terms_t`。业务代码可调用 `module_motor_pid_get_terms()`，不要直接改历史误差、积分或累计输出。

## 约束

- 目标接口的单位由派生模块固定，App 不应靠同一个 float 猜测单位。
- `module_motor_update()` 的 dt 必须有限且大于零。
- 反馈离线、过温或驱动故障时必须清零最终命令。
- 注册表和总线都只保存对象指针，对象存储必须在整个运行期有效。
- DM 电机使用驱动器内部闭环模式；软件三级 PID 当前用于具有连续电流反馈的 DJI 控制链。

健康监控见 [README_health.md](README_health.md)，DJI 三级控制链见 [../module_dji_motor/README.md](../module_dji_motor/README.md)。
