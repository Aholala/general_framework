# 电机健康监控模块 (module_motor_health) —— 完整使用指南

## 1. 模块概述

`module_motor_health` 是一个多电机健康聚合器，将注册、在线、故障、使能和温度等状态转换为稳定的可用性数组，供底盘降级运动学和 App 安全状态机使用。它还支持过流、编码器突跳、跟踪误差、堵转、饱和和总线错误等扩展诊断。

**核心功能**：

- 聚合多电机的健康状态（注册、在线、故障、使能、温度）
- 扩展诊断：过流、编码器突跳、跟踪误差、堵转、输出饱和、总线错误
- 故障确认和恢复防抖（避免瞬时抖动误报）
- 逐电机阈值配置（数组方式）
- 通过 `observer` 回调获取控制器内部状态

**设计哲学**：

- **原因位掩码**：使用位掩码表示多个健康原因，便于诊断和遥测
- **防抖机制**：故障确认时间和恢复确认时间，避免瞬时状态变化导致误判
- **可扩展性**：通过 `observer` 回调获取任意电机派生类的内部状态
- **零侵入**：不发送电机命令、不清除故障、不自动重新使能

## 2. 设计边界

| **模块负责**         | **模块不负责**       |
| :------------------- | :------------------- |
| 多电机健康状态聚合   | 发送电机命令         |
| 健康原因检测和位掩码 | 清除电机故障         |
| 故障/恢复防抖确认    | 自动重新使能电机     |
| 可用性状态输出       | 电机对象的所有权管理 |
| 反馈超时管理（可选） | 控制策略决策         |

## 3. 对象关系

```text
module_motor_health_t
    ├── motors[]          → module_motor_t 数组（引用）
    ├── states[]          → 健康状态（由本模块维护）
    └── observer          → 外部诊断回调
```

## 4. 核心概念

### 4.1 健康原因位掩码

| 位掩码             | 说明                         |
| :----------------- | :--------------------------- |
| `NOT_REGISTERED`   | 电机未注册到注册表           |
| `OFFLINE`          | 反馈离线（超时未收到数据）   |
| `MOTOR_FAULT`      | 电机基类处于 FAULT 状态      |
| `NOT_ENABLED`      | 电机未使能（可选检查）       |
| `OVER_TEMPERATURE` | 电机过温                     |
| `OVER_CURRENT`     | 过流                         |
| `ENCODER_JUMP`     | 编码器突跳                   |
| `TRACKING_ERROR`   | 跟踪误差超限                 |
| `STALL`            | 堵转（电流大、速度小）       |
| `OUTPUT_SATURATED` | 输出饱和                     |
| `BUS_ERROR`        | 总线错误（CAN 错误计数变化） |

### 4.2 故障确认与恢复

- **故障确认**：原因持续达到 `fault_confirmation_time_ms` 后才标记不可用
- **恢复确认**：所有原因消失并持续达到 `recovery_confirmation_time_ms` 后才恢复可用
- 防抖机制避免瞬时噪声导致状态频繁切换

### 4.3 观察者回调

```c
typedef bool (*module_motor_health_observer_t)(
    const module_motor_t *motor,
    module_motor_health_observation_t *observation,
    void *user_context);
```

调用者通过此回调提供：

- `commanded_effort`：控制器输出命令值
- `tracking_error`：跟踪误差（命令值 - 实际值）
- `output_limit`：输出限幅值
- `bus_error_count`：总线错误计数

## 5. 使用示例

### 5.1 配置阈值数组

```c
// 假设有 4 个电机
static const float max_temp[] = {60.0F, 60.0F, 65.0F, 65.0F};
static const float max_current[] = {20.0F, 20.0F, 15.0F, 15.0F};
static const uint32_t max_encoder_step[] = {100, 100, 100, 100};
static const uint32_t encoder_modulus[] = {8192, 8192, 8192, 8192};
static const float max_tracking_error[] = {0.1F, 0.1F, 0.1F, 0.1F};
static const float stall_current[] = {15.0F, 15.0F, 10.0F, 10.0F};
static const float stall_velocity[] = {0.5F, 0.5F, 0.5F, 0.5F};
```

### 5.2 实现观察者回调

```c
static bool my_observer(const module_motor_t *motor,
                        module_motor_health_observation_t *obs,
                        void *ctx) {
    // 从电机派生类获取内部状态
    module_my_motor_t *me = MODULE_MOTOR_CONTAINER_OF(motor, module_my_motor_t, super);
    obs->commanded_effort = me->last_command;    // 控制器输出
    obs->tracking_error = me->target - me->feedback.position_rad;  // 跟踪误差
    obs->output_limit = me->command_limit;       // 输出限幅
    obs->bus_error_count = me->can_error_count;  // 总线错误计数
    obs->is_valid = true;
    return true;
}
```

### 5.3 初始化健康模块

```c
static module_motor_t *motors[] = {&motor1.super, &motor2.super, &motor3.super, &motor4.super};
static module_motor_health_state_t states[4];
static module_motor_health_t health;

module_motor_health_config_t cfg = {
    .motors = motors,
    .motor_count = 4,
    .state_storage = states,
    .maximum_temperature_c = max_temp,
    .maximum_current_a = max_current,
    .maximum_encoder_step = max_encoder_step,
    .encoder_modulus = encoder_modulus,
    .maximum_tracking_error = max_tracking_error,
    .stall_current_a = stall_current,
    .stall_velocity_rad_per_s = stall_velocity,
    .output_saturation_ratio = 0.95F,
    .stall_confirmation_time_ms = 100,
    .saturation_confirmation_time_ms = 50,
    .observer = my_observer,
    .observer_user_context = NULL,
    .fault_confirmation_time_ms = 50,
    .recovery_confirmation_time_ms = 100,
    .require_enabled_state = true,
    .manage_feedback_time = true,
};

module_motor_health_init(&health, &cfg);
```

### 5.4 周期更新与查询

```c
void control_loop(uint32_t dt_ms) {
    // 1. 更新健康状态
    module_motor_health_status_t status = module_motor_health_update(&health, dt_ms);

    // 2. 获取可用性数组
    bool available[4];
    module_motor_health_get_availability(&health, available, 4);

    // 3. 根据可用性决定控制策略
    if (available[0] && available[1]) {
        // 正常行驶
        move_forward(speed);
    } else if (!available[0]) {
        // 左电机故障，右转
        turn_right();
    } else {
        // 降级模式
        stop();
    }

    // 4. 诊断信息
    for (int i = 0; i < 4; i++) {
        const module_motor_health_state_t *s = module_motor_health_get_state(&health, i);
        if (!s->is_available) {
            printf("Motor %d unavailable, reason: 0x%08X\n", i, s->reason_mask);
        }
    }
}
```

### 5.5 与运动学连接

```c
void chassis_update(bool *motor_available, float *cmd_vel) {
    // 直接传给麦轮/全向轮运动学算法
    // 不可用的电机对应输出为 0
    for (int i = 0; i < 4; i++) {
        if (!motor_available[i]) {
            cmd_vel[i] = 0.0F;   // 该轮不输出
        }
    }
    // 运动学计算...
}
```

## 6. 配置参数详解

| 参数                       | 说明                 | NULL 表示          |
| :------------------------- | :------------------- | :----------------- |
| `maximum_temperature_c`    | 逐电机最大温度       | 禁用过温检查       |
| `maximum_current_a`        | 逐电机最大电流       | 禁用过流检查       |
| `maximum_encoder_step`     | 逐电机最大编码器步长 | 禁用编码器突跳检查 |
| `encoder_modulus`          | 逐电机编码器模数     | 不使用回绕最短路径 |
| `maximum_tracking_error`   | 逐电机最大跟踪误差   | 禁用跟踪误差检查   |
| `stall_current_a`          | 逐电机堵转电流阈值   | 禁用堵转检测       |
| `stall_velocity_rad_per_s` | 逐电机堵转速度阈值   | 禁用堵转检测       |
| `output_saturation_ratio`  | 输出饱和比例（0~1）  | -                  |
| `observer`                 | 诊断观察回调         | 禁用扩展诊断       |

## 7. 反馈时间所有权

- `manage_feedback_time = true`：本模块调用 `module_motor_update_feedback_time`
- `manage_feedback_time = false`：调用者自行管理反馈超时
- **关键**：系统只能有一个时间推进所有者，不能 App 和健康模块同时累加

## 8. 错误码速查

| 状态码             | 触发场景                         |
| :----------------- | :------------------------------- |
| `OK`               | 所有电机可用                     |
| `DEGRADED`         | 部分电机不可用                   |
| `INVALID_ARGUMENT` | 参数为空、阈值无效、输出容量不足 |
| `NOT_INITIALIZED`  | 对象未初始化                     |
| `MOTOR_ERROR`      | 电机反馈更新失败                 |

## 9. 建议验证测试项

- [ ] 每个原因位单独触发和组合触发
- [ ] 故障确认时间边界（小于/大于配置值）
- [ ] 恢复确认时间边界
- [ ] 温度数组为 NULL 时跳过检查
- [ ] `require_enabled_state` true/false 不同行为
- [ ] 反馈时间单一所有者（不重复累加）
- [ ] 多电机不同健康状态
- [ ] 时间累加溢出保护
- [ ] 输出容量不足时返回错误

---

**总结**：`module_motor_health` 提供了完整的多电机健康监控框架，通过原因位掩码精确定位问题，通过故障/恢复防抖避免误报，通过 `observer` 回调支持任意电机派生类的扩展诊断。配合运动学算法，可实现底盘降级控制和安全决策。
