# 发射机构控制模块 (module_shooter)

## 1. 模块概述

`module_shooter` 是 RoboMaster 发射机构的状态机控制模块，组合左右摩擦轮电机和拨弹电机，提供摩擦轮启停、排队发射、位置步进、堵转确认、自动回退、有限重试和故障锁存功能。该模块基于 `module_motor` 基类，可与 M3508、M2006 或其他实现同一基类的电机配合使用。

**核心功能**：

- 左右摩擦轮独立方向控制与速度设定
- 拨弹盘步进式位置控制（每次射击步进固定角度）
- 排队发射（支持多发连续射击，带队列上限保护）
- 堵转检测（速度阈值 + 电流阈值双重判断）
- 堵转后自动回退并重试（有限重试次数）
- 故障锁存（超过最大重试次数或电机异常）
- 取消待发请求（清空队列）

**设计哲学**：

- **状态机驱动**：通过有限状态机管理发射流程，逻辑清晰可预测。
- **故障安全**：堵重试次数限制和故障锁存，防止硬件损坏。
- **非阻塞设计**：`update` 函数仅推进状态和设置电机目标，不阻塞主循环。
- **模块化**：与具体电机型号解耦，通过 `module_motor_t *` 注入依赖。

## 2. 设计边界

| **模块负责**               | **模块不负责**                      |
| :------------------------- | :---------------------------------- |
| 拨弹电机步进控制和堵转检测 | 电机 PID 参数调优（由电机层负责）   |
| 摩擦轮使能/禁用和速度控制  | 热量限制和裁判系统弹速限制          |
| 发射队列管理和流量控制     | 射频控制和摩擦轮到速判定策略        |
| 堵转回退和重试机制         | 拨弹盘的实际安装位置（由 App 配置） |
| 故障锁存和恢复             | 裁判系统帧协议解析                  |

## 3. 状态机

```text
                    ┌──────────────────────────────────────┐
                    │              DISABLED               │
                    │        (所有目标归零)               │
                    └──────────────────┬───────────────────┘
                                       │ module_shooter_enable()
                                       ▼
                    ┌──────────────────────────────────────┐
                    │               READY                 │
                    │      (摩擦轮可运行，等待射击)        │
                    └──────────────────┬───────────────────┘
                                       │ pending_shots > 0
                                       ▼
                    ┌──────────────────────────────────────┐
                    │              FEEDING                │
                    │    (拨弹盘向目标位置运动)             │
                    └──────────┬────────────┬─────────────┘
                               │            │
                    ┌──────────▼────────┐   │ 正常到位
                    │     堵转检测       │   │
                    │  (速度低+电流高)   │   │
                    └──────────┬────────┘   │
                               │            │
                               ▼            ▼
                    ┌──────────────────────────────────────┐
                    │              ROLLBACK               │
                    │      (反向退让，释放卡弹)             │
                    └──────────────────┬───────────────────┘
                                       │ 回退到位
                                       ▼
                    ┌──────────────────────────────────────┐
                    │              FEEDING                │
                    │        (再次尝试送弹)                 │
                    └──────────────────┬───────────────────┘
                                       │ 超过最大重试
                                       ▼
                    ┌──────────────────────────────────────┐
                    │               FAULT                 │
                    │          (故障锁存，停机)             │
                    └──────────────────────────────────────┘
```

| 状态       | 说明                         | 电机目标                                 |
| :--------- | :--------------------------- | :--------------------------------------- |
| `DISABLED` | 所有电机禁用，目标归零       | 无输出                                   |
| `READY`    | 就绪，等待射击请求           | 摩擦轮按设定运行，拨弹保持当前位置       |
| `FEEDING`  | 送弹中，拨弹盘向目标位置运动 | 拨弹目标 = 当前 + step_rad               |
| `ROLLBACK` | 堵转后退让                   | 拨弹目标 = 当前位置 - rollback_angle_rad |
| `FAULT`    | 故障锁存，停止所有操作       | 无输出                                   |

## 4. 核心配置参数

| 参数                               | 说明                             | 典型值     |
| :--------------------------------- | :------------------------------- | :--------- |
| `feeder_step_rad`                  | 单次射击拨弹盘步进角度           | 2.0 rad    |
| `feeder_position_tolerance_rad`    | 拨弹到位容差                     | 0.05 rad   |
| `jam_velocity_threshold_rad_per_s` | 堵转速度阈值（低于此值视为堵转） | 0.5 rad/s  |
| `jam_current_threshold_a`          | 堵转电流阈值（优先使用）         | 3.0 A      |
| `jam_current_threshold_raw`        | 堵转电流原始值阈值（备用）       | 需实车标定 |
| `jam_confirmation_time_s`          | 堵转确认时间                     | 0.2 s      |
| `rollback_angle_rad`               | 堵转回退角度                     | 0.5 rad    |
| `rollback_position_tolerance_rad`  | 回退到位容差                     | 0.05 rad   |
| `maximum_jam_retries`              | 最大堵转重试次数                 | 3 次       |
| `maximum_pending_shots`            | 最大待发弹量                     | 20 发      |

## 5. 堵转检测机制

堵转在 `FEEDING` 状态下检测，需同时满足以下条件：

1. **速度低于阈值**：`|feeder_feedback.velocity_rad_per_s| <= jam_velocity_threshold_rad_per_s`
2. **电流超过阈值**：优先使用 `jam_current_threshold_a`（安培），若 `is_current_a_valid` 为 false，则使用 `jam_current_threshold_raw`（原始值）
3. **持续确认时间**：上述条件持续 `jam_confirmation_time_s` 后确认堵转

**检测到堵转后的处理流程**：

```text
堵转确认
  ↓
jam_retry_count++
  ↓
超过 maximum_jam_retries? ──是──→ FAULT（故障锁存）
  ↓ 否
ROLLBACK（回退 rollback_angle_rad）
  ↓
等待回退到位（误差 ≤ rollback_position_tolerance_rad）
  ↓
FEEDING（再次尝试送弹）
```

## 6. API 参考

| 函数                                 | 说明                                       | 返回值                              |
| :----------------------------------- | :----------------------------------------- | :---------------------------------- |
| `module_shooter_init`                | 初始化发射机构                             | `OK` / `INVALID_ARGUMENT`           |
| `module_shooter_enable`              | 使能发射机构（使能三个电机，状态 → READY） | `OK` / `MOTOR_ERROR` / `NOT_READY`  |
| `module_shooter_disable`             | 禁用发射机构（禁用电机，清空队列）         | `OK` / `MOTOR_ERROR`                |
| `module_shooter_set_friction`        | 设置摩擦轮使能状态和目标速度               | `OK` / `INVALID_ARGUMENT`           |
| `module_shooter_request_shots`       | 请求发射（加入队列）                       | `OK` / `INVALID_ARGUMENT` / `FAULT` |
| `module_shooter_cancel_shots`        | 取消所有待发射请求                         | `OK`                                |
| `module_shooter_reset_fault`         | 清除故障状态（需电机在线）                 | `OK` / `NOT_READY`                  |
| `module_shooter_update`              | 周期更新状态机                             | `OK` / `FAULT` / `NOT_READY`        |
| `module_shooter_get_state`           | 获取当前状态                               | 状态枚举                            |
| `module_shooter_get_pending_shots`   | 获取待发弹量                               | 弹量                                |
| `module_shooter_get_jam_retry_count` | 获取当前堵重重试次数                       | 重试次数                            |

## 7. 使用示例

### 7.1 初始化

```c
static module_shooter_t s_shooter;

const module_shooter_config_t cfg = {
    .left_friction_motor = &left_friction_motor.super,
    .right_friction_motor = &right_friction_motor.super,
    .feeder_motor = &feeder_motor.super,
    .left_friction_direction_sign = 1.0F,
    .right_friction_direction_sign = -1.0F,   // 左右摩擦轮反向旋转
    .feeder_direction_sign = 1.0F,
    .feeder_step_rad = 2.0F,
    .feeder_position_tolerance_rad = 0.05F,
    .jam_velocity_threshold_rad_per_s = 0.5F,
    .jam_current_threshold_a = 3.0F,
    .jam_current_threshold_raw = 3000,
    .jam_confirmation_time_s = 0.2F,
    .rollback_angle_rad = 0.5F,
    .rollback_position_tolerance_rad = 0.05F,
    .maximum_jam_retries = 3,
    .maximum_pending_shots = 20,
};

module_shooter_init(&s_shooter, &cfg);
```

### 7.2 启用和设置摩擦轮

```c
// 使能发射机构
module_shooter_enable(&s_shooter);

// 启动摩擦轮（目标速度 300 rad/s）
module_shooter_set_friction(&s_shooter, true, 300.0F);

// 请求发射 1 发
module_shooter_request_shots(&s_shooter, 1);
```

### 7.3 周期更新

```c
void control_loop(float dt) {
    // 更新发射机构状态机
    module_shooter_status_t status = module_shooter_update(&s_shooter, dt);
    if (status == MODULE_SHOOTER_STATUS_FAULT) {
        // 故障处理：停止摩擦轮，记录错误
        module_shooter_set_friction(&s_shooter, false, 0.0F);
    }

    // 检查状态
    if (module_shooter_get_state(&s_shooter) == MODULE_SHOOTER_STATE_READY) {
        // 可以继续请求发射
    }
}
```

### 7.4 故障恢复

```c
// 故障发生后，需检查电机状态并手动恢复
if (module_shooter_get_state(&s_shooter) == MODULE_SHOOTER_STATE_FAULT) {
    // 确保所有电机在线
    if (电机已恢复) {
        module_shooter_reset_fault(&s_shooter);
        module_shooter_enable(&s_shooter);
    }
}
```

## 8. 错误码速查

| 状态码             | 触发场景                                       |
| :----------------- | :--------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、阈值非法、方向符号非法、步进角度 ≤ 0 |
| `NOT_INITIALIZED`  | 对象未初始化                                   |
| `NOT_READY`        | 电机离线或反馈无效                             |
| `MOTOR_ERROR`      | 电机使能/禁用/设置目标/更新失败                |
| `FAULT`            | 超过最大堵重重试次数                           |

## 9. 注意事项

- **电机依赖**：三个电机必须在调用 `enable` 前已注册、初始化且在线。
- **故障恢复**：`reset_fault` 需要拨弹电机在线且反馈有效，否则返回 `NOT_READY`。
- **队列上限**：`maximum_pending_shots` 防止遥控器抖动导致无界累积。
- **摩擦轮方向**：左右摩擦轮通常反向旋转，通过 `direction_sign` 配置。
- **电流阈值**：优先使用安培阈值（需电机层正确换算），否则使用原始值。

## 10. 建议验证测试项

- [ ] 摩擦轮启停方向正确（左右反向）
- [ ] 单发：从 READY → FEEDING → READY 完整流程
- [ ] 多发：连续请求多发射击，队列正常递减
- [ ] 队列上限：超过 `maximum_pending_shots` 返回错误
- [ ] 正常到位：拨弹盘到达目标位置后状态回到 READY
- [ ] 瞬时大电流不误判（未达到确认时间不触发）
- [ ] 持续堵转：触发 ROLLBACK → FEEDING 重试
- [ ] 超过最大重试：进入 FAULT 并锁存
- [ ] 任一电机离线：`update` 返回 `NOT_READY`
- [ ] 故障恢复：`reset_fault` 后状态回到 READY

---

**总结**：`module_shooter` 提供了完整的 RoboMaster 发射机构控制方案，通过状态机管理送弹流程、堵转检测和故障恢复。其模块化设计使其可与任意 `module_motor` 派生类配合，适应不同电机型号和安装方向。配合电机层和上层控制逻辑，可构建可靠的弹丸发射系统。

## 一页式接入顺序与可读信息

```c
/* 1. 三个电机必须先初始化、注册并能提供有效反馈。 */
static module_shooter_t shooter;

/* 2. 配置中注入左右摩擦轮和拨弹电机，以及速度、堵转和重试参数。 */
module_shooter_status_t status = module_shooter_init(&shooter, &shooter_config);

/* 3. 使能模块，再设置摩擦轮目标；未使能时不接受发射流程。 */
status = module_shooter_enable(&shooter);
status = module_shooter_set_friction(
    &shooter, friction_enabled, target_friction_velocity_rad_per_s);

/* 4. 请求发射只增加待发数量，真正送弹由 update 状态机执行。 */
status = module_shooter_request_shots(&shooter, shot_count);

/* 5. 固定周期调用并检查返回值；电机离线时立即进入上层安全处理。 */
status = module_shooter_update(&shooter, delta_time_s);

/* 6. 停机先 cancel_shots，再 disable；FAULT 只能显式 reset_fault。 */
```

| 可读取信息 | API | 说明 |
| --- | --- | --- |
| `module_shooter_state_t` | `module_shooter_get_state()` | 禁用、准备、送弹、回退或故障状态 |
| 待发弹数 | `module_shooter_get_pending_shots()` | 尚未完成的发射请求数量 |
| 堵转重试次数 | `module_shooter_get_jam_retry_count()` | 当前发射流程已经执行的回退次数 |
| 电机反馈 | `module_motor_get_feedback()` | 三个依赖电机的位置、速度、温度和在线状态 |
| `module_shooter_t` | 调试器只读查看 | 状态机计时、目标值和内部阶段；应用不要直接修改 |
