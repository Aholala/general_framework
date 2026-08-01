# 舵轮执行模块 (module_swerve)

## 1. 模块概述

`module_swerve` 是单个舵轮（Swerve Module）的执行模块，负责将 `alg_swerve_module_target_t` 运动学目标转换为驱动电机速度目标和舵向电机位置目标，并驱动两个物理电机执行。底盘几何解算和运动学优化由 `alg_swerve` 负责，本模块只处理单个物理舵轮的电机控制。

**核心功能**：

- 将线速度目标转换为驱动电机角速度（考虑轮半径、减速比、方向符号）
- 将舵向角度目标转换为舵向电机位置目标（叠加机械零位、方向符号）
- 启用/禁用单个舵轮的两个电机
- 读取当前舵向角度（扣除零位并应用方向符号）
- 调用 `alg_swerve_optimize_target` 优化目标（选择最短转向路径）

**设计哲学**：

- **单一职责**：只负责一个物理舵轮的电机控制，不涉及整车运动学
- **模块化**：通过 `module_motor_t *` 注入驱动电机和舵向电机，与具体电机型号解耦
- **安全优先**：任一电机未使能或离线时返回错误，避免单边输出

## 2. 设计边界

| **模块负责**                          | **模块不负责**                                       |
| :------------------------------------ | :--------------------------------------------------- |
| 单个舵轮的驱动电机和舵向电机控制      | 整车运动学计算（由 `alg_swerve` 负责）               |
| 线速度 → 电机角速度换算               | 舵向目标优化（由 `alg_swerve_optimize_target` 负责） |
| 舵向角度 → 电机位置换算（含零位补偿） | 舵轮零位标定（由 App 或标定流程负责）                |
| 电机使能/禁用状态管理                 | 电机故障自动清除                                     |
| 舵向角度反馈读取                      | 多舵轮之间的协调和降级策略                           |

## 3. 对象模型

```text
module_swerve_t
    ├── drive_motor      → module_motor_t *（驱动电机，如 M3508）
    ├── steering_motor   → module_motor_t *（舵向电机，如 GM6020）
    ├── 配置参数         → 轮半径、减速比、零位偏移、方向符号
    └── 状态             → is_enabled, is_initialized
```

两个电机必须已注册到电机注册表并具有有效反馈。

## 4. 核心配置参数

| 参数                       | 说明                                         | 单位 |
| :------------------------- | :------------------------------------------- | :--- |
| `drive_motor`              | 驱动电机基类指针                             | —    |
| `steering_motor`           | 舵向电机基类指针                             | —    |
| `wheel_radius_m`           | 轮子半径                                     | 米   |
| `drive_reduction_ratio`    | 驱动电机到轮子的减速比                       | —    |
| `steering_zero_offset_rad` | 舵向机械零位偏移（电机零位与物理零位的偏差） | 弧度 |
| `drive_direction_sign`     | 驱动电机方向符号（+1 或 -1）                 | —    |
| `steering_direction_sign`  | 舵向电机方向符号（+1 或 -1）                 | —    |

## 5. 目标转换公式

### 5.1 驱动电机

```
电机角速度 (rad/s) = (轮线速度 / 轮半径) × 减速比 × 方向符号
```

### 5.2 舵向电机

```
电机位置目标 (rad) = (优化后舵角 + 零位偏移) × 方向符号
```

### 5.3 舵向角度反馈

```
物理舵角 (rad) = (电机位置 × 方向符号) - 零位偏移
```

## 6. API 参考

| 函数                               | 说明                                           | 返回值                             |
| :--------------------------------- | :--------------------------------------------- | :--------------------------------- |
| `module_swerve_init`               | 初始化舵轮模块                                 | `OK` / `INVALID_ARGUMENT`          |
| `module_swerve_enable`             | 使能舵轮（按顺序使能驱动电机和舵向电机）       | `OK` / `MOTOR_ERROR`               |
| `module_swerve_disable`            | 禁用舵轮（禁用两个电机）                       | `OK` / `MOTOR_ERROR`               |
| `module_swerve_apply_target`       | 应用运动学目标（转换并设置电机目标，更新电机） | `OK` / `NOT_READY` / `MOTOR_ERROR` |
| `module_swerve_get_steering_angle` | 读取当前舵向角度                               | `OK` / `NOT_READY`                 |

## 7. 使用示例

### 7.1 初始化舵轮

```c
static module_swerve_t s_wheel_front_left;

const module_swerve_config_t cfg = {
    .drive_motor = &front_left_drive.super,
    .steering_motor = &front_left_steering.super,
    .wheel_radius_m = 0.05F,
    .drive_reduction_ratio = 19.0F,
    .steering_zero_offset_rad = 0.0F,
    .drive_direction_sign = 1.0F,
    .steering_direction_sign = 1.0F,
};

module_swerve_init(&s_wheel_front_left, &cfg);
module_swerve_enable(&s_wheel_front_left);
```

### 7.2 周期控制循环

```c
void swerve_control_loop(float dt) {
    // 1. 获取当前舵向角度
    float current_steering_rad;
    if (module_swerve_get_steering_angle(&s_wheel_front_left, &current_steering_rad) !=
        MODULE_SWERVE_STATUS_OK) {
        // 舵向电机离线，标记模块不可用
        return;
    }

    // 2. 计算原始目标（由运动学算法生成）
    alg_swerve_module_target_t target = {
        .wheel_velocity_m_per_s = 1.0F,
        .steering_angle_rad = 0.5F,
    };

    // 3. 应用目标（内部会调用 alg_swerve_optimize_target 优化）
    if (module_swerve_apply_target(&s_wheel_front_left, &target, dt) !=
        MODULE_SWERVE_STATUS_OK) {
        // 电机错误处理
    }
}
```

### 7.3 禁用舵轮

```c
// 停止舵轮（先停止控制，再禁用）
module_swerve_disable(&s_wheel_front_left);
```

## 8. 目标优化

`module_swerve_apply_target` 内部会调用 `alg_swerve_optimize_target` 对目标进行优化：

- 当舵向需要旋转超过 90° 时，选择旋转另一个方向并反转轮速
- 这可以减少舵向电机的运动行程，提高响应速度

```text
原始目标: 轮速 1.0 m/s, 舵角 180°
优化后:   轮速 -1.0 m/s, 舵角 0°  （反转轮速，舵角转到 0°）
```

## 9. 故障处理

| 错误码            | 触发场景                          | 处理建议               |
| :---------------- | :-------------------------------- | :--------------------- |
| `NOT_READY`       | 舵向电机反馈无效或离线            | 检查电机连接和反馈状态 |
| `MOTOR_ERROR`     | 电机使能/设置目标/更新失败        | 检查电机注册状态和通信 |
| `ALGORITHM_ERROR` | `alg_swerve_optimize_target` 失败 | 检查目标值是否有效     |

**故障时上层应将对应舵轮标记为不可用**，交由 `alg_swerve` 进行降级处理，而不是继续向故障舵轮发送单边输出。

## 10. 注意事项

- **电机依赖**：两个电机必须在调用 `enable` 前已注册、初始化且在线
- **不拥有电机**：本模块不拥有电机对象，电机生命周期由调用者管理
- **不计算整车运动学**：本模块只处理单个舵轮，整车运动学由上层 `alg_swerve` 负责
- **不做零位标定**：`steering_zero_offset_rad` 需由外部标定流程确定
- **故障不自动清除**：电机故障后需由上层调用 `module_motor_clear_fault` 恢复
- **单任务调用**：一个舵轮对象应由一个控制任务周期调用

## 11. 建议验证测试项

- [ ] 轮速单位换算正确（m/s → rad/s，考虑轮半径和减速比）
- [ ] 舵向角度换算正确（含零位偏移和方向符号）
- [ ] 驱动方向和舵向方向符号正确（正反向安装）
- [ ] 两电机启停失败时的回滚（一个电机使能失败，另一个也会被禁用）
- [ ] 舵向角度跨圈（编码器多圈累计正确）
- [ ] 驱动电机或舵向电机离线时返回 `NOT_READY`
- [ ] 优化后的反轮速目标生效（舵角优化时轮速反向）
- [ ] 四个独立舵轮实例同时工作

---

**总结**：`module_swerve` 提供了完整的舵轮执行控制，将运动学目标安全可靠地转换为电机控制指令。它通过 `module_motor` 基类与具体电机解耦，通过 `alg_swerve` 与运动学算法解耦，职责清晰。配合 `alg_swerve` 运动学库和底盘控制器，可构建完整的舵轮机器人运动控制系统。

## 一页式接入顺序与可读信息

```c
/* 1. 驱动和舵向电机必须先初始化、注册，并具有有效反馈。 */
static module_swerve_t wheel_module;

/* 2. 注入两个 module_motor_t、轮半径、减速比和舵向零偏。 */
module_swerve_status_t status = module_swerve_init(&wheel_module, &swerve_config);

/* 3. 电机在线后使能舵轮。 */
status = module_swerve_enable(&wheel_module);

/* 4. 先由 alg_swerve 计算 module target，再交给执行模块。 */
alg_swerve_module_target_t target;
/* alg_swerve_inverse(..., &target); */
status = module_swerve_apply_target(&wheel_module, &target, delta_time_s);

/* 5. 每个控制周期更新两个电机；停机调用 disable。 */
```

| 可读取信息 | 推荐读取方式 | 说明 |
| --- | --- | --- |
| 当前舵向角 | `module_swerve_get_steering_angle(me, &angle)` | 已扣除配置零偏后的机械角度 |
| 运动学目标 | `alg_swerve_module_target_t` | 目标轮速、舵角及优化后的方向 |
| 两个电机反馈 | `module_motor_get_feedback()` | 驱动速度、舵向位置、温度、在线状态 |
| `module_swerve_t` | 调试器只读查看 | 配置引用和最近执行状态，不应由 App 直接改写 |

调用顺序必须是“运动学解算 → `module_swerve_apply_target`”，不要让本模块承担底盘整体运动学。
