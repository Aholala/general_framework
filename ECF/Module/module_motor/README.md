# module_motor — 通用电机基类

所有电机的公共基类（完整 OOP：`vptr` 虚表）。派生类（M2006/M3508/GM6020/DM4310）通过 `super` 第一成员继承。

## 关键结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `module_motor_t` | 电机基类 | `vptr`, `motor_name`, `state`(DISABLED/ENABLED/FAULT), `feedback`, `is_initialized` |
| `module_motor_feedback_t` | 统一反馈 | `position_rad`, `velocity_rad_per_s`, `torque_nm`, `current_a`, `motor_temperature_c`, `is_online`, `update_count` |
| `module_motor_ops_t` | 虚表（5 个纯虚函数） | `enable`, `disable`, `can_clear_fault`, `set_target`, `update` |
| `module_motor_registry_t` | 注册表 | `motor_storage[]`, `motor_count` — 统一管理所有电机 |
| `module_motor_pid_t` | 内嵌 PID 环 | `form`, `controller`(union: positional/incremental), `is_initialized` |

## 读取反馈

```c
const module_motor_feedback_t *fb = module_motor_get_feedback(motor);
if (fb != NULL && fb->is_online) {
    float pos  = fb->position_rad;          // 位置 (rad)
    float vel  = fb->velocity_rad_per_s;    // 速度 (rad/s)
    float temp = fb->motor_temperature_c;   // 温度 (°C)
    float cur  = fb->current_a;             // 电流 (A)，需检查 is_current_a_valid
}
```

## 注册表

```c
module_motor_t *storage[10];
module_motor_registry_t reg;
module_motor_registry_init(&reg, storage, 10);

module_motor_registry_register(&reg, &m3508);
module_motor_registry_register(&reg, &gm6020);

module_motor_t *m = module_motor_registry_find(&reg, KEY_PITCH);
size_t n = module_motor_registry_get_count(&reg);
for (size_t i = 0; i < n; i++) module_motor_update(storage[i], dt);
```

## 状态机

```
DISABLED → enable() → ENABLED
ENABLED  → disable() → DISABLED
ENABLED  → 反馈超时 → FAULT
FAULT    → clear_fault() → DISABLED（需重新 enable）
```

`update()` 在使能状态下自动检测 `elapsed_time > feedback_timeout_ms` → 自动 FAULT。

## API 速查

| 函数 | 功能 |
|------|------|
| `module_motor_init_base(me, vptr, name, key, id)` | 初始化基类 |
| `module_motor_enable(me)` | 使能 |
| `module_motor_disable(me)` | 禁用 |
| `module_motor_clear_fault(me)` | 清除故障（→DISABLED） |
| `module_motor_set_target(me, value)` | 设目标（含义取决于派生类） |
| `module_motor_update(me, dt_s)` | 周期更新 + 自动故障检测 |
| `module_motor_get_feedback(me)` | 获取反馈只读指针 |
| `module_motor_set_feedback_timeout(me, ms)` | 设超时 |
| `module_motor_notify_feedback(me)` | 通知收到反馈（重置计时） |
| `module_motor_registry_init/find/register/unregister/get_count` | 注册表操作 |
| `module_motor_pid_init/update/reset/get_terms` | 内嵌 PID 环 |
