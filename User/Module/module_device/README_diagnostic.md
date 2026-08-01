# 统一健康诊断模块 (module_diagnostic) —— 完整使用指南

## 1. 模块概述

`module_diagnostic` 是一个统一的健康诊断注册表。各 BSP、模块或控制器只需提供“当前是否健康”的只读探针；诊断模块负责故障确认、防抖恢复、严重度聚合、发生次数、锁存与事件通知。

**核心功能**：

- 注册多个诊断条目（每个条目一个探针）
- 探针返回布尔健康状态和详细错误码
- 故障确认防抖（`confirmation_time_ms`）
- 恢复确认防抖（`recovery_time_ms`）
- 严重等级聚合（INFO → WARNING → ERROR → FATAL）
- 故障发生次数统计
- 锁存机制（故障后保持，需手动清除）
- 状态变化事件回调
- 查询接口：获取状态、检查是否有指定严重等级的故障

**设计哲学**：

- **只读探针**：探针不得修改设备状态，仅返回健康与否。
- **防抖**：避免瞬时抖动导致的误报。
- **非阻塞**：所有操作在任务上下文执行，不阻塞控制任务。
- **静态内存**：条目和状态数组由调用者提供，无动态内存分配。

## 2. 设计边界

| **模块负责**       | **模块不负责**                     |
| :----------------- | :--------------------------------- |
| 故障确认、恢复防抖 | 具体设备状态的恢复操作             |
| 严重等级聚合和统计 | 控制策略（降级、停机、复位）       |
| 锁存和事件通知     | 探针函数的具体实现（由调用者提供） |
| 状态查询接口       | 硬件寄存器读取或设备初始化         |

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_diagnostic_t            (诊断对象：条目表、状态数组、聚合信息)
```

## 4. 核心类型

### 4.1 严重等级 (`module_diagnostic_severity_t`)

```c
typedef enum {
    MODULE_DIAGNOSTIC_SEVERITY_INFO = 0,    // 信息（正常）
    MODULE_DIAGNOSTIC_SEVERITY_WARNING,     // 警告
    MODULE_DIAGNOSTIC_SEVERITY_ERROR,       // 错误
    MODULE_DIAGNOSTIC_SEVERITY_FATAL        // 致命
} module_diagnostic_severity_t;
```

### 4.2 诊断条目 (`module_diagnostic_entry_t`)

```c
typedef struct {
    uint16_t diagnostic_id;                     // 唯一 ID
    module_diagnostic_severity_t severity;      // 严重等级
    module_diagnostic_probe_t probe;            // 探针函数
    void *user_context;                         // 探针上下文
    uint32_t confirmation_time_ms;              // 故障确认时间（防抖）
    uint32_t recovery_time_ms;                  // 恢复确认时间（防抖）
    bool is_latched;                            // 是否锁存
} module_diagnostic_entry_t;
```

### 4.3 诊断状态 (`module_diagnostic_state_t`)

```c
typedef struct {
    uint32_t detail_code;               // 详细错误码
    uint32_t fault_elapsed_time_ms;     // 故障累积时间
    uint32_t recovery_elapsed_time_ms;  // 恢复累积时间
    uint32_t occurrence_count;          // 发生次数
    bool is_active;                     // 是否活动故障
    bool is_latched;                    // 是否锁存
} module_diagnostic_state_t;
```

## 5. API 参考

| 函数                              | 说明                               | 返回值                                         |
| :-------------------------------- | :--------------------------------- | :--------------------------------------------- |
| `module_diagnostic_init`          | 初始化诊断模块                     | `OK` / `INVALID_ARGUMENT`                      |
| `module_diagnostic_clear_latched` | 清除锁存标志（仅故障恢复后）       | `OK` / `OPERATION_FAILED` / `INVALID_ARGUMENT` |
| `module_diagnostic_get_state`     | 获取指定诊断的状态                 | 状态指针 / `NULL`                              |
| `module_diagnostic_has_severity`  | 检查是否存在达到指定严重等级的故障 | `true` / `false`                               |
| `module_device_start`             | 启动诊断（开始更新）               | `OK` / `NOT_INITIALIZED`                       |
| `module_device_stop`              | 停止诊断（停止更新）               | `OK` / `NOT_INITIALIZED`                       |
| `module_device_update`            | 周期更新（调用探针并更新状态）     | `OK` / `OPERATION_FAILED`                      |

## 6. 使用示例

### 6.1 定义诊断条目

```c
// 传感器健康探针
static bool sensor_probe(void *ctx, uint32_t *detail) {
    sensor_t *s = (sensor_t *)ctx;
    if (s->is_communication_ok) {
        *detail = 0;
        return true;
    }
    *detail = s->error_code;
    return false;
}

// 电源电压探针
static bool voltage_probe(void *ctx, uint32_t *detail) {
    float voltage = read_voltage();
    if (voltage > 4.5F && voltage < 5.5F) {
        *detail = 0;
        return true;
    }
    *detail = (uint32_t)(voltage * 1000); // 毫伏
    return false;
}

static const module_diagnostic_entry_t entries[] = {
    {
        .diagnostic_id = 0x1001,
        .severity = MODULE_DIAGNOSTIC_SEVERITY_ERROR,
        .probe = sensor_probe,
        .user_context = &sensor_ctx,
        .confirmation_time_ms = 50,
        .recovery_time_ms = 50,
        .is_latched = false,
    },
    {
        .diagnostic_id = 0x1002,
        .severity = MODULE_DIAGNOSTIC_SEVERITY_WARNING,
        .probe = voltage_probe,
        .user_context = NULL,
        .confirmation_time_ms = 100,
        .recovery_time_ms = 200,
        .is_latched = true,
    },
};

static module_diagnostic_state_t states[ARRAY_SIZE(entries)];
```

### 6.2 初始化诊断模块

```c
module_diagnostic_t diag;

module_diagnostic_config_t cfg = {
    .entries = entries,
    .state_storage = states,
    .entry_count = ARRAY_SIZE(entries),
    .event_callback = my_diagnostic_event_callback,  // 可选
    .event_user_context = NULL,
    .logical_name = "health_diag",
    .registration_key = 0,
};

module_diagnostic_init(&diag, &cfg);
module_device_start(&diag.super);
```

### 6.3 周期更新

```c
void main_loop(void) {
    uint32_t dt_ms = get_delta_time_ms();
    module_device_update(&diag.super, dt_ms);

    // 检查是否有致命故障
    if (module_diagnostic_has_severity(&diag, MODULE_DIAGNOSTIC_SEVERITY_FATAL)) {
        emergency_shutdown();
    }
}
```

### 6.4 事件回调

```c
void my_diagnostic_event_callback(const module_diagnostic_entry_t *entry,
                                  const module_diagnostic_state_t *state,
                                  bool became_active, void *ctx) {
    if (became_active) {
        printf("Diagnostic %04X became active (severity=%d, code=%lu)\n",
               entry->diagnostic_id, entry->severity, state->detail_code);
    } else {
        printf("Diagnostic %04X recovered\n", entry->diagnostic_id);
    }
}
```

### 6.5 清除锁存

```c
// 在故障已恢复后，清除锁存标志
if (module_diagnostic_get_state(&diag, 0x1002)->is_active == false) {
    module_diagnostic_clear_latched(&diag, 0x1002);
}
```

## 7. 防抖机制

- **故障确认**：探针返回 `false` 后，累积 `fault_elapsed_time_ms`，达到 `confirmation_time_ms` 才判定为活动故障。
- **恢复确认**：探针返回 `true` 后，累积 `recovery_elapsed_time_ms`，达到 `recovery_time_ms` 才清除活动故障。
- **锁存**：若 `is_latched = true`，故障清除后 `is_active` 恢复为 `false`，但 `is_latched` 仍为 `true`，需手动调用 `clear_latched`。

## 8. 状态与统计

- `is_active`：当前是否处于活动故障状态（影响 `active_count` 和 `highest_active_severity`）
- `is_latched`：是否被锁存（独立于 `is_active`，用于提示操作员）
- `occurrence_count`：从健康→故障的转换次数（每次计数一次）
- `fault_elapsed_time_ms` / `recovery_elapsed_time_ms`：用于防抖的内部计时

## 9. 错误码速查

| 状态码             | 触发场景                                                     |
| :----------------- | :----------------------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、条目数组为空、probe 为 NULL、严重等级非法、ID 重复 |
| `NOT_INITIALIZED`  | 对象未初始化                                                 |
| `OPERATION_FAILED` | 清除锁存时故障仍活动，或未启动时调用 update                  |

## 10. 集成约束

- 探针函数必须快速返回，不得阻塞或修改设备状态。
- 诊断条目和状态数组由调用者静态分配，对象仅保存指针。
- 事件回调在 `update` 中调用（任务上下文），仍应保持有界执行时间。
- 建议更新周期为 10~100ms，确保防抖时间精度。

## 11. 建议验证测试项

- [ ] 探针返回 false 后，经过 `confirmation_time_ms` 才变为活动故障。
- [ ] 探针返回 true 后，经过 `recovery_time_ms` 才恢复。
- [ ] 瞬时抖动（小于防抖时间）不会触发误报。
- [ ] 锁存条目在故障恢复后 `is_latched` 仍为 true。
- [ ] `clear_latched` 在活动故障时返回错误。
- [ ] 事件回调在状态变化时正确触发。
- [ ] `has_severity` 正确反映最高等级。
- [ ] 多次发生计数准确。

---

## 一页式接入顺序与可读信息

```c
/* 1. 为每个诊断项定义只读 entry，并准备等长的 state 存储。 */
static const module_diagnostic_entry_t entries[] = { /* probe、等级、防抖时间 */ };
static module_diagnostic_state_t states[DIAGNOSTIC_COUNT];
static module_diagnostic_t diagnostics;

/* 2. 配置条目、状态数组和可选事件回调。 */
module_device_status_t status = module_diagnostic_init(&diagnostics, &diagnostic_config);

/* 3. 通过统一设备接口启动。 */
status = module_device_start(&diagnostics.super);

/* 4. 在任务中周期更新；探针和事件回调都在这里执行。 */
status = module_device_update(&diagnostics.super, elapsed_time_ms);

/* 5. 读取状态并由 App 决定降级、停机或提示。 */
const module_diagnostic_state_t *state =
    module_diagnostic_get_state(&diagnostics, diagnostic_id);

/* 6. 锁存故障恢复后，才允许 clear_latched。 */
```

| 可读取结构体/信息           | 读取方式                           | 重点字段                                                   |
| --------------------------- | ---------------------------------- | ---------------------------------------------------------- |
| `module_diagnostic_state_t` | `module_diagnostic_get_state()`    | `detail_code`、故障/恢复累计时间、发生次数、活动和锁存标志 |
| 最高故障等级                | `module_diagnostic_has_severity()` | 判断是否存在达到指定等级的活动故障                         |
| `module_diagnostic_t`       | 调试器只读查看                     | `highest_active_severity`、`active_count`、`is_started`    |
| `module_diagnostic_entry_t` | 调用者持有                         | ID、等级、探针、防抖和锁存策略                             |

getter 返回的状态指针属于诊断对象，下一次 `module_device_update()` 后内容可能变化。
