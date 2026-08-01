# BSP 硬件看门狗通用抽象层 (bsp_watchdog)

## 1. 模块概述

`bsp_watchdog` 提供了对硬件看门狗（Watchdog Timer）的通用抽象，支持刷新（喂狗）、实际超时时间查询和看门狗复位来源检测。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **刷新（Refresh）**：在允许的时间窗口内喂狗，防止系统复位。
- **超时时间查询（Get Timeout）**：获取平台实际超时时间（受低速时钟和分频影响，存在器差和温漂）。
- **复位来源检测（Get Reset Detected）**：检测上次复位是否由看门狗导致，用于故障诊断。

**适用场景**：

- 系统级故障恢复（检测任务死锁、死循环）。
- 安全关键应用（确保系统在异常时自动复位）。
- 生产测试（验证看门狗功能是否正常）。

**设计哲学**：

- **轻量抽象**：通用层不决定独立看门狗（IWDG）或窗口看门狗（WWDG），也不配置具体寄存器。
- **灵活扩展**：`get_timeout_ms` 和 `get_reset_detected` 为可选接口，适应不同平台能力。
- **启动不可逆**：某些 MCU 看门狗启动后无法停止，平台 `deinit` 可以返回 `UNSUPPORTED`。

## 2. 设计边界

| **模块负责**                             | **模块不负责**                               |
| :--------------------------------------- | :------------------------------------------- |
| 看门狗刷新、超时查询、复位检测的统一接口 | 看门狗类型选择（IWDG/WWDG）                  |
| 设备对象生命周期管理                     | 看门狗寄存器配置（时钟源、分频、窗口值）     |
| 多态接口和驱动解耦                       | 硬件复位标志清除（由平台驱动在 init 中处理） |
| 参数校验（空指针）                       | 系统健康管理策略（由上层实现）               |

**重要说明**：

- **启动不可逆**：某些 MCU 看门狗启动后无法软件停止，平台 `deinit` 可以返回 `BSP_STATUS_UNSUPPORTED`，不能伪造已关闭。
- **超时偏差**：超时时间通常由低速时钟和分频决定，存在器差和温漂。`get_timeout_ms` 应返回平台计算后的实际值。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_watchdog_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_watchdog_device_t  (派生类：持有 driver_ops)
```

- **`bsp_watchdog_t`**：应用层使用的基类指针，所有看门狗操作均通过此指针进行。
- **`bsp_watchdog_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_watchdog_ops_t` 继承自 `bsp_device_ops_t`，新增 `refresh`、`get_timeout_ms`、`get_reset_detected`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_watchdog_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_watchdog_driver_ops_t *driver_ops; // 底层驱动表
} bsp_watchdog_config_t;
```

### 4.2 底层驱动操作表 (`bsp_watchdog_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*refresh)(void *handle);
    bsp_status_t (*get_timeout_ms)(const void *handle, uint32_t *timeout_ms);
    bsp_status_t (*get_reset_detected)(const void *handle, bool *reset_detected);
} bsp_watchdog_driver_ops_t;
```

**必须实现的函数**：`refresh`。`init`/`deinit` 可选，`get_timeout_ms`/`get_reset_detected` 可选。

## 5. API 参考

| 函数                              | 说明                 | 返回值                              |
| :-------------------------------- | :------------------- | :---------------------------------- |
| `bsp_watchdog_init`               | 初始化看门狗设备     | `OK` / `INVALID_ARGUMENT`           |
| `bsp_watchdog_as_base`            | 向上转型             | 基类指针或 `NULL`                   |
| `bsp_watchdog_refresh`            | 刷新看门狗（喂狗）   | `OK` / `NOT_INITIALIZED` / 平台错误 |
| `bsp_watchdog_get_timeout_ms`     | 获取实际超时时间     | `OK` / `UNSUPPORTED`                |
| `bsp_watchdog_get_reset_detected` | 检测是否由看门狗复位 | `OK` / `UNSUPPORTED`                |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_iwdg_driver.c
static bsp_status_t stm32_iwdg_init(void *handle) {
    bsp_watchdog_context_t *ctx = (bsp_watchdog_context_t *)handle;
    // 检测是否由看门狗复位
    ctx->reset_detected = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    // 配置 IWDG
    IWDG1->KR = 0x5555U;   // 使能写访问
    IWDG1->PR = 4U;        // 预分频器（约 2 秒超时）
    IWDG1->RLR = 1000U;    // 重载值
    while (IWDG1->SR != 0U) {}
    IWDG1->KR = 0xCCCCU;   // 启动看门狗
    ctx->timeout_ms = 2000U;
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_iwdg_refresh(void *handle) {
    IWDG1->KR = 0xAAAAU;   // 重载计数器
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_iwdg_deinit(void *handle) {
    // IWDG 启动后无法停止，返回 UNSUPPORTED
    (void)handle;
    return BSP_STATUS_UNSUPPORTED;
}

const bsp_watchdog_driver_ops_t stm32_iwdg_driver = {
    .init = stm32_iwdg_init,
    .deinit = stm32_iwdg_deinit,
    .refresh = stm32_iwdg_refresh,
    .get_timeout_ms = stm32_iwdg_get_timeout,
    .get_reset_detected = stm32_iwdg_get_reset_detected,
};
```

### 6.2 应用层初始化

```c
static bsp_watchdog_device_t s_watchdog_dev;
static bsp_watchdog_t *s_watchdog_ptr = NULL;

void board_watchdog_init(void) {
    bsp_watchdog_config_t cfg = {
        .device_handle = &watchdog_ctx,
        .driver_ops = &stm32_iwdg_driver,
    };
    bsp_watchdog_init(&s_watchdog_dev, &cfg);
    s_watchdog_ptr = bsp_watchdog_as_base(&s_watchdog_dev);
}
```

### 6.3 复位原因检测

```c
void system_startup(void) {
    bool reset_by_watchdog;
    if (bsp_watchdog_get_reset_detected(s_watchdog_ptr, &reset_by_watchdog) == BSP_STATUS_OK) {
        if (reset_by_watchdog) {
            // 上次复位由看门狗导致，记录故障
            fault_logger_record(FAULT_WATCHDOG_RESET);
            // 可选择恢复关键状态
        }
    }
    // 继续正常启动流程
}
```

### 6.4 健康监督任务喂狗

```c
// 关键任务心跳计数器
volatile uint32_t control_heartbeat = 0;
volatile uint32_t comm_heartbeat = 0;
volatile uint32_t safety_heartbeat = 0;

// 健康监督任务（周期 5ms）
void health_supervisor_task(void *arg) {
    static uint32_t last_control = 0;
    static uint32_t last_comm = 0;
    static uint32_t last_safety = 0;

    while (1) {
        // 检查所有关键任务是否都在预期内更新
        if (control_heartbeat != last_control &&
            comm_heartbeat != last_comm &&
            safety_heartbeat != last_safety) {
            // 所有任务健康，喂狗
            bsp_watchdog_refresh(s_watchdog_ptr);
            last_control = control_heartbeat;
            last_comm = comm_heartbeat;
            last_safety = safety_heartbeat;
        }
        // 否则不喂狗，让看门狗复位系统
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

### 6.5 获取超时时间

```c
uint32_t timeout_ms;
if (bsp_watchdog_get_timeout_ms(s_watchdog_ptr, &timeout_ms) == BSP_STATUS_OK) {
    // 根据超时时间设定健康监督任务的报警窗口
    // 报警窗口 = timeout_ms * 0.7（留出 30% 裕量）
}
```

## 7. 正确喂狗策略

**关键原则**：不要在定时器 ISR 中无条件喂狗。否则主控制、通信或安全任务死锁后看门狗仍不会复位，失去保护意义。

**推荐架构**：

```
┌─────────────────────────────────────────────────────────────┐
│                    Health Supervisor Task                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Check Control Task Heartbeat                        │   │
│  │  Check Communication Task Heartbeat                  │   │
│  │  Check Safety Task Heartbeat                         │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│                          ▼                                  │
│            All tasks healthy? ──No──> Do NOT refresh       │
│                          │                                  │
│                         Yes                                 │
│                          ▼                                  │
│                   ┌──────────────┐                         │
│                   │   Refresh    │                         │
│                   │   Watchdog   │                         │
│                   └──────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

**喂狗时机**：

- 由健康监督任务定期执行（周期应小于超时时间的 1/2）。
- 只有所有关键任务都在预期窗口内更新时才喂狗。
- 任意关键任务超时或异常时停止喂狗，让系统复位。

## 8. 超时配置与裕量

- 超时时间由低速时钟（LSI/LSE）和分频器决定，存在 ±10%~±30% 的器差和温漂。
- 实际超时时间应通过 `get_timeout_ms` 获取，而非依赖配置值。
- **裕量建议**：健康监督任务的报警窗口应设置为 `timeout_ms * 0.7`，喂狗周期设置为 `timeout_ms * 0.3`。
- 比赛项目应留出调度抖动和最坏执行时间（WCET）裕量。

## 9. 故障安全

| 场景       | 处理方式                                          |
| :--------- | :------------------------------------------------ |
| 系统死锁   | 看门狗超时 → 硬件复位                             |
| 任务死循环 | 心跳停止 → 不喂狗 → 硬件复位                      |
| 复位后恢复 | 读取 `get_reset_detected` → 记录故障 → 安全初始化 |

**关键原则**：

- 复位前应尽可能由硬件保证电机使能和功率输出进入安全状态，**不能依赖软件来得及执行退出逻辑**。
- 复位后保持故障记录，并要求遥控和通信重新进入有效状态后再使能执行器。

## 10. 生命周期与并发

- **初始化顺序**：`bsp_watchdog_init` → 使用 → `bsp_device_deinit`（可选）。
- **启动不可逆**：某些 MCU 看门狗启动后无法停止，`deinit` 可能返回 `UNSUPPORTED`。
- **并发约束**：`refresh` 是硬件操作，通常不可重入。若多任务可能同时调用，需由上层提供互斥保护。
- **ISR 安全**：`refresh` 通常可以在中断中调用（平台驱动应确保安全），但健康监督任务才是正确的喂狗位置。

## 11. 错误码速查

| 错误码                        | 触发场景                                                                   |
| :---------------------------- | :------------------------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空                                                                   |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                                               |
| `BSP_STATUS_UNSUPPORTED`      | 调用可选函数但驱动未实现（`get_timeout_ms`/`get_reset_detected`/`deinit`） |
| `BSP_STATUS_IO_ERROR`         | 硬件错误                                                                   |
| `BSP_STATUS_TIMEOUT`          | 喂狗窗口错误（过早或过晚）                                                 |

## 12. 移植要求

平台移植者需实现 `bsp_watchdog_driver_ops_t`：

| 函数                 | 要求                                                   |
| :------------------- | :----------------------------------------------------- |
| `init`               | 配置看门狗（时钟源、分频、重载值），检测复位标志并清除 |
| `deinit`             | 若硬件支持停止则停止，否则返回 `UNSUPPORTED`           |
| `refresh`            | 执行喂狗操作（重载计数器）                             |
| `get_timeout_ms`     | 返回实际超时时间（考虑时钟偏差，返回计算值）           |
| `get_reset_detected` | 返回是否由看门狗复位（应在 init 时读取并缓存）         |

**关键注意事项**：

- **复位标志清除**：在 `init` 中读取复位标志后必须清除，否则下次启动仍会读到旧值。
- **超时偏差**：`get_timeout_ms` 应基于实际低速时钟频率计算（而非配置值）。
- **窗口支持**：若使用窗口看门狗，`refresh` 需要在窗口内调用，驱动应检查并返回错误。

## 13. 建议验证测试项

- [ ] 正常心跳持续刷新，系统不复位。
- [ ] 任一关键任务停跳后不再刷新，系统在超时后复位。
- [ ] 复位后 `get_reset_detected` 返回 `true`。
- [ ] `get_timeout_ms` 返回值与实际复位时间一致（误差在可接受范围内）。
- [ ] 窗口看门狗过早喂狗返回错误（若硬件支持）。
- [ ] 平台不支持反初始化时，`deinit` 返回 `UNSUPPORTED`。
- [ ] 低速时钟偏差下超时时间符合预期。
- [ ] 上电后执行器保持安全状态（硬件层面确保）。

---

## 一页式接入顺序与可读信息

1. 平台确定看门狗窗口/超时并实现 driver ops。
2. 启动早期调用 `bsp_watchdog_init()`，读取并保存复位原因。
3. 关键任务分别提交心跳，由一个健康监督任务统一判断。
4. 只有全部必要心跳正常时才调用 `bsp_watchdog_refresh()`。
5. 看门狗一旦启动通常无法停止；不要在任意循环或 ISR 中无条件喂狗。

可读取 `bsp_watchdog_get_timeout_ms()` 和 `bsp_watchdog_get_reset_detected()`。后者用于启动日志和故障追踪，不应在读取后自动清除业务层记录。
