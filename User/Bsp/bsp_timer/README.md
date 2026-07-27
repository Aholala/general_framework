# BSP 通用定时器抽象层 (bsp_timer)

## 1. 模块概述

`bsp_timer` 提供了对基本硬件定时器的通用抽象，支持启动/停止、计数器读写、周期设置、时钟频率查询和周期到期通知。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 启动/停止定时器。
- 设置/获取计数器值。
- 设置/获取周期值（自动重载值）。
- 获取定时器时钟频率。
- 注册到期回调并通知（`bsp_timer_notify_elapsed`）。

**适用场景**：

- 固定周期任务调度（控制循环、传感器轮询）。
- 通用硬件定时器计数。
- 软件定时器模拟的基础。

## 2. 设计边界

| **模块负责**                     | **模块不负责**                                       |
| :------------------------------- | :--------------------------------------------------- |
| 定时器启停、计数、周期的统一接口 | 预分频器、时钟树配置（由平台端配置）                 |
| 到期事件回调通知                 | 自动重装模式、单次/连续模式（由平台端配置）          |
| 计数器与周期值的读写             | IRQ 优先级配置                                       |
| 频率查询                         | 输出比较、输入捕获（使用 `bsp_pwm` / `bsp_encoder`） |
| 多态对象管理                     | 微秒级延时（使用 `bsp_timebase`）                    |

**与其他模块的区别**：
| 模块 | 用途 |
| :--- | :--- |
| `bsp_pwm` | PWM 输出（占空比、频率） |
| `bsp_encoder` | 正交编码器计数 |
| `bsp_timebase` | 高精度时间测量和短延时 |
| `bsp_timer` | 固定周期调度和通用计数 |

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_timer_t             (基类：增加 callback、user_context)
    └── bsp_timer_device_t  (派生类：持有 driver_ops)
```

- **`bsp_timer_t`**：应用层使用的基类指针，包含事件回调和用户上下文。
- **`bsp_timer_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_timer_ops_t` 继承自 `bsp_device_ops_t`，新增 `start`、`stop`、`set_counter`、`get_counter`、`set_period`、`get_period`、`get_frequency`。

## 4. 核心类型

### 4.1 回调函数类型 (`bsp_timer_callback_t`)

```c
typedef void (*bsp_timer_callback_t)(bsp_timer_t *const me, void *user_context);
```

- 回调在 ISR 上下文中执行，必须快速返回，不可阻塞。

### 4.2 配置结构 (`bsp_timer_config_t`)

```c
typedef struct {
    void *device_handle;
    const bsp_timer_driver_ops_t *driver_ops;
    bsp_timer_callback_t callback;
    void *user_context;
} bsp_timer_config_t;
```

### 4.3 底层驱动操作表 (`bsp_timer_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*start)(void *handle);
    bsp_status_t (*stop)(void *handle);
    bsp_status_t (*set_counter)(void *handle, uint32_t counter_ticks);
    bsp_status_t (*get_counter)(const void *handle, uint32_t *counter_ticks);
    bsp_status_t (*set_period)(void *handle, uint32_t period_ticks);
    bsp_status_t (*get_period)(const void *handle, uint32_t *period_ticks);
    bsp_status_t (*get_frequency)(const void *handle, uint32_t *frequency_hz);
} bsp_timer_driver_ops_t;
```

**所有函数均为必须实现**（由 `bsp_timer_init` 校验），`init`/`deinit` 可选。

## 5. API 参考

| 函数                       | 说明                    | 返回值                           |
| :------------------------- | :---------------------- | :------------------------------- |
| `bsp_timer_init`           | 初始化定时器设备        | `OK` / `INVALID_ARGUMENT`        |
| `bsp_timer_as_base`        | 向上转型                | 基类指针或 `NULL`                |
| `bsp_timer_set_callback`   | 设置到期回调            | `OK` / `NOT_INITIALIZED`         |
| `bsp_timer_start`          | 启动定时器              | `OK` / `NOT_INITIALIZED`         |
| `bsp_timer_stop`           | 停止定时器              | `OK` / `NOT_INITIALIZED`         |
| `bsp_timer_reset`          | 复位计数器为 0          | `OK` / `NOT_INITIALIZED`         |
| `bsp_timer_set_counter`    | 设置计数器值            | `OK` / `NOT_INITIALIZED`         |
| `bsp_timer_get_counter`    | 获取计数器值            | `OK` / `INVALID_ARGUMENT`        |
| `bsp_timer_set_period`     | 设置周期（tick）        | `OK` / `OUT_OF_RANGE`（周期为0） |
| `bsp_timer_get_period`     | 获取周期值              | `OK` / `INVALID_ARGUMENT`        |
| `bsp_timer_get_frequency`  | 获取时钟频率            | `OK` / `INVALID_ARGUMENT`        |
| `bsp_timer_notify_elapsed` | 到期通知（由 ISR 调用） | 无返回值                         |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_timer_driver.c
static bsp_status_t stm32_timer_start(void *handle) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_Base_Start_IT(htim);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_stop(void *handle) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_Base_Stop_IT(htim);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_set_counter(void *handle, uint32_t counter) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    __HAL_TIM_SET_COUNTER(htim, counter);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_get_counter(const void *handle, uint32_t *counter) {
    const TIM_HandleTypeDef *htim = (const TIM_HandleTypeDef *)handle;
    *counter = __HAL_TIM_GET_COUNTER(htim);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_set_period(void *handle, uint32_t period) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    __HAL_TIM_SET_AUTORELOAD(htim, period - 1);  // 硬件 ARR = period - 1
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_get_period(const void *handle, uint32_t *period) {
    const TIM_HandleTypeDef *htim = (const TIM_HandleTypeDef *)handle;
    *period = __HAL_TIM_GET_AUTORELOAD(htim) + 1; // 硬件 ARR + 1
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_timer_get_frequency(const void *handle, uint32_t *freq) {
    // 示例：假设 APB1 时钟为 100MHz
    *freq = 100000000;
    return BSP_STATUS_OK;
}

const bsp_timer_driver_ops_t stm32_timer_driver = {
    .init = NULL,
    .deinit = NULL,
    .start = stm32_timer_start,
    .stop = stm32_timer_stop,
    .set_counter = stm32_timer_set_counter,
    .get_counter = stm32_timer_get_counter,
    .set_period = stm32_timer_set_period,
    .get_period = stm32_timer_get_period,
    .get_frequency = stm32_timer_get_frequency,
};
```

### 6.2 应用层初始化

```c
static bsp_timer_device_t s_control_timer;
static bsp_timer_t *s_timer_ptr = NULL;

void board_timer_init(void) {
    bsp_timer_config_t cfg = {
        .device_handle = &htim3,
        .driver_ops = &stm32_timer_driver,
        .callback = control_tick_callback,
        .user_context = &control_context,
    };
    bsp_timer_init(&s_control_timer, &cfg);
    s_timer_ptr = bsp_timer_as_base(&s_control_timer);
}
```

### 6.3 启动周期定时器

```c
// 设置周期为 1000 个 tick（假设频率 100kHz，则周期为 10ms）
bsp_timer_set_period(s_timer_ptr, 1000);
bsp_timer_start(s_timer_ptr);
```

### 6.4 到期回调处理

```c
void control_tick_callback(bsp_timer_t *const me, void *ctx) {
    control_context_t *ctrl = (control_context_t *)ctx;
    // 仅做轻量操作：设置标志、累加计数、释放信号量
    ctrl->tick_count++;
    xSemaphoreGiveFromISR(ctrl->control_sem, NULL);
}
```

### 6.5 动态修改周期和计数器

```c
// 修改周期为 2000 tick
bsp_timer_set_period(s_timer_ptr, 2000);
// 重置计数器为 0
bsp_timer_reset(s_timer_ptr);
```

### 6.6 HAL 中断回调路由

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) {
        bsp_timer_notify_elapsed(s_timer_ptr);
    }
}
```

## 7. 时间换算说明

定时周期由 `period_ticks` 和 `frequency_hz` 共同决定：

```
周期时间 (秒) = period_ticks / frequency_hz
周期时间 (微秒) = period_ticks * 1,000,000 / frequency_hz
```

**重要**：是否包含自动重装寄存器的 `+1` 语义属于平台端实现。通用接口中的 `period_ticks` 必须表示逻辑周期（例如 `period_ticks = 1000` 表示 1000 个时钟周期），平台驱动负责转换为硬件 ARR 值（如 `ARR = period_ticks - 1`）。获取周期时，驱动应将硬件 ARR 值还原为逻辑周期。

## 8. 中断规则

- 到期回调运行在 ISR 上下文中。
- 回调中**只允许**：
  - 设置任务标志。
  - 增加饱和计数。
  - 释放 RTOS 同步量（信号量、消息队列）。
- **禁止**：
  - 阻塞（`vTaskDelay`、`printf`）。
  - 执行长时间循环。
  - 修改定时器配置（除非明确安全）。

## 9. 生命周期与并发

- **初始化顺序**：`bsp_timer_init` → `bsp_timer_set_period` → `bsp_timer_start`。
- **停止顺序**：先 `bsp_timer_stop`，再 `bsp_device_deinit`。
- **回调替换**：如果中断可能并发发生，更换回调前应禁用中断或使用临界区。
- **并发约束**：同一定时器实例的操作（如 `set_period` 与 `start`）应由单一任务控制，或由上层提供互斥保护。

## 10. 错误码速查

| 错误码                        | 触发场景                |
| :---------------------------- | :---------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、输出指针为空  |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化            |
| `BSP_STATUS_OUT_OF_RANGE`     | 周期设置为 0            |
| `BSP_STATUS_IO_ERROR`         | 硬件错误（频率为 0 等） |

## 11. 移植要求

平台移植者需实现 `bsp_timer_driver_ops_t` 的所有函数：

| 函数            | 说明                                             |
| :-------------- | :----------------------------------------------- |
| `start`         | 使能定时器并启动中断（或计数）                   |
| `stop`          | 禁用定时器和中断                                 |
| `set_counter`   | 写入计数器寄存器                                 |
| `get_counter`   | 读取计数器寄存器                                 |
| `set_period`    | 写入自动重载寄存器（注意逻辑周期与硬件值的转换） |
| `get_period`    | 读取自动重载寄存器并还原为逻辑周期               |
| `get_frequency` | 返回定时器输入时钟频率（Hz）                     |

**关键注意事项**：

- **周期换算**：若硬件需要 `ARR = period_ticks - 1`，则 `set_period` 做减法，`get_period` 做加法。
- **预分频与时钟**：预分频器和时钟树由硬件配置（CubeMX 或 `init` 中）决定，本层不管理。
- **中断清除**：在调用 `bsp_timer_notify_elapsed` 前，平台驱动必须清除定时器中断标志。

## 12. 建议验证测试项

- [ ] 启动/停止功能正常。
- [ ] 计数器读写准确。
- [ ] 周期设置和读取一致。
- [ ] 频率返回正确。
- [ ] 到期回调在预期时间触发。
- [ ] 多次到期通知计数准确。
- [ ] 回调中释放信号量等轻量操作正常。
- [ ] 周期为 0 时返回 `OUT_OF_RANGE`。
- [ ] 两个定时器实例独立工作。
- [ ] 停止后回调不再触发。
- [ ] 反初始化后对象拒绝访问。

---

**总结**：`bsp_timer` 提供了简洁、可移植的基本定时器抽象，适用于固定周期调度和通用计数场景。其完整的启停、计数、周期控制接口配合 ISR 回调机制，能够很好地支持实时控制任务。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
