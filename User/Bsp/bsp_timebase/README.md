# BSP 时间基准通用抽象层 (bsp_timebase)

## 1. 模块概述

`bsp_timebase` 提供了基于**自由运行周期计数器**的单调时间基准抽象，可由 Cortex DWT（Data Watchpoint and Trace）、通用 32 位定时器或其他硬件实现。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **时间点快照**：`bsp_timebase_now()` 获取当前时间点（周期计数值快照）。
- **回绕安全的耗时计算**：`bsp_timebase_elapsed_cycles()` 计算两次时间点之间的周期差，正确处理 32 位回绕。
- **周期与微秒转换**：`cycles_to_us` / `us_to_cycles` 在高精度和高分辨率之间转换。
- **微秒级忙等待**：`bsp_timebase_delay_us()` 同步延时，适用于极短硬件时序。
- **非阻塞超时判断**：`bsp_timebase_has_elapsed_us()` 用于驱动轮询超时。

- **单调递增**：计数器必须单调递增并按 `uint32_t` 自然回绕。
- **频率稳定**：频率在对象使用期间应保持稳定（动态变频需由平台端同步更新）。
- **回绕安全**：无符号减法 `current - start` 正确处理一次 32 位回绕（例如 0xFFFFFFFF → 0x00000000）。
- **高精度**：基于 CPU 周期计数，精度可达纳秒级（取决于 CPU 频率）。

## 2. 设计边界

| **模块负责**           | **模块不负责**                            |
| :--------------------- | :---------------------------------------- |
| 单调时间基准的统一抽象 | 日历时间（年月日时分秒）—— 使用 `bsp_rtc` |
| 时间点快照与耗时计算   | RTOS 系统 tick 和任务调度延时             |
| 周期与微秒的换算       | 低功耗模式下的计数器保持                  |
| 微秒级忙等待延时       | 长周期（> 秒级）时间测量                  |
| 非阻塞超时判断         | 多时间基准同步                            |
| 32 位回绕的安全处理    | 64 位时间戳扩展（需由上层自行累加）       |

**适用场景**：

- 驱动超时检测（I2C/SPI 等待、外设就绪轮询）。
- 性能测量（函数执行时间、中断响应延迟）。
- 微秒级硬件时序（传感器复位、脉冲生成）。
- 高精度周期任务调度（如 1kHz 控制循环的精确计时）。

**不适用场景**：

- 需要跨天/跨年的时间戳 → 使用 `bsp_rtc`。
- 需要 RTOS 任务阻塞延时 → 使用 `vTaskDelay` 或 `osDelay`。
- 需要 64 位时间戳且测量时间超过 32 位计数器溢出周期 → 需上层扩展 64 位。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_timebase_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_timebase_device_t  (派生类：持有 driver_ops)
```

- **`bsp_timebase_t`**：应用层使用的基类指针，所有时间基准操作均通过此指针进行。
- **`bsp_timebase_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_timebase_ops_t` 继承自 `bsp_device_ops_t`，新增 `reset`、`get_cycle_count`、`get_frequency`。

## 4. 核心类型

### 4.1 时间点结构体 (`bsp_timebase_time_point_t`)

```c
typedef struct {
    uint32_t cycle_count;          // 周期计数值（快照）
} bsp_timebase_time_point_t;
```

### 4.2 配置结构 (`bsp_timebase_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_timebase_driver_ops_t *driver_ops; // 底层驱动表
} bsp_timebase_config_t;
```

### 4.3 底层驱动操作表 (`bsp_timebase_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *device_handle);
    bsp_status_t (*deinit)(void *device_handle);
    bsp_status_t (*reset)(void *device_handle);
    bsp_status_t (*get_cycle_count)(const void *device_handle, uint32_t *cycle_count);
    bsp_status_t (*get_frequency)(const void *device_handle, uint32_t *frequency_hz);
} bsp_timebase_driver_ops_t;
```

**必须实现的函数**：`get_cycle_count`、`get_frequency`。`init`/`deinit`/`reset` 为可选。

### 4.4 计数器要求

- **单调递增**：每次读取的计数值必须 ≥ 上次读取的值（回绕除外）。
- **32 位自然回绕**：从 `0xFFFFFFFF` 回绕到 `0x00000000`。
- **频率稳定**：在对象生命周期内频率应保持不变（若变频，需驱动层处理）。

## 5. API 参考

| 函数                           | 说明                        | 返回值                        |
| :----------------------------- | :-------------------------- | :---------------------------- |
| `bsp_timebase_init`            | 初始化时间基准              | `OK` / `INVALID_ARGUMENT`     |
| `bsp_timebase_as_base`         | 向上转型                    | 基类指针或 `NULL`             |
| `bsp_timebase_reset`           | 复位周期计数器（可选）      | `OK` / `UNSUPPORTED`          |
| `bsp_timebase_get_cycle_count` | 获取当前周期计数值          | `OK` / `INVALID_ARGUMENT`     |
| `bsp_timebase_get_frequency`   | 获取时间基准频率            | `OK` / `IO_ERROR`（频率为 0） |
| `bsp_timebase_now`             | 获取当前时间点              | `OK` / `INVALID_ARGUMENT`     |
| `bsp_timebase_elapsed_cycles`  | 计算经过的周期数            | `OK` / `INVALID_ARGUMENT`     |
| `bsp_timebase_cycles_to_us`    | 周期 → 微秒换算             | `OK` / `OUT_OF_RANGE`         |
| `bsp_timebase_us_to_cycles`    | 微秒 → 周期换算（向上取整） | `OK` / `OUT_OF_RANGE`         |
| `bsp_timebase_delay_us`        | 微秒级忙等待                | `OK` / `OUT_OF_RANGE`         |
| `bsp_timebase_has_elapsed_us`  | 非阻塞超时判断              | `OK` / `OUT_OF_RANGE`         |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_dwt_timebase_driver.c
static bsp_status_t stm32_dwt_init(void *handle) {
    // 使能 DWT 跟踪
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? BSP_STATUS_OK : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t stm32_dwt_get_cycle_count(const void *handle, uint32_t *cycle_count) {
    if (cycle_count == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    *cycle_count = DWT->CYCCNT;
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_dwt_get_frequency(const void *handle, uint32_t *frequency_hz) {
    if (frequency_hz == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    *frequency_hz = SystemCoreClock;
    return BSP_STATUS_OK;
}

const bsp_timebase_driver_ops_t stm32_dwt_driver = {
    .init = stm32_dwt_init,
    .deinit = NULL,
    .reset = NULL,
    .get_cycle_count = stm32_dwt_get_cycle_count,
    .get_frequency = stm32_dwt_get_frequency,
};
```

### 6.2 应用层初始化

```c
static bsp_timebase_device_t s_timebase_dev;
static bsp_timebase_t *s_timebase_ptr = NULL;

void board_timebase_init(void) {
    bsp_timebase_config_t cfg = {
        .device_handle = NULL,   // DWT 不需要设备句柄
        .driver_ops = &stm32_dwt_driver,
    };
    bsp_timebase_init(&s_timebase_dev, &cfg);
    s_timebase_ptr = bsp_timebase_as_base(&s_timebase_dev);
}
```

### 6.3 时间点与耗时计算

```c
// 测量某段代码执行时间
bsp_timebase_time_point_t start, end;
bsp_timebase_now(s_timebase_ptr, &start);
// ... 执行被测代码 ...
bsp_timebase_now(s_timebase_ptr, &end);
uint32_t elapsed_cycles;
bsp_timebase_elapsed_cycles(s_timebase_ptr, start, &elapsed_cycles);
uint32_t elapsed_us;
bsp_timebase_cycles_to_us(s_timebase_ptr, elapsed_cycles, &elapsed_us);
```

### 6.4 微秒级忙等待

```c
// 传感器复位需要 100us 等待
bsp_timebase_delay_us(s_timebase_ptr, 100);
```

### 6.5 非阻塞超时判断

```c
// 轮询外设状态，超时 1ms
bsp_timebase_time_point_t start;
bsp_timebase_now(s_timebase_ptr, &start);
bool timeout = false;
do {
    // 读取外设状态寄存器
    if (peripheral_is_ready()) {
        break;
    }
    bsp_timebase_has_elapsed_us(s_timebase_ptr, start, 1000, &timeout);
} while (!timeout);
```

## 7. 关键机制详解

### 7.1 32 位回绕安全

`bsp_timebase_elapsed_cycles` 使用无符号减法：

```c
*elapsed_cycles = current_cycle_count - start_time.cycle_count;
```

由于 `uint32_t` 的算术是模 `2^32` 的，即使计数器回绕，结果也是正确的。例如：

- `start = 0xFFFFFFF0`，`current = 0x00000010`（回绕后）
- `elapsed = 0x00000010 - 0xFFFFFFF0 = 0x00000020`（正确）

**限制**：仅能正确处理**一次**回绕。若测量时间超过完整计数周期，将无法区分多次回绕。

### 7.2 微秒换算的精度

- `cycles_to_us`：`(cycles * 1,000,000) / frequency`，精度取决于频率。
- `us_to_cycles`：向上取整 `(us * frequency + 999,999) / 1,000,000`，确保延时至少达到要求。

### 7.3 忙等待的安全限制

`bsp_timebase_delay_us` 设置了最大安全延时周期数 `UINT32_MAX / 2`，防止循环溢出。超过此值的延时应使用 RTOS 延时或其他机制。

## 8. 生命周期与并发

- **初始化顺序**：`bsp_timebase_init` 后即可使用。
- **复位**：`reset` 会破坏所有已有时间点的参考，建议仅在系统启动阶段调用一次。
- **并发约束**：读取操作（`get_cycle_count`）可在多个上下文（中断/任务）中安全调用，但 `reset` 不应在运行期调用。
- **ISR 安全**：`get_cycle_count`、`elapsed_cycles`、`has_elapsed_us` 可在 ISR 中调用（非阻塞）。`delay_us` 不应在 ISR 中调用。

## 9. 错误码速查

| 错误码                        | 触发场景                            |
| :---------------------------- | :---------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、输出指针为空              |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                        |
| `BSP_STATUS_IO_ERROR`         | 频率为 0（硬件异常）                |
| `BSP_STATUS_OUT_OF_RANGE`     | 周期/微秒换算结果超出 uint32_t 范围 |
| `BSP_STATUS_UNSUPPORTED`      | 调用 `reset` 但驱动未实现           |

## 10. 移植要求（DWT / 定时器）

平台移植者需实现 `bsp_timebase_driver_ops_t`：

| 函数              | 要求                                                             |
| :---------------- | :--------------------------------------------------------------- |
| `init`            | 使能周期计数器，处理硬件初始化（DWT 需使能 TRCENA 和 CYCCNTENA） |
| `deinit`          | 释放资源（可选）                                                 |
| `reset`           | 将计数器复位为 0（可选）                                         |
| `get_cycle_count` | 读取当前 32 位周期计数值                                         |
| `get_frequency`   | 返回计数器的时钟频率（Hz），必须 > 0                             |

**DWT 特别注意**：

1. 检查内核是否支持周期计数器（通过 `CoreDebug` 或 `DWT->CTRL`）。
2. 使能调试跟踪（`CoreDebug_DEMCR_TRCENA_Msk`）和周期计数器（`DWT_CTRL_CYCCNTENA_Msk`）。
3. 频率为 `SystemCoreClock`（CPU 实际频率）。
4. 调试器暂停或低功耗模式可能影响 DWT，需在文档中说明。

**定时器实现特别注意**：

1. 选择 32 位自由运行定时器（如 STM32 的 TIM2/TIM5）。
2. 配置为向上计数，自动重载值为 `0xFFFFFFFF`。
3. 频率为定时器输入时钟（通常为 APB 时钟）。

## 11. 建议验证测试项

- [ ] 连续读取周期计数，验证单调递增。
- [ ] 32 位回绕测试（从 0xFFFFFF00 到 0x00000010，`elapsed_cycles` 正确）。
- [ ] `cycles_to_us` 和 `us_to_cycles` 相互转换误差在 1us 以内。
- [ ] 大数值换算（接近 UINT32_MAX）返回 `OUT_OF_RANGE`。
- [ ] `delay_us(1)`、`delay_us(1000)`、`delay_us(10000)` 实际延时误差在可接受范围。
- [ ] `has_elapsed_us` 边界判断准确（恰好等于 duration 时返回 true）。
- [ ] 非阻塞超时判断在循环中正常工作。
- [ ] 对象未初始化时所有 API 返回 `NOT_INITIALIZED`。
- [ ] 多次 `init` / `deinit` 无状态残留。

---

**总结**：`bsp_timebase` 提供了高精度、低开销的时间基准抽象，是驱动超时、性能测量和短延时场景的理想选择。其回绕安全的耗时计算和灵活的微秒换算接口，使得上层代码能够以统一的方式处理时间相关逻辑。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
