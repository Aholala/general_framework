# BSP USART/UART 通用抽象层 (bsp_usart)

## 1. 模块概述

`bsp_usart` 提供了对 USART/UART 串行通信接口的通用抽象，支持固定长度接收、空闲线接收（不定长协议）、阻塞/中断/DMA 传输、中止和忙状态查询。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **发送（Transmit）**：支持阻塞/中断/DMA 三种模式发送数据。
- **固定长度接收（Receive）**：接收指定长度的数据。
- **空闲线接收（Receive to Idle）**：适用于 DR16、裁判系统等不定长协议，收到空闲线或缓冲区满时返回实际长度。
- **中止（Abort）**：终止当前异步事务（中断/DMA 模式）。
- **忙状态查询（Get Busy）**：查询当前是否有传输正在进行。

**适用场景**：

- 调试日志输出（printf 重定向）。
- DR16 遥控器数据接收（不定长协议）。
- 裁判系统数据交互（不定长协议）。
- 传感器数据采集（固定长度协议）。
- 板间通信（自定义协议）。

## 2. 设计边界

| **模块负责**                             | **模块不负责**                                   |
| :--------------------------------------- | :----------------------------------------------- |
| 发送/接收的统一接口（固定长度 + 空闲线） | 波特率、字长、校验位、停止位配置（由平台端配置） |
| 三种传输模式（阻塞/中断/DMA）的抽象      | 引脚复用、反相、硬件流控配置                     |
| 事务中止和忙状态查询                     | DMA 通道配置和缓存一致性维护（由平台驱动负责）   |
| 事件回调通知（完成/错误）                | 协议解析（由 Module 层处理）                     |
| 设备对象生命周期管理                     | 环形缓冲区、帧队列管理（由 Module 层实现）       |

**重要说明**：

- **空闲线接收**：最适合 DR16、裁判系统和不定长协议。调用者提供缓冲区容量，平台在收到空闲线或缓冲区满时通过 `transferred_size` 报告实际长度。
- **异步所有权**：异步发送与接收不会由通用 BSP 自动复制缓冲区。缓冲区在完成、中止或错误事件前必须保持有效且不能被覆盖。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_usart_t             (基类：增加 callback、user_context)
    └── bsp_usart_device_t  (派生类：持有 driver_ops)
```

- **`bsp_usart_t`**：应用层使用的基类指针，包含事件回调和用户上下文。
- **`bsp_usart_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_usart_ops_t` 继承自 `bsp_device_ops_t`，新增 `transmit`、`receive`、`receive_to_idle`、`abort`、`get_busy`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_usart_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_usart_driver_ops_t *driver_ops;   // 底层驱动表
    bsp_event_callback_t callback;            // 事件回调（可为 NULL）
    void *user_context;                       // 回调用户上下文
} bsp_usart_config_t;
```

### 4.2 底层驱动操作表 (`bsp_usart_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*transmit)(void *handle, const uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t (*receive)(void *handle, uint8_t *data, size_t size,
                            bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t (*receive_to_idle)(void *handle, uint8_t *data, size_t capacity,
                                    bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t (*abort)(void *handle);
    bsp_status_t (*get_busy)(const void *handle, bool *is_busy);
} bsp_usart_driver_ops_t;
```

**必须实现的函数**：`transmit`、`receive`。`receive_to_idle`、`abort`、`get_busy` 为可选，若未实现公共接口会返回 `BSP_STATUS_UNSUPPORTED`。`init`/`deinit` 可选。

## 5. API 参考

| 函数                        | 说明                    | 返回值                               |
| :-------------------------- | :---------------------- | :----------------------------------- |
| `bsp_usart_init`            | 初始化 USART 设备       | `OK` / `INVALID_ARGUMENT`            |
| `bsp_usart_as_base`         | 向上转型                | 基类指针或 `NULL`                    |
| `bsp_usart_set_callback`    | 设置事件回调            | `OK` / `NOT_INITIALIZED`             |
| `bsp_usart_transmit`        | 发送数据                | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_usart_receive`         | 固定长度接收            | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_usart_receive_to_idle` | 空闲线接收（不定长）    | `OK` / `UNSUPPORTED` / 平台错误      |
| `bsp_usart_abort`           | 中止当前事务            | `OK` / `UNSUPPORTED`                 |
| `bsp_usart_get_busy`        | 查询是否忙              | `OK` / `UNSUPPORTED`                 |
| `bsp_usart_notify`          | 事件通知（由 ISR 调用） | 无返回值                             |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_usart_driver.c
static bsp_status_t stm32_usart_transmit(void *handle, const uint8_t *data, size_t size,
                                         bsp_transfer_mode_t mode, uint32_t timeout) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle;
    switch (mode) {
        case BSP_TRANSFER_MODE_BLOCKING:
            return bsp_stm32h723_status(HAL_UART_Transmit(huart, data, size, timeout));
        case BSP_TRANSFER_MODE_INTERRUPT:
            return bsp_stm32h723_status(HAL_UART_Transmit_IT(huart, data, size));
        case BSP_TRANSFER_MODE_DMA:
            return bsp_stm32h723_status(HAL_UART_Transmit_DMA(huart, data, size));
        default:
            return BSP_STATUS_INVALID_ARGUMENT;
    }
}

static bsp_status_t stm32_usart_receive_to_idle(void *handle, uint8_t *data, size_t capacity,
                                                bsp_transfer_mode_t mode, uint32_t timeout) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle;
    switch (mode) {
        case BSP_TRANSFER_MODE_BLOCKING: {
            uint16_t received = 0;
            return bsp_stm32h723_status(HAL_UARTEx_ReceiveToIdle(huart, data, capacity, &received, timeout));
        }
        case BSP_TRANSFER_MODE_INTERRUPT:
            return bsp_stm32h723_status(HAL_UARTEx_ReceiveToIdle_IT(huart, data, capacity));
        case BSP_TRANSFER_MODE_DMA:
            return bsp_stm32h723_status(HAL_UARTEx_ReceiveToIdle_DMA(huart, data, capacity));
        default:
            return BSP_STATUS_INVALID_ARGUMENT;
    }
}

const bsp_usart_driver_ops_t stm32_usart_driver = {
    .init = NULL,
    .deinit = NULL,
    .transmit = stm32_usart_transmit,
    .receive = stm32_usart_receive,
    .receive_to_idle = stm32_usart_receive_to_idle,
    .abort = stm32_usart_abort,
    .get_busy = stm32_usart_busy,
};
```

### 6.2 应用层初始化

```c
static bsp_usart_device_t s_dr16_usart;
static bsp_usart_t *s_dr16_ptr = NULL;

void board_usart_init(void) {
    bsp_usart_config_t cfg = {
        .device_handle = &huart5,   // DR16 使用 UART5
        .driver_ops = &stm32_usart_driver,
        .callback = dr16_rx_callback,
        .user_context = &dr16_ctx,
    };
    bsp_usart_init(&s_dr16_usart, &cfg);
    s_dr16_ptr = bsp_usart_as_base(&s_dr16_usart);
}
```

### 6.3 固定长度接收（阻塞模式）

```c
uint8_t rx_buffer[64];
if (bsp_usart_receive(s_dr16_ptr, rx_buffer, 64, BSP_TRANSFER_MODE_BLOCKING, 100) == BSP_STATUS_OK) {
    // 成功接收 64 字节
}
```

### 6.4 空闲线接收（DMA 模式，不定长协议）

```c
static uint8_t dr16_rx_buffer[128];

// 启动 DMA 空闲接收
bsp_usart_receive_to_idle(s_dr16_ptr, dr16_rx_buffer, sizeof(dr16_rx_buffer),
                          BSP_TRANSFER_MODE_DMA, 0);

// 回调函数（在 ISR 中执行）
void dr16_rx_callback(bsp_event_t event, bsp_status_t status, size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_COMPLETE) {
        // size 为实际接收的字节数（空闲线触发或缓冲区满）
        // 立即重启 DMA 接收
        bsp_usart_receive_to_idle(s_dr16_ptr, dr16_rx_buffer, sizeof(dr16_rx_buffer),
                                  BSP_TRANSFER_MODE_DMA, 0);
        // 将数据交给任务处理（通过信号量或消息队列）
        xSemaphoreGiveFromISR(dr16_sem, NULL);
    }
}
```

### 6.5 异步发送

```c
uint8_t tx_buffer[] = "Hello World\n";
// 注册回调以获取发送完成通知
bsp_usart_transmit(s_dr16_ptr, tx_buffer, sizeof(tx_buffer) - 1,
                   BSP_TRANSFER_MODE_DMA, 0);
// 在回调中处理发送完成事件
```

### 6.6 中止与忙查询

```c
// 超时恢复：中止当前传输
if (bsp_usart_abort(s_dr16_ptr) == BSP_STATUS_OK) {
    // 等待中止完成
    bool busy;
    do {
        bsp_usart_get_busy(s_dr16_ptr, &busy);
    } while (busy);
    // 重新启动接收
    bsp_usart_receive_to_idle(s_dr16_ptr, buffer, capacity, BSP_TRANSFER_MODE_DMA, 0);
}
```

## 7. 空闲线接收详解

`receive_to_idle` 是 `bsp_usart` 的特色功能，特别适合不定长协议：

- **工作原理**：启动接收后，硬件持续接收数据直到：
  1. 缓冲区已满（`capacity` 字节）。
  2. 收到空闲线（IDLE 标志）。
- **通知方式**：通过 `bsp_event_callback_t` 回调，`transferred_size` 为实际接收字节数。
- **典型应用**：
  - DR16 遥控器（数据包长度不固定）。
  - 裁判系统（不同命令长度不同）。
  - 自定义 Modbus 协议。

**最佳实践**：

```c
// 在回调中立即重启 DMA，避免丢失数据
void rx_callback(bsp_event_t event, bsp_status_t status, size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_COMPLETE) {
        // 1. 保存数据（复制到处理队列）
        // 2. 立即重启接收
        bsp_usart_receive_to_idle(usart, buffer, capacity, BSP_TRANSFER_MODE_DMA, 0);
        // 3. 通知任务处理数据
    }
}
```

## 8. 传输模式说明

| 模式                          | 行为                              | 适用场景                      |
| :---------------------------- | :-------------------------------- | :---------------------------- |
| `BSP_TRANSFER_MODE_BLOCKING`  | 函数阻塞直到传输完成或超时        | 初始化、调试、低频通信        |
| `BSP_TRANSFER_MODE_INTERRUPT` | 启动异步传输，完成后通过回调通知  | 中等速率、需保持 CPU 响应能力 |
| `BSP_TRANSFER_MODE_DMA`       | 使用 DMA 传输，完成后通过回调通知 | 大量数据传输、高吞吐量需求    |

## 9. 异步缓冲区生命周期

- **中断/DMA 模式下**，用户提供的 `data` 缓冲区在传输完成、错误或中止前必须保持有效（不可释放或出作用域）。
- **双缓冲/环形缓冲**：推荐使用双缓冲或环形缓冲策略，确保接收不中断。
- **示例**：

```c
// 错误示例：缓冲区在栈上，函数返回后失效
void bad_example(void) {
    uint8_t local_buf[64];
    bsp_usart_receive_to_idle(usart, local_buf, 64, BSP_TRANSFER_MODE_DMA, 0);
    // 函数返回后 local_buf 失效，DMA 仍在访问！
}

// 正确示例：静态或全局缓冲区
static uint8_t g_rx_buffer[128];
void good_example(void) {
    bsp_usart_receive_to_idle(usart, g_rx_buffer, 128, BSP_TRANSFER_MODE_DMA, 0);
}
```

## 10. 中断与回调

- **平台 HAL 回调**：映射到具体对象后调用 `bsp_usart_notify`。
- **回调上下文**：在 ISR 中执行，必须快速返回。
- **回调中允许**：设置标志、累加计数、释放信号量。
- **回调中禁止**：阻塞、`printf`、修改配置（除非明确安全）。

**HAL 回调路由示例**：

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    bsp_usart_t *usart = find_usart_by_handle(huart);
    if (usart != NULL) {
        bsp_usart_notify(usart, BSP_EVENT_TRANSMIT_COMPLETE, BSP_STATUS_OK, huart->TxXferSize);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    bsp_usart_t *usart = find_usart_by_handle(huart);
    if (usart != NULL) {
        bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, size);
    }
}
```

## 11. 错误恢复

帧错误、噪声、溢出或 DMA 错误应通知 `BSP_EVENT_ERROR`。Module 应统计错误并决定重新启动接收。

**推荐恢复流程**：

1. 调用 `bsp_usart_abort` 中止当前传输。
2. 等待 `get_busy` 返回 `false`。
3. 重新启动接收（`receive` 或 `receive_to_idle`）。
4. 错误计数上报健康管理。

**禁止**：在 ISR 中打印日志或进行复杂错误处理。

## 12. 生命周期与并发

- **初始化顺序**：`bsp_usart_init` → 使用 → `bsp_device_deinit`。
- **反初始化**：先 `bsp_usart_abort`（若有异步传输），再 `bsp_device_deinit`。
- **回调替换**：若中断可能并发发生，更换回调前应禁用中断或使用临界区。
- **并发约束**：同一串口的发送队列和接收状态需要单一所有者或外部锁。`get_busy` 只能反映平台状态，不能替代多任务事务管理。

## 13. 错误码速查

| 错误码                        | 触发场景                                                   |
| :---------------------------- | :--------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、数据指针为空、大小为 0、传输模式非法             |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                               |
| `BSP_STATUS_UNSUPPORTED`      | 调用可选函数但驱动未实现（receive_to_idle/abort/get_busy） |
| `BSP_STATUS_TIMEOUT`          | 阻塞模式下超时                                             |
| `BSP_STATUS_IO_ERROR`         | 帧错误、噪声、溢出、DMA 错误等                             |

## 14. 移植要求

平台移植者需实现 `bsp_usart_driver_ops_t`：

- **`transmit`**（必须）：发送数据，支持三种传输模式。
- **`receive`**（必须）：固定长度接收，支持三种传输模式。
- **`receive_to_idle`**（可选）：空闲线接收，支持三种传输模式。
- **`abort`**（可选）：中止当前异步事务。
- **`get_busy`**（可选）：查询是否有事务正在进行。

**关键注意事项**：

- DMA 模式下需处理缓冲区对齐和缓存一致性（D-Cache 清理/失效）。
- 中断模式下需在中断服务程序中调用 `bsp_usart_notify`。
- 空闲线接收的实现因平台而异（STM32 使用 `HAL_UARTEx_ReceiveToIdle`）。
- 接收缓冲区在 DMA 模式下的生命周期管理由调用者负责。

## 15. 建议验证测试项

- [ ] 固定长度发送/接收（阻塞/中断/DMA）功能正常。
- [ ] 空闲线接收（不定长协议）功能正常。
- [ ] 零长度参数返回 `INVALID_ARGUMENT`。
- [ ] 缓冲区满时正确触发回调。
- [ ] 连续短帧接收不丢数据。
- [ ] 发送忙、接收忙状态查询准确。
- [ ] 中止功能正常（中断/DMA 模式下）。
- [ ] 错误恢复流程（帧错误、DMA 错误）。
- [ ] 两个串口实例独立工作。
- [ ] 异步缓冲区在传输期间保持有效。

---

## 一页式接入顺序与可读信息

1. CubeMX/平台配置波特率、数据位、校验、停止位、DMA 和 IRQ，再注入 USART driver ops。
2. init 后先注册 callback；固定帧用 receive，不定长流用 receive_to_idle，DR16 使用 double-buffer 版本。
3. 启动一次接收后，由平台在 HAL 回调中调用 `bsp_usart_notify()` 或 `notify_double_buffer()`。
4. Module 回调只搬运最小数据并立即重启接收，任务上下文再解析协议。
5. 发送选择阻塞/中断/DMA；停机或重配前 abort/deinit。

可读信息来自接收 buffer、回调的 `transferred_size`/event/status 和 `bsp_usart_get_busy()`。双缓冲回调还会指出完成的 buffer；未完成的 DMA buffer 不得覆盖。
