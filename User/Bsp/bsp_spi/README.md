# BSP SPI 通用抽象层 (bsp_spi)

## 1. 模块概述

`bsp_spi` 提供了对 SPI 总线主机的通用抽象，支持发送、接收、全双工交换、中止和忙状态查询。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **发送（Transmit）**：仅发送数据，不关心接收内容。
- **接收（Receive）**：仅接收数据，平台端负责产生时钟。
- **全双工交换（Exchange）**：等长数据的发送与接收同时进行。
- **中止（Abort）**：终止当前异步事务。
- **忙状态查询（Get Busy）**：查询总线当前是否有事务正在进行。

**设计哲学**：

- **三种传输模式**：阻塞（Blocking）、中断（Interrupt）、DMA 统一接口。
- **片选独立性**：SPI 模块不自动控制片选（CS），由上层 Module 通过 `bsp_gpio_t *` 或专用平台接口控制。
- **灵活配置**：SPI 模式（CPOL/CPHA）、位宽、时钟频率和片选策略由平台端与设备 Module 共同决定。

## 2. 设计边界

| **模块负责**                        | **模块不负责**                                 |
| :---------------------------------- | :--------------------------------------------- |
| 发送、接收、全双工交换的统一接口    | SPI 模式（CPOL/CPHA）配置                      |
| 三种传输模式（阻塞/中断/DMA）的抽象 | 时钟频率和位宽配置                             |
| 事务中止和忙状态查询                | 片选信号控制（由上层 Module 管理）             |
| 事件回调通知（完成/错误/中止）      | 多设备共享总线的串行化调度                     |
| 设备对象生命周期管理                | DMA 通道配置和缓存一致性维护（由平台驱动负责） |

**重要说明**：

- **片选控制**：通用 SPI 不自动控制片选，因为不同器件对事务边界、延时和多段读写要求不同。设备 Module 通过注入的 `bsp_gpio_t *` 或专用平台事务接口控制片选，并保证一次协议事务期间总线不被其他设备抢占。
- **共享总线**：多个传感器共用 SPI 时，外部总线管理器必须串行化事务，并在切换设备时处理不同的 CPOL、CPHA、频率或位宽。不能只靠各自片选而并发调用同一 SPI 对象。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_spi_t             (基类：增加 callback、user_context)
    └── bsp_spi_device_t  (派生类：持有 driver_ops)
```

- **`bsp_spi_t`**：应用层使用的基类指针，包含事件回调和用户上下文。
- **`bsp_spi_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_spi_ops_t` 继承自 `bsp_device_ops_t`，新增 `transmit`、`receive`、`exchange`、`abort`、`get_busy`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_spi_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_spi_driver_ops_t *driver_ops;   // 底层驱动表
    bsp_event_callback_t callback;            // 事件回调（可为 NULL）
    void *user_context;                       // 回调用户上下文
} bsp_spi_config_t;
```

### 4.2 底层驱动操作表 (`bsp_spi_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *);
    bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *, const uint8_t *, size_t,
                             bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive)(void *, uint8_t *, size_t,
                            bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*exchange)(void *, const uint8_t *, uint8_t *, size_t,
                             bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*abort)(void *);
    bsp_status_t (*get_busy)(const void *, bool *);
} bsp_spi_driver_ops_t;
```

**必须实现的函数**：`transmit`、`receive`。`exchange`、`abort`、`get_busy` 为可选，若未实现公共接口会返回 `BSP_STATUS_UNSUPPORTED`。`init`/`deinit` 可选。

## 5. API 参考

| 函数                   | 说明                        | 返回值                          |
| :--------------------- | :-------------------------- | :------------------------------ |
| `bsp_spi_init`         | 初始化 SPI 设备             | `OK` / `INVALID_ARGUMENT`       |
| `bsp_spi_as_base`      | 向上转型，获取基类指针      | 基类指针或 `NULL`               |
| `bsp_spi_set_callback` | 设置事件回调                | `OK` / `NOT_INITIALIZED`        |
| `bsp_spi_transmit`     | 发送数据（只发不收）        | `OK` / `UNSUPPORTED` / 平台错误 |
| `bsp_spi_receive`      | 接收数据（只收不发）        | `OK` / `UNSUPPORTED` / 平台错误 |
| `bsp_spi_exchange`     | 全双工交换（收发同时）      | `OK` / `UNSUPPORTED` / 平台错误 |
| `bsp_spi_abort`        | 中止当前事务                | `OK` / `UNSUPPORTED`            |
| `bsp_spi_get_busy`     | 查询总线是否忙              | `OK` / `UNSUPPORTED`            |
| `bsp_spi_notify`       | 事件通知（由平台 ISR 调用） | 无返回值                        |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_spi_driver.c
static bsp_status_t stm32_spi_transmit(void *handle, const uint8_t *data, size_t size,
                                       bsp_transfer_mode_t mode, uint32_t timeout) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    // 根据 mode 选择 HAL_SPI_Transmit / HAL_SPI_Transmit_IT / HAL_SPI_Transmit_DMA
    // ...
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_spi_receive(void *handle, uint8_t *data, size_t size,
                                      bsp_transfer_mode_t mode, uint32_t timeout) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    // 根据 mode 选择 HAL_SPI_Receive / HAL_SPI_Receive_IT / HAL_SPI_Receive_DMA
    // ...
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_spi_exchange(void *handle, const uint8_t *tx, uint8_t *rx,
                                       size_t size, bsp_transfer_mode_t mode, uint32_t timeout) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    // 根据 mode 选择 HAL_SPI_TransmitReceive / ..._IT / ..._DMA
    // ...
    return BSP_STATUS_OK;
}

const bsp_spi_driver_ops_t stm32_spi_driver = {
    .init = stm32_spi_init,
    .deinit = NULL,
    .transmit = stm32_spi_transmit,
    .receive = stm32_spi_receive,
    .exchange = stm32_spi_exchange,
    .abort = stm32_spi_abort,
    .get_busy = stm32_spi_get_busy,
};
```

### 6.2 应用层初始化

```c
static bsp_spi_device_t s_sensor_spi;
static bsp_spi_t *s_spi_ptr = NULL;

void board_spi_init(void) {
    bsp_spi_config_t cfg = {
        .device_handle = &hspi1,
        .driver_ops = &stm32_spi_driver,
        .callback = spi_event_callback,
        .user_context = &sensor,
    };
    bsp_spi_init(&s_sensor_spi, &cfg);
    s_spi_ptr = bsp_spi_as_base(&s_sensor_spi);
}
```

### 6.3 带片选控制的传感器读取

```c
// 设备 Module 层控制片选
static void sensor_read(bsp_spi_t *spi, bsp_gpio_t *cs, uint8_t *data, size_t size) {
    bsp_gpio_write(cs, false);   // 片选有效（低电平选中）
    bsp_spi_receive(spi, data, size, BSP_TRANSFER_MODE_BLOCKING, 100);
    bsp_gpio_write(cs, true);    // 片选释放
}

// 带片选的全双工交换
static void sensor_exchange(bsp_spi_t *spi, bsp_gpio_t *cs,
                            const uint8_t *tx, uint8_t *rx, size_t size) {
    bsp_gpio_write(cs, false);
    bsp_spi_exchange(spi, tx, rx, size, BSP_TRANSFER_MODE_BLOCKING, 100);
    bsp_gpio_write(cs, true);
}
```

### 6.4 异步 DMA 传输

```c
uint8_t dma_tx_buffer[256];
uint8_t dma_rx_buffer[256];

// 注册回调
bsp_spi_set_callback(s_spi_ptr, spi_dma_callback, NULL);

// 启动异步 DMA 发送
bsp_spi_transmit(s_spi_ptr, dma_tx_buffer, 256, BSP_TRANSFER_MODE_DMA, 0);

void spi_dma_callback(bsp_event_t event, bsp_status_t status, size_t size, void *ctx) {
    if (event == BSP_EVENT_TRANSMIT_COMPLETE) {
        // DMA 发送完成
    }
}
```

### 6.5 中止与忙查询

```c
// 超时恢复：先中止，再释放片选
if (bsp_spi_abort(s_spi_ptr) == BSP_STATUS_OK) {
    // 等待中止完成（通过回调或轮询 get_busy）
    bool busy;
    do {
        bsp_spi_get_busy(s_spi_ptr, &busy);
    } while (busy);
    bsp_gpio_write(cs, true);   // 释放片选
}
```

## 7. 传输模式说明

| 模式                          | 行为                              | 适用场景                      |
| :---------------------------- | :-------------------------------- | :---------------------------- |
| `BSP_TRANSFER_MODE_BLOCKING`  | 函数阻塞直到传输完成或超时        | 初始化、低频通信、简单配置    |
| `BSP_TRANSFER_MODE_INTERRUPT` | 启动异步传输，完成后通过回调通知  | 中等速率、需保持 CPU 响应能力 |
| `BSP_TRANSFER_MODE_DMA`       | 使用 DMA 传输，完成后通过回调通知 | 大量数据传输、高吞吐量需求    |

## 8. 片选控制

SPI 模块**不自动控制片选**（CS/NSS），由上层 Module 负责：

- **独立片选 GPIO**：通过 `bsp_gpio_t *` 控制片选，适用于大多数场景。
- **硬件片选（NSS）**：部分平台支持硬件自动控制，但需注意不同器件的时序要求。
- **多段事务**：对于需要连续多段读写的器件（如带地址的传感器），片选需在整个事务期间保持有效。

```c
// 典型的多段事务：发送地址 → 读取数据（片选全程有效）
bsp_gpio_write(cs, false);
bsp_spi_transmit(spi, &address, 1, BSP_TRANSFER_MODE_BLOCKING, 10);
bsp_spi_receive(spi, data, size, BSP_TRANSFER_MODE_BLOCKING, 10);
bsp_gpio_write(cs, true);
```

## 9. 异步缓冲区生命周期

- IRQ/DMA 模式下，发送和接收缓冲区必须**持续有效**直到传输完成、中止或错误通知。
- 平台端处理 D-Cache 清理/失效后调用 `bsp_spi_notify`。
- 通用层不复制缓冲区，调用者负责生命周期管理。

```c
// 错误示例（禁止）：缓冲区在栈上，函数返回后失效
void bad_example(bsp_spi_t *spi) {
    uint8_t local_buffer[64];
    bsp_spi_transmit(spi, local_buffer, 64, BSP_TRANSFER_MODE_DMA, 0);
    // 函数返回后 local_buffer 失效，DMA 仍在访问
}

// 正确示例：静态或全局缓冲区
static uint8_t g_tx_buffer[64];
void good_example(bsp_spi_t *spi) {
    bsp_spi_transmit(spi, g_tx_buffer, 64, BSP_TRANSFER_MODE_DMA, 0);
}
```

## 10. 错误恢复

超时或 DMA 错误后的标准恢复流程：

1. 调用 `bsp_spi_abort` 中止当前事务。
2. 通过回调或轮询 `bsp_spi_get_busy` 确认中止完成。
3. 释放片选和总线锁。
4. 上报设备健康状态。
5. **禁止**在控制周期内无限重试。

## 11. 生命周期与并发

- **初始化顺序**：`bsp_spi_init` 后即可使用。
- **反初始化**：通过 `bsp_device_deinit` 触发虚析构。
- **共享总线**：多个设备共享同一 SPI 对象时，必须由外部总线管理器串行化事务。
- **并发约束**：SPI 总线不支持并发调用，必须由上层提供互斥保护。

## 12. 错误码速查

| 错误码                        | 触发场景                                            |
| :---------------------------- | :-------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、数据指针为空、大小为 0、传输模式非法      |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                        |
| `BSP_STATUS_UNSUPPORTED`      | 调用可选函数但驱动未实现（exchange/abort/get_busy） |
| `BSP_STATUS_TIMEOUT`          | 阻塞模式下超时                                      |
| `BSP_STATUS_IO_ERROR`         | 硬件错误（仲裁丢失、过载等）                        |
| `BSP_STATUS_BUSY`             | 总线忙                                              |

## 13. 移植要求

平台移植者需实现 `bsp_spi_driver_ops_t`：

- **`transmit`**（必须）：发送数据，支持三种传输模式。
- **`receive`**（必须）：接收数据，支持三种传输模式。
- **`exchange`**（可选）：全双工交换，支持三种传输模式。
- **`abort`**（可选）：中止当前异步事务。
- **`get_busy`**（可选）：查询是否有事务正在进行。

**关键注意事项**：

- 接收模式下，平台驱动必须产生 SPI 时钟（通过发送 dummy 数据）。
- DMA 模式下需处理缓冲区对齐和缓存一致性（D-Cache 清理/失效）。
- 中断模式下需在中断服务程序中调用 `bsp_spi_notify`。
- 模式切换（CPOL/CPHA、频率、位宽）由平台驱动或上层 Module 管理，通用层不干涉。

## 14. 建议验证测试项

- [ ] 三种传输接口（transmit/receive/exchange）功能正常。
- [ ] 三种传输模式（阻塞/中断/DMA）均正常工作。
- [ ] 0 字节、最大长度及空指针返回 `INVALID_ARGUMENT`。
- [ ] CPOL/CPHA 和频率切换后通信正常。
- [ ] DMA 传输完成、中止和错误回调正确触发。
- [ ] 片选事务完整性（多段事务片选持续有效）。
- [ ] 多设备共享总线时串行化正确，无数据冲突。
- [ ] 忙查询状态准确（`get_busy`）。
- [ ] 异步缓冲区在传输期间保持有效，无悬空指针。
- [ ] 缓存一致性（D-Cache）正确处理。

---

**总结**：`bsp_spi` 提供了灵活、可移植的 SPI 主机抽象，适用于各种传感器、存储器和外设的通信。其三种传输模式和三种传输接口的组合满足了从简单配置到高速数据采集的各类场景。片选控制的独立性设计让上层 Module 能够精确控制事务边界，而可选函数（exchange/abort/get_busy）的 `UNSUPPORTED` 返回机制则兼容了不同硬件平台的能力差异。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
