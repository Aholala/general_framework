# BSP I2C 通用抽象层 (bsp_i2c)

## 1. 模块概述

`bsp_i2c` 提供了对 I2C 主机（Master）接口的通用抽象，支持直接收发、8/16 位寄存器地址访问、设备就绪探测、中止和忙状态查询。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **直接收发**：7 位设备地址的发送和接收（阻塞/中断/DMA）。
- **内存访问**：支持 8 位或 16 位寄存器地址的读写（适用于 EEPROM、传感器等）。
- **设备探测**：通过 `is_device_ready` 检查设备是否响应。
- **事务管理**：`abort` 中止当前事务，`get_busy` 查询总线忙状态。
- **三种传输模式**：阻塞（Blocking）、中断（Interrupt）、DMA。

**设计哲学**：

- **地址规则**：所有公共接口使用 **7 位设备地址**，调用方**不允许预先左移**。平台驱动负责转换为 HAL 或寄存器所需的格式。
- **多态接口**：通过 `bsp_i2c_ops_t` 虚表实现多态，底层可替换为不同平台的 I2C 驱动。
- **异步支持**：中断/DMA 模式下，回调通知传输完成，用户缓冲区在传输期间必须保持有效。

## 2. 设计边界

| **模块负责**                               | **模块不负责**                                                 |
| :----------------------------------------- | :------------------------------------------------------------- |
| I2C 传输的公共接口（发送、接收、内存读写） | I2C 引脚复用、上拉电阻、时钟速度配置（由平台端或 CubeMX 配置） |
| 7 位地址验证和参数校验                     | 10 位地址支持（本模块仅支持 7 位）                             |
| 三种传输模式（阻塞/中断/DMA）的统一抽象    | 多主机仲裁、时钟拉伸、总线恢复的硬件细节                       |
| 设备就绪探测、中止、忙状态查询             | SMBus 协议、PMBus 协议等特殊功能                               |
| 事件回调通知（完成/错误/中止）             | 具体传感器的寄存器协议（由 Module 层处理）                     |
| 异步缓冲区的生命周期管理（调用者负责）     | 动态内存分配                                                   |

**适用场景**：

- 读取传感器数据（温度、加速度、磁力计、陀螺仪）。
- 读写 EEPROM 存储器。
- 配置 I2C 外设寄存器（ADC、DAC、IO 扩展器）。
- 设备在线检测和初始化。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_i2c_t             (基类：增加 callback、user_context)
    └── bsp_i2c_device_t  (派生类：持有 driver_ops)
```

- **`bsp_i2c_t`**：应用层使用的基类指针，包含事件回调和用户上下文。
- **`bsp_i2c_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_i2c_ops_t` 继承自 `bsp_device_ops_t`，新增 `transmit`、`receive`、`memory_write`、`memory_read`、`is_device_ready`、`abort`、`get_busy`。

## 4. 核心类型

### 4.1 内存地址大小枚举 (`bsp_i2c_memory_address_size_t`)

```c
typedef enum {
    BSP_I2C_MEMORY_ADDRESS_8_BIT = 1,   // 8 位寄存器地址
    BSP_I2C_MEMORY_ADDRESS_16_BIT = 2   // 16 位寄存器地址
} bsp_i2c_memory_address_size_t;
```

### 4.2 配置结构 (`bsp_i2c_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_i2c_driver_ops_t *driver_ops;   // 底层驱动表
    bsp_event_callback_t callback;            // 事件回调（可为 NULL）
    void *user_context;                       // 回调用户上下文
} bsp_i2c_config_t;
```

### 4.3 底层驱动操作表 (`bsp_i2c_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *);
    bsp_status_t (*deinit)(void *);
    bsp_status_t (*transmit)(void *, uint16_t, const uint8_t *, size_t,
                             bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*receive)(void *, uint16_t, uint8_t *, size_t,
                            bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*memory_write)(void *, uint16_t, uint16_t,
                                 bsp_i2c_memory_address_size_t, const uint8_t *,
                                 size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*memory_read)(void *, uint16_t, uint16_t,
                                bsp_i2c_memory_address_size_t, uint8_t *,
                                size_t, bsp_transfer_mode_t, uint32_t);
    bsp_status_t (*is_device_ready)(void *, uint16_t, uint32_t, uint32_t);
    bsp_status_t (*abort)(void *, uint16_t);
    bsp_status_t (*get_busy)(const void *, bool *);
} bsp_i2c_driver_ops_t;
```

**必须实现的函数**：`transmit`、`receive`。其余函数为可选，若未实现公共接口会返回 `BSP_STATUS_UNSUPPORTED`。`init`/`deinit` 可选。

## 5. API 参考

| 函数                      | 说明                           | 返回值                           |
| :------------------------ | :----------------------------- | :------------------------------- |
| `bsp_i2c_init`            | 初始化 I2C 设备                | `OK` / `INVALID_ARGUMENT`        |
| `bsp_i2c_as_base`         | 向上转型，获取基类指针         | 基类指针或 `NULL`                |
| `bsp_i2c_set_callback`    | 设置事件回调                   | `OK` / `NOT_INITIALIZED`         |
| `bsp_i2c_transmit`        | 发送数据到从设备（7位地址）    | `OK` / `UNSUPPORTED` / 平台错误  |
| `bsp_i2c_receive`         | 从从设备接收数据（7位地址）    | `OK` / `UNSUPPORTED` / 平台错误  |
| `bsp_i2c_memory_write`    | 向寄存器地址写入数据（8/16位） | `OK` / `UNSUPPORTED` / 平台错误  |
| `bsp_i2c_memory_read`     | 从寄存器地址读取数据（8/16位） | `OK` / `UNSUPPORTED` / 平台错误  |
| `bsp_i2c_is_device_ready` | 检查设备是否响应               | `OK` / `TIMEOUT` / `UNSUPPORTED` |
| `bsp_i2c_abort`           | 中止当前事务                   | `OK` / `UNSUPPORTED` / 平台错误  |
| `bsp_i2c_get_busy`        | 查询总线是否忙                 | `OK` / `UNSUPPORTED`             |
| `bsp_i2c_notify`          | 事件通知（由平台 ISR 调用）    | 无返回值                         |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_i2c_driver.c
static bsp_status_t stm32_i2c_transmit(void *handle, uint16_t addr, const uint8_t *data,
                                       size_t size, bsp_transfer_mode_t mode, uint32_t timeout) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    uint16_t dev_addr = addr << 1;  // 7位地址左移为8位地址
    HAL_StatusTypeDef hal_status;
    // 根据 mode 选择阻塞/中断/DMA
    // ...
    return hal_status == HAL_OK ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

const bsp_i2c_driver_ops_t stm32_i2c_driver = {
    .init = stm32_i2c_init,
    .deinit = NULL,
    .transmit = stm32_i2c_transmit,
    .receive = stm32_i2c_receive,
    .memory_write = stm32_i2c_memory_write,
    .memory_read = stm32_i2c_memory_read,
    .is_device_ready = stm32_i2c_is_device_ready,
    .abort = stm32_i2c_abort,
    .get_busy = stm32_i2c_get_busy,
};
```

### 6.2 应用层初始化

```c
static bsp_i2c_device_t s_i2c_dev;
static bsp_i2c_t *s_i2c_ptr = NULL;

void board_i2c_init(void) {
    bsp_i2c_config_t cfg = {
        .device_handle = &hi2c1,
        .driver_ops = &stm32_i2c_driver,
        .callback = i2c_event_callback,
        .user_context = NULL,
    };
    bsp_i2c_init(&s_i2c_dev, &cfg);
    s_i2c_ptr = bsp_i2c_as_base(&s_i2c_dev);
}
```

### 6.3 基本收发

```c
// 发送数据到设备地址 0x68
uint8_t tx_data[] = {0x01, 0x02, 0x03};
bsp_i2c_transmit(s_i2c_ptr, 0x68, tx_data, sizeof(tx_data),
                 BSP_TRANSFER_MODE_BLOCKING, 100);

// 从设备接收数据
uint8_t rx_data[8];
bsp_i2c_receive(s_i2c_ptr, 0x68, rx_data, sizeof(rx_data),
                BSP_TRANSFER_MODE_BLOCKING, 100);
```

### 6.4 寄存器读写（如传感器、EEPROM）

```c
// 读取 8 位寄存器 0x0F 的值
uint8_t reg_value;
bsp_i2c_memory_read(s_i2c_ptr, 0x68, 0x0F, BSP_I2C_MEMORY_ADDRESS_8_BIT,
                    &reg_value, 1, BSP_TRANSFER_MODE_BLOCKING, 10);

// 写入 16 位寄存器地址 0x0100
uint16_t eeprom_addr = 0x0100;
uint8_t write_data = 0xAA;
bsp_i2c_memory_write(s_i2c_ptr, 0x50, eeprom_addr, BSP_I2C_MEMORY_ADDRESS_16_BIT,
                     &write_data, 1, BSP_TRANSFER_MODE_BLOCKING, 100);
```

### 6.5 设备探测与异步传输

```c
// 设备就绪探测
if (bsp_i2c_is_device_ready(s_i2c_ptr, 0x68, 3, 10) == BSP_STATUS_OK) {
    // 设备在线
}

// 异步 DMA 接收
uint8_t dma_buffer[256];
bsp_i2c_set_callback(s_i2c_ptr, my_i2c_callback, NULL);
bsp_i2c_receive(s_i2c_ptr, 0x68, dma_buffer, 256,
                BSP_TRANSFER_MODE_DMA, 0);

void my_i2c_callback(bsp_event_t event, bsp_status_t status, size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_COMPLETE) {
        // DMA 传输完成，处理 dma_buffer
    }
}
```

## 7. 传输模式说明

| 模式                          | 行为                              | 适用场景                      |
| :---------------------------- | :-------------------------------- | :---------------------------- |
| `BSP_TRANSFER_MODE_BLOCKING`  | 函数阻塞直到传输完成或超时        | 初始化、低频读取、简单配置    |
| `BSP_TRANSFER_MODE_INTERRUPT` | 启动异步传输，完成后通过回调通知  | 中等速率、需保持 CPU 响应能力 |
| `BSP_TRANSFER_MODE_DMA`       | 使用 DMA 传输，完成后通过回调通知 | 大量数据传输、高吞吐量需求    |

## 8. 中断与事件通知

平台端在中断或 DMA 完成回调中调用：

```c
bsp_i2c_notify(i2c_ptr, BSP_EVENT_TRANSFER_COMPLETE, BSP_STATUS_OK, transferred_size);
```

事件类型包括：`TRANSMIT_COMPLETE`、`RECEIVE_COMPLETE`、`TRANSFER_COMPLETE`、`ABORT_COMPLETE`、`ERROR`。

**回调约束**：回调在 ISR 上下文中执行，必须快速返回，不可阻塞。

## 9. 异步缓冲区生命周期

- 中断/DMA 模式下，用户提供的 `data` 缓冲区在传输完成、错误或中止前必须保持有效（不可释放或出作用域）。
- 若使用 DMA，平台驱动需确保缓存一致性（如使用 D-Cache 时执行 `Cache_Invalidate` 或 `Clean`）。
- `bsp_i2c_abort` 可用于强制中止传输，中止后缓冲区可安全释放。

## 10. 生命周期与并发

- **初始化顺序**：`bsp_i2c_init` 后即可使用。
- **反初始化**：通过 `bsp_device_deinit` 触发虚析构。
- **并发约束**：I2C 总线是共享资源，多个任务或模块共享同一 I2C 对象时必须由上层串行化（总线管理器或互斥锁）。
- **忙状态查询**：`bsp_i2c_get_busy` 可用于检查是否有异步传输进行中，避免覆盖正在进行的传输。

## 11. 错误码速查

| 错误码                        | 触发场景                                                     |
| :---------------------------- | :----------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、地址 > 0x7F、数据指针为空、size 为 0、传输模式非法 |
| `BSP_STATUS_OUT_OF_RANGE`     | 地址超出 7 位范围、寄存器地址大小非法                        |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                                 |
| `BSP_STATUS_UNSUPPORTED`      | 调用可选函数但驱动未实现                                     |
| `BSP_STATUS_TIMEOUT`          | 阻塞模式下超时                                               |
| `BSP_STATUS_IO_ERROR`         | NACK、仲裁丢失、总线错误等                                   |

## 12. 移植要求

平台移植者需实现 `bsp_i2c_driver_ops_t`：

- **`transmit`**（必须）：向从设备发送数据。
- **`receive`**（必须）：从从设备接收数据。
- **`memory_write`**（可选）：带寄存器地址的写入（8/16 位）。
- **`memory_read`**（可选）：带寄存器地址的读取（8/16 位）。
- **`is_device_ready`**（可选）：发送 START + 地址，检测 NACK。
- **`abort`**（可选）：生成 STOP 或复位总线。
- **`get_busy`**（可选）：检查是否有传输进行中。

**关键注意事项**：

- 7 位地址在驱动层需左移 1 位（加上 R/W 位），具体取决于 HAL 或寄存器要求。
- 寄存器字节序（大端/小端）需根据目标设备协议决定。
- DMA 模式下需处理缓冲区对齐和缓存一致性问题。
- 中断模式下需在中断服务程序中调用 `bsp_i2c_notify`。
- 总线恢复（SDA 被拉低时发时钟脉冲、STOP 等）由平台驱动负责。

## 13. 建议验证测试项

- [ ] 7 位地址边界（0x00~0x7F）的发送/接收。
- [ ] 8 位和 16 位寄存器地址的内存读写。
- [ ] 阻塞、中断、DMA 三种模式均正常工作。
- [ ] NACK、超时、仲裁丢失等错误正确处理。
- [ ] `is_device_ready` 对在线/离线设备返回正确结果。
- [ ] 异步传输中 `abort` 能成功中止。
- [ ] `get_busy` 能正确反映总线状态。
- [ ] 多个设备（不同地址）共享同一总线。
- [ ] 中断/DMA 模式下缓冲区在传输期间保持有效。
- [ ] 反初始化后对象拒绝访问。

---

**总结**：`bsp_i2c` 提供了完整、可移植的 I2C 主机抽象，适用于各种传感器、存储器和外设的通信。其灵活的传输模式（阻塞/中断/DMA）和寄存器地址支持（8/16 位）使其能够适应从简单配置到高速数据采集的各种场景。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性，同时通过可选函数设计（`UNSUPPORTED`）兼容了不同硬件平台的能力差异。
