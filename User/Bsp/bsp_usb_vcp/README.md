# BSP USB CDC 虚拟串口通用抽象层 (bsp_usb_vcp)

## 1. 模块概述

`bsp_usb_vcp` 提供了对 USB CDC（Communications Device Class）虚拟串口的通用抽象，向 Module 层提供异步字节流接口，而不暴露 USB Device 中间件类型（如 `USBD_CDC_HandleTypeDef`）。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **发送（Transmit）**：异步发送数据（忙时返回 `BUSY`）。
- **接收（Receive）**：登记接收缓冲区，异步接收数据。
- **中止（Abort）**：终止当前发送事务。
- **连接状态查询（Get Connected）**：查询 USB 主机枚举/连接状态。
- **忙状态查询（Get Busy）**：查询发送是否正在占用。

**适用场景**：

- 上位机通信（视觉数据处理、调试信息输出）。
- 固件升级（通过 USB 传输固件数据）。
- 遥测数据传输（传感器数据、状态信息）。

## 2. 设计边界

| **模块负责**                             | **模块不负责**                                |
| :--------------------------------------- | :-------------------------------------------- |
| USB 虚拟串口发送/接收的抽象接口          | USB 端点号、描述符配置（由平台端配置）        |
| 异步事务中止和状态查询                   | USB Device 类对象和中间件回调（由平台端管理） |
| 事件回调通知（发送完成、接收完成、错误） | 协议解析（由 Module 层处理）                  |
| 设备对象生命周期管理                     | 接收缓冲区管理（由调用者持有）                |
| 连接状态和忙状态查询                     | USB 枚举状态机（由 USB 协议栈管理）           |

**重要说明**：

- **异步发送**：许多 USB CDC 实现不会立即复制发送数组。调用者必须让发送缓冲区保持有效直到发送完成通知。
- **接收登记**：`receive` 登记调用者持有的缓冲区。平台收到 USB 数据后完成缓存处理，再通过回调通知实际长度。
- **连接与容错**：拔线、主机休眠或重新枚举都是正常状态。断开时 Module 应超时离线，并回退到安全控制源。
- **平台边界**：端点号、描述符、USB Device 类对象和中间件回调均留在平台 Port。通用头文件不包含 `usbd_cdc_if.h`。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_usb_vcp_t             (基类：增加 callback、user_context)
    └── bsp_usb_vcp_device_t  (派生类：持有 driver_ops)
```

- **`bsp_usb_vcp_t`**：应用层使用的基类指针，包含事件回调和用户上下文。
- **`bsp_usb_vcp_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_usb_vcp_ops_t` 继承自 `bsp_device_ops_t`，新增 `transmit`、`receive`、`abort`、`get_connected`、`get_busy`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_usb_vcp_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_usb_vcp_driver_ops_t *driver_ops; // 底层驱动表
    bsp_event_callback_t callback;            // 事件回调（可为 NULL）
    void *user_context;                       // 回调用户上下文
} bsp_usb_vcp_config_t;
```

### 4.2 底层驱动操作表 (`bsp_usb_vcp_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*transmit)(void *handle, const uint8_t *data, size_t size, uint32_t timeout_ms);
    bsp_status_t (*receive)(void *handle, uint8_t *data, size_t capacity);
    bsp_status_t (*abort)(void *handle);
    bsp_status_t (*get_connected)(const void *handle, bool *is_connected);
    bsp_status_t (*get_busy)(const void *handle, bool *is_busy);
} bsp_usb_vcp_driver_ops_t;
```

**必须实现的函数**：`transmit`、`receive`。`abort`、`get_connected`、`get_busy` 为可选，若未实现公共接口会返回 `BSP_STATUS_UNSUPPORTED`。`init`/`deinit` 可选。

## 5. API 参考

| 函数                        | 说明                       | 返回值                                        |
| :-------------------------- | :------------------------- | :-------------------------------------------- |
| `bsp_usb_vcp_init`          | 初始化 USB VCP 设备        | `OK` / `INVALID_ARGUMENT`                     |
| `bsp_usb_vcp_as_base`       | 向上转型                   | 基类指针或 `NULL`                             |
| `bsp_usb_vcp_set_callback`  | 设置事件回调               | `OK` / `NOT_INITIALIZED`                      |
| `bsp_usb_vcp_transmit`      | 发送数据（异步）           | `OK` / `INVALID_ARGUMENT` / `BUSY` / 平台错误 |
| `bsp_usb_vcp_receive`       | 登记接收缓冲区（异步）     | `OK` / `INVALID_ARGUMENT` / `BUSY` / 平台错误 |
| `bsp_usb_vcp_abort`         | 中止当前事务               | `OK` / `UNSUPPORTED`                          |
| `bsp_usb_vcp_get_connected` | 查询 USB 连接状态          | `OK` / `UNSUPPORTED`                          |
| `bsp_usb_vcp_get_busy`      | 查询发送是否忙             | `OK` / `UNSUPPORTED`                          |
| `bsp_usb_vcp_notify`        | 事件通知（由平台回调调用） | 无返回值                                      |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_usb_vcp_driver.c
static bsp_status_t stm32_usb_transmit(void *handle, const uint8_t *data,
                                       size_t size, uint32_t timeout_ms) {
    uint32_t started = HAL_GetTick();
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)handle;
    do {
        if (USBD_CDC_TransmitPacket(hcdc, (uint8_t *)data, size) == USBD_OK) {
            return BSP_STATUS_OK;
        }
        if (hcdc->TxState != 0U) {
            return BSP_STATUS_BUSY;
        }
        if ((HAL_GetTick() - started) >= timeout_ms) {
            return BSP_STATUS_TIMEOUT;
        }
    } while (true);
}

static bsp_status_t stm32_usb_receive(void *handle, uint8_t *data, size_t capacity) {
    // 平台端维护接收缓冲区，在 usb_cdc_receive_callback 中填充
    // 这里返回是否有待接收数据
    // ...
}

static bsp_status_t stm32_usb_get_connected(const void *handle, bool *is_connected) {
    *is_connected = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
    return BSP_STATUS_OK;
}

const bsp_usb_vcp_driver_ops_t stm32_usb_vcp_driver = {
    .init = NULL,
    .deinit = NULL,
    .transmit = stm32_usb_transmit,
    .receive = stm32_usb_receive,
    .abort = stm32_usb_abort,
    .get_connected = stm32_usb_get_connected,
    .get_busy = stm32_usb_get_busy,
};
```

### 6.2 应用层初始化

```c
static bsp_usb_vcp_device_t s_vision_usb;
static bsp_usb_vcp_t *s_usb_vcp_ptr = NULL;

void board_usb_vcp_init(void) {
    bsp_usb_vcp_config_t cfg = {
        .device_handle = &hUsbDeviceFS,
        .driver_ops = &stm32_usb_vcp_driver,
        .callback = usb_event_callback,
        .user_context = &vision_ctx,
    };
    bsp_usb_vcp_init(&s_vision_usb, &cfg);
    s_usb_vcp_ptr = bsp_usb_vcp_as_base(&s_vision_usb);
}
```

### 6.3 发送数据（异步）

```c
// 建议使用静态/全局缓冲区，而非栈数组
static uint8_t g_tx_buffer[256];

void send_vision_data(void) {
    bool connected;
    bool busy;
    bsp_usb_vcp_get_connected(s_usb_vcp_ptr, &connected);
    bsp_usb_vcp_get_busy(s_usb_vcp_ptr, &busy);
    if (connected && !busy) {
        // 填充数据到 g_tx_buffer
        bsp_usb_vcp_transmit(s_usb_vcp_ptr, g_tx_buffer, data_len, 0);
    }
    // 如果忙，丢弃或放入队列
}
```

### 6.4 接收数据（登记缓冲区）

```c
static uint8_t g_rx_buffer[512];

void usb_event_callback(bsp_event_t event, bsp_status_t status,
                        size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_PENDING) {
        // 有数据待接收，立即读取
        if (bsp_usb_vcp_receive(s_usb_vcp_ptr, g_rx_buffer, sizeof(g_rx_buffer)) == BSP_STATUS_OK) {
            // 数据已复制到 g_rx_buffer，实际长度为 size
            // 将数据交给任务处理（通过信号量或消息队列）
        }
    } else if (event == BSP_EVENT_TRANSMIT_COMPLETE) {
        // 发送完成，可以释放缓冲区
    }
}
```

### 6.5 连接状态与容错

```c
void vision_control_task(void *arg) {
    while (1) {
        bool connected;
        bsp_usb_vcp_get_connected(s_usb_vcp_ptr, &connected);
        if (!connected) {
            // 断开连接：回退到安全控制源（如本地决策）
            set_safety_mode();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## 7. 异步发送详解

USB 虚拟串口的发送是异步的，有以下关键点：

- **缓冲区生命周期**：调用者必须让发送缓冲区保持有效直到发送完成通知。
- **忙状态**：发送忙时返回 `BSP_STATUS_BUSY`，上层应丢弃可丢遥测或进入有界队列，**不能阻塞控制任务等待主机**。
- **超时处理**：`timeout_ms` 用于等待硬件就绪，但不保证传输完成。

**推荐实践**：

```c
// 使用对象内部持久缓冲区，而不是栈数组
typedef struct {
    uint8_t tx_buffer[256];
    bool tx_pending;
} module_vision_comm_t;

static void module_vision_comm_send(module_vision_comm_t *mod, const uint8_t *data, size_t len) {
    if (len > sizeof(mod->tx_buffer)) return;
    memcpy(mod->tx_buffer, data, len);
    if (bsp_usb_vcp_transmit(mod->usb_vcp, mod->tx_buffer, len, 0) != BSP_STATUS_OK) {
        // 发送忙或失败，丢弃或重试
        mod->tx_pending = false;
    } else {
        mod->tx_pending = true;
    }
}
```

## 8. 接收机制

- **登记模式**：调用 `receive` 登记一个缓冲区，平台收到数据后填充并通知。
- **事件通知**：通过 `BSP_EVENT_RECEIVE_PENDING` 事件通知有数据待接收。
- **实际长度**：回调的 `transferred_size` 参数报告实际接收字节数。
- **协议解析**：必须在 Module 层检查长度、帧头、序号和校验。

**推荐实践**：

```c
// 双缓冲接收
static uint8_t rx_buffer_a[512];
static uint8_t rx_buffer_b[512];
static uint8_t *active_rx_buffer = rx_buffer_a;

void usb_event_callback(bsp_event_t event, bsp_status_t status,
                        size_t size, void *ctx) {
    if (event == BSP_EVENT_RECEIVE_PENDING) {
        // 立即登记另一个缓冲区（双缓冲）
        uint8_t *next_buffer = (active_rx_buffer == rx_buffer_a) ? rx_buffer_b : rx_buffer_a;
        bsp_usb_vcp_receive(s_usb_vcp_ptr, next_buffer, sizeof(rx_buffer_a));
        // 处理 active_rx_buffer 中的数据（复制到队列或交给任务）
        process_rx_data(active_rx_buffer, size);
        active_rx_buffer = next_buffer;
    }
}
```

## 9. 连接状态与容错

| 状态               | 行为            | 应用层处理                   |
| :----------------- | :-------------- | :--------------------------- |
| 连接（Configured） | 可正常发送/接收 | 正常通信                     |
| 断开（未配置）     | 发送失败        | 回退到安全控制源             |
| 主机休眠           | 发送可能阻塞    | 超时处理                     |
| 重新枚举           | 短暂断开后恢复  | 自动恢复（需处理缓冲区状态） |

**容错原则**：不能让 USB 状态阻塞云台和底盘周期。断开时视觉/调试 Module 应超时离线，并回退到安全控制源。

## 10. 生命周期与并发

- **初始化顺序**：`bsp_usb_vcp_init` → 使用 → `bsp_device_deinit`。
- **反初始化**：先 `bsp_usb_vcp_abort`（若有异步传输），再 `bsp_device_deinit`。
- **回调替换**：如果回调可能并发发生，更换回调前应禁用中断或使用临界区。
- **并发约束**：发送和接收操作应由单一任务控制，或由外部锁保护。

## 11. 错误码速查

| 错误码                        | 触发场景                                                 |
| :---------------------------- | :------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、数据指针为空、大小为 0                         |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                             |
| `BSP_STATUS_UNSUPPORTED`      | 调用可选函数但驱动未实现（abort/get_connected/get_busy） |
| `BSP_STATUS_BUSY`             | 发送忙（上一次发送未完成）或接收忙（已有待接收数据）     |
| `BSP_STATUS_TIMEOUT`          | 等待硬件就绪超时                                         |
| `BSP_STATUS_IO_ERROR`         | USB 硬件错误                                             |

## 12. 移植要求

平台移植者需实现 `bsp_usb_vcp_driver_ops_t`：

- **`transmit`**（必须）：发送数据，支持超时等待（忙时返回 `BUSY`）。
- **`receive`**（必须）：登记接收缓冲区（若有待接收数据则返回 `BUSY`）。
- **`abort`**（可选）：中止当前发送事务。
- **`get_connected`**（可选）：返回 USB 是否已枚举和配置。
- **`get_busy`**（可选）：返回发送是否正在占用。

**关键注意事项**：

- **发送缓冲区**：USB CDC 通常不会立即复制发送数据，需确保缓冲区在发送完成前有效。
- **接收缓存**：平台端需维护接收缓冲区（或在 USB 回调中直接填充用户缓冲区）。
- **USB 回调**：在 `usbd_cdc_if.c` 的 `CDC_Receive_FS` 等回调中调用 `bsp_usb_vcp_notify`。
- **连接状态**：通过 `hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED` 判断。
- **通用头文件隔离**：平台 Port 不将 `usbd_cdc_if.h` 暴露给通用 BSP 头文件。

## 13. 建议验证测试项

- [ ] 枚举前后连接状态正确（`get_connected` 准确）。
- [ ] 连续发送（忙返回、完成通知）。
- [ ] 主机拔插与重新枚举后自动恢复。
- [ ] 0 字节发送/接收返回 `INVALID_ARGUMENT`。
- [ ] 最大包（64 字节/512 字节）发送正常。
- [ ] 接收分包/粘包（不定长数据）正确处理。
- [ ] 发送缓冲区生命周期（在栈上分配会失效）。
- [ ] 两个逻辑协议共享 USB 时的外部仲裁。
- [ ] 断开连接时 Module 回退到安全控制源。
- [ ] 反初始化后对象拒绝访问。

---

## 一页式接入顺序与可读信息

1. 先初始化 USB Device/CDC 栈，平台再用 CDC handle 和 driver ops 构造 VCP BSP。
2. init 后注册事件 callback，并提交 receive buffer。
3. CDC RX/TX 完成从平台调用 `bsp_usb_vcp_notify()`，不要在 USB ISR 中解析业务协议。
4. 发送前检查 connected/busy；BUSY 时由任务稍后重试，不要阻塞 USB 回调。
5. 停机或重新枚举前 abort/deinit。

可读取 `bsp_usb_vcp_get_connected()`、`bsp_usb_vcp_get_busy()`、接收 buffer 和回调传输长度。connected 只表示链路枚举/端点状态，不代表上位机应用已经准备好协议交互。
