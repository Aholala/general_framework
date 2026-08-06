# BSP FDCAN 通用抽象层与 Classic 适配器 (bsp_fdcan)

## 1. 模块概述

`bsp_fdcan` 提供了与芯片厂商无关的 **CAN FD（CAN with Flexible Data-Rate）** 通用抽象，同时在同一目录下提供了 **Classic CAN 适配器**，使得基于 FDCAN 硬件的设备能够无缝兼容现有的 Classic CAN 上层模块（如 `bsp_can_dispatcher`、电机控制模块等）。

该接口是可选扩展，不是所有 Module 的共同依赖。需要与 F405 bxCAN 共用的
协议应依赖 `bsp_can_t`；只有 CAN FD 长帧、BRS 和协议状态等能力才依赖
`bsp_fdcan_t`。F405 平台不实现本扩展接口。

目录结构：

- `bsp_fdcan.h/.c`：CAN FD 基类、驱动注入、多态接口、帧校验、协议状态查询和事件通知。
- `bsp_fdcan_classic_adapter.h/.c`：将 FDCAN 的 Classic 帧能力适配为 `bsp_can_t *` 接口，使上层模块无需感知底层是 FDCAN 还是 Classic CAN 硬件。

**核心能力**：

- 支持 Classic CAN、CAN FD 无 BRS、CAN FD 带 BRS 三种帧格式。
- 支持 0~64 字节可变数据长度（有效长度：0~8、12、16、20、24、32、48、64）。
- 标准/扩展标识符、数据/远程帧。
- FIFO0/FIFO1 接收选择。
- 硬件过滤器配置（掩码+ID）。
- 协议状态查询（Bus-Off、Error Passive、Warning、错误计数）。
- 发送邮箱余量查询。
- Classic 适配器将 FDCAN Classic 帧桥接到 `bsp_can_t` 多态接口。

## 2. 设计边界

| **模块负责**                                   | **模块不负责**                                    |
| :--------------------------------------------- | :------------------------------------------------ |
| CAN FD 帧结构、格式、数据长度的抽象与校验      | CAN FD 位时序、采样点、数据段波特率配置           |
| 标准/扩展 ID 过滤器的统一配置接口              | 消息 RAM 分配和管理                               |
| 发送/接收（阻塞）、协议状态、邮箱余量查询      | GPIO 引脚复用、收发器 STBY 引脚控制               |
| FDCAN → Classic CAN 的帧格式转换与适配         | 厂商 HAL 库头文件包含（如 stm32h7xx_hal_fdcan.h） |
| 事件通知（发送完成、接收完成、错误）的中断转发 | 中断向量表配置和 NVIC 优先级设置                  |
| 多实例管理（多个 FDCAN 控制器）                | 具体电机协议、裁判系统协议或应用层数据解析        |

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_fdcan_t                         (FDCAN 基类：增加 callback、user_context)
    └── bsp_fdcan_device_t              (FDCAN 具体设备：持有 driver_ops)

bsp_can_t
└── bsp_can_device_t                    (Classic CAN 基类)
    └── bsp_fdcan_classic_adapter_t     (适配器：组合一个 bsp_fdcan_t *)
```

- **`bsp_fdcan_t`**：应用层使用的 FDCAN 基类指针，包含事件回调和用户上下文。
- **`bsp_fdcan_device_t`**：实际分配的对象，保存底层驱动操作表。
- **`bsp_fdcan_classic_adapter_t`**：适配器对象，本身是一个 `bsp_can_device_t` 派生类，内部组合一个 `bsp_fdcan_t *`，不拥有该指针（生命周期由调用者管理）。

## 4. 核心类型

### 4.1 FDCAN 帧结构 (`bsp_fdcan_frame_t`)

```c
typedef struct {
    uint32_t identifier;               // 11 或 29 位 ID
    bsp_can_id_type_t id_type;         // BSP_CAN_ID_STANDARD / EXTENDED
    bsp_can_frame_type_t frame_type;   // BSP_CAN_FRAME_DATA / REMOTE
    bsp_fdcan_format_t format;         // 帧格式
    uint8_t data_length;               // 有效字节数
    uint8_t data[64];                  // 数据负载
} bsp_fdcan_frame_t;
```

- **`format`**：
  - `BSP_FDCAN_FORMAT_CLASSIC`：Classic CAN 帧，最大 8 字节。
  - `BSP_FDCAN_FORMAT_FD_NO_BRS`：CAN FD 帧，数据段速率与仲裁段相同。
  - `BSP_FDCAN_FORMAT_FD_BRS`：CAN FD 帧，数据段切换到更高波特率（BRS 使能）。
- **`data_length`**：有效长度，必须为 0~8、12、16、20、24、32、48、64 之一。

### 4.2 协议状态结构 (`bsp_fdcan_protocol_status_t`)

```c
typedef struct {
    bool is_bus_off;                   // Bus-Off 状态
    bool is_error_passive;             // Error Passive 状态
    bool has_warning;                  // 是否达到错误警告阈值
    uint8_t transmit_error_count;      // 发送错误计数
    uint8_t receive_error_count;       // 接收错误计数
    uint32_t last_error_code;          // 平台相关错误码
} bsp_fdcan_protocol_status_t;
```

### 4.3 底层驱动操作表 (`bsp_fdcan_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*start)(void *handle);
    bsp_status_t (*stop)(void *handle);
    bsp_status_t (*configure_filter)(void *handle, const bsp_can_filter_t *);
    bsp_status_t (*transmit)(void *handle, const bsp_fdcan_frame_t *, uint32_t);
    bsp_status_t (*receive)(void *handle, bsp_can_receive_fifo_t, bsp_fdcan_frame_t *);
    bsp_status_t (*get_protocol_status)(const void *handle, bsp_fdcan_protocol_status_t *);
    bsp_status_t (*get_transmit_free_level)(const void *handle, uint32_t *);
} bsp_fdcan_driver_ops_t;
```

**必须实现的函数**：`start`、`stop`、`configure_filter`、`transmit`、`receive`。`init`/`deinit` 可选，`get_protocol_status` 和 `get_transmit_free_level` 若硬件不支持可返回 `BSP_STATUS_UNSUPPORTED`。

## 5. 初始化与使用流程

### 5.1 FDCAN 设备初始化

```c
static bsp_fdcan_device_t s_fdcan_dev;
static bsp_fdcan_t *s_fdcan_ptr = NULL;

static const bsp_fdcan_config_t s_fdcan_cfg = {
    .device_handle = &hfdcan1,                 // 平台句柄（如 FDCAN_HandleTypeDef*）
    .driver_ops    = &platform_fdcan_driver_ops,
    .callback      = fdcan_event_callback,
    .user_context  = NULL,
};

void board_fdcan_init(void) {
    bsp_fdcan_init(&s_fdcan_dev, &s_fdcan_cfg);
    s_fdcan_ptr = bsp_fdcan_as_base(&s_fdcan_dev);
    bsp_fdcan_start(s_fdcan_ptr);
}
```

### 5.2 发送 CAN FD 帧

```c
bsp_fdcan_frame_t tx_frame = {
    .identifier   = 0x120,
    .id_type      = BSP_CAN_ID_STANDARD,
    .frame_type   = BSP_CAN_FRAME_DATA,
    .format       = BSP_FDCAN_FORMAT_FD_BRS,
    .data_length  = 16,
    .data         = { ... },
};
if (bsp_fdcan_transmit(s_fdcan_ptr, &tx_frame, 100) == BSP_STATUS_OK) {
    // 发送成功
}
```

### 5.3 接收 FDCAN 帧

```c
bsp_fdcan_frame_t rx_frame;
if (bsp_fdcan_receive(s_fdcan_ptr, BSP_CAN_RX_FIFO_0, &rx_frame) == BSP_STATUS_OK) {
    // 处理 rx_frame
}
```

### 5.4 配置硬件过滤器

```c
bsp_can_filter_t filter = {
    .identifier   = 0x200,
    .mask         = 0x7FF,
    .id_type      = BSP_CAN_ID_STANDARD,
    .receive_fifo = BSP_CAN_RX_FIFO_0,
    .filter_index = 0,                     // 平台端解释
};
bsp_fdcan_configure_filter(s_fdcan_ptr, &filter);
```

### 5.5 查询协议状态

```c
bsp_fdcan_protocol_status_t status;
if (bsp_fdcan_get_protocol_status(s_fdcan_ptr, &status) == BSP_STATUS_OK) {
    if (status.is_bus_off) {
        // 执行 Bus-Off 恢复流程
    }
}
```

## 6. Classic CAN 适配器

适配器用于将 FDCAN 硬件降级为 8 字节 Classic CAN，使上层模块（如电机控制、裁判系统、`bsp_can_dispatcher`）无需感知底层是 FDCAN 还是 Classic CAN。

### 6.1 适配器初始化

```c
static bsp_fdcan_classic_adapter_t s_classic_adapter;

static const bsp_fdcan_classic_adapter_config_t adapter_cfg = {
    .fdcan        = s_fdcan_ptr,           // 已初始化的 FDCAN 对象
    .callback     = NULL,                  // 可选用户回调
    .user_context = NULL,
};

void board_classic_adapter_init(void) {
    bsp_fdcan_classic_adapter_init(&s_classic_adapter, &adapter_cfg);
    bsp_can_t *can_ptr = bsp_fdcan_classic_adapter_as_can(&s_classic_adapter);
    // 现在 can_ptr 可传入 bsp_can_dispatcher 或电机模块
}
```

### 6.2 适配器行为约束

- 只接受 `BSP_FDCAN_FORMAT_CLASSIC` 格式的帧。
- `data_length` 必须 ≤ 8。
- 发送时自动将 `bsp_can_frame_t` 转换为 `bsp_fdcan_frame_t`，格式固定为 Classic。
- 接收时若收到 FD 帧（非 Classic）或长度 > 8，返回 `BSP_STATUS_UNSUPPORTED`。
- 过滤器和 FIFO 配置直接转发到 FDCAN 底层。

## 7. 中断与事件通知

平台端在 CAN 中断中调用：

```c
bsp_fdcan_notify(fdcan_ptr, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, 1U);
```

- 事件类型包括：`TRANSMIT_COMPLETE`、`RECEIVE_COMPLETE`、`TRANSFER_COMPLETE`、`RECEIVE_PENDING`、`ERROR`。
- 回调在 ISR 上下文中执行，必须快速返回，不可阻塞。

## 8. 协议状态与故障恢复

`bsp_fdcan_get_protocol_status` 提供 Bus-Off、Error Passive、Warning 及错误计数信息。通用层只报告状态，**不擅自重置硬件**。上层健康管理器应执行：

1. 立即禁止相关执行器输出（设置安全状态）。
2. 记录故障和时间戳。
3. 停止总线（`bsp_fdcan_stop`）。
4. 重新初始化硬件（调用 `driver_ops->init` 或设备复位）。
5. 重新配置过滤器。
6. 恢复关键设备反馈。
7. 解除故障锁存并重新使能（`bsp_fdcan_start`）。

## 9. API 参考

| 函数                                | 说明                        | 返回值                           |
| :---------------------------------- | :-------------------------- | :------------------------------- |
| `bsp_fdcan_init`                    | 初始化 FDCAN 设备           | `OK` / `INVALID_ARGUMENT`        |
| `bsp_fdcan_as_base`                 | 向上转型，获取基类指针      | 基类指针或 `NULL`                |
| `bsp_fdcan_set_callback`            | 设置事件回调                | `OK` / `NOT_INITIALIZED`         |
| `bsp_fdcan_start`                   | 启动 CAN FD 总线            | 状态码                           |
| `bsp_fdcan_stop`                    | 停止 CAN FD 总线            | 状态码                           |
| `bsp_fdcan_configure_filter`        | 配置硬件过滤器（ID+掩码）   | `OK` / `INVALID_ARGUMENT`        |
| `bsp_fdcan_transmit`                | 发送 FDCAN 帧（阻塞）       | `OK` / `OUT_OF_RANGE` / 平台错误 |
| `bsp_fdcan_receive`                 | 从 FIFO 接收一帧            | `OK` / `IO_ERROR`                |
| `bsp_fdcan_get_protocol_status`     | 获取协议状态                | `OK` / `UNSUPPORTED`             |
| `bsp_fdcan_get_transmit_free_level` | 获取发送邮箱空闲数          | `OK` / `UNSUPPORTED`             |
| `bsp_fdcan_notify`                  | 事件通知（由平台 ISR 调用） | 无返回值                         |
| `bsp_fdcan_classic_adapter_init`    | 初始化 Classic 适配器       | `OK` / `INVALID_ARGUMENT`        |
| `bsp_fdcan_classic_adapter_as_can`  | 获取 `bsp_can_t *` 指针     | `bsp_can_t *` 或 `NULL`          |

## 10. 生命周期与并发约束

- **初始化顺序**：`bsp_fdcan_init` → `bsp_fdcan_start`。
- **适配器依赖**：适配器依赖的 `bsp_fdcan_t` 必须在整个适配器生命周期内保持有效，不可先销毁 FDCAN 对象。
- **状态冲突**：发送、接收和状态查询可并发执行，但需注意硬件资源共享（由平台驱动处理）。
- **缓冲区生命周期**：`bsp_fdcan_transmit` 的帧指针在函数返回前必须有效。若平台驱动异步发送，平台端必须复制数据或使用 DMA 安全机制。
- **回调约束**：用户回调在 ISR 上下文中执行，必须非阻塞（仅释放信号量或发送消息）。

## 11. 移植要求

平台移植者需实现 `bsp_fdcan_driver_ops_t`：

- **`init`**（可选）：配置消息 RAM、FIFO 大小、过滤器数量、位时序等。
- **`start`**：使能 CAN FD 通信（进入 Normal 模式）。
- **`stop`**：禁用通信。
- **`configure_filter`**：将 `bsp_can_filter_t` 转换为硬件过滤器配置。
- **`transmit`**：将帧写入发送邮箱，支持超时机制。
- **`receive`**：从指定 FIFO 读取一帧，若 FIFO 空返回 `BSP_STATUS_NO_DATA`。
- **`get_protocol_status`**：读取协议状态寄存器（Bus-Off、错误计数等）。
- **`get_transmit_free_level`**：返回可用发送邮箱数。

**关键注意事项**：

- 数据长度与硬件 DLC 编码的转换必须在驱动层完成。
- FDF 和 BRS 标志需根据 `format` 正确设置。
- 标准帧与扩展帧的过滤器配置需分别处理。
- 若使用 D-Cache，需确保消息缓冲区的一致性维护。
- 适配器的 `device_handle` 指向适配器自身，而非 FDCAN 句柄。

## 12. 建议验证测试项

- [ ] Classic 帧（0~8 字节）收发正常。
- [ ] FD 帧（12、16、20、24、32、48、64 字节）收发正常。
- [ ] BRS 使能/禁用的 FD 帧收发正常。
- [ ] 标准/扩展 ID、数据/远程帧的发送接收。
- [ ] 非法数据长度（如 9、10）被拒绝。
- [ ] FIFO0/FIFO1 独立接收。
- [ ] 过滤器精确匹配与掩码匹配。
- [ ] 发送邮箱满时超时返回 `TIMEOUT`。
- [ ] Bus-Off、Error Passive、Warning 状态正确上报。
- [ ] Classic 适配器正确转换双向帧。
- [ ] 适配器收到 FD 长帧返回 `UNSUPPORTED`。
- [ ] 两个 FDCAN 实例/两个适配器互不干扰。
- [ ] 停止、反初始化后接口拒绝访问。
- [ ] 故障恢复流程（Bus-Off → 恢复）正确执行。

---

## 一页式接入顺序与可读信息

1. 平台配置 arbitration/data phase 时序和 message RAM，注入 FDCAN driver ops。
2. init 后配置 filter、注册 callback，再 start。
3. 使用 `bsp_fdcan_frame_t` 发送/接收；明确 Classic/FD、BRS 和有效数据长度。
4. ISR 经 `bsp_fdcan_notify()` 路由，任务读取并处理帧。
5. 旧模块需要 Classic CAN 接口时创建 `bsp_fdcan_classic_adapter_t`，并只通过 `as_can()` 使用。

| 可读取结构体                  | 主要信息                                       |
| ----------------------------- | ---------------------------------------------- |
| `bsp_fdcan_frame_t`           | ID、帧格式、位速率切换、长度和最多 64 字节数据 |
| `bsp_fdcan_protocol_status_t` | 总线状态、错误状态和错误计数                   |
| TX 空闲量                     | `bsp_fdcan_get_transmit_free_level()`          |

Classic 适配器只接受 Classic 帧语义，不能借适配器偷偷发送 64 字节 FD 帧。
