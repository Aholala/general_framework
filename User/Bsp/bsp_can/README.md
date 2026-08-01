# BSP CAN 通用抽象层与帧分发器 (bsp_can)

## 1. 模块概述

`bsp_can` 是一套 **与芯片厂商无关** 的 Classic CAN（经典 CAN）通用抽象层，并附带一个轻量级的 **CAN 帧分发器 (Dispatcher)**。该模块采用面向对象的设计，将底层硬件差异完全封装在平台驱动中，为上层应用（如电机控制、板间通信、裁判系统解析）提供统一的操作接口。

目录按功能组织如下：

- `bsp_can.h/.c`：CAN 基类、多态接口、事件通知及生命周期管理。
- `bsp_can_dispatcher.h/.c`：接收帧路由表管理、任务上下文分发机制及运行统计。

## 2. 设计边界

模块严格遵循“高内聚、低耦合”原则，明确划分职责：

| **模块负责 (Core Responsibilities)**         | **模块不负责 (Non-Responsibilities)**              |
| :------------------------------------------- | :------------------------------------------------- |
| 标准帧 (11-bit) 与扩展帧 (29-bit) 的数据模型 | CAN 总线位时序、波特率计算与配置                   |
| 数据帧与远程帧的区分与处理                   | MCU 引脚复用 (Pin Mux) 与收发器 (Transceiver) 使能 |
| 硬件掩码过滤器 (Filter) 的抽象配置           | 具体厂商 HAL 库寄存器操作（如 STM32 HAL/FDCAN）    |
| 阻塞式发送、接收及发送邮箱余量查询           | 具体的电机协议、裁判系统协议或应用层数据解析       |
| 中断通知到任务上下文的解耦与帧路由分发       | 在中断服务程序 (ISR) 中执行耗时或阻塞的用户回调    |
| 多协议共享单条 CAN 总线时的帧匹配与回调      | 动态内存分配（所有存储由调用者静态分配）           |

## 3. 对象模型与继承关系

模块采用分层继承结构，所有操作均基于基类指针进行多态调用：

```text
bsp_device_t               (基类：包含 vptr、device_handle、is_initialized)
└── bsp_can_t              (CAN 基类：增加 event_callback、user_context)
    └── bsp_can_device_t   (具体设备类：持有 bsp_can_driver_ops_t)
```

- **`bsp_can_t`**：应用层依赖的抽象基类，所有 API（如发送、接收）均接收此指针，确保模块可替换性。
- **`bsp_can_device_t`**：实际分配的对象，用于保存底层驱动操作表（`driver_ops`）。
- **`bsp_can_dispatcher_t`**：独立的分发器对象，通过组合方式关联 `bsp_can_t`，管理帧路由表。

## 4. 核心数据类型

### 4.1 CAN 帧结构 (`bsp_can_frame_t`)

- `identifier`：11 位（标准）或 29 位（扩展）标识符。
- `id_type`：`BSP_CAN_ID_STANDARD` 或 `BSP_CAN_ID_EXTENDED`。
- `frame_type`：`BSP_CAN_FRAME_DATA`（数据帧）或 `BSP_CAN_FRAME_REMOTE`（远程帧）。
- `data_length`：有效数据长度，范围 0～8。
- `data[8]`：数据负载。

### 4.2 硬件过滤器配置 (`bsp_can_filter_t`)

用于底层硬件匹配：

- `identifier` / `mask`：匹配值与掩码（掩码位为 1 表示需精确匹配）。
- `id_type`：过滤标准帧或扩展帧。
- `receive_fifo`：匹配成功后帧进入 `BSP_CAN_RX_FIFO_0` 或 `FIFO_1`。
- `filter_index`：由平台端解释的硬件过滤器槽位（如 STM32 的 Filter Bank 索引）。

### 4.3 分发器路由表项 (`bsp_can_route_t`)

- `callback`：匹配成功后调用的用户函数（在任务上下文执行）。
- `is_enabled`：可动态启用或禁用该路由。

> **匹配规则**：`(received_identifier & mask) == (route_identifier & mask)`

## 5. 初始化与启动流程

### 5.1 板级设备初始化

```c
// 1. 静态分配设备对象
static bsp_can_device_t s_can_dev;

// 2. 配置初始化参数
static const bsp_can_config_t s_can_cfg = {
    .device_handle = &hfdcan1,                    // 平台驱动句柄
    .driver_ops    = &stm32_fdcan_driver_ops,     // 平台驱动函数表
    .callback      = NULL,                        // 默认不使用基类回调
    .user_context  = NULL,
};

// 3. 初始化并启动
void board_can_init(void) {
    bsp_can_init(&s_can_dev, &s_can_cfg);
    bsp_can_t *can_ptr = bsp_can_as_base(&s_can_dev);
    bsp_can_start(can_ptr);
}
```

### 5.2 配置硬件过滤器（可选）

```c
bsp_can_filter_t filter = {
    .identifier   = 0x200,
    .mask         = 0x7FF,       // 完全匹配 ID 0x200
    .id_type      = BSP_CAN_ID_STANDARD,
    .receive_fifo = BSP_CAN_RX_FIFO_0,
    .filter_index = 0,
};
bsp_can_configure_filter(can_ptr, &filter);
```

## 6. 发送与接收接口

### 6.1 阻塞发送

```c
bsp_can_frame_t tx_frame = {
    .identifier = 0x100,
    .id_type = BSP_CAN_ID_STANDARD,
    .frame_type = BSP_CAN_FRAME_DATA,
    .data_length = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
};
if (bsp_can_transmit(can_ptr, &tx_frame, 100) == BSP_STATUS_OK) {
    // 发送成功，超时时间 100ms
}
```

### 6.2 手动轮询接收（非分发器模式）

```c
bsp_can_frame_t rx_frame;
if (bsp_can_receive(can_ptr, BSP_CAN_RX_FIFO_0, &rx_frame) == BSP_STATUS_OK) {
    // 处理接收到的帧
}
```

### 6.3 查询发送邮箱空闲数

用于流量控制，防止发送缓冲区溢出：

```c
uint32_t free_level;
if (bsp_can_get_transmit_free_level(can_ptr, &free_level) == BSP_STATUS_OK) {
    // free_level 表示当前可用的发送邮箱数量
}
```

## 7. 帧分发器 (Dispatcher) 使用指南

分发器用于解决多协议共享总线的问题，将中断触发与协议回调分离。

### 7.1 初始化分发器与路由表

```c
// 1. 静态分配路由存储（最大支持 16 条路由）
static bsp_can_route_t s_route_storage[16];
static bsp_can_dispatcher_t s_dispatcher;

// 2. 配置分发器
static const bsp_can_dispatcher_config_t disp_cfg = {
    .can = can_ptr,                              // 关联的 CAN 基类
    .receive_fifo = BSP_CAN_RX_FIFO_0,
    .route_storage = s_route_storage,
    .route_capacity = 16,                        // 最大路由数
    .maximum_frames_per_process = 8,             // 单次任务最多处理 8 帧
};

// 3. 初始化并添加路由
void board_dispatcher_init(void) {
    bsp_can_dispatcher_init(&s_dispatcher, &disp_cfg);

    // 添加电机反馈路由 (完全匹配 0x201)
    bsp_can_dispatcher_add_route(&s_dispatcher, 0x201, 0x7FF,
                                 BSP_CAN_ID_STANDARD,
                                 motor_feedback_callback, &motor_ctx, NULL);

    // 添加裁判系统路由 (匹配 0x300 ~ 0x3FF)
    bsp_can_dispatcher_add_route(&s_dispatcher, 0x300, 0x700,
                                 BSP_CAN_ID_STANDARD,
                                 referee_data_callback, NULL, NULL);
}
```

### 7.2 中断与任务协作

**步骤 A：在 CAN 中断 ISR 中调用通知函数**

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    // 仅设置事件标志，不执行具体协议处理
    bsp_can_notify(g_can_ptr, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, 1U);
}
```

**步骤 B：分发器自动捕获事件**
分发器内部注册的回调只做一件事：将 `receive_pending` 置为 `true`。

**步骤 C：在 RTOS 任务或主循环中处理**

```c
void can_process_task(void *arg) {
    while (1) {
        if (bsp_can_dispatcher_has_pending_receive(&s_dispatcher)) {
            size_t processed = 0;
            // 这里会依次读取 FIFO 中的帧，匹配路由并调用用户回调
            bsp_can_dispatcher_process(&s_dispatcher, &processed);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 或其他同步机制
    }
}
```

### 7.3 路由运行时管理

- **启用/禁用**：`bsp_can_dispatcher_set_route_enabled(dispatcher, idx, false)`
- **移除路由**：`bsp_can_dispatcher_remove_route(dispatcher, idx)`
- **清空所有**：`bsp_can_dispatcher_clear_routes(dispatcher)`
- **注意**：路由修改操作不能在 `process` 执行期间进行，否则返回 `BSP_STATUS_BUSY`。

## 8. 统计与诊断

分发器内置运行计数器，便于监控总线健康：

- `received_frame_count`：累计成功读取的帧总数。
- `unmatched_frame_count`：没有匹配到任何路由的帧数（用于检测未知 ID 攻击或配置错误）。
- `receive_error_count`：底层读取失败次数（如 FIFO 溢出）。

建议在调试或监控任务中周期上报这些计数。

## 9. 生命周期与并发约束

| **阶段**   | **操作**                                 | **注意事项**          |
| :--------- | :--------------------------------------- | :-------------------- |
| 初始化     | `bsp_can_init` → `bsp_can_start`         | 必须先于分发器初始化  |
| 分发器启动 | `bsp_can_dispatcher_init`                | 依赖 CAN 对象已初始化 |
| 运行时     | `bsp_can_transmit` / `process`           | 发送和接收处理可并发  |
| 停止       | `bsp_can_stop`                           | 停止硬件接收          |
| 反初始化   | `bsp_can_dispatcher_deinit` → (设备销毁) | 先注销回调再停止硬件  |

**严格约束**：

- 所有存储（CAN 对象、路由表、用户上下文）必须在整个生命周期内保持有效，不得释放或出作用域。
- **用户回调**（`bsp_can_frame_callback_t`）**严禁阻塞**（不可包含 `vTaskDelay`、`printf` 或信号量获取），且**严禁在回调中增删路由**，以免引发重入问题。
- `bsp_can_dispatcher_process` 不可重入，内部已通过 `is_processing` 标志保护。

## 10. 平台移植要求

若需适配新的 MCU（如 NXP S32K、TI TMS320），平台端需实现 `bsp_can_driver_ops_t` 结构体：

```c
typedef struct {
    bsp_status_t (*init)(void *);                    // 可选
    bsp_status_t (*deinit)(void *);                  // 可选
    bsp_status_t (*start)(void *);                   // 必须：进入 Normal 模式
    bsp_status_t (*stop)(void *);                    // 必须：退出通信模式
    bsp_status_t (*configure_filter)(void *, const bsp_can_filter_t *); // 必须
    bsp_status_t (*transmit)(void *, const bsp_can_frame_t *, uint32_t); // 必须
    bsp_status_t (*receive)(void *, bsp_can_receive_fifo_t, bsp_can_frame_t *); // 必须
    bsp_status_t (*get_tx_free_level)(const void *, uint32_t *); // 可选
} bsp_can_driver_ops_t;
```

**实现注意事项**：

- `transmit` 必须支持超时机制，超时返回 `BSP_STATUS_TIMEOUT`。
- `receive` 若 FIFO 为空，应返回 `BSP_STATUS_NO_DATA`（或 `BSP_STATUS_AGAIN`，由平台统一定义）。
- 在 CAN 接收中断中必须调用 `bsp_can_notify`，以激活分发器的事件回调。

## 11. 建议验证测试项

为确保模块稳定运行，建议在集成时覆盖以下测试：

- [ ] 空指针、未初始化对象调用 API 返回 `BSP_STATUS_INVALID_ARGUMENT` / `NOT_INITIALIZED`。
- [ ] 标准帧 (0x000~0x7FF) 与扩展帧 (0x00000000~0x1FFFFFFF) 边界值测试。
- [ ] 数据长度 0~8 字节的发送与接收，远程帧（`data_length=0`）处理。
- [ ] 硬件过滤器精确匹配与掩码匹配（如匹配 0x100~0x1FF）。
- [ ] 分发器路由匹配：精确匹配、掩码匹配、路由禁用（`is_enabled=false`）。
- [ ] 未匹配帧统计计数（`unmatched_frame_count`）递增验证。
- [ ] 单次 `process` 处理帧数上限（`maximum_frames_per_process`）的分批处理验证。
- [ ] `process` 处理期间收到中断不会导致重入（检查 `is_processing` 保护）。
- [ ] 多实例（两个独立 CAN 总线）独立工作，路由表互不干扰。
- [ ] 停止与反初始化后，所有接口拒绝访问（返回 `NOT_INITIALIZED`）。

---

## 一页式接入顺序与可读信息

1. 平台配置 CAN 时序和引脚，注入 `bsp_can_driver_ops_t` 后 init。
2. start 前配置硬件 filter；注册 callback 或初始化 dispatcher 路由表。
3. 调用 start；发送使用 `bsp_can_frame_t`，阻塞等待由 timeout 控制。
4. RX ISR 只 notify/置 pending；任务中调用 `bsp_can_dispatcher_process()` 分发给模块。
5. 停机先停止上层发送，再 `stop → dispatcher_deinit → bsp_device_deinit`。

| 可读取结构体       | 主要信息                                        |
| ------------------ | ----------------------------------------------- |
| `bsp_can_frame_t`  | ID、标准/扩展帧、数据/远程帧、DLC 和 8 字节数据 |
| `bsp_can_filter_t` | ID、掩码、类型和目标 FIFO                       |
| `bsp_can_route_t`  | 路由匹配条件、处理器、上下文和启用状态          |
| TX 空闲量          | `bsp_can_get_transmit_free_level()`             |
| dispatcher pending | `bsp_can_dispatcher_has_pending_receive()`      |

模块处理器应在任务上下文解析帧；不要在 CAN ISR 中运行电机控制或协议状态机。
