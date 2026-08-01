# 通用蓝牙串口模块 (module_bluetooth) —— 完整使用指南

## 1. 模块概述

`module_bluetooth` 是基于 `bsp_usart_t` 的通用蓝牙串口模块，负责异步接收、双缓冲转交、在线超时、原始数据发送和 AT 命令发送。它不绑定 HC-05、HC-06 或某个厂商协议，适用于任意 UART 接口的蓝牙透传模块。

**核心功能**：

- DMA 空闲中断异步接收，ISR 仅拷贝数据，不解析。
- 双缓冲机制：`receive_buffer`（DMA）→ `processing_buffer`（ISR）→ 任务回调。
- 在线超时检测：收到数据重置计时，超时自动离线。
- 发送二进制数据和 AT 命令。
- 接收覆盖计数和 DMA 重启错误计数。
- 设备基类接口（`module_device_t`）。

**设计哲学**：

- **ISR 最小化**：中断中仅拷贝数据并重启 DMA，所有回调在任务上下文执行。
- **非阻塞接收**：使用 DMA 空闲中断，不丢失数据。
- **安全优先**：离线超时提供链路状态指示，供上层决策。

## 2. 设计边界

| **模块负责**                 | **模块不负责**                                   |
| :--------------------------- | :----------------------------------------------- |
| USART 异步接收管理（双缓冲） | 蓝牙芯片的具体 AT 指令集解析                     |
| 在线超时检测                 | 蓝牙配对、名称、PIN 码配置（由 AT 命令自行完成） |
| 原始数据发送和 AT 命令发送   | 协议解析（由上层或用户回调处理）                 |
| 设备生命周期管理             | 串口波特率、校验位等硬件参数配置                 |
| 状态统计（覆盖、重启错误）   | 连接加密、认证等高级功能                         |

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_bluetooth_t             (蓝牙对象：USART、缓冲区、回调、状态)
```

通过 `module_bluetooth_as_device`（未显式提供，但可通过 `&me->super` 获取）可接入 `module_device` 框架进行统一调度。

## 4. 核心类型

### 4.1 配置结构 (`module_bluetooth_config_t`)

```c
typedef struct {
    bsp_usart_t *usart;                          // USART BSP 基类
    uint8_t *receive_buffer;                     // DMA 接收缓冲区
    size_t receive_capacity;                     // 接收缓冲区大小
    uint8_t *processing_buffer;                  // 任务处理缓冲区
    size_t processing_capacity;                  // 处理缓冲区大小（>= receive_capacity）
    uint32_t transmit_timeout_ms;                // 发送超时
    uint32_t receive_timeout_ms;                 // 接收超时（用于 USART 接收）
    uint32_t offline_timeout_ms;                 // 离线超时
    bsp_transfer_mode_t receive_mode;            // 接收模式（仅 INTERRUPT 或 DMA）
    const char *logical_name;                    // 设备逻辑名称
    uint32_t registration_key;                   // 注册键值
    module_bluetooth_receive_callback_t receive_callback; // 接收回调（可为 NULL）
    void *user_context;                          // 回调用户上下文
} module_bluetooth_config_t;
```

### 4.2 接收回调类型

```c
typedef void (*module_bluetooth_receive_callback_t)(const uint8_t *receive_data,
                                                    size_t data_size, void *user_context);
```

回调在 `module_bluetooth_update` 中执行（任务上下文），数据在回调期间有效，如需长期保存需自行复制。

## 5. API 参考

| 函数                            | 说明                                 | 返回值                    |
| :------------------------------ | :----------------------------------- | :------------------------ |
| `module_bluetooth_init`         | 初始化蓝牙模块                       | `OK` / `INVALID_ARGUMENT` |
| `module_bluetooth_start`        | 启动模块（注册回调 + 启动 DMA 接收） | `OK` / `TRANSPORT_ERROR`  |
| `module_bluetooth_stop`         | 停止模块（中止接收 + 注销回调）      | `OK` / `TRANSPORT_ERROR`  |
| `module_bluetooth_transmit`     | 发送原始二进制数据                   | `OK` / `TRANSPORT_ERROR`  |
| `module_bluetooth_send_command` | 发送 AT 命令（阻塞）                 | `OK` / `TRANSPORT_ERROR`  |
| `module_bluetooth_update`       | 周期更新（处理接收 + 超时更新）      | `OK` / `NOT_STARTED`      |
| `module_bluetooth_is_online`    | 查询在线状态                         | `true` / `false`          |

## 6. 使用示例

### 6.1 初始化与启动

```c
static module_bluetooth_t s_bluetooth;
static uint8_t rx_buffer[128];
static uint8_t proc_buffer[128];

void bluetooth_receive_callback(const uint8_t *data, size_t size, void *ctx) {
    // 处理接收数据（任务上下文）
    // 数据仅回调期间有效，需要长期保存需复制
}

const module_bluetooth_config_t cfg = {
    .usart = board_usart_ptr,
    .receive_buffer = rx_buffer,
    .receive_capacity = sizeof(rx_buffer),
    .processing_buffer = proc_buffer,
    .processing_capacity = sizeof(proc_buffer),
    .transmit_timeout_ms = 100,
    .receive_timeout_ms = 100,
    .offline_timeout_ms = 500,
    .receive_mode = BSP_TRANSFER_MODE_DMA,
    .logical_name = "bluetooth",
    .registration_key = 0,
    .receive_callback = bluetooth_receive_callback,
    .user_context = NULL,
};

module_bluetooth_init(&s_bluetooth, &cfg);
module_bluetooth_start(&s_bluetooth);
```

### 6.2 周期更新

```c
void main_loop(void) {
    uint32_t dt_ms = get_delta_time_ms();
    module_bluetooth_update(&s_bluetooth, dt_ms);

    if (module_bluetooth_is_online(&s_bluetooth)) {
        // 蓝牙在线
    }
}
```

### 6.3 发送 AT 命令

```c
// 发送 AT 命令（阻塞）
module_bluetooth_send_command(&s_bluetooth, "AT+NAME=MyDevice\r\n");

// 发送二进制数据
uint8_t data[] = {0x01, 0x02, 0x03};
module_bluetooth_transmit(&s_bluetooth, data, sizeof(data), BSP_TRANSFER_MODE_DMA);
```

## 7. 接收路径

```text
USART DMA 空闲中断 (ISR)
  -> 拷贝 receive_buffer 到 processing_buffer
  -> 立即重启 DMA
  -> 设置 is_receive_pending = true

任务: module_bluetooth_update
  -> 调用用户回调（任务上下文）
  -> 重置超时计时
  -> 清除 pending 标志
```

## 8. 在线状态

- 成功接收数据后，`is_online = true`，并重置超时计时。
- 超过 `offline_timeout_ms` 未收到数据，`is_online = false`。
- `module_bluetooth_is_online` 仅表示链路最近有活动，不代表远端协议或控制权限有效。

## 9. 统计信息

对象内部维护以下计数（位于 `module_bluetooth_t`）：

- `receive_overrun_count`：接收覆盖次数（上一帧未处理时新帧到达）。
- `receive_restart_error_count`：重启 DMA 接收失败次数。

可在调试或监控任务中读取这些值。

## 10. 生命周期与并发

- **初始化**：`module_bluetooth_init` → `module_bluetooth_start`
- **停止**：`module_bluetooth_stop`（中止接收 + 注销回调）
- **并发约束**：发送与更新最好由同一通信任务管理；多任务发送需要外部队列。
- **回调**：运行在任务上下文，但仍应保持有界执行时间（非阻塞）。

## 11. 错误码速查

| 状态码             | 触发场景                                                                         |
| :----------------- | :------------------------------------------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、缓冲区容量为 0、处理缓冲区小于接收缓冲区、接收模式为阻塞、离线超时为 0 |
| `NOT_INITIALIZED`  | 对象未初始化                                                                     |
| `NOT_STARTED`      | 未调用 `start`                                                                   |
| `TRANSPORT_ERROR`  | USART 回调注册失败、DMA 启停失败、发送失败                                       |
| `OFFLINE`          | 设备离线（超时未收到数据）                                                       |

## 12. 集成约束

- `receive_mode` 不支持阻塞模式，必须使用 `INTERRUPT` 或 `DMA`。
- `processing_capacity` 必须 ≥ `receive_capacity`。
- 一个 `bsp_usart_t` 对象只能保存一个事件回调，蓝牙使用的串口不应被其他模块覆盖回调。
- 调用方负责 AT 命令的结束符（如 `\r\n`）和模块所需波特率。

## 13. 建议验证测试项

- [ ] DMA 空闲接收和立即重启
- [ ] 连续帧、粘包和分包由上层协议处理（模块透传原始数据）
- [ ] 处理未完成时的 overrun 计数递增
- [ ] 接收重启失败计数递增
- [ ] 在线/离线超时：收到数据后 `is_online = true`，超时后变为 `false`
- [ ] AT 命令和二进制发送
- [ ] 停止后拒绝操作（`NOT_STARTED`）
- [ ] 回调在任务上下文正确执行

---

**总结**：`module_bluetooth` 提供了通用的蓝牙串口透传模块，通过 DMA 空闲中断和双缓冲机制实现高效接收，在线超时检测提供链路状态指示，适用于需要蓝牙通信的各种嵌入式应用。其设计与 BSP 层解耦，可移植到任意 MCU 平台。配合 `module_device` 框架，可统一接入系统调度。

## 一页式接入顺序与可读信息

```c
/* 1. 先初始化 USART BSP，并准备两个生命周期覆盖 bluetooth 的静态缓冲区。 */
static module_bluetooth_t bluetooth;
static uint8_t receive_buffer[128];
static uint8_t processing_buffer[128];

/* 2. 填写 module_bluetooth_config_t：注入 USART、缓冲区、超时和接收回调。 */
module_bluetooth_status_t status = module_bluetooth_init(&bluetooth, &bluetooth_config);

/* 3. init 成功后启动 DMA/中断接收。 */
if (status == MODULE_BLUETOOTH_STATUS_OK) {
    status = module_bluetooth_start(&bluetooth);
}

/* 4. 在任务中周期调用；这里才执行接收回调并推进离线计时。 */
status = module_bluetooth_update(&bluetooth, elapsed_time_ms);

/* 5. 需要发送时调用 transmit/send_command；退出前先 stop。 */
status = module_bluetooth_stop(&bluetooth);
```

| 可读取信息 | 推荐读取方式 | 说明 |
| --- | --- | --- |
| 在线状态 | `module_bluetooth_is_online()` | 是否在离线超时内收到过数据 |
| 接收内容 | `module_bluetooth_receive_callback_t` | 数据只在回调期间有效，长期保存必须复制 |
| `module_bluetooth_t` | 调试器只读查看 | `receive_overrun_count`、`receive_restart_error_count`、`receive_elapsed_time_ms`、`is_started` |
| `module_device_t super` | `module_device_is_initialized()` 等 | 逻辑名称、注册键和初始化状态 |

不要从应用代码修改 `module_bluetooth_t` 的运行字段；它们公开是为了静态分配和调试，不是控制接口。
