# 视觉通信模块 (module_vision_comm)

## 1. 模块概述

`module_vision_comm` 是视觉设备（如摄像头、AI 处理板）与主控之间通过 USB CDC 虚拟串口进行固定格式二进制帧通信的模块。它定义了 5 字节帧结构，包含双字节帧头、两个数据字节和 CRC8 校验，支持发送、接收流解析和帧同步。

**核心功能**：

- 发送两个数据字节（带 CRC8 校验）
- 接收数据流解析（支持分包、粘包、帧前噪声）
- CRC8 校验（初值 0xFF，多项式 0x8C，LSB first）
- 最新有效帧数据缓存
- USB 发送忙状态检测

- **固定帧格式**：极简 5 字节帧，适合高速、低开销通信。
- **流式解析**：支持任意数据流输入，自动同步和丢弃无效字节。
- **非阻塞发送**：检测 USB 忙状态，避免覆盖未完成发送。

## 2. 设计边界

| **模块负责**                 | **模块不负责**                    |
| :--------------------------- | :-------------------------------- |
| 固定帧的组帧、CRC 计算和发送 | 具体的视觉数据处理算法            |
| 接收字节流的帧同步和校验     | 高层协议（如 JSON、Protobuf）解析 |
| 最新有效帧数据的缓存         | 帧序号或更复杂的数据结构          |
| USB 发送忙状态检查           | USB 硬件初始化或配置              |

## 3. 帧格式

帧固定为 5 字节：

| 字节偏移 | 内容          | 说明                  |
| :------- | :------------ | :-------------------- |
| 0        | `0xA5`        | 帧头 1                |
| 1        | `0x5A`        | 帧头 2                |
| 2        | `data_first`  | 第一个数据字节        |
| 3        | `data_second` | 第二个数据字节        |
| 4        | CRC8          | 对字节 0～3 计算 CRC8 |

### CRC8 参数

- 初始值：`0xFF`
- 多项式：`0x8C`（CRC-8/SAE-J1850 常用变体）
- 处理顺序：LSB first（右移）
- 结果异或值：`0x00`

> 注意：CRC 校验**不包含**帧头本身？不，CRC 是对帧头加数据（4 个字节）计算，然后存放于第 4 字节。公式为 `CRC = crc8(帧头1, 帧头2, data_first, data_second)`。

## 4. 依赖

- `bsp_usb_vcp`：USB 虚拟串口 BSP 抽象层，提供 `transmit`、`get_busy` 等接口。

## 5. API 参考

| 函数                      | 说明                                 | 返回值                            |
| :------------------------ | :----------------------------------- | :-------------------------------- |
| `module_vision_comm_init`      | 初始化模块，保存 USB VCP 句柄和超时  | `OK` / `INVALID_ARGUMENT`         |
| `module_vision_comm_send`      | 发送两个数据字节（组帧+CRC+发送）    | `OK` / `BUSY` / `TRANSPORT_ERROR` |
| `module_vision_comm_feed_data` | 注入接收数据流，解析并更新最新有效帧 | `OK` / `INVALID_FRAME`            |
| `module_vision_comm_get_data`  | 获取最新有效帧的两个数据字节         | `OK` / `NO_DATA`                  |
| `module_vision_comm_crc8`      | 计算 CRC8（可单独使用）              | CRC 值                            |

## 6. 使用示例

### 6.1 初始化

```c
static module_vision_comm_t s_vision;

const module_vision_comm_config_t cfg = {
    .usb_vcp = board_usb_vcp_ptr,          // 已初始化的 USB VCP
    .transmit_timeout_ms = 10,
};

module_vision_comm_init(&s_vision, &cfg);
```

### 6.2 发送数据

```c
// 发送两个字节（例如目标检测结果和置信度）
uint8_t result = 0x01;
uint8_t confidence = 0x64;
module_vision_comm_status_t st = module_vision_comm_send(&s_vision, result, confidence);
if (st == MODULE_VISION_COMM_STATUS_BUSY) {
    // USB 发送忙，稍后重试
}
else if (st != MODULE_VISION_COMM_STATUS_OK) {
    // 传输错误处理
}
```

### 6.3 接收数据（在 USB 回调中）

```c
// USB 接收回调（通常由 bsp_usb_vcp 触发）
void usb_vcp_receive_callback(const uint8_t *data, size_t size, void *ctx) {
    module_vision_comm_t *vision = (module_vision_comm_t *)ctx;
    module_vision_comm_feed_data(vision, data, size);
}

// 在任务中定期检查最新数据
module_vision_comm_process_data_t rx_data;
if (module_vision_comm_get_data(&s_vision, &rx_data) == MODULE_VISION_COMM_STATUS_OK) {
    // 使用 rx_data.data_first, rx_data.data_second
    // rx_data.update_count 可用于检测是否更新
}
```

### 6.4 手动注入测试数据

```c
uint8_t test_frame[] = {0xA5, 0x5A, 0x01, 0x02, 0x??}; // CRC 需正确
module_vision_comm_feed_data(&s_vision, test_frame, sizeof(test_frame));
```

## 7. 解析状态机

- **流式解析**：输入任意字节流，自动查找帧头 `0xA5 0x5A`。
- **丢弃无效字节**：若在帧头后收到非预期字节，会重置同步。
- **CRC 校验**：只有当 CRC 正确时才更新 `received_data`。
- **覆盖策略**：总是保留最新有效的帧数据；`update_count` 递增可用于检测更新。

## 8. 注意事项

- **USB 发送忙**：调用 `send_data` 前模块会检查 USB 是否忙，若忙则返回 `BUSY`，**不会**阻塞或覆盖发送缓冲区。
- **发送缓冲区**：`module_vision_comm_t` 内部有一个 5 字节发送缓冲区，因此调用 `send_data` 后数据会被立即复制并发送，调用者可安全释放源数据。
- **接收数据有效期**：`get_data` 返回的 `module_vision_comm_process_data_t` 在下次有效帧到来前保持不变，`update_count` 可辅助判断是否更新。
- **CRC 工具**：`module_vision_comm_crc8` 是公开函数，可用于其他需要 CRC8 的场景。
- **帧头冲突**：数据字节若为 `0xA5` 或 `0x5A` 不影响解析，因为帧头是两个连续字节，解析器会正确区分。

## 9. 建议验证测试项

- [ ] 发送正常帧，USB 未忙时返回 `OK`
- [ ] USB 忙时返回 `BUSY`
- [ ] 接收完整正确帧后，`get_data` 返回有效数据
- [ ] 接收 CRC 错误的帧，`received_data` 不更新
- [ ] 接收半帧（如只收到 `0xA5`）后继续接收剩余字节，能正常解析
- [ ] 接收粘包（连续两帧），能正确解析两帧（但只保留最新一帧）
- [ ] 接收帧前噪声（如 `0x00 0xA5 0x5A ...`），能自动同步
- [ ] 发送后 USB 发送错误返回 `TRANSPORT_ERROR`
- [ ] `data_first` 和 `data_second` 取边界值（0x00, 0xFF, 0xA5, 0x5A）时通信正常

---

**总结**：`module_vision_comm` 提供了极简、可靠的视觉通信基础，适用于需要低延迟、高可靠性的实时视觉数据（如目标检测结果、舵机角度等）。固定帧格式和 CRC 校验确保了数据完整性，流式解析器能稳定处理实际通信中的各种异常情况。

## 一页式接入顺序与可读信息

```c
/* 1. 先初始化 USB VCP BSP。 */
static module_vision_comm_t vision_comm;

/* 2. 注入 USB VCP 和发送超时。 */
module_vision_comm_status_t status =
    module_vision_comm_init(&vision_comm, &vision_config);

/* 3. USB 接收回调只把收到的字节流交给解析器。 */
status = module_vision_comm_feed_data(&vision_comm, receive_data, data_size);

/* 4. 任务中读取最新一帧；NO_DATA 表示还没有有效帧。 */
module_vision_comm_process_data_t data;
status = module_vision_comm_get_data(&vision_comm, &data);
if ((status == MODULE_VISION_COMM_STATUS_OK) && data.is_valid) {
    uint8_t first = data.data_first;
}

/* 5. 发送时只传两个数据字节，模块自动添加 A5 5A 和 CRC8。 */
status = module_vision_comm_send(&vision_comm, data_first, data_second);
```

`module_vision_comm_process_data_t` 是唯一业务数据结构体：`data_first`、`data_second` 是最近有效帧的数据位，`update_count` 用来判断是否收到新帧，`is_valid` 表示是否至少成功解析过一帧。`module_vision_comm_get_data()` 会复制快照，调用者可以安全保存该副本。
