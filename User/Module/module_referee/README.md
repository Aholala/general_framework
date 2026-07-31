# module_referee —— RoboMaster 裁判系统协议模块

## 1. 模块概述

`module_referee` 是一个 RoboMaster 裁判系统串口协议的流式收发框架，提供帧同步、CRC8/CRC16 校验、分包/粘包处理、命令路由分发、在线超时检测、发送序号管理和完整运行统计。它基于 BSP USART 抽象层构建，与具体 MCU 硬件解耦，可移植到任意支持串口的平台。

**核心功能**：

- **帧解析**：自动识别 `0xA5` 起始字节，处理分包和粘包。
- **双重 CRC 校验**：CRC8 校验帧头，CRC16 校验整帧。
- **命令路由**：支持命令 ID 精确匹配路由和默认处理器。
- **帧发送**：支持阻塞/中断/DMA 三种传输模式。
- **在线检测**：根据接收帧超时自动判断在线状态。
- **运行统计**：提供完整的接收、错误、丢弃等统计信息。
- **强类型数据仓库**：提供常用命令的解析和存储。

## 2. 文件说明

| 文件                         | 说明                                      | 详细文档 |
| :--------------------------- | :---------------------------------------- | :------- |
| `module_referee.h/.c`        | 核心框架：流解析、路由、发送和生命周期    | 本文档 |
| `module_referee_crc.h/.c`    | CRC8 与 CRC16 计算、验证和追加            | 本文档 |
| `module_referee_data.h/.c`   | 强类型比赛、机器人、功率和射击数据仓库    | 本文档 |
| `module_referee_ui.h/.c`     | 客户端图形编码、队列、批量发送和发送限频  | [README_UI.md](README_UI.md) |

## 3. 帧格式

```
+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+
|  SOF   | DataLen (LE) | Seq   | CRC8  | CmdID (LE) |         Payload           | CRC16 (LE) |
| 0xA5   | byte0 | byte1 | byte  | byte  | byte0 | byte1 |         ...              | byte0 | byte1 |
+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+
|  1     |       2       |  1    |  1    |       2       |          payload_size     |    2      |
+--------+----------------+-------+-------+---------------+---------------------------+-----------+
|                              MODULE_REFEREE_HEADER_SIZE (5)                                 |
```

- **SOF**：起始字节 `0xA5`
- **DataLen**：负载长度（小端序，最大 65535）
- **Seq**：序列号（发送时递增）
- **CRC8**：帧头 CRC（从 SOF 到 CRC8 前一个字节，初值 0xFF，多项式 0x8C）
- **CmdID**：命令 ID（小端序）
- **Payload**：负载数据
- **CRC16**：整帧 CRC（从 SOF 到 CRC16 前一个字节，初值 0xFFFF，多项式 0x8408）

## 4. 接收路径

```text
USART DMA/idle ISR
  -> 拷贝 receive_buffer 到 processing_buffer
  -> 立即重启 DMA
  -> 设置 receive_pending

任务: module_referee_update
  -> 追加到 stream_buffer
  -> 查找 0xA5
  -> 验证 CRC8 帧头
  -> 等待完整帧
  -> 验证 CRC16
  -> 路由命令
```

## 5. 依赖

- `bsp_usart`：BSP USART 抽象层，提供串口收发和空闲中断。
- `module_device`：模块设备基类，提供统一的启动/停止/更新接口。

## 6. 使用示例

### 6.1 定义配置和对象

```c
/* 静态分配缓冲区 */
static uint8_t rx_buffer[256];
static uint8_t proc_buffer[256];
static uint8_t stream_buffer[512];
static uint8_t tx_buffer[256];

/* 路由表（静态常量） */
static const module_referee_route_t routes[] = {
    {
        .command_id = MODULE_REFEREE_COMMAND_ROBOT_STATUS,
        .handler = my_robot_status_handler,
        .user_context = &my_ctx,
    },
    {
        .command_id = MODULE_REFEREE_COMMAND_POWER_HEAT,
        .handler = my_power_heat_handler,
        .user_context = &my_ctx,
    },
};

/* 对象 */
static module_referee_t ref;
static module_referee_data_t ref_data;  // 可选：强类型数据仓库
```

### 6.2 初始化

```c
module_referee_config_t cfg = {
    .usart = usart_ptr,                           // BSP USART 基类
    .receive_buffer = rx_buffer,
    .receive_capacity = sizeof(rx_buffer),
    .processing_buffer = proc_buffer,
    .processing_capacity = sizeof(proc_buffer),
    .stream_buffer = stream_buffer,
    .stream_capacity = sizeof(stream_buffer),
    .transmit_buffer = tx_buffer,
    .transmit_capacity = sizeof(tx_buffer),
    .routes = routes,
    .route_count = ARRAY_SIZE(routes),
    .default_handler = NULL,                      // 可选默认处理器
    .default_user_context = NULL,
    .receive_timeout_ms = 100,
    .transmit_timeout_ms = 100,
    .offline_timeout_ms = 500,
    .receive_mode = BSP_TRANSFER_MODE_DMA,
    .logical_name = "referee",
    .registration_key = 0,
};
module_referee_init(&ref, &cfg);
module_referee_start(&ref);
```

### 6.3 周期更新

```c
void main_loop(void) {
    uint32_t dt_ms = get_delta_time_ms();   // 获取距上次更新的毫秒数
    module_referee_update(&ref, dt_ms);

    // 检查在线状态
    if (module_referee_is_online(&ref)) {
        // 裁判系统在线，执行相关逻辑
    }

    // 其他任务...
}
```

### 6.4 发送帧

```c
/* 发送数据到裁判系统 */
uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
module_referee_transmit(&ref, 0x0301, payload, sizeof(payload),
                        BSP_TRANSFER_MODE_DMA);
```

### 6.5 使用数据仓库（可选）

将 `module_referee_data_route_handler` 注册为默认处理器或特定命令路由，自动解析并存储数据。

```c
/* 配置默认处理器指向数据仓库 */
cfg.default_handler = module_referee_data_route_handler;
cfg.default_user_context = &ref_data;

/* 在任务中消费数据 */
if (module_referee_data_has_update(&ref_data, MODULE_REFEREE_COMMAND_ROBOT_STATUS)) {
    uint16_t hp = ref_data.robot_status.current_hp;
    uint16_t heat = ref_data.robot_status.shooter_barrel_heat_limit;
    // 处理机器人状态数据...
}
module_referee_data_clear_updates(&ref_data);  // 清除标记
```

## 7. 命令处理器

```c
void my_command_handler(uint16_t command_id, const uint8_t *payload,
                        size_t payload_size, uint8_t sequence, void *user_context) {
    /* 注意：payload 仅在回调期间有效 */
    if (payload_size >= expected_size) {
        // 解析 payload（小端序）
        uint16_t value = module_referee_read_uint16_le(payload);
        // 需要长期保存的数据必须复制
    }
}
```

## 8. 配置要点

| 配置项                                    | 要求                                                   |
| :---------------------------------------- | :----------------------------------------------------- |
| `receive_capacity <= processing_capacity` | 处理缓冲区必须 >= 接收缓冲区                           |
| `stream_capacity >= 最大帧大小`           | 建议 512 字节以上                                      |
| `receive_mode`                            | 不支持阻塞模式，必须使用 `INTERRUPT` 或 `DMA`          |
| `routes` 表                               | 建议 `static const`，回调中 `payload` 仅在回调期间有效 |
| `offline_timeout_ms`                      | 应大于 2 倍帧间隔，避免误判离线                        |

## 9. 在线状态

- 收到有效的 CRC16 帧时自动刷新在线时间（`receive_elapsed_time_ms = 0`）
- 超过 `offline_timeout_ms` 未收到帧则置离线
- 通过 `module_referee_is_online()` 查询

## 10. 统计信息

```c
module_referee_statistics_t stats;
module_referee_get_statistics(&ref, &stats);
```

| 字段                          | 说明                     |
| :---------------------------- | :----------------------- |
| `received_frame_count`        | 成功接收并校验通过的帧数 |
| `handled_frame_count`         | 已分发的帧数             |
| `unknown_command_count`       | 无路由匹配且无默认处理器 |
| `crc8_error_count`            | 帧头 CRC8 错误数         |
| `crc16_error_count`           | 帧体 CRC16 错误数        |
| `oversize_frame_count`        | 超大帧数                 |
| `discarded_byte_count`        | 丢弃的非法字节数         |
| `receive_overrun_count`       | 接收覆盖次数             |
| `receive_restart_error_count` | 重启 DMA 接收失败次数    |

## 11. 中断回调

USART 回调由模块内部管理，应用层无需关心。模块内部实现：

- 在 ISR 中拷贝数据到 `processing_buffer`
- 立即重启 DMA/中断接收
- 设置 `receive_pending` 标志
- 任务上下文的 `module_referee_update` 处理数据

## 12. 协议版本边界

本框架**不硬编码**所有裁判命令结构，避免官方协议升级导致底层重写。上层解析器必须按当前赛事手册核对：

- 命令 ID
- payload 长度
- 字节序（小端序）
- 字段缩放（如浮点数、单位）

不要把官方 packed 结构直接强制转换到接收缓冲区。

## 13. 建议验证测试项

- [ ] 单帧接收正确解析
- [ ] 分包（一帧分多次接收）正确拼接
- [ ] 粘包（多帧连续接收）正确拆分
- [ ] 帧前噪声（非 0xA5 字节）正确丢弃
- [ ] CRC8 错误正确丢弃并计数
- [ ] CRC16 错误正确丢弃并计数
- [ ] 超长 payload 正确处理
- [ ] 流缓冲区溢出正确处理
- [ ] 未知命令路由到默认处理器或计数
- [ ] DMA 接收立即重启（无数据丢失）
- [ ] 异步发送 BUSY 状态阻止并发发送
- [ ] 发送序列号正常回绕
- [ ] 在线超时正确触发离线
- [ ] 统计计数饱和（UINT32_MAX）
- [ ] 停止后不再接收数据

---

**总结**：`module_referee` 提供了完整的裁判系统协议收发框架，具备流式解析、双重 CRC 校验、命令路由、在线检测和运行统计等特性。其设计与 BSP 层解耦，可移植到任意 MCU 平台。强类型数据仓库为常用命令提供了便捷的解析和存储，减少上层重复劳动。协议版本变更时只需更新数据仓库的解析逻辑，核心框架保持稳定。
