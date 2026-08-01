# nRF24L01 点对点通信模块 (module_nrf24l01)

## 1. 模块概述

`module_nrf24l01` 是基于 `bsp_spi_t`、CE GPIO、CSN GPIO 和注入的微秒延时回调的 nRF24L01(+) 2.4GHz 收发器驱动。它提供了完整的硬件寄存器操作、地址管道管理、速率功率配置、自动应答与重发，并在其之上封装了一层点对点应用协议，便于双机可靠通信。

**核心功能**：

- 硬件初始化与配置（频道、地址、速率、功率、自动应答/重发）
- 接收/发送管道管理（最多 6 个接收管道）
- 原始固定载荷收发（`transmit` / `receive` / `poll_transmit`）
- **点对点应用协议**（`send_packet` / `receive_packet`）：帧头 + 消息类型 + 序号 + 长度 + CRC16-CCITT
- 发送观察统计（丢包计数、重发计数）
- FIFO 管理（清空发送/接收 FIFO）

- **双机配对**：两个节点共用 `link_address`，模块自动将发送地址和自动应答管道 0 地址设置为该地址。
- **应用层帧**：在固定射频载荷内封装消息类型、序号、长度和 CRC16，提供跨层数据完整性校验。
- **可靠性**：nRF24L01 硬件自动应答 + 应用层 CRC16 双重保障。

## 2. 两个模块必须一致的参数

两个 nRF24L01 节点必须使用完全相同的以下参数：

| 参数                            | 说明                             |
| :------------------------------ | :------------------------------- |
| `channel`                       | 射频频道（0~125）                |
| `address_size`                  | 地址宽度（3~5 字节）             |
| `link_address`                  | 共用链路地址（内容必须完全一致） |
| `payload_size`                  | 固定射频载荷长度（1~32 字节）    |
| `data_rate`                     | 空中速率                         |
| `automatic_retransmit_count`    | 自动重发次数                     |
| `automatic_retransmit_delay_us` | 重发延时                         |
| `automatic_acknowledge_enabled` | 自动应答使能                     |

**建议点对点链路**：使用 5 字节地址、32 字节固定载荷、250 kbps、自动应答和自动重发。

## 3. 推荐双机配置

```c
/* 节点 A 与节点 B 使用相同配置；硬件对象由各自板级代码注入。 */
const module_nrf24l01_config_t radio_config = {
    .spi = board_spi,
    .chip_enable_gpio = board_ce_gpio,
    .chip_select_gpio = board_csn_gpio,
    .channel = 76U,
    .address_size = MODULE_NRF24L01_ACE_ADDRESS_SIZE,
    .link_address = module_nrf24l01_ace_address,   // 默认 ACE 地址
    .payload_size = 32U,
    .automatic_retransmit_count = 10U,
    .automatic_retransmit_delay_us = 750U,
    .data_rate = MODULE_NRF24L01_DATA_RATE_250_KBPS,
    .output_power = MODULE_NRF24L01_OUTPUT_POWER_0_DBM,
    .automatic_acknowledge_enabled = true,
    .spi_timeout_ms = 10U,
    .delay_us = board_delay_us,
    .delay_user_context = NULL,
    .logical_name = "nrf24_ace_link",
    .registration_key = 0U,
};
```

模块提供默认地址 `module_nrf24l01_ace_address`，内容为 3 字节 ASCII `{'A', 'C', 'E'}`。双方写入 nRF24L01 的地址字节数组必须完全一致，**不要**在一端自行倒序。

## 4. 应用层协议帧格式

模块在固定射频载荷内封装应用层帧：

| 字节偏移    | 字段     | 说明                             |
| :---------- | :------- | :------------------------------- |
| 0           | 帧头 1   | 固定 `0xA5`                      |
| 1           | 帧头 2   | 固定 `0x5A`                      |
| 2           | 消息类型 | 由应用定义（0~255）              |
| 3           | 序号     | 每次成功装入发送 FIFO 后自动加一 |
| 4           | 数据长度 | 0~25 字节（载荷 32 时）          |
| 5...        | 应用数据 | 仅校验有效长度                   |
| 末尾 2 字节 | CRC16    | 低字节在前，高字节在后           |

**CRC16-CCITT-FALSE 参数**：

- 初始值：`0xFFFF`
- 多项式：`0x1021`
- 校验范围：从第一个帧头到最后一个有效应用数据字节

> nRF24L01 自身的 2 字节硬件 CRC 仍保持开启，应用层 CRC16 用于检查帧格式和跨层数据完整性。

## 5. 发送流程

```c
module_nrf24l01_t radio;

/* 1. 初始化并启动 */
module_nrf24l01_init(&radio, &radio_config);
module_nrf24l01_start(&radio);

/* 2. 发送应用数据（消息类型 0x01，2 字节数据） */
const uint8_t control_data[2] = {0x12U, 0x34U};
module_nrf24l01_send_packet(&radio, 0x01U, control_data, sizeof(control_data));

/* 3. 轮询直到发送完成 */
module_nrf24l01_status_t status;
do {
    status = module_nrf24l01_poll_transmit(&radio);
} while (status == MODULE_NRF24L01_STATUS_BUSY);

if (status == MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) {
    // 对端未应答或信号差，需要重试
}

/* 4. 恢复接收模式 */
module_nrf24l01_start_receive(&radio);
```

## 6. 接收流程

```c
module_nrf24l01_packet_t received_packet;

/* 1. 启动接收模式 */
module_nrf24l01_start_receive(&radio);

/* 2. 轮询或由 EXTI 中断触发接收 */
module_nrf24l01_status_t status;
status = module_nrf24l01_receive_packet(&radio, &received_packet, NULL);

if (status == MODULE_NRF24L01_STATUS_OK) {
    // 收到有效数据包
    uint8_t type = received_packet.message_type;
    uint8_t seq = received_packet.sequence;
    uint8_t *data = received_packet.data;
    size_t len = received_packet.data_size;
    // 处理数据...
}
else if (status == MODULE_NRF24L01_STATUS_NO_DATA) {
    // FIFO 空，稍后重试
}
else if (status == MODULE_NRF24L01_STATUS_INVALID_PACKET) {
    // 帧头或长度错误
}
else if (status == MODULE_NRF24L01_STATUS_CHECKSUM_ERROR) {
    // CRC16 校验失败
}
```

## 7. 原始接口（自定义协议）

模块保留原始固定载荷接口，适合需要自定义协议的场景：

```c
// 发送原始载荷（不添加应用层帧）
uint8_t raw_payload[32];
module_nrf24l01_transmit(&radio, raw_payload, 32);

// 接收原始载荷
uint8_t rx_payload[32];
uint8_t pipe;
module_nrf24l01_receive(&radio, rx_payload, 32, &pipe);
```

## 8. 关键时序

| 操作     | 延时       | 说明                  |
| :------- | :--------- | :-------------------- |
| 上电稳定 | 1.5ms      | `set_mode` 后自动等待 |
| CE 脉冲  | 15us       | 触发发送              |
| 重发延时 | 250~4000us | 配置的自动重发间隔    |

## 9. 链路地址管理

- 初始化时 `link_address` 被写入 `TX_ADDR` 和 `RX_ADDR_P0`
- 发送时模块自动确保 `TX_ADDR` 和 `RX_ADDR_P0` 指向 `link_address`
- 接收模式启动前也会恢复 `RX_ADDR_P0`，避免被其他操作修改

## 10. 错误码速查

| 状态码               | 触发场景                     |
| :------------------- | :--------------------------- |
| `OK`                 | 操作成功                     |
| `NO_DATA`            | 接收 FIFO 空                 |
| `BUSY`               | 发送进行中                   |
| `MAXIMUM_RETRANSMIT` | 达到最大重发次数             |
| `INVALID_ARGUMENT`   | 参数为空、数据过长、地址非法 |
| `NOT_INITIALIZED`    | 对象未初始化                 |
| `NOT_STARTED`        | 未调用 `start`               |
| `TRANSPORT_ERROR`    | SPI 传输错误                 |
| `DEVICE_NOT_FOUND`   | CONFIG 寄存器回读不匹配      |
| `INVALID_PACKET`     | 帧头或长度错误               |
| `CHECKSUM_ERROR`     | CRC16 校验失败               |

## 11. 建议验证测试项

- [ ] 两端 CONFIG 寄存器读回成功（`start` 不报 `DEVICE_NOT_FOUND`）
- [ ] 两端频道、速率、地址宽度和载荷长度完全相同
- [ ] 两端 `link_address` 内容完全一致（推荐 `ACE`）
- [ ] 连续发送递增序号，观察自动重发和丢包计数
- [ ] 发送后轮询 `poll_transmit` 直到完成
- [ ] 发送完成后调用 `start_receive` 恢复接收
- [ ] 接收端校验帧头、长度和 CRC16
- [ ] 错误帧头、错误长度、错误 CRC 正确处理
- [ ] 链路断开后的超时和重试（应用层序号和超时）
- [ ] 最大重发后 FIFO 被正确清空

---

**重要提醒**：nRF24L01 不应作为唯一的安全控制链路。应用层仍需实现序号、超时、数据校验和失联安全状态。本模块的应用层 CRC16 提供数据完整性校验，但**不替代**应用层的超时重传和故障安全逻辑。

## 一页式双机接入顺序与可读信息

```c
/* 1. 两端分别初始化 SPI、CE GPIO、CSN GPIO，并提供微秒延时。 */
static module_nrf24l01_t radio;

/* 2. 两端必须使用相同频道、地址宽度、ACE 地址、载荷长度、速率和自动应答参数。 */
module_nrf24l01_status_t status = module_nrf24l01_init(&radio, &radio_config);

/* 3. start 写入并验证寄存器；接收端随后进入 RX 模式。 */
status = module_nrf24l01_start(&radio);
status = module_nrf24l01_start_receive(&radio);

/* 4. 发送端打包 message_type + sequence + data + CRC16。 */
status = module_nrf24l01_send_packet(&radio, message_type, payload, payload_size);

/* 5. 非阻塞发送需要周期 poll_transmit；接收端周期 receive_packet。 */
status = module_nrf24l01_poll_transmit(&radio);
module_nrf24l01_packet_t packet;
uint8_t receive_pipe_index;
status = module_nrf24l01_receive_packet(&radio, &packet, &receive_pipe_index);

/* 6. 退出前 stop；应用层还必须根据 sequence 和时间实现失联保护。 */
```

| 可读取信息 | 读取方式 | 说明 |
| --- | --- | --- |
| `module_nrf24l01_packet_t` | `module_nrf24l01_receive_packet()` | 消息类型、序号、有效数据长度和数据数组 |
| 重发/丢包观察值 | `module_nrf24l01_get_observe_transmit()` | 芯片 `OBSERVE_TX` 寄存器，用于统计重发和丢包 |
| 当前模式 | `module_nrf24l01_t.mode`，仅调试读取 | 关机、待机、发送或接收 |
| 链路配置 | `module_nrf24l01_t`，仅调试读取 | 频道、地址、载荷长度、寄存器缓存和发送待完成标志 |

`module_nrf24l01_receive_packet()` 会把数据复制到调用者的 packet，因此返回后可以保存该结构体。
