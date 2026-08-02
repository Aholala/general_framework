# nRF24L01 原始驱动与 ACE 链路协议

本目录分成两个职责明确的组件：

| 文件 | 职责 |
| --- | --- |
| `module_nrf24l01.h/.c` | 芯片寄存器、地址、管道、固定载荷、自动应答、重发和原始收发 |
| `module_nrf24l01_ace_link.h/.c` | ACE 地址、`A5 5A` 数据包、序号和 CRC16 |

芯片驱动不依赖 `alg_crc`，也不知道 ACE 帧格式；ACE 链路协议通过公开的
`module_nrf24l01_transmit()` 和 `module_nrf24l01_receive()` 使用无线驱动。

## 1. 两端无线参数

两个 nRF24L01 必须使用相同的频道、地址宽度、地址内容、固定载荷长度、
数据率和自动应答参数。原始驱动支持 1~32 字节固定载荷，ACE 协议由于有
7 字节固定开销，要求载荷长度为 7~32 字节。

推荐点对点配置：32 字节固定载荷、250 kbps、自动应答和自动重发。

```c
#include "module_nrf24l01.h"
#include "module_nrf24l01_ace_link.h"

static module_nrf24l01_t radio;
static module_nrf24l01_ace_link_t ace_link;

static const module_nrf24l01_config_t radio_config = {
    .spi = board_spi,
    .chip_enable_gpio = board_ce_gpio,
    .chip_select_gpio = board_csn_gpio,
    .channel = 76U,
    .address_size = MODULE_NRF24L01_ACE_LINK_ADDRESS_SIZE,
    .link_address = module_nrf24l01_ace_link_address,
    .payload_size = 32U,
    .automatic_retransmit_count = 10U,
    .automatic_retransmit_delay_us = 750U,
    .data_rate = MODULE_NRF24L01_DATA_RATE_250_KBPS,
    .output_power = MODULE_NRF24L01_OUTPUT_POWER_0_DBM,
    .automatic_acknowledge_enabled = true,
    .spi_timeout_ms = 10U,
    .delay_us = board_delay_us,
    .delay_user_context = NULL,
    .logical_name = "nrf24_radio",
    .registration_key = 0U,
};
```

默认地址 `module_nrf24l01_ace_link_address` 是 ASCII `{'A', 'C', 'E'}`。
两端必须按相同顺序写入，不能只在一端倒序。

## 2. ACE 数据包

```text
[A5] [5A] [message_type] [sequence] [data_size]
[data[0] ... data[N-1]，固定载荷剩余部分补0]
[CRC16_L] [CRC16_H]
```

- `sequence` 在数据成功装入发送 FIFO 后递增。
- `data_size <= payload_size - 7`。
- CRC16-CCITT-FALSE：初值 `0xFFFF`，多项式 `0x1021`。
- 应用 CRC16 调用 `alg_crc`；芯片内部2字节空中 CRC 仍独立开启。

## 3. 初始化顺序

```c
module_nrf24l01_status_t radio_status;
module_nrf24l01_ace_link_status_t link_status;

/* 1. BSP SPI、CE GPIO、CSN GPIO 必须先初始化。 */
radio_status = module_nrf24l01_init(&radio, &radio_config);

/* 2. 启动芯片并验证寄存器。 */
radio_status = module_nrf24l01_start(&radio);

/* 3. 将独立 ACE 协议对象绑定到已初始化的无线对象。 */
link_status = module_nrf24l01_ace_link_init(&ace_link, &radio);

/* 4. 接收节点进入 RX；发送节点也可在发送完成后恢复 RX。 */
radio_status = module_nrf24l01_start_receive(&radio);
```

## 4. 发送

```c
const uint8_t control_data[2] = {0x12U, 0x34U};

link_status = module_nrf24l01_ace_link_send(
    &ace_link, 0x01U, control_data, sizeof(control_data));

if (link_status == MODULE_NRF24L01_ACE_LINK_STATUS_OK)
{
    do
    {
        radio_status = module_nrf24l01_poll_transmit(&radio);
    } while (radio_status == MODULE_NRF24L01_STATUS_BUSY);
}

if (radio_status == MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT)
{
    /* 对端未应答：上层决定重试、降级或进入安全状态。 */
}

module_nrf24l01_start_receive(&radio);
```

## 5. 接收

```c
module_nrf24l01_ace_link_packet_t packet;
uint8_t receive_pipe_index;

link_status = module_nrf24l01_ace_link_receive(
    &ace_link, &packet, &receive_pipe_index);

if (link_status == MODULE_NRF24L01_ACE_LINK_STATUS_OK)
{
    /* packet.message_type / sequence / data_size / data[] 可直接读取。 */
}
else if (link_status == MODULE_NRF24L01_ACE_LINK_STATUS_NO_DATA)
{
    /* FIFO 为空，稍后再读。 */
}
else if (link_status == MODULE_NRF24L01_ACE_LINK_STATUS_INVALID_FRAME)
{
    /* 帧头或长度错误。 */
}
else if (link_status == MODULE_NRF24L01_ACE_LINK_STATUS_CHECKSUM_ERROR)
{
    /* 应用层 CRC16 错误。 */
}
```

## 6. 自定义协议

不使用 ACE 帧时只包含 `module_nrf24l01.h`：

```c
uint8_t transmit_payload[32] = {0U};
uint8_t receive_payload[32];
uint8_t receive_pipe_index;

module_nrf24l01_transmit(&radio, transmit_payload, sizeof(transmit_payload));
module_nrf24l01_receive(
    &radio, receive_payload, sizeof(receive_payload), &receive_pipe_index);
```

## 7. 可读信息

| 数据 | 读取方式 |
| --- | --- |
| 消息类型、序号、有效长度和数据 | `module_nrf24l01_ace_link_packet_t` |
| 下一发送序号 | `module_nrf24l01_ace_link_get_next_sequence()` |
| 当前射频模式和发送待完成状态 | `module_nrf24l01_t.mode/transmit_pending` |
| 频道、地址、固定载荷长度 | `module_nrf24l01_t` 调试读取 |
| 自动重发和丢包计数 | `module_nrf24l01_get_observe_transmit()` |

## 8. 验证清单

- [ ] 两端频道、地址、载荷长度、速率和自动应答配置一致
- [ ] 原始1~32字节固定载荷均能被驱动正确检查
- [ ] ACE 链路拒绝小于7字节的固定载荷
- [ ] 发送后持续轮询，直到成功或达到最大重发次数
- [ ] 正确处理错误帧头、错误长度和错误 CRC
- [ ] 失联时由 App 根据时间和序号进入安全状态

nRF24L01 不应作为唯一安全控制链路。CRC 和硬件自动应答只能检查传输，
不能替代超时、目标清零和故障安全状态机。
