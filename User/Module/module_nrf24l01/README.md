# nRF24L01 无线收发器模块 (module_nrf24l01) —— 使用指南

## 1. 模块概述

`module_nrf24l01` 是 nRF24L01(+) 2.4GHz 收发器的驱动模块，基于 `bsp_spi_t`、CE GPIO、CSN GPIO 和注入的微秒延时回调。支持地址、管道、速率、功率、自动应答、自动重发、发送轮询和接收。

## 2. 硬件依赖

配置需要：

- SPI 基类（已初始化）
- CE 和 CSN 两个 GPIO
- 频道 0～125
- 3～5 字节地址宽度
- 1～32 字节固定载荷长度
- 数据速率（1M/2M/250k）与输出功率（-18～0dBm）
- 自动应答和自动重发参数
- SPI 超时（毫秒）
- 微秒延时回调（用于 CE 脉冲和上电等待）

## 3. 典型初始化流程

```c
static module_nrf24l01_t s_radio;

static uint8_t tx_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

const module_nrf24l01_config_t cfg = {
    .spi = board_spi_ptr,
    .chip_enable_gpio = board_ce_gpio,
    .chip_select_gpio = board_csn_gpio,
    .channel = 10,
    .address_size = 5,
    .payload_size = 32,
    .automatic_retransmit_count = 5,
    .automatic_retransmit_delay_us = 1000,
    .data_rate = MODULE_NRF24L01_DATA_RATE_1_MBPS,
    .output_power = MODULE_NRF24L01_OUTPUT_POWER_0_DBM,
    .automatic_acknowledge_enabled = true,
    .spi_timeout_ms = 10,
    .delay_us = board_delay_us,
    .delay_user_context = NULL,
    .logical_name = "nrf24l01",
    .registration_key = 0,
};

module_nrf24l01_init(&s_radio, &cfg);
module_nrf24l01_start(&s_radio);

// 设置地址
module_nrf24l01_set_transmit_address(&s_radio, tx_address, 5);
module_nrf24l01_set_receive_address(&s_radio, 0, tx_address, 5);
```

## 4. 发送流程

```c
// 1. 发送数据
uint8_t payload[32];
if (module_nrf24l01_transmit(&s_radio, payload, 32) == MODULE_NRF24L01_STATUS_OK) {
    // 2. 轮询发送状态
    module_nrf24l01_status_t status;
    do {
        status = module_nrf24l01_poll_transmit(&s_radio);
    } while (status == MODULE_NRF24L01_STATUS_BUSY);
    if (status == MODULE_NRF24L01_STATUS_OK) {
        // 发送成功
    } else if (status == MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) {
        // 达到最大重发次数
    }
}
```

## 5. 接收流程

```c
// 1. 启用接收管道
module_nrf24l01_set_receive_pipe_enabled(&s_radio, 0, true);

// 2. 启动接收模式
module_nrf24l01_start_receive(&s_radio);

// 3. 轮询接收（可配合 EXTI 中断）
uint8_t rx_payload[32];
uint8_t pipe_index;
module_nrf24l01_status_t status = module_nrf24l01_receive(&s_radio, rx_payload, 32, &pipe_index);
if (status == MODULE_NRF24L01_STATUS_OK) {
    // 处理收到的数据
}
```

## 6. 管道说明

| 管道     | 地址大小             | 说明                |
| :------- | :------------------- | :------------------ |
| 管道 0   | 完整地址（3~5 字节） | 发送自动应答使用    |
| 管道 1   | 完整地址（3~5 字节） | 可作为标准接收管道  |
| 管道 2~5 | 仅 1 字节（LSB）     | 与管道 1 共享高字节 |

## 7. 关键时序

| 操作     | 延时       | 说明                  |
| :------- | :--------- | :-------------------- |
| 上电稳定 | 1.5ms      | `set_mode` 后自动等待 |
| CE 脉冲  | 15us       | 触发发送              |
| 重发延时 | 250~4000us | 配置的自动重发间隔    |

## 8. 注意事项

- **可靠性**：nRF24L01 不应作为唯一安全控制链路，应用协议仍需序号、超时、数据校验和失联安全状态。
- **总线共享**：SPI 与其他设备共享时需要外部总线互斥，并保证一次命令期间 CSN 连续有效。
- **中断**：IRQ 引脚可通过 `bsp_exti` 在 App/板级连接，本模块也支持轮询状态寄存器。
- **发送阻塞**：发送期间不能覆盖当前事务，必须等 `poll_transmit` 返回非 BUSY。

---

**总结**：`module_nrf24l01` 提供了完整的 nRF24L01(+) 驱动，涵盖初始化、配置、发送接收和状态监控。通过 SPI 和 GPIO 抽象层，与具体 MCU 解耦，可移植到任意平台。配合 `module_device` 基类，可接入统一设备管理框架。
