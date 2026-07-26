# module_nrf24l01

nRF24L01(+) 2.4 GHz 收发器驱动，基于 `bsp_spi_t`、CE GPIO、CSN GPIO 和注入的微秒延时。
支持地址、管道、速率、功率、自动应答、自动重发、发送轮询和接收。

## 硬件依赖

配置需要：

- SPI 基类；
- CE 和 CSN 两个 GPIO；
- 频道 0～125；
- 3～5 字节地址宽度；
- 1～32 字节固定载荷长度；
- 数据速率与输出功率；
- 自动应答和自动重发参数；
- SPI 超时；
- 非阻塞环境可接受的微秒延时回调。

IRQ 引脚可通过 `bsp_exti` 在 App/板级连接，本模块也支持轮询状态寄存器。

## 初始化

```c
module_nrf24l01_init(&radio, &config);
module_nrf24l01_start(&radio);
module_nrf24l01_set_transmit_address(&radio, address, address_size);
module_nrf24l01_set_receive_address(&radio, 0U, address, address_size);
```

启动时会配置寄存器并验证设备响应。地址长度必须与配置一致。

## 接收

启用所需管道后调用 `module_nrf24l01_start_receive`。收到数据时调用
`module_nrf24l01_receive`，返回载荷和管道号；FIFO 为空返回 `STATUS_NO_DATA`。

## 发送

`module_nrf24l01_transmit` 装载载荷并启动发送，随后周期调用
`module_nrf24l01_poll_transmit`：

- 成功：清状态并结束；
- 仍发送：返回 BUSY；
- 达到最大重发：返回 MAXIMUM_RETRANSMIT，并需要清 FIFO/状态。

发送期间不能覆盖当前事务。

## 时序与总线共享

CE 脉冲和上电等待使用注入延时。不要在高优先级 ISR 中调用完整收发流程。SPI 与其他设备
共享时需要外部总线互斥，并保证一次命令期间 CSN 连续有效。

## 可靠性

nRF24L01 不应作为唯一安全控制链路。应用协议仍需序号、超时、数据校验和失联安全状态。
丢包计数与重发计数可通过 `get_observe_transmit` 上报。

## 建议验证

- 频道、地址宽度和载荷边界；
- 三种速率和四种功率；
- 六个接收管道；
- 自动应答开关；
- 成功发送、最大重发和 FIFO 清理；
- 接收空、满和管道号；
- SPI/CE/CSN 错误与设备不存在。
