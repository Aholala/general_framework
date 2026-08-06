# bsp_usart — USART/UART 抽象层（完整 OOP）

完整 OOP 模型（`super + vptr + container_of`）。支持阻塞/中断/DMA、空闲中断接收、双缓冲 DMA。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `bsp_usart_t` | 基类：`super`(bsp_device_t), `callback`, `user_context` |
| `bsp_usart_device_t` | 派生设备：`super`(bsp_usart_t), `driver_ops` |
| `bsp_usart_config_t` | 配置：`device_handle`(UART_HandleTypeDef*), `driver_ops` |
| `bsp_usart_driver_ops_t` | 平台实现：`transmit/receive/receive_to_idle/double_buffer/abort/get_busy` |

## 用法

```c
// 获取（board_config 已初始化）
bsp_usart_t *usart = board_config_get_usart(BOARD_CONFIG_UART_DR16);

// 发送（三种模式）
bsp_usart_transmit(usart, data, size, BSP_TRANSFER_MODE_BLOCKING, 100);
bsp_usart_transmit(usart, data, size, BSP_TRANSFER_MODE_INTERRUPT, 0);
bsp_usart_transmit(usart, data, size, BSP_TRANSFER_MODE_DMA, 0);

// 接收
bsp_usart_receive(usart, buf, 18, BSP_TRANSFER_MODE_DMA, 0);

// 空闲中断接收（不定长协议）
bsp_usart_receive_to_idle(usart, buf, 256, BSP_TRANSFER_MODE_DMA, 100);

// 双缓冲 DMA（DR16 用）
bsp_usart_receive_to_idle_double_buffer(usart, buf0, buf1, 18);

// 查询忙状态
bool busy;
bsp_usart_get_busy(usart, &busy);

// 中止
bsp_usart_abort(usart);
```

## 传输模式

| 模式 | 特点 |
|------|------|
| `BLOCKING` | 轮询等待完成，可设超时 |
| `INTERRUPT` | 中断驱动，非阻塞 |
| `DMA` | DMA 传输，CPU 零开销 |

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_usart_init(me, cfg)` | 初始化 |
| `bsp_usart_transmit(me, data, n, mode, ms)` | 发送 |
| `bsp_usart_receive(me, buf, n, mode, ms)` | 定长接收 |
| `bsp_usart_receive_to_idle(me, buf, cap, mode, ms)` | 不定长接收（空闲中断） |
| `bsp_usart_receive_to_idle_double_buffer(me, b0, b1, cap)` | 双缓冲 DMA |
| `bsp_usart_abort(me)` | 中止传输 |
| `bsp_usart_get_busy(me, &busy)` | 查询忙 |
| `bsp_usart_as_base(me)` | 向上转型 |
