# bsp_spi — SPI 总线抽象层（完整 OOP）

完整 OOP 模型。支持阻塞/中断/DMA、全双工交换。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `bsp_spi_t` | 基类：`super`(bsp_device_t), `callback`, `user_context` |
| `bsp_spi_device_t` | 派生设备 |
| `bsp_spi_config_t` | 配置：`device_handle`(SPI_HandleTypeDef*), `driver_ops` |
| `bsp_spi_driver_ops_t` | 平台实现：`transmit/receive/exchange/abort/get_busy` |

## 用法

```c
bsp_spi_t *spi = board_config_get_bmi088_spi();

// 发送
uint8_t tx[4] = {0x80, 0x00, 0x00, 0x00};
bsp_spi_transmit(spi, tx, 4, BSP_TRANSFER_MODE_BLOCKING, 10);

// 接收
uint8_t rx[8];
bsp_spi_receive(spi, rx, 8, BSP_TRANSFER_MODE_DMA, 0);

// 全双工（同时收发）
bsp_spi_exchange(spi, tx, rx, 4, BSP_TRANSFER_MODE_BLOCKING, 5);

// 中止
bsp_spi_abort(spi);
bool busy;
bsp_spi_get_busy(spi, &busy);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_spi_init(me, cfg)` | 初始化 |
| `bsp_spi_transmit(me, data, n, mode, ms)` | 发送 |
| `bsp_spi_receive(me, buf, n, mode, ms)` | 接收 |
| `bsp_spi_exchange(me, tx, rx, n, mode, ms)` | 全双工交换 |
| `bsp_spi_abort(me)` | 中止 |
| `bsp_spi_get_busy(me, &busy)` | 查询忙 |
| `bsp_spi_as_base(me)` | 向上转型 |
