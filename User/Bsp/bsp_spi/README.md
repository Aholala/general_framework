# bsp_spi

通用 SPI 总线接口，支持发送、接收、全双工交换、中止和忙状态查询。SPI 模式、位宽、时钟
和片选策略由平台端与设备 Module 共同决定。

## 传输接口

- `bsp_spi_transmit`：只发送；
- `bsp_spi_receive`：只接收，平台端负责产生时钟；
- `bsp_spi_exchange`：等长全双工交换；
- `bsp_spi_abort`：终止异步事务；
- `bsp_spi_get_busy`：查询当前事务。

全部接口接受 Blocking、Interrupt 或 DMA 模式。不存在的模式返回
`BSP_STATUS_UNSUPPORTED`。

## 初始化

```c
static bsp_spi_device_t sensor_spi;
static const bsp_spi_config_t config = {
    .device_handle = &platform_spi,
    .driver_ops = &platform_spi_driver_ops,
    .callback = spi_event_callback,
    .user_context = &sensor,
};

bsp_spi_init(&sensor_spi, &config);
```

## 片选边界

通用 SPI 不自动控制片选，因为不同器件对事务边界、延时和多段读写要求不同。设备 Module
通过注入的 `bsp_gpio_t *` 或专用平台事务接口控制片选，并保证一次协议事务期间总线不被
其他设备抢占。

## 异步缓冲区

IRQ/DMA 模式下发送和接收数组必须持续有效到完成、中止或错误通知。平台端处理 D-Cache
清理/失效后调用 `bsp_spi_notify`。通用层不隐藏复制大缓冲区。

## 共享总线

多个传感器共用 SPI 时，外部总线管理器必须串行化事务，并在切换设备时处理不同的 CPOL、
CPHA、频率或位宽。不能只靠各自片选而并发调用同一 SPI 对象。

## 错误恢复

超时或 DMA 错误后先调用 `abort`，确认完成事件，再释放片选和总线锁。重复失败应上报设备
健康管理，不应在控制周期无限重试。

## 建议验证

- 三种传输接口和三种模式；
- 0 字节、最大长度及空指针；
- CPOL/CPHA 和频率切换；
- DMA 完成、中止和错误；
- 片选事务完整性；
- 多设备共享和忙冲突；
- 缓冲区缓存一致性。
