# bsp_i2c

通用 I2C 主机接口，支持直接收发、8/16 位寄存器地址访问、设备就绪探测、中止和忙状态
查询，三种传输模式统一使用同一对象。

## 地址规则

公共接口参数命名为 `address_7bit`，必须传 7 位设备地址，不允许调用方预先左移。平台
驱动负责转换成 HAL 或寄存器需要的格式。

寄存器访问使用 `BSP_I2C_MEMORY_ADDRESS_8_BIT` 或
`BSP_I2C_MEMORY_ADDRESS_16_BIT`。寄存器字节序由目标器件协议和平台操作确定。

## 传输模式

- `BSP_TRANSFER_MODE_BLOCKING`：在 `timeout_ms` 内同步完成；
- `BSP_TRANSFER_MODE_INTERRUPT`：启动异步传输，完成后通知；
- `BSP_TRANSFER_MODE_DMA`：启动 DMA，完成后通知。

平台不支持的模式返回 `BSP_STATUS_UNSUPPORTED`。

## 初始化与寄存器读取

```c
bsp_i2c_init(&i2c_device, &config);

uint8_t chip_id;
bsp_i2c_memory_read(
    bsp_i2c_as_base(&i2c_device),
    0x68U,
    chip_id_register,
    BSP_I2C_MEMORY_ADDRESS_8_BIT,
    &chip_id,
    sizeof(chip_id),
    BSP_TRANSFER_MODE_BLOCKING,
    10U);
```

`bsp_i2c_is_device_ready` 只用于初始化和诊断，不应在高频控制周期扫描整条总线。

## 异步缓冲区

Interrupt/DMA 模式下，发送和接收缓冲区必须保持有效，直到完成、错误或中止事件。平台端
调用 `bsp_i2c_notify`，回调只做轻量事件转交。

`bsp_i2c_get_busy` 可防止覆盖尚未完成的事务，`bsp_i2c_abort` 用于超时恢复。

## 总线恢复

SDA 被从机拉低时，恢复时钟脉冲、STOP 生成和外设重初始化属于平台端。上层应统计超时，
必要时停止相关设备输出后执行恢复，而不是无限重试阻塞控制任务。

## 并发和所有权

一条 I2C 总线通常一次只有一个事务。多个 Module 共享对象时必须通过总线管理器或外部
互斥串行化。对象不复制异步数据，也不拥有用户缓冲区。

## 建议验证

- 7 位地址边界；
- 8/16 位寄存器地址；
- 阻塞、IRQ、DMA 三种模式；
- NACK、仲裁丢失、超时和总线忙；
- 中止与恢复；
- 多设备共享总线；
- 缓冲区生命周期和缓存一致性。
