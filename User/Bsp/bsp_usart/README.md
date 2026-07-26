# bsp_usart

通用 USART/UART 字节流接口，支持固定长度接收、空闲线接收、阻塞/中断/DMA 传输、中止和
忙状态查询。

## 功能接口

- `bsp_usart_transmit`；
- `bsp_usart_receive`；
- `bsp_usart_receive_to_idle`；
- `bsp_usart_abort`；
- `bsp_usart_get_busy`；
- `bsp_usart_set_callback`；
- `bsp_usart_notify`。

波特率、字长、校验、停止位、反相和 DMA 通道由平台端配置。

## 空闲线接收

`receive_to_idle` 最适合 DR16、裁判系统和不定长协议。调用者提供缓冲区容量，平台在收到
空闲线或缓冲区满时通过 `transferred_size` 报告实际长度。

```c
bsp_usart_receive_to_idle(
    usart,
    receive_buffer,
    receive_capacity,
    BSP_TRANSFER_MODE_DMA,
    0U);
```

Module 的中断回调应立即保存长度、重启 DMA，并把解析留给任务上下文。

## 异步所有权

异步发送与接收不会由通用 BSP 自动复制。缓冲区在完成、中止或错误事件前必须保持有效且
不能被覆盖。双缓冲、环形缓冲或帧队列由 Module 根据协议流量选择。

## 回调

平台 HAL 回调映射到具体对象后调用 `bsp_usart_notify`。不得通过比较全局 `huart`
在通用层寻找对象。每个实例独立保存回调与用户上下文。

## 错误恢复

帧错误、噪声、溢出或 DMA 错误应通知 `BSP_EVENT_ERROR`。Module 统计错误并决定重新启动
接收。高频错误不应在 ISR 中打印日志。

## 并发

同一串口的发送队列和接收状态需要单一所有者或外部锁。`get_busy` 只能反映平台状态，
不能替代多任务事务管理。

## 建议验证

- 固定长度和空闲线接收；
- Blocking、IRQ 和 DMA；
- 零长度、缓冲区满和连续短帧；
- 发送忙、接收忙和中止；
- 错误后重启 DMA；
- 两个串口实例；
- 异步缓冲区生命周期。
