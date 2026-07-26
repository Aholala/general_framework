# module_referee

RoboMaster 裁判系统串口协议的流式收发框架，提供帧同步、CRC8/CRC16、分包/粘包处理、
命令路由、在线超时、发送序号和完整统计。具体比赛命令数据结构由上层按官方协议版本定义。

## 文件

- `module_referee.h/.c`：流解析、路由、发送和生命周期；
- `module_referee_crc.h/.c`：CRC8 与 CRC16；
- `README.md`：接口和集成约束。

## 调用者缓冲区

配置需要四块独立存储：

- `receive_buffer`：USART DMA 正在写入；
- `processing_buffer`：任务解析当前接收块；
- `stream_buffer`：保存跨回调残留和粘连帧；
- `transmit_buffer`：构造并异步发送完整帧。

处理缓冲容量不得小于接收容量。流缓冲和发送缓冲必须容纳项目允许的最大裁判帧。所有内存
由调用者静态持有。

## 接收路径

```text
USART DMA/idle ISR
  -> copy receive_buffer to processing_buffer
  -> immediately restart DMA
  -> set receive pending
task: module_referee_update
  -> append to stream_buffer
  -> find 0xA5
  -> verify CRC8 header
  -> wait for complete frame
  -> verify CRC16
  -> route command
```

无效字节逐步丢弃以重新同步。未知命令交给默认处理器或只计数，不会破坏后续帧。

## 路由

`module_referee_route_t` 由命令 ID、处理函数和用户上下文组成，配置数组建议
`static const`。回调收到 payload 指针、长度、序号和命令 ID；payload 只在回调期间有效，
需要长期保存的数据必须复制到调用者状态结构。

## 发送

`module_referee_build_frame` 可独立构造带序号和 CRC 的帧。
`module_referee_transmit` 使用对象发送缓冲区，异步发送完成前返回 BUSY，禁止覆盖。

## 在线和统计

有效 CRC16 帧刷新在线时间。统计包含接收、处理、未知命令、CRC8/CRC16 错误、超长帧、
丢弃字节、接收覆盖和 DMA 重启错误。比赛中建议低频上报这些数据。

## 协议版本边界

本框架不硬编码所有裁判命令结构，避免官方协议升级导致底层重写。上层解析器必须按当前
赛事手册核对命令 ID、payload 长度、字节序和字段缩放。

## 建议验证

- 单帧、分包、粘包和帧前噪声；
- CRC8/CRC16 错误；
- 超长 payload 和流缓冲不足；
- 未知命令与默认路由；
- DMA 立即重启和 overrun；
- 异步发送 BUSY 与序号回绕；
- 在线超时和统计饱和。
