# bsp_fdcan

`bsp_fdcan` 提供与芯片厂商无关的 CAN FD 接口，并在同一目录提供 Classic CAN
适配器：

- `bsp_fdcan.h/.c`：CAN FD 基类和驱动注入；
- `bsp_fdcan_classic_adapter.h/.c`：把 FDCAN 的 Classic 帧能力适配为
  `bsp_can_t`。

适配器是 FDCAN 的一种使用方式，不再作为独立外设目录。

## 功能范围

- Classic CAN、CAN FD 无 BRS、CAN FD 带 BRS 三种格式；
- 标准/扩展标识符和数据/远程帧；
- 0～64 字节有效载荷；
- FIFO0、FIFO1 接收；
- 过滤器配置、启动、停止、发送和接收；
- 发送队列余量查询；
- Bus-Off、Error Passive、Warning 和错误计数查询；
- 将 FDCAN Classic 帧桥接到 `bsp_can_t` 多态接口。

位时序、消息 RAM、GPIO、收发器 STBY 引脚和厂商 HAL 均属于平台端。

## 对象模型

```text
bsp_device_t
└── bsp_fdcan_t
    └── bsp_fdcan_device_t

bsp_can_t
└── bsp_can_device_t
    └── bsp_fdcan_classic_adapter_t
```

`bsp_fdcan_device_t` 持有平台 `device_handle` 和
`bsp_fdcan_driver_ops_t`。Classic 适配器本身是一个 `bsp_can_device_t` 派生对象，
内部组合一个 `bsp_fdcan_t *`，因此无需复制 FDCAN 对象。

## 帧格式

`bsp_fdcan_frame_t` 的 `format` 可取：

- `BSP_FDCAN_FORMAT_CLASSIC`：Classic CAN，最大 8 字节；
- `BSP_FDCAN_FORMAT_FD_NO_BRS`：CAN FD，仲裁段与数据段相同速率；
- `BSP_FDCAN_FORMAT_FD_BRS`：CAN FD，数据段切换到更高比特率。

平台端必须校验实际控制器支持的 DLC。公共层使用字节长度，平台驱动负责字节长度与硬件
DLC 编码之间的转换。

## 初始化与使用

```c
static bsp_fdcan_device_t board_fdcan_device;

static const bsp_fdcan_config_t fdcan_config = {
    .device_handle = &platform_fdcan_handle,
    .driver_ops = &platform_fdcan_driver_ops,
    .callback = fdcan_event_callback,
    .user_context = NULL,
};

bsp_status_t status = bsp_fdcan_init(&board_fdcan_device, &fdcan_config);
if (status == BSP_STATUS_OK)
{
    status = bsp_fdcan_start(bsp_fdcan_as_base(&board_fdcan_device));
}
```

发送 FD 帧：

```c
bsp_fdcan_frame_t frame = {
    .identifier = 0x120U,
    .id_type = BSP_CAN_ID_STANDARD,
    .frame_type = BSP_CAN_FRAME_DATA,
    .format = BSP_FDCAN_FORMAT_FD_BRS,
    .data_length = 16U,
};

bsp_fdcan_transmit(bsp_fdcan_as_base(&board_fdcan_device), &frame, 2U);
```

## Classic CAN 适配器

大疆电机等设备只需要 8 字节 Classic CAN。若硬件外设是 FDCAN，按下列顺序装配：

```c
static bsp_fdcan_classic_adapter_t classic_adapter;

static const bsp_fdcan_classic_adapter_config_t adapter_config = {
    .fdcan = &board_fdcan_device.super,
    .callback = NULL,
    .user_context = NULL,
};

bsp_fdcan_classic_adapter_init(&classic_adapter, &adapter_config);
bsp_can_t *motor_can = bsp_fdcan_classic_adapter_as_can(&classic_adapter);
```

之后 `module_dji_motor`、`module_robot_link` 和 `bsp_can_dispatcher` 只看到
`bsp_can_t *`，不感知底层是 bxCAN 还是 FDCAN。

适配器只接受：

- `BSP_FDCAN_FORMAT_CLASSIC`；
- `data_length <= 8U`；
- `bsp_can_frame_t` 能表达的过滤和 FIFO 配置。

它不会把 CAN FD 长帧截断成 Classic 帧。

## 协议状态与赛场恢复

`bsp_fdcan_get_protocol_status` 返回：

- `is_bus_off`；
- `is_error_passive`；
- `has_warning`；
- 发送和接收错误计数；
- 平台最后错误码。

通用层只报告状态，不擅自重置硬件。比赛项目应由上层健康管理器决定：

1. 立即禁止相关执行器输出；
2. 记录故障和时间戳；
3. 停止并重新初始化总线；
4. 重新配置过滤器；
5. 确认关键设备反馈恢复；
6. 显式解除故障锁存并重新使能。

## 中断入口

平台端在发送完成、收到帧或发生错误时调用：

```c
bsp_fdcan_notify(fdcan, event, status, transferred_size);
```

通知函数只转发事件。协议解析应通过 Classic CAN 分发器或 Module 的任务上下文完成。
中断函数不得等待发送邮箱、打印日志或运行控制算法。

## 内存和所有权

- FDCAN 对象与 Classic 适配器由调用者静态分配；
- 适配器不拥有 `fdcan`，不得先销毁被引用的 FDCAN；
- 发送帧在同步驱动返回前必须有效；
- 若平台发送是异步的，平台端必须明确复制策略或保持调用者缓冲区生命周期；
- 所有操作表使用 `static const`，运行状态只存放在实例内。

## 移植检查

- 把字节长度正确转换为硬件 DLC；
- 根据帧格式设置 FDF 和 BRS；
- 正确区分标准与扩展过滤器；
- 正确路由 FIFO0、FIFO1 中断；
- 提供协议状态和 Bus-Off 错误；
- 若使用 D-Cache，处理消息缓冲区一致性；
- 不在通用 BSP 中包含厂商头文件；
- 由 `board_config.h` 选择逻辑总线、路由容量和恢复策略。

## 建议验证

- Classic 0～8 字节和 FD 12、16、20、24、32、48、64 字节；
- BRS 开关、标准/扩展帧和远程帧限制；
- 非法长度、未实现操作和未初始化对象；
- FIFO0、FIFO1 接收及过滤器配置；
- 发送队列满、超时、Error Passive 和 Bus-Off；
- Classic 适配器双向转换与长帧拒绝；
- 两个 FDCAN 实例和两个 Classic 适配器互不影响；
- 停止、反初始化和故障恢复顺序。
