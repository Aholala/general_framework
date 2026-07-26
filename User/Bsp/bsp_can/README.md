# bsp_can

`bsp_can` 是与芯片厂商无关的 Classic CAN 抽象，同时包含 CAN 帧分发器。目录按“外设能力”组织：

- `bsp_can.h/.c`：CAN 基类、驱动注入、多态接口和事件通知；
- `bsp_can_dispatcher.h/.c`：接收帧路由、任务上下文分发和运行统计。

分发器属于 CAN 接收能力的一部分，因此不再单独占用一个同级目录。

## 设计边界

本目录负责：

- 标准帧与扩展帧的数据模型；
- 数据帧与远程帧的数据模型；
- FIFO0、FIFO1 接收选择；
- 掩码过滤器配置；
- 启动、停止、发送、接收和发送邮箱余量查询；
- 从中断通知到任务分发的解耦；
- 多个协议或设备共享同一 CAN 总线时的帧路由。

本目录不负责：

- CAN 位时序、引脚复用和收发器使能；
- STM32 HAL、FDCAN 寄存器或具体中断名称；
- 电机协议、裁判系统协议或板间通信协议；
- 在中断中执行协议解析和控制算法。

上述硬件行为通过 `bsp_can_driver_ops_t` 注入，业务协议位于 Module 层。

## 对象模型

```text
bsp_device_t
└── bsp_can_t
    └── bsp_can_device_t
```

`bsp_can_t` 是模块依赖的基类。`bsp_can_device_t` 保存不透明 `device_handle` 和只读
`driver_ops`。模块只接收 `bsp_can_t *`，因此同一模块可以使用传统 CAN、FDCAN Classic
适配器或测试替身。

对象和操作表初始化后不得按值复制。对象、路由表和缓冲区均由调用者持有，不使用动态内存。

## 关键类型

### `bsp_can_frame_t`

- `identifier`：11 位或 29 位标识符；
- `id_type`：标准标识符或扩展标识符；
- `frame_type`：数据帧或远程帧；
- `data_length`：Classic CAN 有效载荷长度，范围为 0～8；
- `data[8]`：帧有效载荷。

### `bsp_can_filter_t`

- `identifier`、`mask`：硬件过滤器匹配值；
- `id_type`：过滤标准帧或扩展帧；
- `receive_fifo`：匹配帧进入 FIFO0 或 FIFO1；
- `filter_index`：由平台端解释的过滤器槽位。

### `bsp_can_driver_ops_t`

平台端至少应根据实际能力实现：

- `init`、`deinit`；
- `start`、`stop`；
- `configure_filter`；
- `transmit`、`receive`；
- `get_tx_free_level`。

公共接口会统一检查对象状态和参数。不存在的可选能力返回
`BSP_STATUS_UNSUPPORTED`，平台错误通过 `bsp_status_t` 原样向上传递。

## 初始化顺序

```c
static bsp_can_device_t board_can_device;

static const bsp_can_config_t board_can_config = {
    .device_handle = &platform_can_handle,
    .driver_ops = &platform_can_driver_ops,
    .callback = NULL,
    .user_context = NULL,
};

bsp_status_t status = bsp_can_init(&board_can_device, &board_can_config);
if (status == BSP_STATUS_OK)
{
    status = bsp_can_start(bsp_can_as_base(&board_can_device));
}
```

`device_handle` 的具体类型只允许平台端知道。通用 BSP 和 Module 不应包含厂商 HAL 头文件。

## CAN 分发器

`bsp_can_dispatcher_t` 使用调用者提供的 `bsp_can_route_t` 数组，不分配内存。每条路由由
标识符、掩码、标识符类型、回调和用户上下文组成。

```c
static bsp_can_route_t can_route_storage[16];
static bsp_can_dispatcher_t can_dispatcher;

static const bsp_can_dispatcher_config_t dispatcher_config = {
    .can = &board_can_device.super,
    .receive_fifo = BSP_CAN_RX_FIFO_0,
    .route_storage = can_route_storage,
    .route_capacity = 16,
    .maximum_frames_per_process = 8,
};

bsp_can_dispatcher_init(&can_dispatcher, &dispatcher_config);
bsp_can_dispatcher_add_route(
    &can_dispatcher,
    0x201U,
    0x7FFU,
    BSP_CAN_ID_STANDARD,
    motor_feedback_callback,
    &motor,
    NULL);
```

掩码匹配规则为：

```text
(received_identifier & mask) == (route_identifier & mask)
```

完全匹配标准帧时使用 `mask = 0x7FFU`，完全匹配扩展帧时使用
`mask = 0x1FFFFFFFU`。

## 中断与任务上下文

平台 CAN 中断只调用：

```c
bsp_can_notify(can, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, 1U);
```

分发器注册的事件回调只设置 `receive_pending`，不执行协议回调。主循环或 RTOS 任务调用：

```c
size_t processed_frame_count = 0U;
bsp_can_dispatcher_process(&can_dispatcher, &processed_frame_count);
```

`maximum_frames_per_process` 限制单次处理帧数，防止 CAN 高负载长期占用控制任务。处理期间
再次收到中断不会递归进入用户回调。

## 统计与诊断

分发器维护：

- `received_frame_count`：成功读取的帧数；
- `unmatched_frame_count`：没有路由匹配的帧数；
- `receive_error_count`：底层读取失败次数；
- `receive_pending`：仍有待处理接收事件；
- `is_processing`：防止重入。

比赛项目建议周期上报这些计数，并结合 FDCAN 协议状态监测总线错误和 Bus-Off。

## 移植要求

1. 在平台端实现一份 `bsp_can_driver_ops_t`。
2. 在 `board_config.h` 定义逻辑实例、路由容量和 FIFO 选择。
3. 在板级装配文件绑定 CubeMX 句柄、驱动操作表和中断通知。
4. 将 `bsp_can_t *` 注入电机、板间通信等 Module。
5. 禁止 Module 直接引用 `hcan`、`hfdcan` 或 HAL 回调。

## 生命周期与并发约束

- `bsp_can_init` 成功后才能配置过滤器和启动总线；
- 停止接收后再执行反初始化；
- 路由增删应在分发器未处理帧时完成，或由外部临界区保护；
- 用户回调不得阻塞，不应在回调中修改当前路由表；
- DMA、缓存一致性和 ISR 优先级由平台端负责；
- 路由对象、CAN 对象和用户上下文必须在整个使用期保持有效。

## 建议验证

- 空指针、未初始化对象和缺失操作表；
- 标准/扩展标识符边界与 0～8 字节长度；
- 精确匹配、掩码匹配、禁用路由和未匹配帧；
- FIFO 为空、底层接收错误和单次处理上限；
- 处理期间重复通知以及回调重入保护；
- 两个独立 CAN 实例和两套独立路由表；
- 停止、反初始化后接口拒绝访问。
