# bsp_common

所有多态 BSP 对象共享的基础设施：统一状态码、传输模式、事件回调、设备基类、虚析构和
`container_of`。

## 设备基类

```c
struct bsp_device
{
    const bsp_device_ops_t *vptr;
    void *device_handle;
    uint32_t object_magic;
    bool is_initialized;
};
```

- `vptr` 指向不可变操作表；
- `device_handle` 是平台端不透明句柄；
- `object_magic` 用于识别有效构造对象；
- `is_initialized` 表示构造是否完整提交。

派生对象必须把 `super` 放在第一个成员。初始化后的多态对象不得按值复制，也不得把不相关
结构体强制转换成基类。

## 公共接口

- `bsp_device_init`：初始化基类并绑定虚表、句柄；
- `bsp_device_deinit`：校验对象并虚调用派生析构；
- `bsp_device_is_initialized`：同时检查魔数、虚表和状态；
- `bsp_device_get_handle`：只读取得平台句柄；
- `bsp_transfer_mode_is_valid`：校验 Blocking、Interrupt、DMA。

具体 BSP 的构造函数负责先清空完整对象，再初始化派生字段和基类。任何失败都必须留下可
识别的未初始化状态。

## 状态码

- `BSP_STATUS_OK`：操作完成；
- `INVALID_ARGUMENT`：指针、枚举、长度或范围错误；
- `OUT_OF_RANGE`：数值超出硬件/协议范围；
- `NOT_INITIALIZED`：对象生命周期无效；
- `BUSY`：异步操作仍在进行；
- `TIMEOUT`：等待超时；
- `IO_ERROR`：平台或总线错误；
- `NO_RESOURCE`：队列、路由或硬件资源不足；
- `UNSUPPORTED`：可选虚操作未实现。

禁止用布尔值吞掉错误原因，也不能把 `UNSUPPORTED` 当作成功。

## 事件模型

`bsp_event_callback_t` 统一携带事件、状态、传输长度和用户上下文。事件包括发送完成、接收
完成、传输完成、接收待处理、中止完成和错误。

平台 ISR 调用各外设的 `bsp_xxx_notify`，具体 BSP 再调用实例回调。回调必须非阻塞，协议
解析优先放在任务上下文。

## `container_of`

派生虚函数使用：

```c
bsp_uart_device_t *device =
    BSP_CONTAINER_OF(me, bsp_uart_device_t, super);
```

只读对象使用 `BSP_CONTAINER_OF_CONST`，避免丢失 const 限定。传入成员必须确实属于目标
结构体，错误使用会产生未定义行为。

## 所有权和并发

- 基类不拥有 `device_handle`；
- 操作表和句柄必须覆盖对象生命周期；
- 不使用动态内存；
- 单个对象的并发访问由平台锁或调用方互斥；
- ISR 与任务共享字段需要明确单生产者/单消费者或临界区设计。

## 建议验证

- 空对象、空虚表和无效魔数；
- 重复析构和未初始化析构；
- 缺失可选操作；
- 两个不同派生类通过同一基类接口调用；
- const `container_of`；
- 所有状态码向上传播。
