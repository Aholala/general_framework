# BSP 通用基础设施 (bsp_common)

## 1. 模块概述

`bsp_common` 是 **整个 BSP 抽象层的基石**，它为所有外设对象（UART、SPI、CAN、ADC、PWM 等）提供统一的基础设施。该模块不依赖任何硬件，仅提供纯软件层面的类型定义、工具宏和基类管理，确保了整个 BSP 层在风格和行为上的一致性。

核心功能包括：

- **统一状态码** (`bsp_status_t`)：所有 BSP 函数返回相同类型的错误码，便于上层统一处理。
- **传输模式枚举** (`bsp_transfer_mode_t`)：阻塞、中断、DMA 三种标准模式。
- **事件与回调模型**：统一的事件枚举 (`bsp_event_t`) 和回调函数原型 (`bsp_event_callback_t`)。
- **设备基类** (`bsp_device_t`)：所有外设对象的“根”，包含虚表指针、设备句柄、魔数及初始化状态。
- **虚析构机制**：通过 `vptr->deinit` 实现派生类的多态析构。
- **容器宏**：`BSP_CONTAINER_OF` 和 `BSP_CONTAINER_OF_CONST`，用于从基类指针安全地获取派生类指针。

## 2. 设计思想

### 2.1 面向对象的多态设计

本模块采用 C 语言模拟 C++ 的面向对象机制：

- **继承**：所有具体外设对象（如 `bsp_uart_device_t`）将 `bsp_device_t` 作为**第一个成员**（通常命名为 `super`），实现单继承。
- **多态**：通过虚表指针 (`vptr`) 实现运行时多态。基类 `bsp_device_ops_t` 仅定义 `deinit`，派生类可扩展虚表。
- **封装**：对象内部状态（魔数、初始化标志）对外不可见，通过公共 API 访问。

### 2.2 零动态内存策略

整个 BSP 层**不使用动态内存分配**（`malloc`/`free`）。所有对象、虚表、缓冲区均由调用者静态分配或从内存池获取。这保证了：

- 行为可预测，无内存碎片风险。
- 适合实时系统和安全关键应用。
- 生命周期管理完全由调用者控制。

### 2.3 对象生命周期

每个 BSP 设备对象必须经历以下阶段：

1. **定义**：静态分配对象（`.data` 或 `.bss` 段）。
2. **构造**：先清零完整对象，再初始化派生字段，最后调用 `bsp_device_init` 初始化基类。
3. **运行**：通过基类指针调用虚函数，执行实际 I/O 操作。
4. **析构**：调用 `bsp_device_deinit`（触发虚析构），然后清空派生字段。

**关键规则**：

- `object_magic` 固定为 `0x4253504FU`（即 ASCII "BSP"），用于快速验证对象有效性。
- 初始化失败时必须确保 `is_initialized` 保持 `false`，避免留下“半生不熟”的对象。

## 3. 核心类型详解

### 3.1 状态码 (`bsp_status_t`)

所有 BSP 函数统一返回此枚举，禁止使用 `bool` 或 `int` 吞没错误原因。

| 状态码                        | 含义         | 使用场景                                   |
| :---------------------------- | :----------- | :----------------------------------------- |
| `BSP_STATUS_OK`               | 操作成功     | 一切正常                                   |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数非法     | 传入空指针、枚举值越界、长度超限           |
| `BSP_STATUS_OUT_OF_RANGE`     | 数值超范围   | ID 超限、分辨率非法、频率超出硬件支持      |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化 | 调用 API 时对象尚未 `init` 或已经 `deinit` |
| `BSP_STATUS_BUSY`             | 资源繁忙     | 异步操作正在进行，不能重复启动             |
| `BSP_STATUS_TIMEOUT`          | 等待超时     | 阻塞操作在指定时间内未完成                 |
| `BSP_STATUS_IO_ERROR`         | I/O 错误     | 总线故障、硬件异常、校验失败               |
| `BSP_STATUS_NO_RESOURCE`      | 资源不足     | 队列满、路由表满、内存池耗尽               |
| `BSP_STATUS_UNSUPPORTED`      | 功能不支持   | 可选虚函数未实现（如 `stop_dma`）          |

### 3.2 传输模式 (`bsp_transfer_mode_t`)

用于配置外设的数据传输方式：

```c
typedef enum {
    BSP_TRANSFER_MODE_BLOCKING = 0,  // 轮询等待，阻塞当前线程
    BSP_TRANSFER_MODE_INTERRUPT,     // 中断驱动，配合回调通知
    BSP_TRANSFER_MODE_DMA            // DMA 传输，适合大数据量
} bsp_transfer_mode_t;
```

可通过 `bsp_transfer_mode_is_valid()` 校验枚举值是否合法。

### 3.3 事件类型 (`bsp_event_t`)

用于回调通知的事件类型：

| 事件                          | 含义                                   |
| :---------------------------- | :------------------------------------- |
| `BSP_EVENT_TRANSMIT_COMPLETE` | 发送操作完成                           |
| `BSP_EVENT_RECEIVE_COMPLETE`  | 接收操作完成                           |
| `BSP_EVENT_TRANSFER_COMPLETE` | 通用传输完成（适用于双向操作）         |
| `BSP_EVENT_RECEIVE_PENDING`   | 有数据待接收（如 FIFO 非空）           |
| `BSP_EVENT_ABORT_COMPLETE`    | 中止操作完成                           |
| `BSP_EVENT_ERROR`             | 发生错误（具体错误码见 `status` 参数） |

### 3.4 回调函数原型 (`bsp_event_callback_t`)

```c
typedef void (*bsp_event_callback_t)(bsp_event_t event, bsp_status_t status,
                                     size_t transferred_size, void *user_context);
```

- **`event`**：触发回调的事件类型。
- **`status`**：本次操作的状态码，用于区分“成功完成”和“错误完成”。
- **`transferred_size`**：已传输的数据量（如字节数、帧数）。
- **`user_context`**：注册回调时传入的用户自定义指针。

**重要约束**：回调函数必须**非阻塞**，不应包含延时、信号量获取或长时间循环。推荐在回调中释放信号量或发送消息队列，将实际处理放在任务上下文。

## 4. 设备基类 (`bsp_device_t`)

### 4.1 结构定义

```c
struct bsp_device {
    const bsp_device_ops_t *vptr;   // 虚表指针（只读）
    void *device_handle;            // 平台相关句柄（不透明）
    uint32_t object_magic;          // 魔数，用于验证对象有效性
    bool is_initialized;            // 初始化完成标志
};
```

### 4.2 基类操作表

```c
typedef struct {
    bsp_status_t (*deinit)(bsp_device_t *const me);  // 虚析构函数
} bsp_device_ops_t;
```

### 4.3 公共 API

| 函数                        | 功能                                   | 返回值                     |
| :-------------------------- | :------------------------------------- | :------------------------- |
| `bsp_device_init`           | 初始化基类，绑定虚表和句柄             | `OK` 或 `INVALID_ARGUMENT` |
| `bsp_device_deinit`         | 反初始化，调用虚析构并清空所有基类字段 | 状态码                     |
| `bsp_device_is_initialized` | 综合检查魔数、状态、虚表、句柄         | `true` / `false`           |
| `bsp_device_get_handle`     | 返回设备句柄（用于底层驱动调用）       | 句柄指针或 `NULL`          |

**使用示例**：

```c
static bsp_device_t dev;
static const bsp_device_ops_t ops = { .deinit = my_deinit };

// 初始化
if (bsp_device_init(&dev, &ops, &some_handle) == BSP_STATUS_OK) {
    // 对象已可用
}

// 获取句柄
void *handle = bsp_device_get_handle(&dev);

// 反初始化
bsp_device_deinit(&dev);
```

## 5. 容器宏 (`BSP_CONTAINER_OF`)

### 5.1 作用

在虚函数实现中，通过基类指针反推出包含该基类的派生对象地址。

### 5.2 宏定义

```c
#define BSP_CONTAINER_OF(pointer, type, member) \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))

#define BSP_CONTAINER_OF_CONST(pointer, type, member) \
    ((const type *)((const uint8_t *)(pointer) - offsetof(type, member)))
```

### 5.3 使用规范

**派生类必须将基类作为第一个成员**：

```c
typedef struct {
    bsp_device_t super;           // 必须第一
    const bsp_uart_driver_ops_t *driver_ops;
    uint32_t baudrate;
} bsp_uart_device_t;
```

**在派生虚函数中使用**：

```c
static bsp_status_t bsp_uart_send(bsp_uart_t *base, ...) {
    // 从基类指针获取派生对象
    bsp_uart_device_t *dev = BSP_CONTAINER_OF(base, bsp_uart_device_t, super);
    // 现在可以使用 dev->driver_ops 等派生字段
    return dev->driver_ops->send(dev->super.device_handle, ...);
}
```

**只读操作使用常量版本**：

```c
static bsp_status_t bsp_uart_get_status(const bsp_uart_t *base, uint32_t *status) {
    const bsp_uart_device_t *dev = BSP_CONTAINER_OF_CONST(base, bsp_uart_device_t, super);
    // 只读访问
}
```

## 6. 派生类扩展方法

### 6.1 定义派生对象结构

```c
// 派生对象
typedef struct {
    bsp_device_t super;                     // 基类（第一成员）
    const bsp_uart_driver_ops_t *driver_ops; // 底层驱动表
    uint32_t baudrate;
    bsp_event_callback_t callback;
    void *user_context;
} bsp_uart_device_t;
```

### 6.2 定义派生虚表

```c
// 派生虚表，继承自 bsp_device_ops_t
typedef struct {
    bsp_device_ops_t super;                 // 基类虚表（第一成员）
    bsp_status_t (*send)(bsp_uart_t *, const uint8_t *, size_t, uint32_t);
    bsp_status_t (*receive)(bsp_uart_t *, uint8_t *, size_t, uint32_t);
    bsp_status_t (*set_callback)(bsp_uart_t *, bsp_event_callback_t, void *);
} bsp_uart_ops_t;
```

### 6.3 实现构造函数

```c
bsp_status_t bsp_uart_init(bsp_uart_device_t *dev, const bsp_uart_config_t *cfg) {
    // 1. 参数校验
    if ((dev == NULL) || (cfg == NULL) || (cfg->device_handle == NULL) ||
        (cfg->driver_ops == NULL)) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    // 2. 清零派生字段（可选）
    dev->baudrate = 0;
    dev->callback = NULL;
    dev->user_context = NULL;

    // 3. 初始化基类
    bsp_status_t st = bsp_device_init(&dev->super, &s_uart_ops.super, cfg->device_handle);
    if (st != BSP_STATUS_OK) {
        return st;
    }

    // 4. 初始化派生字段
    dev->driver_ops = cfg->driver_ops;
    dev->baudrate = cfg->baudrate;

    // 5. 调用底层驱动初始化（可选）
    if (dev->driver_ops->init != NULL) {
        return dev->driver_ops->init(dev->super.device_handle, cfg->baudrate);
    }
    return BSP_STATUS_OK;
}
```

### 6.4 实现虚析构函数

```c
// 派生类的 deinit 实现（虚函数）
static bsp_status_t bsp_uart_device_deinit(bsp_device_t *base) {
    bsp_uart_device_t *dev = BSP_CONTAINER_OF(base, bsp_uart_device_t, super);

    // 调用底层驱动去初始化
    if (dev->driver_ops->deinit != NULL) {
        return dev->driver_ops->deinit(dev->super.device_handle);
    }
    return BSP_STATUS_OK;
}

// 将 deinit 填入派生虚表
static const bsp_uart_ops_t s_uart_ops = {
    .super = { .deinit = bsp_uart_device_deinit },
    .send = bsp_uart_send,
    .receive = bsp_uart_receive,
    .set_callback = bsp_uart_set_callback,
};
```

## 7. 事件通知机制

所有外设模块统一使用 `bsp_event_callback_t` 作为回调原型，并在中断中调用各模块的 `bsp_xxx_notify` 函数：

```c
// 在 CAN 中断中
void CAN_IRQHandler(void) {
    // ... 处理中断
    bsp_can_notify(can_ptr, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, 1U);
}

// 在 UART 中断中
void UART_IRQHandler(void) {
    // ... 处理中断
    bsp_uart_notify(uart_ptr, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, rx_count);
}
```

各模块的 `notify` 函数最终会调用用户注册的回调：

```c
void bsp_uart_notify(bsp_uart_t *me, bsp_event_t event, bsp_status_t status, size_t size) {
    if ((me != NULL) && (me->callback != NULL)) {
        me->callback(event, status, size, me->user_context);
    }
}
```

## 8. 并发与线程安全

- **基类本身不提供互斥锁**：多个任务或中断访问同一对象时，必须由上层（应用层或 RTOS 封装）确保临界区保护。
- **ISR 安全操作**：`bsp_device_is_initialized()` 是只读函数，可在中断中安全调用。
- **禁止在中断中修改对象状态**：如 `bsp_device_deinit()` 不应在中断中调用。
- **回调函数**：若从 ISR 触发，回调也运行在中断上下文，必须遵循 ISR 编程规范。

## 9. 移植与使用建议

### 9.1 新模块开发流程

1. 在 `bsp_common.h` 基础上，创建 `bsp_xxx.h` 和 `bsp_xxx.c`。
2. 定义派生虚表，继承自 `bsp_device_ops_t`。
3. 定义派生对象结构，将 `bsp_device_t` 作为第一个成员。
4. 实现构造函数（`bsp_xxx_init`）、虚析构（`xxx_device_deinit`）及各类操作函数。
5. 实现事件通知函数（`bsp_xxx_notify`）。
6. 提供 `bsp_xxx_as_base()` 向上转型函数。

### 9.2 编码规范

- 所有公共函数名以 `bsp_` 为前缀。
- 派生对象的 `super` 成员必须是第一个字段。
- 虚表中 `super` 必须是第一个字段。
- 使用 `BSP_CONTAINER_OF` 而非强制类型转换，保证类型安全。
- 所有错误路径必须返回明确的 `bsp_status_t`。

## 10. 建议验证测试项

- [ ] 空指针、空虚表传入 `init` 返回 `INVALID_ARGUMENT`。
- [ ] 已初始化对象再次初始化（基类不阻止，派生类应自行判断防重入）。
- [ ] 调用 `deinit` 后对象被正确清空，`is_initialized` 返回 `false`。
- [ ] 派生类虚函数能通过 `container_of` 正确转换基类指针。
- [ ] 常量版本宏不丢失 `const` 限定（编译检查）。
- [ ] 所有状态码在不同派生类中一致传播。
- [ ] 回调函数在中断上下文中不阻塞。
- [ ] 多实例对象各自独立，互不干扰。
- [ ] `bsp_device_get_handle` 在对象未初始化时返回 `NULL`。

---

**总结**：`bsp_common` 是构建整个 BSP 框架的“地基”。通过提供统一的状态码、设备基类和容器宏，它确保了所有上层外设模块在接口风格和行为上的一致性。任何新增的外设抽象都应严格遵循此基础设施的规范，从而实现高内聚、低耦合、可移植的嵌入式软件架构。
