# BSP 外部中断通用抽象层 (bsp_exti)

## 1. 模块概述

`bsp_exti` 提供了对外部中断（EXTI）引脚的通用抽象，封装了中断源的启用、禁用、回调注册和事件通知。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 启用/禁用外部中断源。
- 注册/更换中断回调函数（支持用户上下文）。
- 统一的中断通知入口（供平台 ISR 调用）。
- 对象生命周期管理（初始化、反初始化）。

## 2. 设计边界

| **模块负责**                            | **模块不负责**                             |
| :-------------------------------------- | :----------------------------------------- |
| 中断源的启用/禁用抽象接口               | 中断触发边沿配置（上升沿、下降沿、双边沿） |
| 回调函数和用户上下文的存储与管理        | 输入引脚选择和复用配置                     |
| 统一的事件通知函数（`bsp_exti_notify`） | NVIC 优先级设置                            |
| 设备对象生命周期管理                    | 硬件去抖和滤波配置                         |
| 多实例独立回调上下文                    | 中断标志清除（由平台驱动负责）             |

**重要约束**：

- **中断上下文安全**：回调函数在 ISR 中执行，必须遵循 ISR 编程规范（非阻塞、快速返回）。
- **回调可空**：允许不注册回调，此时 `notify` 不做任何操作。
- **多实例独立**：每个 `bsp_exti_device_t` 对象独立保存回调与上下文，可共享同一驱动操作表。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_exti_t             (基类：增加 callback 和 user_context)
    └── bsp_exti_device_t  (派生类：持有 driver_ops)
```

- **`bsp_exti_t`**：应用层使用的基类指针，包含回调函数指针和用户上下文。
- **`bsp_exti_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_exti_ops_t` 继承自 `bsp_device_ops_t`，新增 `enable` 和 `disable`。

## 4. 核心类型

### 4.1 回调函数类型 (`bsp_exti_callback_t`)

```c
typedef void (*bsp_exti_callback_t)(bsp_exti_t *const me, void *user_context);
```

- **参数**：`me` 为触发中断的 EXTI 对象指针，`user_context` 为注册时传入的用户上下文。
- **约束**：回调在 ISR 上下文中执行，必须快速返回，不可阻塞或执行耗时操作。

### 4.2 配置结构 (`bsp_exti_config_t`)

```c
typedef struct {
    void *device_handle;                // 平台句柄
    const bsp_exti_driver_ops_t *driver_ops;
    bsp_exti_callback_t callback;       // 中断回调（可为 NULL）
    void *user_context;                 // 回调用户上下文
} bsp_exti_config_t;
```

### 4.3 底层驱动操作表 (`bsp_exti_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*enable)(void *handle);
    bsp_status_t (*disable)(void *handle);
} bsp_exti_driver_ops_t;
```

**必须实现的函数**：`enable` 和 `disable`。`init`/`deinit` 可选（但 `deinit` 需提供指针，可为空操作）。

## 5. API 参考

| 函数                    | 说明                                   | 返回值                                        |
| :---------------------- | :------------------------------------- | :-------------------------------------------- |
| `bsp_exti_init`         | 初始化 EXTI 对象，绑定句柄、驱动和回调 | `OK` / `INVALID_ARGUMENT`                     |
| `bsp_exti_as_base`      | 向上转型，获取基类指针                 | 基类指针或 `NULL`                             |
| `bsp_exti_set_callback` | 运行时更换回调函数和用户上下文         | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED` |
| `bsp_exti_enable`       | 启用外部中断（使能 NVIC 和硬件触发）   | 状态码                                        |
| `bsp_exti_disable`      | 禁用外部中断                           | 状态码                                        |
| `bsp_exti_notify`       | 中断通知入口（由平台 ISR 调用）        | 无返回值                                      |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_exti_driver.c
static bsp_status_t stm32_exti_init(void *handle) {
    // 配置 GPIO 引脚、触发边沿、NVIC 优先级等
    // 通常由 CubeMX 生成，此处仅作示意
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_exti_enable(void *handle) {
    HAL_NVIC_EnableIRQ(EXTIx_IRQn);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_exti_disable(void *handle) {
    HAL_NVIC_DisableIRQ(EXTIx_IRQn);
    return BSP_STATUS_OK;
}

const bsp_exti_driver_ops_t stm32_exti_driver = {
    .init = stm32_exti_init,
    .deinit = NULL,   // 若无需清理，提供空函数
    .enable = stm32_exti_enable,
    .disable = stm32_exti_disable,
};
```

### 6.2 应用层初始化和使用

```c
static bsp_exti_device_t s_imu_exti_dev;
static bsp_exti_t *s_imu_exti = NULL;

void board_exti_init(void) {
    bsp_exti_config_t cfg = {
        .device_handle = &imu_exti_handle,  // 平台定义的结构体
        .driver_ops = &stm32_exti_driver,
        .callback = imu_data_ready_callback,
        .user_context = &imu_device,
    };
    bsp_exti_init(&s_imu_exti_dev, &cfg);
    s_imu_exti = bsp_exti_as_base(&s_imu_exti_dev);
    bsp_exti_enable(s_imu_exti);
}

// 中断回调（在 ISR 中执行）
static void imu_data_ready_callback(bsp_exti_t *me, void *ctx) {
    imu_t *imu = (imu_t *)ctx;
    // 仅记录时间戳或释放信号量
    imu->last_irq_time = bsp_timebase_get_ms();
    xSemaphoreGiveFromISR(imu->data_sem, NULL);
}

// 平台 ISR（由 HAL 或裸机中断处理函数调用）
void EXTIx_IRQHandler(void) {
    // 清除硬件标志（平台驱动层负责）
    // 调用通知函数
    bsp_exti_notify(s_imu_exti);
}
```

### 6.3 运行时更换回调

```c
// 在任务上下文中更换回调（注意并发保护）
bsp_exti_disable(s_imu_exti);   // 先禁用中断
// 短临界区（或使用互斥锁）
bsp_exti_set_callback(s_imu_exti, new_callback, new_ctx);
bsp_exti_enable(s_imu_exti);
```

## 7. 中断回调约束

- **非阻塞**：回调中不能使用 `vTaskDelay`、`printf`、`HAL_Delay` 等阻塞函数。
- **快速执行**：只做必要操作，如记录时间戳、置标志位、释放信号量/消息队列。
- **不可重入**：若中断可能嵌套，需确保回调函数是线程安全的。
- **不访问外设**：避免在回调中执行 SPI/I2C 读写等长耗时操作，应推迟到任务上下文处理。

对于按键等需要软件去抖的场景，回调只记录边沿时刻，由任务定期查询当前电平并做滤波。

## 8. 生命周期与并发

- **初始化**：`bsp_exti_init` 后对象可用，但默认处于禁用状态（需调用 `enable`）。
- **启用/禁用**：多次调用 `enable`/`disable` 是幂等的（由平台驱动保证）。
- **回调更换**：若中断可能并发发生，更换回调前应先禁用中断，更换完成后再使能，以避免回调指针被部分更新。
- **反初始化**：应先调用 `disable`，再调用 `bsp_device_deinit` 触发虚析构。确保反初始化后不会再调用 `notify`。

## 9. 错误码速查

| 错误码                        | 触发场景                                             |
| :---------------------------- | :--------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空（对象、配置、句柄、驱动表）                 |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化即调用 `enable`/`disable`/`set_callback` |
| 平台驱动返回的其他状态码      | 由 `init`/`enable`/`disable`/`deinit` 返回           |

## 10. 移植要求

平台移植者需实现 `bsp_exti_driver_ops_t`：

- **`init`**（可选）：配置 GPIO 引脚、触发边沿、NVIC 优先级等，通常由硬件初始化代码完成。
- **`deinit`**（可选）：清理资源，如关闭时钟、复位引脚等。
- **`enable`**：使能中断（通常调用 `HAL_NVIC_EnableIRQ` 或等价操作）。
- **`disable`**：禁用中断（`HAL_NVIC_DisableIRQ`）。

**关键注意事项**：

- 平台端需维护一个从 `device_handle` 到 `bsp_exti_t*` 的映射，或在 ISR 中通过全局对象调用 `bsp_exti_notify`。推荐每个 EXTI 对象关联一个全局变量，由平台中断服务程序调用。
- 中断标志的清除必须在调用 `bsp_exti_notify` 之前或之后完成（由平台驱动决定），但必须在 ISR 返回前清除，以避免重复触发。
- 若硬件支持多个 EXTI 通道，平台驱动应确保 `device_handle` 能区分不同通道，并在 `enable`/`disable` 中操作对应的中断寄存器。

## 11. 建议验证测试项

- [ ] 空指针、未初始化对象调用 API 返回 `INVALID_ARGUMENT` / `NOT_INITIALIZED`。
- [ ] 使能后，外部触发能正确进入回调。
- [ ] 禁用后，外部触发不再进入回调。
- [ ] 回调中 `user_context` 与注册时一致。
- [ ] 运行时更换回调（先禁用后使能）后，新回调生效。
- [ ] 多个 EXTI 实例（不同引脚）独立工作，互不干扰。
- [ ] 高频边沿触发下，回调不会阻塞系统（如快速连续的 GPIO 翻转）。
- [ ] 反初始化（`deinit`）后，对象不再响应中断（需先禁用）。
- [ ] 回调设为 `NULL` 时，`notify` 不会产生异常。

---

**总结**：`bsp_exti` 为外部中断提供了轻量、可移植的抽象，将中断控制逻辑与具体应用处理分离。应用层只需注册简单的非阻塞回调，将复杂数据处理移至任务上下文，同时符合 RTOS 和实时系统的最佳实践。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
