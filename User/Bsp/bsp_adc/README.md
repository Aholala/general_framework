# BSP ADC 通用抽象层 —— 完整使用指南

## 1. 模块概述

`bsp_adc` 是一个**面向对象的通用 ADC 通道抽象层**。它的核心设计目标是：

- **统一操作**：屏蔽轮询、中断、DMA 三种读取模式的差异。
- **类型安全**：通过 `bsp_adc_t`（基类）操作对象，通过 `bsp_adc_device_t`（派生类）持有具体驱动句柄。

模块位于 **BSP（板级支持包）** 层，向上为 **Module（模块层）** 或 **Application（应用层）** 提供标准化的 ADC 数值获取接口。

---

## 2. 核心类型解析

| 类型                   | 作用                                                                      | 可见性                |
| :--------------------- | :------------------------------------------------------------------------ | :-------------------- |
| `bsp_adc_t`            | **抽象基类**，包含回调、参考电压、最大量程。应用层主要操作此指针。        | 公开（应用层持有）    |
| `bsp_adc_device_t`     | **派生设备类**，包含底层驱动表(`driver_ops`)和逻辑通道号(`channel`)。     | 静态分配（BSP层持有） |
| `bsp_adc_ops_t`        | **高层虚表**，桥接应用层调用到底层转发函数。通常由 `bsp_adc.c` 自动填充。 | 内部使用              |
| `bsp_adc_driver_ops_t` | **底层驱动契约**，由具体 MCU 移植者实现（如 `stm32_adc_driver.c`）。      | 平台移植者实现        |
| `bsp_adc_config_t`     | **构造参数**，用于初始化时注入句柄、分辨率、参考电压等。                  | 板级配置              |

---

## 3. 如何移植（平台适配者必读）

如果你需要将本模块适配到新的 MCU（如 TI TMS320、NXP S32K），你必须实现 `bsp_adc_driver_ops_t` 结构体。

### 3.1 必须实现的接口（不可为 NULL）

在 `bsp_adc_init` 的参数校验中，以下四个函数指针**必须提供**：

```c
driver_ops->start        // 启动单次/连续转换
driver_ops->stop         // 停止转换
driver_ops->calibrate    // 硬件校准（如内部参考校准、偏移消除）
driver_ops->read_raw     // 阻塞式读取原始寄存器值
```

> **注意**：`init` 和 `deinit` 是**可选**的，若为 NULL 则跳过。`start_dma` / `stop_dma` 若不支持，必须置为 NULL，公共接口会返回 `BSP_STATUS_UNSUPPORTED`。

### 3.2 移植契约（重要）

- **`device_handle`**：由平台驱动提供（如 `ADC_HandleTypeDef *`），在配置中传入。
- **`channel`**：逻辑通道号（如 `ADC_CHANNEL_0`），由板级映射文件定义。
- **`read_raw` 超时**：内部必须使用轮询等待转换完成，超时后返回 `BSP_STATUS_TIMEOUT`。
- **多通道共享**：若多个 `bsp_adc_device_t` 绑定同一个物理 ADC，平台驱动需负责序列扫描仲裁，通用层不处理互斥。

---

## 4. 如何使用（应用开发者视角）

### 4.1 步骤一：定义板级配置（通常在 `board_xxx.c`）

```c
// 1. 声明 ADC 设备对象（通常全局静态分配）
static bsp_adc_device_t s_battery_adc_dev;
static bsp_adc_device_t s_temperature_adc_dev;

// 2. 定义配置（参考电压、分辨率需与硬件严格一致）
static const bsp_adc_config_t s_battery_config = {
    .device_handle      = &hadc1,              // HAL 句柄或自定义句柄
    .driver_ops         = &stm32_adc_driver,   // 平台提供的驱动表
    .channel            = BOARD_ADC_BATTERY,   // 板级宏定义（如 0）
    .resolution_bits    = 12U,                 // 12位分辨率
    .reference_voltage_v = 3.3F,               // VDDA = 3.3V
    .callback           = NULL,                // 暂不注册回调
    .user_context       = NULL,
};

// 3. 初始化
void board_adc_init(void) {
    bsp_adc_init(&s_battery_adc_dev, &s_battery_config);
    // 获取基类指针供后续操作使用
    bsp_adc_t *adc = bsp_adc_as_base(&s_battery_adc_dev);
}
```

### 4.2 步骤二：校准与启动生命周期

```c
bsp_adc_t *adc = bsp_adc_as_base(&s_battery_adc_dev);

// 必须先校准（若硬件需要）
if (bsp_adc_calibrate(adc) == BSP_STATUS_OK) {
    // 启动转换（连续模式或单次模式取决于驱动实现）
    bsp_adc_start(adc);
}
```

> **生命周期顺序**：`init` -> (可选 `set_callback`) -> `calibrate` -> `start` -> `read`/`dma` -> `stop` -> (隐式 `deinit` 随设备销毁)。

### 4.3 步骤三：读取数据（三种方式）

#### A. 阻塞式原始值（适用于低频轮询）

```c
uint32_t raw;
if (bsp_adc_read_raw(adc, &raw, 100) == BSP_STATUS_OK) {
    printf("Raw: %lu\n", raw);
}
```

#### B. 归一化（0.0 ~ 1.0）

```c
float norm;
bsp_adc_read_normalized(adc, &norm, 100);
// 适用于百分比展示或上层算法归一化输入
```

#### C. 直接电压换算（浮点）

```c
float voltage;
bsp_adc_read_voltage(adc, &voltage, 100);
// 注意：此值为 ADC 引脚输入电压。若外部有分压电阻，需在 Module 层换算。
```

### 4.4 步骤四：DMA 批量采样（高性能场景）

```c
#define SAMPLE_COUNT 256
static uint32_t dma_buffer[SAMPLE_COUNT] __attribute__((aligned(32))); // 注意缓存对齐

// 注册回调以接收完成事件
bsp_adc_set_callback(adc, adc_event_handler, NULL);

// 启动 DMA
if (bsp_adc_start_dma(adc, dma_buffer, SAMPLE_COUNT) == BSP_STATUS_OK) {
    // 等待回调或任务通知
}

// 事件回调（通常由中断上下文触发）
static void adc_event_handler(bsp_event_t event, bsp_status_t status,
                              size_t transferred, void *ctx) {
    if (event == BSP_EVENT_DMA_COMPLETE) {
        // 此时 transferred == SAMPLE_COUNT
        // 注意：不可在此处做耗时滤波或 printf，应发送信号量给任务
        xSemaphoreGive(dma_sem);
    }
}

// 停止 DMA（通常在反初始化或切换模式时）
bsp_adc_stop_dma(adc);
```

---

## 5. DMA 缓冲区生命周期与缓存一致性（高风险必看）

- **所有权**：`sample_buffer` 必须由调用者持有，在 `stop_dma` 或回调触发之前**不可被释放或覆盖**（包括局部数组出作用域）。
- **Cache 维护**：若 MCU 带 D-Cache，平台驱动层在 DMA 启动前应做 `Clean`（将数据刷新到内存），在完成回调中应做 `Invalidate`（使缓存失效，保证 CPU 读到最新外设数据）。
- **双缓冲**：本抽象层**未内置**双缓冲切换机制，若需要 Ping-Pong 模式，请在应用层维护两个 buffer 并在回调中切换。

---

## 6. 回调与任务上下文隔离（RTOS 设计规范）

`bsp_adc_notify` 通常由**中断服务程序（ISR）**调用。因此：

- **禁止**在回调函数中执行阻塞操作（`printf`、`vTaskDelay`、`malloc`）。
- **推荐**在回调中释放信号量 (`xSemaphoreGiveFromISR`) 或发送消息队列，将滤波、均值、物理量换算放到任务中处理。

```c
// 错误示例（禁止）
static void my_callback(...) {
    float avg = calculate_average(buffer); // 耗时操作，禁止！
}

// 正确示例
static void my_callback(...) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(adc_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
```

---

## 7. 错误码速查

| 错误码                        | 触发场景                                                            |
| :---------------------------- | :------------------------------------------------------------------ |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、分辨率 >31、参考电压 ≤0 或非有限数、驱动表必要函数为 NULL |
| `BSP_STATUS_NOT_INITIALIZED`  | 未调用 `bsp_adc_init` 直接操作                                      |
| `BSP_STATUS_UNSUPPORTED`      | 调用 `start_dma` 但驱动未实现，或 `stop_dma` 未实现                 |
| `BSP_STATUS_TIMEOUT`          | `read_raw` 在指定毫秒内未完成                                       |
| `BSP_STATUS_IO_ERROR`         | 读取的原始值超过 `maximum_raw_value`（硬件异常或配置错误）          |
| `BSP_STATUS_BUSY`             | 重复启动 DMA 或 ADC（由底层驱动返回）                               |

---

## 8. 精度与外部电路设计建议

- **参考电压**：`reference_voltage_v` 仅用于换算。若 VDDA 不稳定，需在应用层引入 VREF 实测校准系数。
- **分压电阻**：若测量 12V 电池，外部有 `R1/R2` 分压，则真实电压 = `(adc_voltage) * ( (R1+R2) / R2 )`。**此换算不应放入通用 ADC 层**，应由模块层（如 `battery_module.c`）处理。
- **非线性校准**：若 ADC 积分非线性(INL)明显，建议在应用层使用多项式拟合或查表修正。

---

## 9. 典型初始化序列（完整代码示例）

```c
// board_adc.c
static bsp_adc_device_t s_adc_dev;
static bsp_adc_t *s_adc_ptr;

void board_adc_early_init(void) {
    bsp_adc_config_t cfg = {
        .device_handle = &hadc1,
        .driver_ops = &stm32_adc_driver_ops,
        .channel = 5,
        .resolution_bits = 12,
        .reference_voltage_v = 3.3f,
        .callback = app_adc_callback,
        .user_context = NULL,
    };
    bsp_adc_init(&s_adc_dev, &cfg);
    s_adc_ptr = bsp_adc_as_base(&s_adc_dev);
}

void board_adc_start_conversion(void) {
    bsp_adc_calibrate(s_adc_ptr);
    bsp_adc_start(s_adc_ptr);
}

// app_task.c
void adc_reader_task(void *pv) {
    float voltage;
    while(1) {
        if (bsp_adc_read_voltage(s_adc_ptr, &voltage, 500) == BSP_STATUS_OK) {
            // 换算外部分压
            float battery_voltage = voltage * 11.0f; // R1=10k, R2=1k
            update_display(battery_voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## 10. 常见问题排查

| 现象                   | 排查步骤                                                                                     |
| :--------------------- | :------------------------------------------------------------------------------------------- |
| `read_raw` 返回超时    | 检查 `start` 是否已调用；检查 `device_handle` 是否有效；检查通道是否被其他外设占用。         |
| DMA 回调未触发         | 检查 `start_dma` 返回值；确认中断向量已使能；检查缓冲区是否对齐（某些 DMA 要求 32 位对齐）。 |
| 电压值始终为 0 或 3.3V | 检查分辨率配置是否匹配；检查参考电压是否接错；测量外部引脚电平。                             |
| 多通道互相串扰         | 平台驱动未正确切换通道；确保 `read_raw` 中设置了 `channel` 并等待稳定。                      |

---

**总结**：本抽象层通过“继承 + 虚表”的设计，将硬件差异完全封装在平台驱动中，应用层只需关注业务逻辑（电压/电流换算、报警阈值等）。请严格遵守 **生命周期管理** 与 **DMA 内存所有权** 规范，以确保系统长期稳定运行。
