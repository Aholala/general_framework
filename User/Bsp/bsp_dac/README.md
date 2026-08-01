# BSP DAC 通用抽象层 (bsp_dac)

## 1. 模块概述

`bsp_dac` 提供了对硬件数模转换器 (DAC) 通道的通用抽象，支持静态电压输出和 DMA 波形播放。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 单通道 DAC 输出控制（原始码、归一化值、电压值三种表示）。
- DMA 模式输出波形（单次或循环，由平台驱动决定）。
- 统一的对象生命周期管理（初始化、启动、停止、反初始化）。
- 事件回调机制（传输完成、错误通知）。

## 2. 设计边界

| **模块负责**                           | **模块不负责**                                       |
| :------------------------------------- | :--------------------------------------------------- |
| 统一输出接口：原始码、归一化值、电压值 | DAC 引脚复用、输出缓冲、触发源选择（由平台配置）     |
| DMA 启动/停止及样本缓冲区校验          | DMA 循环模式、双缓冲、触发频率等具体配置             |
| 范围检查和错误传播                     | 执行器安全状态管理（应由板级平台在故障时设置安全值） |
| 回调通知（事件和传输状态）             | 波形数据生成（应由应用层提供）                       |

**重要约束**：

- **安全输出**：停止 DAC 后，硬件输出可能保持最后值、高阻或被复位。在控制执行器（如电机、舵机）时，必须在停止前主动设置安全电压（如 0V）或由板级硬件确保安全状态。
- **资源共享**：多个 DAC 通道可能共享定时器和 DMA 通道。平台驱动必须处理资源冲突并返回 `BSP_STATUS_BUSY` 或 `BSP_STATUS_NO_RESOURCE`。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_dac_t             (基类：增加回调、参考电压、最大原始值)
    └── bsp_dac_device_t  (派生类：持有 driver_ops 和 channel)
```

- **`bsp_dac_t`**：应用层使用的基类指针，包含回调、用户上下文、参考电压和最大原始值。
- **`bsp_dac_device_t`**：实际分配的对象，保存底层驱动操作表和逻辑通道号。
- **虚表结构**：`bsp_dac_ops_t` 继承自 `bsp_device_ops_t`，新增 `start`、`stop`、`set_raw`、`get_raw`、`start_dma`、`stop_dma`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_dac_config_t`)

```c
typedef struct {
    void *device_handle;                // 平台句柄
    const bsp_dac_driver_ops_t *driver_ops;
    uint32_t channel;                   // 逻辑通道号
    uint8_t resolution_bits;            // 分辨率（1~31 位）
    float reference_voltage_v;          // 参考电压（>0）
    bsp_event_callback_t callback;
    void *user_context;
} bsp_dac_config_t;
```

### 4.2 底层驱动操作表 (`bsp_dac_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle, uint32_t channel);
    bsp_status_t (*deinit)(void *handle, uint32_t channel);
    bsp_status_t (*start)(void *handle, uint32_t channel);
    bsp_status_t (*stop)(void *handle, uint32_t channel);
    bsp_status_t (*set_raw)(void *handle, uint32_t channel, uint32_t raw_value);
    bsp_status_t (*get_raw)(const void *handle, uint32_t channel, uint32_t *raw_value);
    bsp_status_t (*start_dma)(void *handle, uint32_t channel, const uint32_t *sample_buffer,
                              size_t sample_count);
    bsp_status_t (*stop_dma)(void *handle, uint32_t channel);
} bsp_dac_driver_ops_t;
```

**必须实现的函数**：`start`、`stop`、`set_raw`、`get_raw`。`init`/`deinit` 可选（但 `deinit` 必须提供指针，可为空操作）。`start_dma`/`stop_dma` 可选，若不支持需置 `NULL`，公共接口会返回 `BSP_STATUS_UNSUPPORTED`。

### 4.3 高层虚表 (`bsp_dac_ops_t`)

与驱动表对应，但参数为 `bsp_dac_t *` 而非 `void *`+`channel`。

## 5. 初始化与启动流程

### 5.1 平台驱动实现示例（移植者）

```c
// stm32_dac_driver.c
static bsp_status_t stm32_dac_init(void *handle, uint32_t channel) {
    DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)handle;
    // 配置通道、输出缓冲等
    HAL_DAC_Init(hdac);
    HAL_DAC_ConfigChannel(hdac, ...);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_dac_set_raw(void *handle, uint32_t channel, uint32_t raw) {
    DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)handle;
    HAL_DAC_SetValue(hdac, channel, DAC_ALIGN_12B_R, raw);
    return BSP_STATUS_OK;
}

const bsp_dac_driver_ops_t stm32_dac_driver_ops = {
    .init = stm32_dac_init,
    .deinit = NULL,  // 实际需提供空函数
    .start = stm32_dac_start,
    .stop = stm32_dac_stop,
    .set_raw = stm32_dac_set_raw,
    .get_raw = stm32_dac_get_raw,
    .start_dma = stm32_dac_start_dma,
    .stop_dma = stm32_dac_stop_dma,
};
```

### 5.2 应用层初始化

```c
static bsp_dac_device_t s_dac_dev;
static bsp_dac_t *s_dac_ptr = NULL;

void board_dac_init(void) {
    bsp_dac_config_t cfg = {
        .device_handle = &hdac,
        .driver_ops = &stm32_dac_driver_ops,
        .channel = DAC_CHANNEL_1,
        .resolution_bits = 12,
        .reference_voltage_v = 3.3F,
        .callback = dac_event_callback,
        .user_context = NULL,
    };
    bsp_dac_init(&s_dac_dev, &cfg);
    s_dac_ptr = bsp_dac_as_base(&s_dac_dev);
}
```

### 5.3 启动输出

```c
// 必须先启动 DAC 才能设置值（取决于平台）
bsp_dac_start(s_dac_ptr);
```

## 6. 静态输出 API

三种表示方式，均自动进行范围检查：

| API                      | 输入范围                  | 说明                   |
| :----------------------- | :------------------------ | :--------------------- |
| `bsp_dac_set_raw`        | `0 ~ maximum_raw_value`   | 直接写入硬件码         |
| `bsp_dac_get_raw`        | 输出原始值                | 读取当前输出码         |
| `bsp_dac_set_normalized` | `0.0 ~ 1.0`               | 归一化值，四舍五入取整 |
| `bsp_dac_set_voltage`    | `0 ~ reference_voltage_v` | 电压值，自动换算       |

**示例**：

```c
bsp_dac_set_voltage(s_dac_ptr, 1.65F);   // 输出 1.65V
bsp_dac_set_normalized(s_dac_ptr, 0.5F); // 输出 50% 参考电压
uint32_t raw;
bsp_dac_get_raw(s_dac_ptr, &raw);
```

## 7. DMA 波形输出

### 7.1 准备样本数组

```c
#define WAVE_SAMPLES 256
static uint32_t sine_wave[WAVE_SAMPLES];
// 填充正弦波数据（0 ~ max_raw）
for (int i = 0; i < WAVE_SAMPLES; i++) {
    sine_wave[i] = (uint32_t)( (max_raw / 2) * (1 + sin(2 * PI * i / WAVE_SAMPLES)) + 0.5F );
}
```

### 7.2 启动 DMA

```c
// 注册回调（可选）
bsp_dac_set_callback(s_dac_ptr, my_dac_callback, NULL);
// 启动 DMA（单次或循环由平台决定，通常在驱动 init 时配置）
if (bsp_dac_start_dma(s_dac_ptr, sine_wave, WAVE_SAMPLES) == BSP_STATUS_OK) {
    // DMA 已启动
}
```

### 7.3 停止 DMA

```c
bsp_dac_stop_dma(s_dac_ptr);  // 停止 DMA，输出可能保持最后值
```

### 7.4 回调处理

```c
static void my_dac_callback(bsp_event_t event, bsp_status_t status,
                            size_t transferred, void *ctx) {
    if (event == BSP_EVENT_TRANSFER_COMPLETE) {
        // DMA 传输完成（单次模式）或循环中半完成/完成通知
        // 可在此处重新加载波形或发送信号量
    }
}
```

**注意**：

- 样本缓冲区在 DMA 传输期间必须保持有效，不可被修改或释放。
- 平台驱动负责触发频率和循环模式的配置（通常在 `init` 中完成）。
- 若平台支持循环模式，`BSP_EVENT_TRANSFER_COMPLETE` 可能在每个周期触发（取决于实现）。

## 8. 安全输出管理

DAC 常用于控制外部执行器，安全至关重要：

- **停止顺序**：在停止 DAC 前，应主动设置安全值（如 0V）。

```c
bsp_dac_set_voltage(s_dac_ptr, 0.0F);
bsp_dac_stop(s_dac_ptr);
```

- **硬件安全**：某些平台停止后输出高阻，需外部上拉/下拉电阻确保安全电平。
- **故障处理**：应用层应在检测到错误时立即执行安全停止。

## 9. 错误码速查

| 错误码                        | 触发场景                                                  |
| :---------------------------- | :-------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、分辨率 0 或 >31、参考电压非法                   |
| `BSP_STATUS_OUT_OF_RANGE`     | 原始值超过最大范围、归一化/电压超出 `[0,1]` / `[0, Vref]` |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                              |
| `BSP_STATUS_UNSUPPORTED`      | 调用 `start_dma` / `stop_dma` 但驱动未实现                |
| `BSP_STATUS_BUSY`             | 资源已被占用（如 DMA 正在使用）                           |
| `BSP_STATUS_IO_ERROR`         | 读取的原始值超过最大范围（硬件异常）                      |

## 10. 生命周期与并发

- **初始化顺序**：`bsp_dac_init` → `bsp_dac_start` → 输出设置 → `bsp_dac_stop` → `bsp_device_deinit`（可选）。
- **DMA 与静态输出互斥**：启动 DMA 后，`set_raw`/`set_voltage` 可能无效或被忽略，具体取决于平台驱动。
- **并发安全**：硬件 DAC 通常不支持多任务并发写入。若需多任务共享同一对象，必须由上层提供互斥锁。
- **中断上下文**：`bsp_dac_notify` 可在中断中调用，但 `set_raw` / `set_voltage` 等写操作不应在中断中执行（除非明确安全）。

## 11. 移植要求

平台移植者需实现 `bsp_dac_driver_ops_t`：

- **`init`**（可选）：配置 DAC 通道、输出缓冲、触发源等。
- **`deinit`**（可选）：关闭硬件，释放资源。
- **`start`**：使能 DAC 输出（可能同时启动触发源）。
- **`stop`**：禁用输出（硬件可能保持最后值）。
- **`set_raw`**：立即更新输出值（若 DMA 运行中，可能被忽略）。
- **`get_raw`**：读取当前输出寄存器值。
- **`start_dma`**（可选）：启动 DMA 传输。需确认缓冲区有效、样本值不超范围。
- **`stop_dma`**（可选）：停止 DMA 传输。

**注意事项**：

- 分辨率位数影响 `maximum_raw_value`，平台驱动应确保与硬件匹配。
- 参考电压仅用于换算，平台驱动本身不依赖它。
- DMA 模式下的触发频率和循环/单次模式应在 `init` 或 `start_dma` 中确定，公共层不干涉。

## 12. 验证测试项

- [ ] 空指针、未初始化对象调用返回错误。
- [ ] 设置零码、半量程、满量程，检查输出电压。
- [ ] 归一化值与电压值换算精度（四舍五入误差）。
- [ ] 超出范围调用返回 `OUT_OF_RANGE`。
- [ ] DMA 启动/停止，缓冲区为空或长度为 0 时返回错误。
- [ ] DMA 传输完成回调触发（单次模式）。
- [ ] 停止 DAC 后输出安全值（需结合硬件确认）。
- [ ] 多通道资源冲突处理（若平台支持多通道）。
- [ ] 在 DMA 运行期间调用 `set_raw`（行为由平台定义，应返回适当错误或忽略）。

---

## 一页式接入顺序与可读信息

1. 平台配置 DAC 通道/DMA，提供参考电压和分辨率对应的 driver ops。
2. `bsp_dac_init()` 后按需要注册异步 callback。
3. 先用 `set_raw/set_normalized/set_voltage` 设置安全初值，再 start。
4. 连续波形使用调用者提供的 sample buffer 调用 `start_dma()`。
5. ISR 调用 `bsp_dac_notify()`；停机按 `stop_dma → 安全输出 → stop → deinit`。

`bsp_dac_get_raw()` 可读取当前命令原始值；归一化/电压是写入接口，不代表外部实际模拟电压。DMA buffer 在传输结束前必须保持有效。
