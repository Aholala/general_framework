# BSP 通用数字 GPIO 抽象层 (bsp_gpio)

## 1. 模块概述

`bsp_gpio` 提供了对数字 GPIO（通用输入输出）引脚的通用抽象，支持读取、写入和翻转操作。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 读取 GPIO 逻辑电平（高/低）。
- 写入 GPIO 逻辑电平（高/低）。
- 翻转 GPIO 逻辑电平（高↔低）。
- 对象生命周期管理（初始化、反初始化）。

**设计哲学**：

- **极简抽象**：只提供最基础的 GPIO 操作，不涉及模式、上下拉、速度、引脚复用等配置（这些由平台端或 CubeMX 完成）。
- **句柄封装**：`device_handle` 封装了端口号和引脚号，通用层不保存也不关心具体的硬件引脚。
- **虚表多态**：通过 `bsp_gpio_ops_t` 实现多态，底层可替换为不同平台的 GPIO 驱动。

## 2. 设计边界

| **模块负责**                    | **模块不负责**                              |
| :------------------------------ | :------------------------------------------ |
| GPIO 逻辑电平的读取、写入、翻转 | GPIO 模式配置（输入/输出/复用/模拟）        |
| 设备对象的生命周期管理          | 上下拉电阻配置、输出速度、驱动能力          |
| 虚表多态和驱动解耦              | 引脚复用功能配置（如 UART、SPI 的 IO 功能） |
| 参数校验（空指针、未初始化）    | 边沿中断检测（使用 `bsp_exti` 模块）        |
| 可选操作的 `UNSUPPORTED` 返回   | PWM 输出（使用 `bsp_pwm` 模块）             |

**适用场景**：

- LED 指示灯控制。
- 蜂鸣器使能/禁用。
- 芯片片选信号（CS）控制。
- 复位引脚控制。
- 通用状态输入（按钮、拨码开关、传感器就绪信号）。

**不适用场景**：

- 需要边沿中断的场景 → 使用 `bsp_exti`。
- 需要 PWM 调光的场景 → 使用 `bsp_pwm`。
- 需要双向总线协议（I2C、SPI）的场景 → 使用对应的总线抽象层。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_gpio_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_gpio_device_t  (派生类：持有 driver_ops)
```

- **`bsp_gpio_t`**：应用层使用的基类指针，所有 GPIO 操作均通过此指针进行。
- **`bsp_gpio_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_gpio_ops_t` 继承自 `bsp_device_ops_t`，新增 `read`、`write`、`toggle`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_gpio_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄（封装端口/引脚）
    const bsp_gpio_driver_ops_t *driver_ops;   // 底层驱动表
} bsp_gpio_config_t;
```

### 4.2 底层驱动操作表 (`bsp_gpio_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *device_handle);        // 初始化（可选）
    bsp_status_t (*deinit)(void *device_handle);      // 反初始化（可选）
    bsp_status_t (*read)(const void *device_handle, bool *is_high);   // 读取电平（必须）
    bsp_status_t (*write)(void *device_handle, bool is_high);         // 写入电平（可选）
    bsp_status_t (*toggle)(void *device_handle);                      // 翻转电平（可选）
} bsp_gpio_driver_ops_t;
```

**必须实现的函数**：`read`。`write`/`toggle` 为可选，若未实现公共接口会返回 `BSP_STATUS_UNSUPPORTED`。`init`/`deinit` 可选（但 `deinit` 需提供指针，可为空操作）。

### 4.3 高层虚表 (`bsp_gpio_ops_t`)

```c
typedef struct {
    bsp_device_ops_t super;
    bsp_status_t (*read)(const bsp_gpio_t *const me, bool *is_high);
    bsp_status_t (*write)(bsp_gpio_t *const me, bool is_high);
    bsp_status_t (*toggle)(bsp_gpio_t *const me);
} bsp_gpio_ops_t;
```

## 5. API 参考

| 函数               | 说明                                        | 返回值                                        |
| :----------------- | :------------------------------------------ | :-------------------------------------------- |
| `bsp_gpio_init`    | 初始化 GPIO 设备，绑定句柄和驱动表          | `OK` / `INVALID_ARGUMENT`                     |
| `bsp_gpio_as_base` | 向上转型，获取基类指针                      | 基类指针或 `NULL`                             |
| `bsp_gpio_read`    | 读取 GPIO 逻辑电平（输出到 `is_high`）      | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED` |
| `bsp_gpio_write`   | 写入 GPIO 逻辑电平（`is_high=true` 输出高） | `OK` / `UNSUPPORTED` / `NOT_INITIALIZED`      |
| `bsp_gpio_toggle`  | 翻转 GPIO 逻辑电平                          | `OK` / `UNSUPPORTED` / `NOT_INITIALIZED`      |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_gpio_driver.c
static bsp_status_t stm32_gpio_read(const void *handle, bool *is_high) {
    GPIO_TypeDef *gpio = ((gpio_handle_t *)handle)->port;
    uint16_t pin = ((gpio_handle_t *)handle)->pin;
    *is_high = (HAL_GPIO_ReadPin(gpio, pin) == GPIO_PIN_SET);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_gpio_write(void *handle, bool is_high) {
    GPIO_TypeDef *gpio = ((gpio_handle_t *)handle)->port;
    uint16_t pin = ((gpio_handle_t *)handle)->pin;
    HAL_GPIO_WritePin(gpio, pin, is_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_gpio_toggle(void *handle) {
    GPIO_TypeDef *gpio = ((gpio_handle_t *)handle)->port;
    uint16_t pin = ((gpio_handle_t *)handle)->pin;
    HAL_GPIO_TogglePin(gpio, pin);
    return BSP_STATUS_OK;
}

const bsp_gpio_driver_ops_t stm32_gpio_driver = {
    .init = NULL,      // 无特殊初始化
    .deinit = NULL,    // 无特殊清理
    .read = stm32_gpio_read,
    .write = stm32_gpio_write,
    .toggle = stm32_gpio_toggle,
};
```

### 6.2 应用层初始化

```c
static bsp_gpio_device_t s_led_gpio;
static bsp_gpio_t *s_led_ptr = NULL;

// 平台定义的 GPIO 句柄（封装端口和引脚）
typedef struct { GPIO_TypeDef *port; uint16_t pin; } gpio_handle_t;
static const gpio_handle_t led_handle = { .port = GPIOA, .pin = GPIO_PIN_5 };

void board_gpio_init(void) {
    bsp_gpio_config_t cfg = {
        .device_handle = (void *)&led_handle,
        .driver_ops = &stm32_gpio_driver,
    };
    bsp_gpio_init(&s_led_gpio, &cfg);
    s_led_ptr = bsp_gpio_as_base(&s_led_gpio);
}
```

### 6.3 基本读写操作

```c
// 点亮 LED（假设高电平点亮）
bsp_gpio_write(s_led_ptr, true);

// 熄灭 LED
bsp_gpio_write(s_led_ptr, false);

// 翻转 LED 状态
bsp_gpio_toggle(s_led_ptr);

// 读取按键状态
bool button_pressed;
if (bsp_gpio_read(button_ptr, &button_pressed) == BSP_STATUS_OK) {
    if (button_pressed) {
        // 按键被按下（假设高电平有效）
    }
}
```

### 6.4 低电平有效器件的处理

GPIO 抽象层只处理逻辑电平（高/低），不处理硬件极性。对于低电平有效的器件（如低电平点亮的 LED、低电平有效的片选），应在板级或模块层进行极性转换：

```c
// 板级封装：LED 低电平点亮
typedef struct {
    bsp_gpio_t *gpio;
    bool active_low;
} led_t;

void led_on(led_t *led) {
    bsp_gpio_write(led->gpio, led->active_low ? false : true);
}

void led_off(led_t *led) {
    bsp_gpio_write(led->gpio, led->active_low ? true : false);
}
```

## 7. 生命周期与并发

- **初始化顺序**：`bsp_gpio_init` 后即可使用。
- **反初始化**：通过 `bsp_device_deinit` 触发虚析构，若驱动提供 `deinit` 则调用。
- **并发约束**：GPIO 操作通常不是原子性的。若多个任务或中断同时访问同一 GPIO 实例，必须由上层提供互斥锁（如信号量或临界区）。
- **ISR 安全**：`bsp_gpio_read`、`bsp_gpio_write`、`bsp_gpio_toggle` 理论上可在 ISR 中调用，但需确保平台驱动支持（如使用原子寄存器操作）。

## 8. 错误码速查

| 错误码                        | 触发场景                                                  |
| :---------------------------- | :-------------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空（对象、配置、句柄、驱动表）、`read` 输出指针为空 |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化即调用操作函数                                |
| `BSP_STATUS_UNSUPPORTED`      | 调用 `write` 或 `toggle` 但底层驱动未实现                 |

## 9. 移植要求

平台移植者需实现 `bsp_gpio_driver_ops_t`：

- **`init`**（可选）：配置 GPIO 模式、速度、上下拉等（也可由 CubeMX 完成）。
- **`deinit`**（可选）：清理资源（如关闭时钟）。
- **`read`**（必须）：读取引脚电平，返回逻辑值（`true`=高，`false`=低）。
- **`write`**（可选）：设置引脚电平。若未实现，`bsp_gpio_write` 返回 `UNSUPPORTED`。
- **`toggle`**（可选）：翻转引脚电平。若未实现，`bsp_gpio_toggle` 返回 `UNSUPPORTED`。

**关键注意事项**：

- `device_handle` 的具体类型由平台定义，通常是一个结构体包含端口和引脚信息。
- `toggle` 操作必须保证原子性（使用硬件 Toggle 寄存器或临界区保护），避免多任务竞争导致状态错误。
- 若硬件不支持原子翻转，可在驱动中使用“读-修改-写”方式实现，但需注意并发保护。

## 10. 建议验证测试项

- [ ] 空指针、未初始化对象调用 API 返回 `INVALID_ARGUMENT` / `NOT_INITIALIZED`。
- [ ] 读取输入引脚能正确返回逻辑电平。
- [ ] 写入输出引脚能正确输出高/低电平。
- [ ] `toggle` 能正确翻转输出电平。
- [ ] 多个 GPIO 实例（不同引脚）独立工作，互不干扰。
- [ ] 底层驱动未实现 `write` 时，`bsp_gpio_write` 返回 `UNSUPPORTED`。
- [ ] 底层驱动未实现 `toggle` 时，`bsp_gpio_toggle` 返回 `UNSUPPORTED`。
- [ ] 高频率翻转（如 1MHz）下系统稳定性。
- [ ] 反初始化（`deinit`）后，对象拒绝访问。

---

## 一页式接入顺序与可读信息

1. 平台实现 `bsp_gpio_driver_ops_t`，保存真实端口/引脚的 opaque handle。
2. 填写 `bsp_gpio_config_t`，调用 `bsp_gpio_init()`；库不负责配置 CubeMX 引脚模式。
3. 输出脚调用 `bsp_gpio_write()` 或 `toggle()`，输入脚调用 `bsp_gpio_read()`。
4. 退出或重绑引脚前调用 `bsp_device_deinit(bsp_gpio_as_base())`。

可读取信息是 `bsp_gpio_read()` 输出的 `bool is_high`，以及 `bsp_device_is_initialized()`。`bsp_gpio_config_t` 和 `bsp_gpio_t` 中的句柄/驱动表只用于装配和调试，不是业务数据。
