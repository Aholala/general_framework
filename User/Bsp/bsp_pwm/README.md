# BSP PWM 通用抽象层 (bsp_pwm)

## 1. 模块概述

`bsp_pwm` 提供了对 PWM（脉冲宽度调制）输出的通用抽象，支持启动/停止、频率设置、脉冲宽度（以定时器 tick 为单位）和归一化占空比操作。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 启动/停止 PWM 输出。
- 设置/获取 PWM 频率（Hz）。
- 设置/获取脉冲宽度（比较值，以定时器 tick 为单位）。
- 设置/获取归一化占空比（0.0~1.0）。
- 获取周期值（以定时器 tick 为单位）。

**设计哲学**：

- **双层数据模型**：底层使用 `frequency_hz`、`pulse_ticks`、`period_ticks` 硬件相关值；上层提供 `duty_cycle` 归一化浮点接口。
- **平台适配**：定时器实例、通道号和引脚复用通过平台句柄与配置注入，平台端负责 ARR/CCR 边界和 `+1` 细节。
- **占空比转换**：公共接口 `set_duty_cycle`/`get_duty_cycle` 在 `[0.0, 1.0]` 浮点值与 tick 之间自动转换（四舍五入）。

## 2. 设计边界

| **模块负责**                               | **模块不负责**                                     |
| :----------------------------------------- | :------------------------------------------------- |
| PWM 启停、频率、脉冲宽度、占空比的统一接口 | 定时器时钟配置、预分频器、自动重载值计算           |
| tick 值与占空比浮点数的自动转换            | 引脚复用、输出极性、死区插入                       |
| 脉冲宽度越界检查（不能超过周期值）         | 多通道共享定时器的资源冲突管理（由平台或板级处理） |
| 对象生命周期管理（初始化、反初始化）       | 安全电平配置（由板级实现定义）                     |

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_pwm_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_pwm_device_t  (派生类：持有 driver_ops 和 channel)
```

- **`bsp_pwm_t`**：应用层使用的基类指针，所有 PWM 操作均通过此指针进行。
- **`bsp_pwm_device_t`**：实际分配的对象，保存底层驱动操作表和逻辑通道号。
- **虚表结构**：`bsp_pwm_ops_t` 继承自 `bsp_device_ops_t`，新增 `start`、`stop`、`set_frequency`、`get_frequency`、`set_pulse`、`get_pulse`、`get_period`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_pwm_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_pwm_driver_ops_t *driver_ops;   // 底层驱动表
    uint32_t channel;                         // 逻辑通道号
} bsp_pwm_config_t;
```

### 4.2 底层驱动操作表 (`bsp_pwm_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle, uint32_t channel);
    bsp_status_t (*deinit)(void *handle, uint32_t channel);
    bsp_status_t (*start)(void *handle, uint32_t channel);
    bsp_status_t (*stop)(void *handle, uint32_t channel);
    bsp_status_t (*set_frequency)(void *handle, uint32_t channel, uint32_t freq_hz);
    bsp_status_t (*get_frequency)(const void *handle, uint32_t channel, uint32_t *freq_hz);
    bsp_status_t (*set_pulse)(void *handle, uint32_t channel, uint32_t pulse_ticks);
    bsp_status_t (*get_pulse)(const void *handle, uint32_t channel, uint32_t *pulse_ticks);
    bsp_status_t (*get_period)(const void *handle, uint32_t channel, uint32_t *period_ticks);
} bsp_pwm_driver_ops_t;
```

**必须实现的函数**：`start`、`stop`、`set_frequency`、`get_frequency`、`set_pulse`、`get_pulse`、`get_period`（由 `bsp_pwm_init` 校验）。`init`/`deinit` 可选。

## 5. API 参考

| 函数                     | 说明                         | 返回值                            |
| :----------------------- | :--------------------------- | :-------------------------------- |
| `bsp_pwm_init`           | 初始化 PWM 设备              | `OK` / `INVALID_ARGUMENT`         |
| `bsp_pwm_as_base`        | 向上转型，获取基类指针       | 基类指针或 `NULL`                 |
| `bsp_pwm_start`          | 启动 PWM 输出                | `OK` / `NOT_INITIALIZED`          |
| `bsp_pwm_stop`           | 停止 PWM 输出                | `OK` / `NOT_INITIALIZED`          |
| `bsp_pwm_set_frequency`  | 设置 PWM 频率（Hz）          | `OK` / `OUT_OF_RANGE`（频率为0）  |
| `bsp_pwm_get_frequency`  | 获取 PWM 频率                | `OK` / `INVALID_ARGUMENT`         |
| `bsp_pwm_set_pulse`      | 设置脉冲宽度（比较值，tick） | `OK` / `OUT_OF_RANGE`（超过周期） |
| `bsp_pwm_get_pulse`      | 获取脉冲宽度                 | `OK` / `IO_ERROR`（超过周期）     |
| `bsp_pwm_set_duty_cycle` | 设置归一化占空比（0.0~1.0）  | `OK` / `OUT_OF_RANGE`             |
| `bsp_pwm_get_duty_cycle` | 获取归一化占空比             | `OK` / `IO_ERROR`（周期为0）      |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_pwm_driver.c
static bsp_status_t stm32_pwm_start(void *handle, uint32_t channel) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_PWM_Start(htim, channel);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_pwm_set_frequency(void *handle, uint32_t channel, uint32_t freq) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    uint32_t period = clock / freq - 1;  // 根据硬件调整 +1 逻辑
    __HAL_TIM_SET_AUTORELOAD(htim, period);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_pwm_set_pulse(void *handle, uint32_t channel, uint32_t pulse) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_pwm_get_period(const void *handle, uint32_t channel, uint32_t *period) {
    const TIM_HandleTypeDef *htim = (const TIM_HandleTypeDef *)handle;
    *period = __HAL_TIM_GET_AUTORELOAD(htim) + 1;  // 根据硬件调整
    return BSP_STATUS_OK;
}

const bsp_pwm_driver_ops_t stm32_pwm_driver = {
    .init = NULL,
    .deinit = NULL,
    .start = stm32_pwm_start,
    .stop = stm32_pwm_stop,
    .set_frequency = stm32_pwm_set_frequency,
    .get_frequency = stm32_pwm_get_frequency,
    .set_pulse = stm32_pwm_set_pulse,
    .get_pulse = stm32_pwm_get_pulse,
    .get_period = stm32_pwm_get_period,
};
```

### 6.2 应用层初始化

```c
static bsp_pwm_device_t s_servo_pwm;
static bsp_pwm_t *s_servo_ptr = NULL;

void board_pwm_init(void) {
    bsp_pwm_config_t cfg = {
        .device_handle = &htim2,
        .driver_ops = &stm32_pwm_driver,
        .channel = TIM_CHANNEL_1,
    };
    bsp_pwm_init(&s_servo_pwm, &cfg);
    s_servo_ptr = bsp_pwm_as_base(&s_servo_pwm);
}
```

### 6.3 舵机控制（通过占空比）

```c
// 舵机通常使用 50Hz 频率，脉宽 1ms~2ms 对应 0°~180°
// 占空比 = 脉宽 / 周期（周期 = 1/50 = 20ms）
bsp_pwm_set_frequency(s_servo_ptr, 50);
bsp_pwm_set_duty_cycle(s_servo_ptr, 0.075F);  // 1.5ms / 20ms = 7.5%
bsp_pwm_start(s_servo_ptr);
```

### 6.4 通过脉冲宽度直接控制

```c
uint32_t period;
bsp_pwm_get_period(s_servo_ptr, &period);
// 若 period = 40000（20ms / 0.5us），1.5ms 脉宽对应 3000 ticks
bsp_pwm_set_pulse(s_servo_ptr, 3000);
```

### 6.5 获取当前状态

```c
float duty;
bsp_pwm_get_duty_cycle(s_servo_ptr, &duty);  // 返回当前占空比

uint32_t freq;
bsp_pwm_get_frequency(s_servo_ptr, &freq);   // 返回当前频率
```

## 7. 动态修改与安全

- **频率修改**：修改频率会重新计算 ARR，可能重置计数器（取决于硬件实现）。建议在安全边界更新，避免产生异常脉冲。
- **脉冲修改**：`set_pulse` 应使用预装载机制（由平台驱动实现），避免更新时产生窄脉冲。
- **占空比范围**：`set_duty_cycle` 接受 `[0.0, 1.0]`，0.0 对应持续低电平，1.0 对应持续高电平。
- **停止状态**：停止 PWM 后引脚电平由平台配置决定（高阻、保持最后状态或固定电平）。控制执行器时，板级实现必须定义停止后的安全电平。
- **输出范围**：`set_pulse` 会自动检查 `pulse_ticks > period_ticks` 并返回 `OUT_OF_RANGE`。

## 8. 生命周期与并发

- **初始化顺序**：`bsp_pwm_init` → `bsp_pwm_set_frequency` → `bsp_pwm_set_pulse` → `bsp_pwm_start`。
- **资源耦合**：多个 PWM 通道可能共享同一定时器。修改频率（ARR）会影响同一定时器的所有通道。平台驱动或板级配置必须显式记录这种资源关系。
- **并发约束**：同一 PWM 通道的操作应由单一任务控制，或由上层提供互斥保护。
- **无动态内存**：所有对象由调用者静态分配。

## 9. 错误码速查

| 错误码                        | 触发场景                                             |
| :---------------------------- | :--------------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、驱动表缺失、输出指针为空                   |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                         |
| `BSP_STATUS_OUT_OF_RANGE`     | 频率为 0、脉冲宽度超过周期、占空比超出 [0,1]         |
| `BSP_STATUS_IO_ERROR`         | 读取的脉冲宽度超过周期（硬件异常）、周期为 0（除零） |

## 10. 移植要求

平台移植者需实现 `bsp_pwm_driver_ops_t` 中的所有函数：

- **`start`**：使能定时器通道输出（如 `HAL_TIM_PWM_Start`）。
- **`stop`**：禁用定时器通道输出。
- **`set_frequency`**：计算并设置 ARR 值（需考虑时钟频率、预分频器和 `+1` 边界）。
- **`get_frequency`**：从 ARR 值反算频率。
- **`set_pulse`**：设置 CCR 值（比较值）。
- **`get_pulse`**：读取 CCR 值。
- **`get_period`**：读取 ARR 值（需考虑 `+1` 边界，返回逻辑周期值）。

**关键注意事项**：

- **频率与 tick 关系**：`period_ticks = timer_clock / frequency_hz`（部分硬件需 `-1` 或 `+1`，由平台驱动处理）。
- **通道号映射**：`channel` 参数需正确映射到硬件通道（如 TIM_CHANNEL_1~4）。
- **预装载机制**：建议使用预装载（ARR 和 CCR 预装载），避免频率/脉冲更新时产生异常波形。
- **多通道共享**：修改频率（ARR）会影响同一定时器的所有通道，平台端需提供资源管理或文档说明。

## 11. 建议验证测试项

- [ ] 0%、50%、100% 占空比输出正确（0% 为持续低电平，100% 为持续高电平）。
- [ ] 频率设置与读取一致（如 50Hz、1kHz、100kHz）。
- [ ] 脉冲宽度不能超过周期（越界返回 `OUT_OF_RANGE`）。
- [ ] 动态修改频率和占空比无异常窄脉冲（使用示波器验证）。
- [ ] 同一定时器多通道独立工作（频率共享，占空比独立）。
- [ ] 停止后引脚电平符合安全预期（结合硬件确认）。
- [ ] 未初始化对象调用 API 返回 `NOT_INITIALIZED`。
- [ ] 反初始化后对象拒绝访问。

---

## 一页式接入顺序与可读信息

1. 平台先配置定时器通道并实现 PWM driver ops。
2. `bsp_pwm_init()` 绑定 handle、driver ops 和初始频率/脉宽。
3. 调用 `set_frequency`，再按 ticks 或 `set_duty_cycle` 设置输出。
4. `bsp_pwm_start()` 开启输出；运行中修改参数要考虑共享定时器的其他通道。
5. 停机先设置安全占空比，再 stop/deinit。

可读取 `bsp_pwm_get_frequency()`、`bsp_pwm_get_pulse()` 和 `bsp_pwm_get_duty_cycle()`。这些是命令/寄存器状态，不代表舵机角度或电机速度反馈。
