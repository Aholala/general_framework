# BSP 增量编码器通用抽象层 (bsp_encoder)

## 1. 模块概述

`bsp_encoder` 提供了对**增量编码器计数器**的通用抽象，支持启动/停止计数、读写当前值、获取旋转方向以及带模数回绕处理的增量计算。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 启动/停止编码器计数。
- 设置/获取当前计数值。
- 获取旋转方向（停止、正向、反向）。
- 计算自上次采样以来的增量（自动处理模数回绕）。
- 内部保存历史值以支持增量计算（有状态接口）。

## 2. 设计边界

| **模块负责**                                | **模块不负责**                                       |
| :------------------------------------------ | :--------------------------------------------------- |
| 统一计数读写、方向查询和增量计算接口        | 编码器引脚配置、输入滤波、计数极性（由平台驱动配置） |
| 模数回绕处理（通过 `counter_modulus` 配置） | 高精度速度计算（应由 Module 层结合时间计算）         |
| 内部状态管理（`previous_count` 的更新）     | 溢出中断处理或硬件扩展计数                           |
| 对象生命周期管理（初始化、启动、停止）      | 编码器故障诊断或断线检测                             |

**重要约束**：

- **回绕前提**：单个采样周期内真实增量必须小于**半个模数**，否则仅凭两次计数无法正确判断回绕方向。高速应用应提高采样率或使用硬件溢出中断扩展计数。
- **并发安全**：对象内部保存 `previous_count`，因此 `get_delta` 是有状态接口。同一实例应由单一周期任务调用，`set_count`/`reset` 与 `get_delta` 之间需要串行化（互斥保护）。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_encoder_t             (基类：增加 previous_count 和 counter_modulus)
    └── bsp_encoder_device_t  (派生类：持有 driver_ops)
```

- **`bsp_encoder_t`**：应用层使用的基类指针，包含历史值和模数配置。
- **`bsp_encoder_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_encoder_ops_t` 继承自 `bsp_device_ops_t`，新增 `start`、`stop`、`set_count`、`get_count`、`get_direction`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_encoder_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_encoder_driver_ops_t *driver_ops;
    uint32_t counter_modulus;                 // 计数模数（0 表示无回绕，有效值 >=2）
} bsp_encoder_config_t;
```

- **`counter_modulus`**：表示计数器完整周期长度。例如 16 位向上计数器通常为 `65536U`（从 0 到 65535 后回绕到 0），必须与平台实际计数范围一致。若为 0，表示计数器不产生回绕（如 32 位自由运行计数器）。

### 4.2 底层驱动操作表 (`bsp_encoder_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*start)(void *handle);
    bsp_status_t (*stop)(void *handle);
    bsp_status_t (*set_count)(void *handle, int32_t count);
    bsp_status_t (*get_count)(const void *handle, int32_t *count);
    bsp_status_t (*get_direction)(const void *handle, bsp_encoder_direction_t *direction);
} bsp_encoder_driver_ops_t;
```

**必须实现的函数**：`start`、`stop`、`set_count`、`get_count`、`get_direction`。`init`/`deinit` 可选（但 `deinit` 需提供指针，可为空操作）。

### 4.3 方向枚举 (`bsp_encoder_direction_t`)

```c
typedef enum {
    BSP_ENCODER_DIRECTION_STOPPED = 0,   // 停止（无脉冲变化）
    BSP_ENCODER_DIRECTION_FORWARD,       // 正向（计数值递增）
    BSP_ENCODER_DIRECTION_REVERSE        // 反向（计数值递减）
} bsp_encoder_direction_t;
```

## 5. API 参考

| 函数                        | 说明                                 | 返回值                    |
| :-------------------------- | :----------------------------------- | :------------------------ |
| `bsp_encoder_init`          | 初始化编码器设备，配置模数并绑定驱动 | `OK` / `INVALID_ARGUMENT` |
| `bsp_encoder_as_base`       | 向上转型，获取基类指针               | 基类指针或 `NULL`         |
| `bsp_encoder_start`         | 启动编码器计数（使能硬件）           | 状态码                    |
| `bsp_encoder_stop`          | 停止编码器计数                       | 状态码                    |
| `bsp_encoder_reset`         | 计数值归零并清除内部历史值           | 状态码                    |
| `bsp_encoder_set_count`     | 设置当前计数值（不影响内部历史值）   | 状态码                    |
| `bsp_encoder_get_count`     | 读取当前计数值                       | 状态码                    |
| `bsp_encoder_get_delta`     | 读取增量（带回绕处理），并更新历史值 | `OK` / `OUT_OF_RANGE` 等  |
| `bsp_encoder_get_direction` | 读取旋转方向                         | 状态码                    |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_encoder_driver.c
static bsp_status_t stm32_encoder_init(void *handle) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_Encoder_Init(htim, ...);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_encoder_start(void *handle) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_encoder_get_count(const void *handle, int32_t *count) {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    *count = (int32_t)__HAL_TIM_GET_COUNTER(htim);
    return BSP_STATUS_OK;
}

const bsp_encoder_driver_ops_t stm32_encoder_driver = {
    .init = stm32_encoder_init,
    .deinit = NULL,  // 若无需清理，提供空函数
    .start = stm32_encoder_start,
    .stop = stm32_encoder_stop,
    .set_count = stm32_encoder_set_count,
    .get_count = stm32_encoder_get_count,
    .get_direction = stm32_encoder_get_direction, // 需实现
};
```

### 6.2 应用层初始化和周期性采样

```c
static bsp_encoder_device_t s_encoder_dev;
static bsp_encoder_t *s_encoder_ptr = NULL;

void board_encoder_init(void) {
    bsp_encoder_config_t cfg = {
        .device_handle = &htim2,
        .driver_ops = &stm32_encoder_driver,
        .counter_modulus = 65536U,   // 16位定时器
    };
    bsp_encoder_init(&s_encoder_dev, &cfg);
    s_encoder_ptr = bsp_encoder_as_base(&s_encoder_dev);
    bsp_encoder_start(s_encoder_ptr);
}

// 周期性任务（如 1kHz 控制循环）
void motor_control_loop(void) {
    int32_t delta;
    static int32_t position = 0;
    if (bsp_encoder_get_delta(s_encoder_ptr, &delta) == BSP_STATUS_OK) {
        position += delta;   // 累加得到绝对位置
        float speed = (float)delta / 0.001f; // 速度 = delta / 周期
    }
}
```

### 6.3 重置和设置参考值

```c
// 归零操作
bsp_encoder_reset(s_encoder_ptr);  // 计数清零，内部历史值同步清零

// 手动设置参考位置（如启动时已知机械零点）
bsp_encoder_set_count(s_encoder_ptr, 1000);
// 注意：set_count 不会修改 previous_count，建议之后立即调用 get_delta 以同步
int32_t dummy;
bsp_encoder_get_delta(s_encoder_ptr, &dummy);  // 将 previous_count 同步为当前值
```

## 7. 增量计算与回绕处理（关键逻辑）

`bsp_encoder_get_delta` 内部执行：

1. 读取当前计数值 `current`。
2. 计算 `diff = current - previous_count`（64 位避免溢出）。
3. 若 `counter_modulus > 0`，进行回绕校正：
   - `half = modulus / 2`。
   - 若 `diff > half`，则 `diff -= modulus`（视为负向回绕）。
   - 若 `diff < -half`，则 `diff += modulus`（视为正向回绕）。
4. 检查 `diff` 是否在 `int32_t` 范围内。
5. 更新 `previous_count = current`。

**前提条件**：两次采样间的真实增量绝对值必须小于 `half_modulus`，否则无法区分回绕与真实大位移。

## 8. 方向读取说明

`bsp_encoder_get_direction` 依赖硬件方向检测（如定时器的方向标志位）或由驱动根据计数变化判断。若硬件不支持，可在驱动中缓存上次计数并比较。模块不提供方向滤波，如需可靠的方向信号，应在 Module 层对 `delta` 进行滤波或死区处理。

## 9. 生命周期与并发约束

- **初始化**：`bsp_encoder_init` → `bsp_encoder_start`。
- **停止**：`bsp_encoder_stop`（可选），`bsp_device_deinit` 触发虚析构。
- **状态冲突**：`set_count`、`reset` 和 `get_delta` 均会操作 `previous_count`，必须串行化。推荐单一任务或使用互斥锁。
- **ISR 安全**：`get_count` 和 `get_delta` 可在中断中调用，但需注意 `get_delta` 会修改对象状态，若中断优先级高于任务，需确保非重入。

## 10. 错误码速查

| 错误码                        | 触发场景                        |
| :---------------------------- | :------------------------------ |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、模数非法（非零且 <2） |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                    |
| `BSP_STATUS_OUT_OF_RANGE`     | 计算出的增量超出 int32 范围     |
| `BSP_STATUS_IO_ERROR`         | 底层读取计数失败                |

## 11. 移植要求

平台移植者需实现驱动表所有函数（`deinit` 可为空函数）：

- **`start`**：使能定时器编码器模式计数。
- **`stop`**：停止计数（可保持当前值）。
- **`set_count`**：直接写入计数器寄存器（如 `__HAL_TIM_SET_COUNTER`）。
- **`get_count`**：返回当前计数器值（转为 `int32_t`）。
- **`get_direction`**：返回 `STOPPED`、`FORWARD` 或 `REVERSE`。若硬件支持方向标志（如 TIM_CR1 的 DIR 位），直接读取；否则需根据计数变化或外部逻辑判断。
- **`init`**：配置编码器模式、输入滤波、计数极性、自动重装载值（决定模数）等，确保 `counter_modulus` 与硬件一致。

## 12. 建议验证测试项

- [ ] 正向旋转时增量递增，反向递减。
- [ ] 模数回绕测试（跨越 0 或最大值边界）。
- [ ] 零增量时方向返回 `STOPPED`。
- [ ] 调用 `reset` 后计数归零且 `previous_count` 同步复位。
- [ ] 手工设置计数值后 `get_delta` 计算正确（需同步历史值）。
- [ ] 接近半模数边界时回绕判断准确。
- [ ] 两个独立编码器实例互不干扰。
- [ ] 非法模数（如 1）拒绝初始化。
- [ ] 未初始化对象调用 API 返回 `NOT_INITIALIZED`。
- [ ] 高速旋转下回绕处理稳定性（确保采样率满足条件）。

---

**总结**：`bsp_encoder` 为增量编码器提供了简洁、可移植的抽象，特别适用于电机控制和位置反馈系统。其内置的模数回绕处理简化了应用层代码，同时通过有状态接口的设计保证了增量计算的连续性。配合 `bsp_common`，它保持了 BSP 层的一致性和可维护性。
