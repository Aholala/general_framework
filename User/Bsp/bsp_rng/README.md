# BSP 硬件随机数发生器通用抽象层 (bsp_rng)

## 1. 模块概述

`bsp_rng` 提供了对硬件随机数发生器（RNG / TRNG）的通用抽象，支持获取单个 32 位随机数和填充任意大小的缓冲区。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 获取单个 32 位随机数（`bsp_rng_get_uint32`）。
- 填充用户提供的任意大小缓冲区（`bsp_rng_fill`）。

**设计哲学**：

- **硬件真随机数**：利用 MCU 内置的硬件 RNG 或 TRNG 模块产生真随机数，适用于安全敏感场景（如会话 nonce、密钥生成）。
- **驱动可靠性**：驱动必须检测时钟错误、种子错误和超时，不得在失败时返回未经标记的伪随机值。
- **极简接口**：只提供两种最基本、最通用的操作，满足绝大多数随机数需求。

## 2. 设计边界

| **模块负责**                             | **模块不负责**                           |
| :--------------------------------------- | :--------------------------------------- |
| 统一的随机数获取接口（32位和缓冲区填充） | 随机数质量检测和熵池管理                 |
| 设备对象生命周期管理（初始化、反初始化） | 伪随机数生成器（PRNG）算法实现           |
| 参数校验和错误传播                       | 随机数后处理（如均匀分布转换、范围限制） |
| 多态接口和驱动解耦                       | 硬件异常时的软件回退                     |

**重要说明**：

- 机器人协议的会话 nonce、挑战值等可以使用此接口。
- 控制算法仿真需要**可复现随机数**时应使用独立的软件 PRNG（如 LCG、Mersenne Twister），而不是硬件 RNG。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_rng_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_rng_device_t  (派生类：持有 driver_ops)
```

- **`bsp_rng_t`**：应用层使用的基类指针，所有 RNG 操作均通过此指针进行。
- **`bsp_rng_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_rng_ops_t` 继承自 `bsp_device_ops_t`，新增 `get_uint32` 和 `fill`。

## 4. 核心类型

### 4.1 配置结构 (`bsp_rng_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_rng_driver_ops_t *driver_ops;   // 底层驱动表
} bsp_rng_config_t;
```

### 4.2 底层驱动操作表 (`bsp_rng_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*get_uint32)(void *handle, uint32_t *value);
    bsp_status_t (*fill)(void *handle, void *data, size_t size);
} bsp_rng_driver_ops_t;
```

**必须实现的函数**：`init`、`deinit`、`get_uint32`、`fill`（均由 `bsp_rng_init` 校验）。

### 4.3 高层虚表 (`bsp_rng_ops_t`)

```c
typedef struct {
    bsp_device_ops_t super;
    bsp_status_t (*get_uint32)(bsp_rng_t *me, uint32_t *value);
    bsp_status_t (*fill)(bsp_rng_t *me, void *data, size_t size);
} bsp_rng_ops_t;
```

## 5. API 参考

| 函数                 | 说明                   | 返回值                               |
| :------------------- | :--------------------- | :----------------------------------- |
| `bsp_rng_init`       | 初始化 RNG 设备        | `OK` / `INVALID_ARGUMENT`            |
| `bsp_rng_as_base`    | 向上转型，获取基类指针 | 基类指针或 `NULL`                    |
| `bsp_rng_get_uint32` | 获取单个 32 位随机数   | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_rng_fill`       | 填充缓冲区             | `OK` / `INVALID_ARGUMENT` / 平台错误 |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_rng_driver.c
static bsp_status_t stm32_rng_init(void *handle) {
    RNG_HandleTypeDef *hrng = (RNG_HandleTypeDef *)handle;
    HAL_RNG_Init(hrng);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rng_deinit(void *handle) {
    RNG_HandleTypeDef *hrng = (RNG_HandleTypeDef *)handle;
    HAL_RNG_DeInit(hrng);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rng_get_uint32(void *handle, uint32_t *value) {
    RNG_HandleTypeDef *hrng = (RNG_HandleTypeDef *)handle;
    if (HAL_RNG_GenerateRandomNumber(hrng, value) != HAL_OK) {
        // 检测时钟错误、种子错误等
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rng_fill(void *handle, void *data, size_t size) {
    RNG_HandleTypeDef *hrng = (RNG_HandleTypeDef *)handle;
    uint32_t *ptr = (uint32_t *)data;
    size_t words = size / 4;
    // 填充完整的 4 字节字
    for (size_t i = 0; i < words; i++) {
        if (HAL_RNG_GenerateRandomNumber(hrng, &ptr[i]) != HAL_OK) {
            return BSP_STATUS_IO_ERROR;
        }
    }
    // 处理剩余不足 4 字节的部分
    uint8_t *byte_ptr = (uint8_t *)&ptr[words];
    uint32_t last = 0;
    for (size_t i = 0; i < size - words * 4; i++) {
        if (i == 0) {
            if (HAL_RNG_GenerateRandomNumber(hrng, &last) != HAL_OK) {
                return BSP_STATUS_IO_ERROR;
            }
        }
        byte_ptr[i] = ((uint8_t *)&last)[i];
    }
    return BSP_STATUS_OK;
}

const bsp_rng_driver_ops_t stm32_rng_driver = {
    .init = stm32_rng_init,
    .deinit = stm32_rng_deinit,
    .get_uint32 = stm32_rng_get_uint32,
    .fill = stm32_rng_fill,
};
```

### 6.2 应用层初始化

```c
static bsp_rng_device_t s_rng_dev;
static bsp_rng_t *s_rng_ptr = NULL;

void board_rng_init(void) {
    bsp_rng_config_t cfg = {
        .device_handle = &hrng,
        .driver_ops = &stm32_rng_driver,
    };
    bsp_rng_init(&s_rng_dev, &cfg);
    s_rng_ptr = bsp_rng_as_base(&s_rng_dev);
}
```

### 6.3 获取单个 32 位随机数

```c
uint32_t nonce;
if (bsp_rng_get_uint32(s_rng_ptr, &nonce) == BSP_STATUS_OK) {
    // 使用 nonce 作为会话标识或挑战值
}
```

### 6.4 填充缓冲区

```c
uint8_t random_bytes[64];
if (bsp_rng_fill(s_rng_ptr, random_bytes, sizeof(random_bytes)) == BSP_STATUS_OK) {
    // random_bytes 已填充随机数据
}
```

## 7. 驱动可靠性要求

硬件 RNG 驱动必须满足以下可靠性要求：

| 要求             | 说明                                                               |
| :--------------- | :----------------------------------------------------------------- |
| **时钟错误检测** | RNG 模块时钟失效时，必须返回错误，不得返回无效数据                 |
| **种子错误检测** | 硬件熵源失效或种子不足时，必须返回错误                             |
| **超时检测**     | RNG 生成随机数可能超时（如硬件故障），返回 `TIMEOUT` 或 `IO_ERROR` |
| **错误传播**     | 平台错误码通过 `bsp_status_t` 原样向上传递，不得吞没               |

**严禁行为**：驱动不得在失败时返回未经标记的伪随机值（如固定值或软件 PRNG 退路）。

## 8. 生命周期与并发

- **初始化顺序**：`bsp_rng_init` → 使用 → `bsp_device_deinit`（可选）。
- **并发约束**：硬件 RNG 通常不支持并发访问。若多任务共享同一对象，必须由上层提供互斥锁（如信号量或临界区）。
- **功耗考虑**：RNG 模块功耗较高，不使用时建议通过 `deinit` 关闭（若驱动支持）。
- **ISR 安全**：`get_uint32` 和 `fill` 理论上可在中断中调用，但需确保硬件 RNG 在中断上下文可用且不会阻塞过久。

## 9. 错误码速查

| 错误码                        | 触发场景                                      |
| :---------------------------- | :-------------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、数据指针为空、大小为 0              |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                                  |
| `BSP_STATUS_IO_ERROR`         | 硬件 RNG 错误（时钟错误、种子错误、硬件故障） |
| `BSP_STATUS_TIMEOUT`          | RNG 生成超时（由平台驱动返回）                |

## 10. 移植要求

平台移植者需实现 `bsp_rng_driver_ops_t` 的所有函数：

- **`init`**：使能 RNG 模块时钟，初始化硬件。若初始化失败返回 `IO_ERROR`。
- **`deinit`**：禁用 RNG 模块（可选，用于省电）。若硬件不支持关闭，可返回 `OK`。
- **`get_uint32`**：生成一个 32 位随机数。必须检测硬件错误，失败时返回明确的错误码（`IO_ERROR` 或 `TIMEOUT`）。
- **`fill`**：生成指定大小的随机字节。推荐使用 `get_uint32` 循环实现，注意处理非 4 字节对齐的情况。

**关键注意事项**：

- **字节序**：`fill` 输出字节顺序应与 `get_uint32` 的字节序一致（通常为小端，由平台决定）。
- **错误处理**：任何一次随机数生成失败都应立即停止并返回错误，不得返回部分有效的随机数据。
- **性能优化**：对于大缓冲区，可以考虑使用 DMA 或批量读取模式（如果硬件支持）。
- **初始化检查**：部分硬件 RNG 需要多次读取以完成初始化，`init` 中应处理此情况。

## 11. 建议验证测试项

- [ ] 空指针、未初始化对象调用返回 `INVALID_ARGUMENT` / `NOT_INITIALIZED`。
- [ ] 连续调用 `get_uint32` 产生不同的随机数（基本随机性测试）。
- [ ] `fill` 填充的缓冲区与多次 `get_uint32` 结果一致（按字节序验证）。
- [ ] 非 4 字节对齐的缓冲区填充正确（如 1、2、3、5 字节）。
- [ ] 硬件 RNG 错误（如时钟故障、种子错误）正确传播为 `IO_ERROR`。
- [ ] 多次 `init` / `deinit` 无状态残留。
- [ ] 长时间连续读 RNG（如 10 万次）无卡死或错误。
- [ ] 多任务并发访问（若有互斥保护）不产生竞争。
- [ ] 反初始化后对象拒绝访问。

---

**总结**：`bsp_rng` 提供了硬件随机数发生器的简洁、可靠抽象，适用于需要真随机数的安全场景（会话 nonce、密钥生成等）。其极简的接口设计（仅两个函数）降低了使用复杂度，同时通过强制驱动实现错误检测，保证了随机数的质量和可靠性。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
