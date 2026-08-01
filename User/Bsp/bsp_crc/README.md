# BSP CRC 硬件抽象层 (bsp_crc)

## 1. 模块概述

`bsp_crc` 提供了对硬件 CRC 计算单元的通用抽象接口，允许上层应用以统一的方式调用不同平台（STM32、NXP、TI 等）的硬件 CRC 加速器。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 硬件加速的 CRC 计算（支持任意多项式、位反转、字节顺序等，由平台初始化时配置）。
- 支持链式计算（通过 `initial_value` 参数累加）。
- 统一的对象生命周期管理（初始化、反初始化）。

## 2. 设计边界

| **模块负责**           | **模块不负责**                                   |
| :--------------------- | :----------------------------------------------- |
| CRC 计算的统一调用接口 | 定义 CRC 多项式、位反转、输入/输出反转等具体参数 |
| 设备对象与驱动解耦     | 软件回退实现（当硬件不可用时）                   |
| 初始值传递和结果返回   | 数据对齐或缓存一致性维护（由平台驱动负责）       |

**重要说明**：CRC 的多项式、位反转、字节顺序等属于平台配置范畴，在初始化时由 `driver_ops->init()` 配置完成。调用方只关心数据、初值和结果，无需了解底层细节。如果应用协议使用 CRC8/CRC16 且硬件多项式不匹配，应继续使用软件实现，不要为了复用硬件而改变协议结果。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_crc_t             (基类：目前仅为 bsp_device_t 的包装)
    └── bsp_crc_device_t  (派生类：持有 driver_ops)
```

- **`bsp_crc_t`**：应用层使用的基类指针，所有 CRC 操作均通过此指针进行。
- **`bsp_crc_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_crc_ops_t` 继承自 `bsp_device_ops_t`，新增 `calculate` 虚函数。

## 4. 核心类型

### 4.1 配置结构 (`bsp_crc_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_crc_driver_ops_t *driver_ops;   // 底层驱动表
} bsp_crc_config_t;
```

### 4.2 底层驱动操作表 (`bsp_crc_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*calculate)(void *handle, const void *data, size_t size,
                              uint32_t initial_value, uint32_t *result);
} bsp_crc_driver_ops_t;
```

- **`init`**：初始化硬件 CRC 模块，配置多项式、位反转等参数（由平台实现决定）。
- **`deinit`**：关闭硬件模块或释放资源（必须提供指针，但可为空操作）。
- **`calculate`**：执行 CRC 计算，支持指定初始值（用于链式校验）。

### 4.3 高层虚表 (`bsp_crc_ops_t`)

```c
typedef struct {
    bsp_device_ops_t super;
    bsp_status_t (*calculate)(bsp_crc_t *me, const void *data, size_t size,
                              uint32_t initial_value, uint32_t *result);
} bsp_crc_ops_t;
```

## 5. 初始化与使用流程

### 5.1 平台驱动实现示例（移植者）

```c
// stm32_crc_driver.c
static bsp_status_t stm32_crc_init(void *handle) {
    CRC_HandleTypeDef *hcrc = (CRC_HandleTypeDef *)handle;
    // 默认 CRC-32 配置（多项式 0x04C11DB7，初值 0xFFFFFFFF，输出异或 0xFFFFFFFF）
    HAL_CRC_Init(hcrc);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_crc_calculate(void *handle, const void *data, size_t size,
                                        uint32_t initial_value, uint32_t *result) {
    CRC_HandleTypeDef *hcrc = (CRC_HandleTypeDef *)handle;
    *result = HAL_CRC_Calculate(hcrc, (uint32_t *)data, size / 4, initial_value);
    // 注意：需处理非4字节对齐情况
    return BSP_STATUS_OK;
}

const bsp_crc_driver_ops_t stm32_crc_driver_ops = {
    .init = stm32_crc_init,
    .deinit = NULL,  // 若无需特殊清理可置 NULL，但通用层会校验存在，故需提供空函数
    .calculate = stm32_crc_calculate,
};
```

### 5.2 应用层初始化

```c
static bsp_crc_device_t s_crc_dev;
static bsp_crc_t *s_crc_ptr = NULL;

void board_crc_init(void) {
    bsp_crc_config_t cfg = {
        .device_handle = &hcrc,
        .driver_ops = &stm32_crc_driver_ops,
    };
    bsp_crc_init(&s_crc_dev, &cfg);
    s_crc_ptr = bsp_crc_as_base(&s_crc_dev);
}
```

### 5.3 计算 CRC

```c
uint8_t buffer[] = {0x01, 0x02, 0x03, 0x04};
uint32_t crc_result;
uint32_t initial = 0xFFFFFFFF;

if (bsp_crc_calculate(s_crc_ptr, buffer, sizeof(buffer), initial, &crc_result) == BSP_STATUS_OK) {
    // 使用 crc_result
}
```

### 5.4 链式 CRC（分段校验）

```c
uint32_t crc = 0xFFFFFFFF;
bsp_crc_calculate(s_crc_ptr, part1, len1, crc, &crc);
bsp_crc_calculate(s_crc_ptr, part2, len2, crc, &crc);
// 最终 crc 为整体校验值
```

## 6. API 参考

| 函数                | 说明                                | 返回值                                           |
| :------------------ | :---------------------------------- | :----------------------------------------------- |
| `bsp_crc_init`      | 初始化 CRC 设备，调用驱动 `init`    | `BSP_STATUS_OK` 或错误码                         |
| `bsp_crc_as_base`   | 向上转型，获取基类指针              | 基类指针或 `NULL`                                |
| `bsp_crc_calculate` | 执行 CRC 计算，校验参数后转发到虚表 | `OK` / `INVALID_ARGUMENT` / `NOT_INITIALIZED` 等 |

## 7. 移植要求

平台移植者需实现 `bsp_crc_driver_ops_t` 的全部函数指针（`deinit` 可置为 `NULL`，但通用层检查时会报错，故建议提供空函数）：

- **`init`**：必须配置好 CRC 参数（多项式、位反转、输出反转等），参数应在 `device_handle` 中携带或由 `init` 自行确定。
- **`deinit`**：可执行硬件复位或释放资源，若无操作则返回 `BSP_STATUS_OK`。
- **`calculate`**：
  - 注意 `data` 指针可能未对齐，平台需自行处理（如使用字节循环或拷贝对齐缓冲区）。
  - `size` 为字节数，若硬件按字计算，需处理剩余字节。
  - `initial_value` 若硬件不支持设置初值，可通过软件异或等方式模拟（但应尽量利用硬件支持）。
  - 返回的 `result` 类型固定为 32 位，若硬件只支持 16 位或 8 位，应在平台层进行位宽匹配。

## 8. 生命周期与并发

- **生命周期**：`init` → 使用 → `deinit`（通过 `bsp_device_deinit` 触发）。
- **并发**：硬件 CRC 通常不支持多任务并发访问，若需多任务共享同一对象，必须由上层提供互斥锁（如信号量或临界区）。
- **ISR 安全**：`bsp_crc_calculate` 可被中断调用，但需注意硬件是否可重入。

## 9. 注意事项与最佳实践

- **对齐与 Endianness**：硬件 CRC 通常按 32 位字处理，若数据长度非 4 的倍数，平台驱动需处理剩余字节（通常按字节追加或补零）。字节序（大小端）由硬件决定，调用方需确保数据字节序与硬件匹配。
- **初始值**：不同的 CRC 标准使用不同的初值（如 CRC32 常用 `0xFFFFFFFF`，CRC16 常用 `0x0000` 或 `0xFFFF`）。调用方需根据协议要求传入正确的初值。
- **性能**：硬件 CRC 比软件计算快得多，尤其适合通信协议校验（如 CAN、以太网）和存储校验。
- **协议匹配**：若协议使用 CRC8/CRC16 且硬件仅支持 CRC32，不应强制改用硬件，否则会改变协议结果，应继续使用软件 CRC。

## 10. 建议验证测试项

- [ ] 空指针、未初始化对象调用 `calculate` 返回 `INVALID_ARGUMENT`。
- [ ] 已知数据（如全 0、全 1）的 CRC 结果与预期值对比（需提前获取硬件正确结果）。
- [ ] 链式计算与一次性计算的结果一致性（分段数据）。
- [ ] 数据长度为非 4 倍数时，平台驱动能正确处理。
- [ ] 不同初始值计算结果符合预期。
- [ ] 多次 `init` / `deinit` 无状态残留。
- [ ] 多实例（若硬件支持多个 CRC 单元）独立工作。
- [ ] 在中断上下文中调用 `calculate` 的稳定性。

---

## 一页式接入顺序与可读信息

1. 平台按多项式、输入/输出反转和数据宽度配置 CRC 外设并实现 driver ops。
2. 填写 `bsp_crc_config_t` 后调用 `bsp_crc_init()`。
3. 每次准备只读数据、长度和初值，调用 `bsp_crc_calculate()`。
4. 检查返回状态后读取输出 `uint32_t result`；结束时基类 deinit。

本模块没有持续采样结构体。可读取信息只有计算结果和初始化状态；CRC 参数由平台配置决定，协议层必须确认硬件配置与协议算法完全一致。
