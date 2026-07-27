# BSP 存储设备通用抽象层 (bsp_storage)

## 1. 模块概述

`bsp_storage` 提供了对非易失性存储设备的通用抽象，覆盖片内 Flash、QSPI/OSPI NOR Flash、EEPROM、SDMMC 块设备等多种存储介质。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- **读取（Read）**：从指定地址读取数据。
- **编程（Program）**：向指定地址写入数据（对于需要先擦除的介质，由驱动处理）。
- **擦除（Erase）**：擦除指定范围的存储区域（以块为单位）。
- **同步（Sync）**：确保数据已持久化到物理介质（适用于带缓存的存储系统）。
- **几何信息（Geometry）**：获取容量、对齐要求、擦除块大小等物理特性。

**设计哲学**：

- **统一接口**：上层模块无需关心存储介质的差异，使用同一套 API。
- **显式几何**：通过 `bsp_storage_geometry_t` 明确告知上层读/写对齐、擦除块大小等约束。
- **驱动严格检查**：驱动必须进行地址越界和对齐检查，上层不得假设字节可重复覆盖。

## 2. 设计边界

| **模块负责**                             | **模块不负责**               |
| :--------------------------------------- | :--------------------------- |
| 统一读、编程、擦除、同步、几何查询接口   | 文件系统、磨损均衡、坏块管理 |
| 设备对象生命周期管理（初始化、反初始化） | 具体的存储控制器时序配置     |
| 多态接口和驱动解耦                       | 数据加密、压缩               |
| 参数校验（空指针、零大小）               | 地址空间布局管理             |
| 几何信息的标准化表达                     | 存储分区管理                 |

**适用场景**：

- 参数存储（校准系数、用户配置）。
- 固件升级（Bootloader 写入新固件）。
- 日志存储（飞行记录、故障诊断数据）。
- 数据持久化（传感器校准、运行统计）。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_storage_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_storage_device_t  (派生类：持有 driver_ops)
```

- **`bsp_storage_t`**：应用层使用的基类指针，所有存储操作均通过此指针进行。
- **`bsp_storage_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_storage_ops_t` 继承自 `bsp_device_ops_t`，新增 `read`、`program`、`erase`、`sync`、`get_geometry`。

## 4. 核心类型

### 4.1 几何信息结构体 (`bsp_storage_geometry_t`)

```c
typedef struct {
    uint64_t capacity_bytes;          // 总容量（字节）
    uint32_t read_alignment_bytes;    // 读对齐要求（1=无要求）
    uint32_t program_alignment_bytes; // 编程对齐要求（1=无要求）
    uint32_t erase_block_bytes;       // 擦除块大小（字节）
    bool erase_is_required;           // 写入前是否需要先擦除
    bool supports_memory_mapping;     // 是否支持内存映射（XIP）
} bsp_storage_geometry_t;
```

### 4.2 配置结构 (`bsp_storage_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_storage_driver_ops_t *driver_ops; // 底层驱动表
} bsp_storage_config_t;
```

### 4.3 底层驱动操作表 (`bsp_storage_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*read)(void *handle, uint64_t address, void *data, size_t size);
    bsp_status_t (*program)(void *handle, uint64_t address, const void *data, size_t size);
    bsp_status_t (*erase)(void *handle, uint64_t address, size_t size);
    bsp_status_t (*sync)(void *handle);
    bsp_status_t (*get_geometry)(const void *handle, bsp_storage_geometry_t *geometry);
} bsp_storage_driver_ops_t;
```

**所有函数均为必须实现**（由 `bsp_storage_init` 校验）。

## 5. API 参考

| 函数                       | 说明                   | 返回值                               |
| :------------------------- | :--------------------- | :----------------------------------- |
| `bsp_storage_init`         | 初始化存储设备         | `OK` / `INVALID_ARGUMENT`            |
| `bsp_storage_as_base`      | 向上转型，获取基类指针 | 基类指针或 `NULL`                    |
| `bsp_storage_read`         | 读取数据               | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_storage_program`      | 编程数据               | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_storage_erase`        | 擦除区域               | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_storage_sync`         | 同步（确保数据持久化） | `OK` / `NOT_INITIALIZED` / 平台错误  |
| `bsp_storage_get_geometry` | 获取几何信息           | `OK` / `INVALID_ARGUMENT` / 平台错误 |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_qspi_storage_driver.c
static bsp_status_t stm32_qspi_read(void *handle, uint64_t address, void *data, size_t size) {
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)handle;
    // 执行 QSPI 读取操作
    if (HAL_QSPI_Read(hqspi, data, address, size) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_qspi_program(void *handle, uint64_t address, const void *data, size_t size) {
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)handle;
    // NOR Flash 需要按页编程
    if (HAL_QSPI_Write(hqspi, (uint8_t *)data, address, size) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_qspi_erase(void *handle, uint64_t address, size_t size) {
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)handle;
    // 执行扇区擦除（需地址对齐到扇区边界）
    if (HAL_QSPI_Erase_Block(hqspi, address) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_qspi_get_geometry(const void *handle, bsp_storage_geometry_t *geo) {
    geo->capacity_bytes = 64 * 1024 * 1024;  // 64MB
    geo->read_alignment_bytes = 1;
    geo->program_alignment_bytes = 256;      // 页大小
    geo->erase_block_bytes = 4 * 1024;       // 扇区大小
    geo->erase_is_required = true;
    geo->supports_memory_mapping = true;
    return BSP_STATUS_OK;
}

const bsp_storage_driver_ops_t qspi_storage_driver = {
    .init = stm32_qspi_init,
    .deinit = stm32_qspi_deinit,
    .read = stm32_qspi_read,
    .program = stm32_qspi_program,
    .erase = stm32_qspi_erase,
    .sync = stm32_qspi_sync,
    .get_geometry = stm32_qspi_get_geometry,
};
```

### 6.2 应用层初始化

```c
static bsp_storage_device_t s_storage_dev;
static bsp_storage_t *s_storage_ptr = NULL;

void board_storage_init(void) {
    bsp_storage_config_t cfg = {
        .device_handle = &hqspi,
        .driver_ops = &qspi_storage_driver,
    };
    bsp_storage_init(&s_storage_dev, &cfg);
    s_storage_ptr = bsp_storage_as_base(&s_storage_dev);
}
```

### 6.3 标准操作流程

```c
// 1. 获取几何信息
bsp_storage_geometry_t geo;
bsp_storage_get_geometry(s_storage_ptr, &geo);

// 2. 擦除目标扇区（假设地址已对齐到 erase_block_bytes）
bsp_storage_erase(s_storage_ptr, 0x10000, geo.erase_block_bytes);

// 3. 写入数据（注意对齐要求）
uint8_t write_buf[256] __attribute__((aligned(256)));
memcpy(write_buf, config_data, sizeof(config_data));
bsp_storage_program(s_storage_ptr, 0x10000, write_buf, 256);

// 4. 同步（确保数据已写入物理介质）
bsp_storage_sync(s_storage_ptr);

// 5. 读取验证
uint8_t read_buf[256];
bsp_storage_read(s_storage_ptr, 0x10000, read_buf, 256);
if (memcmp(read_buf, write_buf, 256) == 0) {
    // 写入成功
}
```

### 6.4 参数存储模块适配

```c
// 适配器模式：将 bsp_storage 封装为带分区管理的参数存储
typedef struct {
    bsp_storage_t *storage;
    uint64_t base_address;
    size_t partition_size;
    bsp_storage_geometry_t geo;
} param_storage_t;

bsp_status_t param_storage_write(param_storage_t *ps, const void *data, size_t size) {
    // 先擦除分区
    bsp_storage_erase(ps->storage, ps->base_address, ps->partition_size);
    // 写入数据
    return bsp_storage_program(ps->storage, ps->base_address, data, size);
}
```

## 7. 几何信息详解

| 字段                      | 说明             | 典型值                                |
| :------------------------ | :--------------- | :------------------------------------ |
| `capacity_bytes`          | 总容量           | 64MB (QSPI), 1MB (片内Flash)          |
| `read_alignment_bytes`    | 读对齐要求       | 1（无要求）或 4/8/16                  |
| `program_alignment_bytes` | 编程对齐要求     | 1, 4, 256（NOR Flash页大小）          |
| `erase_block_bytes`       | 擦除块大小       | 4KB, 64KB, 128KB                      |
| `erase_is_required`       | 写入前是否需擦除 | true（Flash类）, false（EEPROM类）    |
| `supports_memory_mapping` | 是否支持 XIP     | true（NOR Flash）, false（NAND/SD卡） |

## 8. 生命周期与并发

- **初始化顺序**：`bsp_storage_init` → 使用 → `bsp_device_deinit`（可选）。
- **并发约束**：存储设备通常不支持并发访问。若多任务共享同一对象，必须由上层提供互斥锁（信号量或临界区）。
- **擦除耗时**：擦除操作可能耗时较长（几毫秒到几秒），需考虑任务调度，避免阻塞实时控制任务。
- **ISR 安全**：存储操作通常耗时较长，不应在中断中调用。

## 9. 错误码速查

| 错误码                        | 触发场景                          |
| :---------------------------- | :-------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、数据指针为空、大小为 0  |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                      |
| `BSP_STATUS_OUT_OF_RANGE`     | 地址越界或超出容量                |
| `BSP_STATUS_IO_ERROR`         | 硬件通信错误（QSPI/SPI 总线错误） |
| `BSP_STATUS_TIMEOUT`          | 操作超时（擦除/编程耗时过长）     |
| `BSP_STATUS_UNSUPPORTED`      | 不支持的操作（本模块未使用）      |

## 10. 移植要求

平台移植者需实现 `bsp_storage_driver_ops_t` 的所有函数：

| 函数           | 要求                                                     |
| :------------- | :------------------------------------------------------- |
| `init`         | 初始化存储控制器（时钟、引脚、模式等）                   |
| `deinit`       | 释放资源（可选，但必须提供指针）                         |
| `read`         | 执行读操作，支持任意地址（驱动内部处理对齐）             |
| `program`      | 执行写操作，若 `erase_is_required` 为 true，上层需先擦除 |
| `erase`        | 擦除指定区域，地址和大小必须对齐到 `erase_block_bytes`   |
| `sync`         | 等待所有挂起操作完成，数据持久化                         |
| `get_geometry` | 返回存储设备的物理特性（容量、对齐、块大小等）           |

**关键注意事项**：

- **地址越界检查**：所有操作必须检查 `address + size <= capacity_bytes`。
- **对齐检查**：`read` 和 `program` 的地址和大小必须满足对齐要求。
- **擦除粒度**：`erase` 的地址和大小必须对齐到 `erase_block_bytes`。
- **错误传播**：所有硬件错误必须映射到对应的 `bsp_status_t`。
- **缓存一致性**：若使用 D-Cache，需在 DMA 操作前后执行缓存维护。

## 11. 建议验证测试项

- [ ] 初始化成功，几何信息正确返回。
- [ ] 读/写功能正常，数据一致。
- [ ] 地址越界返回 `OUT_OF_RANGE`。
- [ ] 对齐要求正确（非对齐操作返回错误或由驱动处理）。
- [ ] 擦除后数据读取为 0xFF（或擦除状态值）。
- [ ] `erase_is_required` 为 true 时，未擦除直接写入返回错误。
- [ ] 同步确保数据持久化（断电测试）。
- [ ] 多次擦除/写入循环稳定性（1000+ 次）。
- [ ] 大块数据传输（接近容量上限）正常。
- [ ] 多任务并发访问（有互斥保护）不产生竞争。

---

**总结**：`bsp_storage` 为各种非易失性存储设备提供了统一的抽象接口，使上层模块能够以一致的方式操作不同介质。几何信息的标准化表达帮助上层应用正确处理对齐和擦除约束，避免了因介质差异导致的隐蔽错误。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
