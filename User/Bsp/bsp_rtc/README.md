# BSP 实时时钟通用抽象层 (bsp_rtc)

## 1. 模块概述

`bsp_rtc` 提供了对硬件实时时钟（RTC）的通用抽象，支持获取/设置结构化时间（年月日时分秒毫秒）和获取 Unix 时间戳。该模块遵循 BSP 通用基础设施（`bsp_common`）的设计规范，通过虚表实现多态，并完全采用静态内存分配。

**核心功能**：

- 获取结构化时间（`bsp_rtc_get_time`）。
- 设置结构化时间（`bsp_rtc_set_time`）。
- 获取 Unix 时间戳（`bsp_rtc_get_unix_time`，64 位避免 2038 年问题）。

**设计哲学**：

- **双接口设计**：同时提供结构化时间和 Unix 时间戳，满足不同场景需求。
- **64 位时间戳**：使用 `uint64_t` 存储 Unix 秒，避免 2038 年问题（32 位时间戳在 2038 年 1 月 19 日溢出）。
- **时间表示**：年份为完整四位数（如 2026），毫秒支持（取决于硬件精度）。
- **跨复位时间戳**：模块日志可使用 RTC 做跨复位时间戳，便于日志分析；实时控制周期的计时应使用单调递增的 `bsp_timebase`（如 SysTick），不能使用可能被校时回拨的 RTC。

## 2. 设计边界

| **模块负责**                                  | **模块不负责**                                   |
| :-------------------------------------------- | :----------------------------------------------- |
| 结构化时间的获取与设置（年月日时分秒毫秒）    | RTC 时钟源选择（LSE/LSI/HSE）                    |
| Unix 时间戳获取（64 位）                      | 备份域电源管理和电池供电                         |
| 时间字段的基本范围校验（月 1-12、日 1-31 等） | 闰年校验、月份天数校验（由平台驱动或调用方负责） |
| 设备对象生命周期管理（初始化、反初始化）      | 时区转换和夏令时处理                             |
| 多态接口和驱动解耦                            | 闹钟、周期性中断等功能                           |

**重要说明**：

- **实时控制**：实时控制周期的计时应使用单调递增的 `bsp_timebase`（如 SysTick），不能使用可能被校时回拨的 RTC。
- **硬件依赖**：当前板未配置 LSE/RTC 备份域时，不创建实例。启用时需决定 LSE/LSI 时钟源、备份电池和首次上电有效标记。

## 3. 对象模型与继承关系

```text
bsp_device_t
└── bsp_rtc_t             (基类：仅为 bsp_device_t 的包装)
    └── bsp_rtc_device_t  (派生类：持有 driver_ops)
```

- **`bsp_rtc_t`**：应用层使用的基类指针，所有 RTC 操作均通过此指针进行。
- **`bsp_rtc_device_t`**：实际分配的对象，保存底层驱动操作表。
- **虚表结构**：`bsp_rtc_ops_t` 继承自 `bsp_device_ops_t`，新增 `get_time`、`set_time`、`get_unix_time`。

## 4. 核心类型

### 4.1 时间结构体 (`bsp_rtc_time_t`)

```c
typedef struct {
    uint16_t year;          // 完整年份（如 2026）
    uint8_t month;          // 月份（1-12）
    uint8_t day;            // 日期（1-31）
    uint8_t hour;           // 小时（0-23）
    uint8_t minute;         // 分钟（0-59）
    uint8_t second;         // 秒（0-59）
    uint16_t millisecond;   // 毫秒（0-999）
} bsp_rtc_time_t;
```

### 4.2 配置结构 (`bsp_rtc_config_t`)

```c
typedef struct {
    void *device_handle;                      // 平台句柄
    const bsp_rtc_driver_ops_t *driver_ops;   // 底层驱动表
} bsp_rtc_config_t;
```

### 4.3 底层驱动操作表 (`bsp_rtc_driver_ops_t`)

```c
typedef struct {
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*get_time)(void *handle, bsp_rtc_time_t *time);
    bsp_status_t (*set_time)(void *handle, const bsp_rtc_time_t *time);
    bsp_status_t (*get_unix_time)(void *handle, uint64_t *unix_time_s);
} bsp_rtc_driver_ops_t;
```

**必须实现的函数**：`init`、`deinit`、`get_time`、`set_time`、`get_unix_time`（均由 `bsp_rtc_init` 校验）。

## 5. API 参考

| 函数                    | 说明                         | 返回值                               |
| :---------------------- | :--------------------------- | :----------------------------------- |
| `bsp_rtc_init`          | 初始化 RTC 设备              | `OK` / `INVALID_ARGUMENT`            |
| `bsp_rtc_as_base`       | 向上转型，获取基类指针       | 基类指针或 `NULL`                    |
| `bsp_rtc_get_time`      | 获取结构化时间               | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_rtc_set_time`      | 设置结构化时间（带范围校验） | `OK` / `INVALID_ARGUMENT` / 平台错误 |
| `bsp_rtc_get_unix_time` | 获取 Unix 时间戳             | `OK` / `INVALID_ARGUMENT` / 平台错误 |

## 6. 使用示例

### 6.1 平台驱动实现（移植者视角）

```c
// stm32_rtc_driver.c
static bsp_status_t stm32_rtc_init(void *handle) {
    RTC_HandleTypeDef *hrtc = (RTC_HandleTypeDef *)handle;
    HAL_RTC_Init(hrtc);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rtc_get_time(void *handle, bsp_rtc_time_t *time) {
    RTC_HandleTypeDef *hrtc = (RTC_HandleTypeDef *)handle;
    RTC_DateTypeDef date;
    RTC_TimeTypeDef sTime;
    HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, &date, RTC_FORMAT_BIN);
    time->year = 2000 + date.Year;
    time->month = date.Month;
    time->day = date.Date;
    time->hour = sTime.Hours;
    time->minute = sTime.Minutes;
    time->second = sTime.Seconds;
    time->millisecond = sTime.SubSeconds * 1000 / 256; // 近似转换
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rtc_set_time(void *handle, const bsp_rtc_time_t *time) {
    RTC_HandleTypeDef *hrtc = (RTC_HandleTypeDef *)handle;
    RTC_DateTypeDef date = {
        .Year = time->year - 2000,
        .Month = time->month,
        .Date = time->day,
    };
    RTC_TimeTypeDef sTime = {
        .Hours = time->hour,
        .Minutes = time->minute,
        .Seconds = time->second,
    };
    HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc, &date, RTC_FORMAT_BIN);
    return BSP_STATUS_OK;
}

static bsp_status_t stm32_rtc_get_unix(void *handle, uint64_t *unix_time_s) {
    bsp_rtc_time_t time;
    bsp_status_t status = stm32_rtc_get_time(handle, &time);
    if (status != BSP_STATUS_OK) return status;
    // 调用时间库函数或自行计算 Unix 时间戳
    *unix_time_s = rtc_to_unix(time.year, time.month, time.day,
                               time.hour, time.minute, time.second);
    return BSP_STATUS_OK;
}

const bsp_rtc_driver_ops_t stm32_rtc_driver = {
    .init = stm32_rtc_init,
    .deinit = stm32_rtc_deinit,
    .get_time = stm32_rtc_get_time,
    .set_time = stm32_rtc_set_time,
    .get_unix_time = stm32_rtc_get_unix,
};
```

### 6.2 应用层初始化

```c
static bsp_rtc_device_t s_rtc_dev;
static bsp_rtc_t *s_rtc_ptr = NULL;

void board_rtc_init(void) {
    bsp_rtc_config_t cfg = {
        .device_handle = &hrtc,
        .driver_ops = &stm32_rtc_driver,
    };
    bsp_rtc_init(&s_rtc_dev, &cfg);
    s_rtc_ptr = bsp_rtc_as_base(&s_rtc_dev);
}
```

### 6.3 获取结构化时间

```c
bsp_rtc_time_t now;
if (bsp_rtc_get_time(s_rtc_ptr, &now) == BSP_STATUS_OK) {
    printf("%04d-%02d-%02d %02d:%02d:%02d.%03d\n",
           now.year, now.month, now.day,
           now.hour, now.minute, now.second, now.millisecond);
}
```

### 6.4 设置结构化时间

```c
bsp_rtc_time_t new_time = {
    .year = 2026,
    .month = 7,
    .day = 27,
    .hour = 14,
    .minute = 30,
    .second = 0,
    .millisecond = 0,
};
bsp_rtc_set_time(s_rtc_ptr, &new_time);
```

### 6.5 获取 Unix 时间戳

```c
uint64_t timestamp;
if (bsp_rtc_get_unix_time(s_rtc_ptr, &timestamp) == BSP_STATUS_OK) {
    // timestamp 为自 1970-01-01 00:00:00 UTC 以来的秒数
}
```

## 7. 时间格式说明

- **年份**：完整四位数（如 2026），非两位数。
- **毫秒**：取值范围 0-999，精度取决于硬件（有些 RTC 仅支持秒级精度）。
- **Unix 时间戳**：使用 `uint64_t` 存储，避免 2038 年溢出（32 位 `time_t` 在 2038 年 1 月 19 日溢出）。

## 8. 生命周期与并发

- **初始化顺序**：`bsp_rtc_init` → 使用 → `bsp_device_deinit`（可选）。
- **并发约束**：RTC 通常不支持并发访问。若多任务共享同一对象，必须由上层提供互斥锁。
- **备份域**：RTC 通常由备份电池供电，断电后时间可保持（取决于硬件配置）。

## 9. 错误码速查

| 错误码                        | 触发场景                                 |
| :---------------------------- | :--------------------------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数为空、输出指针为空、时间字段超出范围 |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                             |
| `BSP_STATUS_IO_ERROR`         | 硬件 RTC 错误                            |

## 10. 移植要求

平台移植者需实现 `bsp_rtc_driver_ops_t` 的所有函数：

- **`init`**：初始化 RTC 硬件（时钟源、格式、备份域等）。
- **`deinit`**：关闭 RTC 模块（可选）。
- **`get_time`**：读取硬件 RTC 寄存器，填充 `bsp_rtc_time_t` 结构。
- **`set_time`**：将 `bsp_rtc_time_t` 写入硬件 RTC 寄存器。
- **`get_unix_time`**：计算 Unix 时间戳（自 1970-01-01 以来的秒数）。

**关键注意事项**：

- **年份处理**：若硬件存储两位数年份，需与世纪标记结合转换为四位年份。
- **月份天数校验**：`set_time` 中的基本校验（1-31）无法覆盖所有情况（如 2 月 31 日），平台驱动应进行深度校验。
- **毫秒精度**：若硬件不支持毫秒，`get_time` 可将该字段置 0。
- **初始化状态**：部分 RTC 需要首次上电标记来判断是否已初始化。

## 11. 建议验证测试项

- [ ] 空指针、未初始化对象调用返回 `INVALID_ARGUMENT` / `NOT_INITIALIZED`。
- [ ] `get_time` 返回的时间与 `set_time` 设置的时间一致。
- [ ] 闰年日期（如 2024-02-29）正确识别。
- [ ] 各字段边界值（0:00:00, 23:59:59, 毫秒 999）处理正确。
- [ ] Unix 时间戳与结构化时间互相转换一致。
- [ ] 跨越大时间范围（如 2038 年后）64 位时间戳正确。
- [ ] 多次 `init` / `deinit` 无状态残留。
- [ ] 断电重启后 RTC 时间保持（需硬件支持）。

---

**总结**：`bsp_rtc` 提供了简洁、可移植的实时时钟抽象，适用于日志时间戳和用户时间显示等场景。其双接口设计（结构化时间 + Unix 时间戳）兼顾了人类可读性和机器处理便利性。配合 `bsp_common`，该模块保持了 BSP 层的一致性和可维护性。
