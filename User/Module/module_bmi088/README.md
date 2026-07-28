# BMI088 六轴 IMU 传感器模块 (module_bmi088)

## 1. 模块概述

`module_bmi088` 是一个完整的 BMI088 六轴惯性测量单元（IMU）驱动模块，通过通用 `bsp_spi_t` 抽象层与硬件通信，不依赖具体 MCU 平台。两个传感器（加速度计和陀螺仪）共用同一 SPI 总线，片选控制、延时和可选的微秒时间源由配置回调注入。

**核心功能**：

- 加速度计和陀螺仪的初始化、配置与数据读取。
- 双 Chip ID 验证与寄存器回读校验，确保通信可靠。
- 加速度量程（±3G/±6G/±12G/±24G）和陀螺仪量程（±125°/s ~ ±2000°/s）可配置。
- 调用者定义的坐标轴重映射与方向映射（支持任意安装方向）。
- SI 单位换算（加速度 m/s²、角速度 rad/s、温度 °C）。
- 陀螺仪静止零偏标定与外部零偏加载。
- 加速度计和陀螺仪自检流程。
- 样本时间戳、采样间隔、有效状态和失败计数统计。

**设计哲学**：

- 通过回调函数（片选、延时、时间戳）与具体硬件解耦，平台无关。
- 所有 SPI 传输使用阻塞模式，确保时序确定。
- 寄存器写操作附带回读校验，提高配置可靠性。
- 提供原始数据和物理量数据两套接口，兼顾调试和算法开发。

## 2. 设计边界

| **模块负责**                    | **模块不负责**                             |
| :------------------------------ | :----------------------------------------- |
| BMI088 寄存器读写、Chip ID 验证 | SPI 总线的时钟、模式和 GPIO 初始化         |
| 加速度/角速度量程配置           | 数据就绪中断（EXTI）的配置与处理           |
| 坐标轴重映射与正负方向校正      | IMU 安装朝向和载体坐标系定义（由上层决定） |
| 原始值到 SI 单位换算            | 传感器融合算法（EKF/互补滤波）             |
| 陀螺仪零偏标定与补偿            | 采样周期调度（由调用者控制 `read` 时机）   |
| 自检流程控制与判定              | 硬件片选引脚的 GPIO 初始化                 |
| 数据有效标志和失败统计          | 温度补偿以外的温漂处理                     |

## 3. 对象模型

```text
module_device_t                    (设备基类：is_initialized、vptr)
└── module_bmi088_t                (BMI088 设备对象：SPI、回调、轴映射、数据缓存)
```

`module_bmi088_t` 内部包含两个传感器的原始数据、解算后数据和换算因子。通过 `module_bmi088_as_device` 获取基类指针后可接入 `module_device` 框架进行统一调度（start/stop/update）。

## 4. 核心类型

### 4.1 量程枚举

```c
typedef enum {
    MODULE_BMI088_ACCEL_RANGE_3G = 0,   // ±3G
    MODULE_BMI088_ACCEL_RANGE_6G,       // ±6G
    MODULE_BMI088_ACCEL_RANGE_12G,      // ±12G
    MODULE_BMI088_ACCEL_RANGE_24G       // ±24G
} module_bmi088_accel_range_t;

typedef enum {
    MODULE_BMI088_GYRO_RANGE_2000_DPS = 0,  // ±2000°/s
    MODULE_BMI088_GYRO_RANGE_1000_DPS,      // ±1000°/s
    MODULE_BMI088_GYRO_RANGE_500_DPS,       // ±500°/s
    MODULE_BMI088_GYRO_RANGE_250_DPS,       // ±250°/s
    MODULE_BMI088_GYRO_RANGE_125_DPS        // ±125°/s
} module_bmi088_gyro_range_t;
```

### 4.2 轴映射结构体

```c
typedef struct {
    uint8_t source_axis;   // 传感器原始轴索引（0=X, 1=Y, 2=Z）
    float direction_sign;  // 方向符号（+1.0 或 -1.0）
} module_bmi088_axis_map_t;
```

### 4.3 数据输出结构体

```c
// 原始 AD 计数值（便于调试）
typedef struct {
    int16_t acceleration[3];      // 加速度计原始值
    int16_t angular_velocity[3];  // 陀螺仪原始值
    int16_t temperature;          // 温度原始值
} module_bmi088_raw_data_t;

// 物理量数据（供算法使用）
typedef struct {
    float acceleration_m_per_s2[3];       // 加速度（m/s²）
    float angular_velocity_rad_per_s[3];  // 角速度（rad/s）
    float temperature_c;                  // 温度（°C）
    uint32_t timestamp_us;                // 采样时间戳（微秒）
    uint32_t sample_interval_us;          // 与上次采样的间隔（微秒）
    uint32_t sample_count;                // 成功采样总数
    uint32_t failed_sample_count;         // 失败采样总数
    bool is_valid;                        // 当前数据是否有效
} module_bmi088_data_t;
```

### 4.4 配置结构体

```c
typedef struct {
    const char *logical_name;                     // 设备逻辑名称
    uint32_t registration_key;                    // 模块注册键值
    bsp_spi_t *spi;                               // SPI BSP 基类
    module_bmi088_chip_select_t set_chip_select;  // 片选控制回调
    module_bmi088_delay_ms_t delay_ms;            // 毫秒延时回调
    module_bmi088_get_time_us_t get_time_us;      // 微秒时间戳回调（可选）
    void *user_context;                           // 回调用户上下文
    module_bmi088_accel_range_t acceleration_range;   // 加速度量程
    module_bmi088_gyro_range_t angular_velocity_range; // 陀螺仪量程
    module_bmi088_axis_map_t axis_map[3];         // 轴映射表
    uint32_t transfer_timeout_ms;                 // SPI 传输超时（ms）
} module_bmi088_config_t;
```

## 5. API 参考

| 函数                                | 说明                                                               | 返回值                                      |
| :---------------------------------- | :----------------------------------------------------------------- | :------------------------------------------ |
| `module_bmi088_init`                | 初始化 BMI088（软复位 → 验证 Chip ID → 配置寄存器 → 设置换算因子） | `OK` / `ACCEL_NOT_FOUND` / `GYRO_NOT_FOUND` |
| `module_bmi088_configure`           | 重新配置量程（无需重新初始化）                                     | `OK` / `NOT_INITIALIZED`                    |
| `module_bmi088_read`                | 读取加速度、角速度和温度，按轴映射和量程转换为物理量               | `OK` / `TRANSPORT_ERROR`                    |
| `module_bmi088_get_data`            | 获取解算后的物理量数据指针                                         | 数据指针 / `NULL`                           |
| `module_bmi088_get_raw_data`        | 获取原始 AD 计数值指针                                             | 原始数据指针 / `NULL`                       |
| `module_bmi088_set_gyroscope_bias`  | 手动设置陀螺仪零偏                                                 | `OK` / `OUT_OF_RANGE`                       |
| `module_bmi088_calibrate_gyroscope` | 静止状态下采样取平均，标定零偏                                     | `OK` / `CALIBRATION_MOTION`                 |
| `module_bmi088_run_self_test`       | 执行加速度计和陀螺仪自检                                           | `OK` / `SELF_TEST_FAILED`                   |
| `module_bmi088_as_device`           | 获取 `module_device_t` 基类指针                                    | `module_device_t` 指针                      |

## 6. 使用示例

### 6.1 配置轴映射

轴映射用于将传感器原始轴映射到应用所需的逻辑轴，并校正方向。

```c
// 标准安装：传感器 X/Y/Z 对应逻辑 X/Y/Z，方向为正
module_bmi088_axis_map_t axis_map[3] = {
    {.source_axis = 0, .direction_sign = 1.0F},   // 逻辑 X ← 传感器 X
    {.source_axis = 1, .direction_sign = 1.0F},   // 逻辑 Y ← 传感器 Y
    {.source_axis = 2, .direction_sign = 1.0F},   // 逻辑 Z ← 传感器 Z
};

// 传感器倒装（绕 X 轴旋转 180°）：Z 轴反向
module_bmi088_axis_map_t axis_map_inverted[3] = {
    {.source_axis = 0, .direction_sign = 1.0F},   // 逻辑 X ← 传感器 X
    {.source_axis = 1, .direction_sign = 1.0F},   // 逻辑 Y ← 传感器 Y
    {.source_axis = 2, .direction_sign = -1.0F},  // 逻辑 Z ← 传感器 Z（反向）
};

// 传感器与逻辑轴交换（如 X 轴物理安装指向逻辑 Y 方向）
module_bmi088_axis_map_t axis_map_swap[3] = {
    {.source_axis = 1, .direction_sign = 1.0F},   // 逻辑 X ← 传感器 Y
    {.source_axis = 0, .direction_sign = -1.0F},  // 逻辑 Y ← 传感器 X（反向）
    {.source_axis = 2, .direction_sign = 1.0F},   // 逻辑 Z ← 传感器 Z
};
```

### 6.2 初始化

```c
static module_bmi088_t imu;
static bsp_spi_t *spi_ptr;      // 已初始化的 SPI 基类
static void *user_ctx;          // 用户上下文

// 片选控制回调
void imu_chip_select(void *ctx, module_bmi088_sensor_t sensor, bool selected) {
    if (sensor == MODULE_BMI088_SENSOR_ACCEL) {
        bsp_gpio_write(accel_cs_gpio, selected ? false : true);  // 低电平有效
    } else {
        bsp_gpio_write(gyro_cs_gpio, selected ? false : true);
    }
}

// 延时回调
void imu_delay_ms(void *ctx, uint32_t ms) {
    HAL_Delay(ms);  // 或使用 RTOS 延时
}

// 时间戳回调（可选）
uint32_t imu_get_time_us(void *ctx) {
    return get_microsecond_timestamp();  // 高精度时间源
}

const module_bmi088_config_t config = {
    .logical_name = "imu",
    .registration_key = 1U,
    .spi = spi_ptr,
    .set_chip_select = imu_chip_select,
    .delay_ms = imu_delay_ms,
    .get_time_us = imu_get_time_us,
    .user_context = user_ctx,
    .acceleration_range = MODULE_BMI088_ACCEL_RANGE_6G,
    .angular_velocity_range = MODULE_BMI088_GYRO_RANGE_2000_DPS,
    .axis_map = {axis_map[0], axis_map[1], axis_map[2]},
    .transfer_timeout_ms = 10U,
};

module_bmi088_init(&imu, &config);
```

### 6.3 周期读取数据

```c
void imu_task(void *param) {
    while (1) {
        if (module_bmi088_read(&imu) == MODULE_BMI088_STATUS_OK) {
            const module_bmi088_data_t *data = module_bmi088_get_data(&imu);
            if (data->is_valid) {
                // 加速度（m/s²）
                float ax = data->acceleration_m_per_s2[0];
                float ay = data->acceleration_m_per_s2[1];
                float az = data->acceleration_m_per_s2[2];
                // 角速度（rad/s）
                float gx = data->angular_velocity_rad_per_s[0];
                float gy = data->angular_velocity_rad_per_s[1];
                float gz = data->angular_velocity_rad_per_s[2];
                // 温度
                float temp = data->temperature_c;
                // 采样间隔（用于 EKF 的 dt）
                float dt = data->sample_interval_us / 1000000.0F;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // 200Hz
    }
}
```

### 6.4 陀螺仪零偏校准

```c
// 静止状态下执行 200 次采样，每次间隔 5ms，允许最大偏差 0.05 rad/s
if (module_bmi088_calibrate_gyroscope(&imu, 200, 5, 0.05F) == MODULE_BMI088_STATUS_OK) {
    // 零偏已自动补偿到 data.angular_velocity_rad_per_s
} else {
    // 校准失败（运动过大或通信错误）
}
```

### 6.5 自检

```c
if (module_bmi088_run_self_test(&imu) == MODULE_BMI088_STATUS_OK) {
    // 自检通过，传感器工作正常
} else {
    // 自检失败，需要检查硬件连接
}
```

### 6.6 使用 module_device 框架

```c
// 获取设备基类指针
module_device_t *dev = module_bmi088_as_device(&imu);

// 注册到模块设备管理器
module_device_manager_register(dev);

// 统一调度（由管理器调用）
module_device_start(dev);
module_device_update(dev, elapsed_ms);
```

## 7. 数据有效性

- **成功读取后**：`is_valid = true`，`sample_count += 1`
- **SPI 读取失败后**：`is_valid = false`，`failed_sample_count += 1`
- **使用建议**：
  - 上层必须检查 `is_valid`，丢弃无效数据。
  - 使用真实的 `sample_interval_us` 计算 EKF 的 `delta_time`，避免使用固定周期。
  - `failed_sample_count` 可用于诊断通信稳定性。

## 8. 错误码速查

| 错误码                   | 触发场景                                                                   |
| :----------------------- | :------------------------------------------------------------------------- |
| `INVALID_ARGUMENT`       | 参数为空、轴映射非法（source_axis 重复或超出范围、direction_sign 不是 ±1） |
| `NOT_INITIALIZED`        | 对象未初始化                                                               |
| `ACCEL_NOT_FOUND`        | 加速度计 Chip ID 不匹配（期望 0x1E）                                       |
| `GYRO_NOT_FOUND`         | 陀螺仪 Chip ID 不匹配（期望 0x0F）                                         |
| `REGISTER_VERIFY_FAILED` | 寄存器写后回读值不一致                                                     |
| `SELF_TEST_FAILED`       | 自检未通过（加速度差值不足或陀螺仪状态错误）                               |
| `CALIBRATION_MOTION`     | 校准时传感器运动量超过阈值                                                 |
| `TRANSPORT_ERROR`        | SPI 传输超时或失败                                                         |
| `OUT_OF_RANGE`           | 参数超出范围（如零偏分量不是有限数）                                       |

## 9. 移植要求

平台需要实现以下回调：

| 回调                  | 原型                                             | 说明                                                  |
| :-------------------- | :----------------------------------------------- | :---------------------------------------------------- |
| `set_chip_select`     | `void (*)(void *, module_bmi088_sensor_t, bool)` | 片选控制：`true`=选中，`false`=取消选中。低电平有效。 |
| `delay_ms`            | `void (*)(void *, uint32_t)`                     | 毫秒级延时（用于软复位等待、寄存器操作间隔）          |
| `get_time_us`（可选） | `uint32_t (*)(void *)`                           | 微秒级时间戳。为 `NULL` 时模块不提供时间戳。          |

**关键注意事项**：

- 两个传感器共用 SPI 总线，片选必须独立控制。
- SPI 模式为 **Mode 3（CPOL=1, CPHA=1）**，由 BSP SPI 配置保证。
- 加速度计读操作需要 **2 字节协议开销**（命令字节 + 填充字节），陀螺仪为 **1 字节开销**，模块已自动处理。
- 寄存器写操作后模块会自动执行回读校验，确保写入成功。

## 10. 建议验证测试项

- [ ] 加速度计和陀螺仪 Chip ID 正确识别。
- [ ] 各量程配置正确，回读校验通过。
- [ ] 轴重映射：三个逻辑轴的 source_axis 互异，方向 ±1。
- [ ] 数据读取：加速度、角速度、温度值在合理范围内。
- [ ] 陀螺仪零偏校准：静止状态下标定后输出接近 0。
- [ ] 自检：加速度计和陀螺仪自检通过。
- [ ] 数据有效性：SPI 通信失败后 `is_valid = false`，`failed_sample_count` 递增。
- [ ] 时间戳：`get_time_us` 非 NULL 时，`timestamp_us` 和 `sample_interval_us` 正确。
- [ ] 异常处理：SPI 未初始化、空指针等返回正确错误码。
- [ ] 多实例：两个独立的 BMI088 可同时工作（共用 SPI 总线需注意片选切换）。

---

**总结**：`module_bmi088` 提供了完整的 BMI088 驱动，涵盖初始化、配置、数据读取、零偏校准和自检。其轴映射、量程配置和回调注入的设计使其能够适应各种安装方向和硬件平台。配合 `module_device` 框架，可统一接入系统调度。数据有效性和统计信息为上层算法提供了可靠的诊断依据。
