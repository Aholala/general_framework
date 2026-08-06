# module_bmi088 — BMI088 六轴 IMU

SPI 通信 + EXTI 中断驱动。读取加速度计和陀螺仪原始值，换算为 SI 单位，支持零偏标定和轴映射。

## 关键结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `module_bmi088_t` | IMU 对象 | 内部状态，通过 getter 读取数据 |
| `module_bmi088_config_t` | 配置 | `spi`, `accel_exti`, `gyro_exti`, `gyro_range_dps`, `accel_range_g`, `axis_map` |
| `module_bmi088_raw_data_t` | 原始数据 | `acceleration[3]`(LSB), `angular_velocity[3]`(LSB), `temperature` |
| `module_bmi088_process_data_t` | 已换算数据 | `acceleration_m_per_s2[3]`, `angular_velocity_rad_per_s[3]`, `temperature_c`, `timestamp_us`, `is_valid` |

## 读取数据

```c
// 原始值（标定/调试用）
const module_bmi088_raw_data_t *raw = module_bmi088_get_raw_data(&bmi088);
int16_t accel_x = raw->acceleration[0];

// 已换算（可直接送姿态解算）
const module_bmi088_process_data_t *d = module_bmi088_get_data(&bmi088);
if (d->is_valid) {
    float ax = d->acceleration_m_per_s2[0];       // m/s²
    float gz = d->angular_velocity_rad_per_s[2];  // rad/s
    float temp = d->temperature_c;                 // °C
    uint32_t ts = d->timestamp_us;                 // 采样时刻
}
```

## 用法

```c
// 1. 初始化（SPI + EXTI 需由 board_config 预先初始化）
module_bmi088_t bmi088;
module_bmi088_config_t cfg = {
    .spi = board_config_get_bmi088_spi(),
    .accel_exti = board_config_get_exti(BOARD_CONFIG_EXTI_BMI088_ACCELEROMETER),
    .gyro_exti  = board_config_get_exti(BOARD_CONFIG_EXTI_BMI088_GYROSCOPE),
    .gyro_range_dps  = 2000,  // ±2000°/s
    .accel_range_g    = 24,    // ±24g
    .spi_timeout_ms   = 5,
};
module_bmi088_init(&bmi088, &cfg);  // 自动读取芯片 ID 校验

// 2. 零偏标定（静止时调用，采样 1000 次取平均）
module_bmi088_calibrate(&bmi088, 1000);

// 3. ISR 中通知数据就绪（由 EXTI 回调触发）
void on_imu_ready(bsp_exti_t *me, void *ctx) {
    module_bmi088_t *imu = (module_bmi088_t *)ctx;
    module_bmi088_read(imu);  // 读取寄存器到内部缓冲
}
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_bmi088_init(me, cfg)` | 初始化 + 读取 ID 校验 |
| `module_bmi088_read(me)` | SPI 读取传感器寄存器（ISR 安全） |
| `module_bmi088_calibrate(me, samples)` | 零偏标定 |
| `module_bmi088_get_raw_data(me)` | 获取原始数据只读指针 |
| `module_bmi088_get_data(me)` | 获取已换算数据只读指针 |
| `module_bmi088_set_axis_map(me, map)` | 运行时换轴映射 |
