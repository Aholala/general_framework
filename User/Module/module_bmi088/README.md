# BMI088六轴IMU模块

本模块通过通用 `bsp_spi_t` 驱动BMI088，不依赖具体MCU。加速度计和陀螺仪
共用SPI总线，片选、延时和可选微秒时间源由配置回调注入。

## 主要能力

- 双Chip ID检查和寄存器回读验证；
- 加速度和角速度量程配置；
- 调用者定义的坐标轴交换与方向映射；
- SI单位换算；
- 陀螺仪静止零偏标定和外部零偏加载；
- 加速度计、陀螺仪自检；
- 样本时间戳、采样间隔、有效状态和失败计数；
- `module_device_t` 基类接口。

## 配置边界

```c
const module_bmi088_config_t imu_config = {
    .logical_name = "imu",
    .registration_key = registration_key,
    .spi = board_spi,
    .set_chip_select = board_set_imu_chip_select,
    .delay_ms = board_delay_ms,
    .get_time_us = board_get_time_us,
    .user_context = board_context,
    .acceleration_range = acceleration_range,
    .angular_velocity_range = angular_velocity_range,
    .axis_map = axis_map,
    .transfer_timeout_ms = transfer_timeout_ms,
};
```

`get_time_us` 可以为 `NULL`。提供时间源时，每次成功读取会更新
`timestamp_us` 和 `sample_interval_us`；32位计数器自然回绕仍可通过无符号
减法得到间隔。

## 数据有效性

成功读取后：

```text
is_valid = true
sample_count += 1
```

SPI读取失败后：

```text
is_valid = false
failed_sample_count += 1
```

上层必须检查 `is_valid`，并使用真实 `sample_interval_us` 计算EKF的
`delta_time_s`。数据就绪EXTI和SPI调度属于板级/App，不写死在传感器模块中。

## 接口

- `module_bmi088_init`
- `module_bmi088_configure`
- `module_bmi088_read`
- `module_bmi088_get_data`
- `module_bmi088_get_raw_data`
- `module_bmi088_set_gyroscope_bias`
- `module_bmi088_calibrate_gyroscope`
- `module_bmi088_run_self_test`
- `module_bmi088_as_device`
