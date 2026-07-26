# bsp_adc

通用 ADC 通道抽象，支持校准、阻塞原始值读取、归一化/电压转换和 DMA 序列采样。分辨率、
参考电压和通道由配置注入，不绑定具体 MCU。

## 配置与对象

`bsp_adc_config_t` 包含平台句柄、驱动操作表、逻辑通道、分辨率、参考电压、回调和用户
上下文。构造时根据 `resolution_bits` 计算 `maximum_raw_value`。

```c
static bsp_adc_device_t battery_adc;
static const bsp_adc_config_t config = {
    .device_handle = &platform_adc,
    .driver_ops = &platform_adc_driver_ops,
    .channel = board_battery_adc_channel,
    .resolution_bits = 12U,
    .reference_voltage_v = 3.3F,
    .callback = adc_event_callback,
    .user_context = NULL,
};

bsp_adc_init(&battery_adc, &config);
```

## 读取接口

- `bsp_adc_read_raw`：返回硬件原始码；
- `bsp_adc_read_normalized`：返回 `[0, 1]`；
- `bsp_adc_read_voltage`：按参考电压换算输入电压。

外部电阻分压、运放增益、传感器偏置和工厂校准属于 Module 或板级配置，不应塞进 ADC
通用层。

## DMA 采样

`bsp_adc_start_dma` 接收调用者持有的 `uint32_t` 数组和样本数。缓冲区必须在停止 DMA 或
完成回调前持续有效。平台端完成缓存维护后调用 `bsp_adc_notify`。

回调只报告事件和传输数量，滤波、均值和物理量换算放在任务上下文。重复启动时平台应返回
`BSP_STATUS_BUSY`。

## 校准和生命周期

按平台要求在启动前调用 `bsp_adc_calibrate`。反初始化前先停止 DMA、停止 ADC，并确保
中断不会再访问缓冲区。多个逻辑通道共享一个 ADC 时，平台端负责扫描序列和资源仲裁。

## 精度说明

`read_voltage` 使用配置参考电压，不能替代 VDDA 实测和器件校准。高精度项目应在上层应用
校准系数，并处理采样时间、源阻抗和 ADC 非线性。

## 建议验证

- 分辨率边界和最大码；
- 0、半量程、满量程电压；
- 校准失败；
- DMA 完成、半传输和错误通知；
- 缓冲区生命周期和缓存一致性；
- 共享 ADC 多通道；
- 无效参考电压、未初始化对象和超时。
