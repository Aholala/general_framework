# bsp_encoder

增量编码器计数器抽象，提供启动、停止、计数读写、方向读取和带回绕处理的增量计算。

## 配置

`bsp_encoder_config_t` 包含平台句柄、驱动操作表和 `counter_modulus`。模数表示计数器完整
周期，例如 16 位向上计数器通常为 `65536U`，必须与平台计数范围一致。

平台端负责定时器编码器模式、输入滤波、计数极性和硬件启动。

## 初始化与采样

```c
static bsp_encoder_device_t steering_encoder;
bsp_encoder_init(&steering_encoder, &config);
bsp_encoder_start(bsp_encoder_as_base(&steering_encoder));

int32_t count_delta;
bsp_encoder_get_delta(
    bsp_encoder_as_base(&steering_encoder),
    &count_delta);
```

`get_delta` 读取当前计数，与对象内部 `previous_count` 比较并处理模数回绕，然后更新历史
值。首次需要确定参考值时可调用 `set_count` 或 `reset`。

## 方向

`bsp_encoder_get_direction` 返回停止、正向或反向。具体停止判定由硬件驱动决定；需要可靠
速度时，建议使用固定周期的 `count_delta / delta_time_s`，并在 Module 层滤波。

## 回绕前提

单个采样周期内真实增量必须小于半个计数周期，否则仅凭两次计数无法判断回绕方向。高速
编码器应提高采样率、扩大硬件计数位数或使用溢出中断扩展计数。

## 所有权与并发

对象保存上一计数，因此 `get_delta` 是有状态接口。同一实例应由单一周期任务调用；直接
`set_count`、`reset` 和 `get_delta` 之间需要串行化。

## 建议验证

- 正向和反向增量；
- 向上和向下回绕；
- 零增量和方向停止；
- 设置计数和复位；
- 接近半模数的边界；
- 两个编码器实例；
- 非法模数、未初始化对象和平台错误。
