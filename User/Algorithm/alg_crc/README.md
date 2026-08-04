# 通用 CRC 算法

`alg_crc` 是无硬件依赖的统一 CRC 入口。协议只选择参数，不依赖 STM32 CRC 外设。

```c
uint32_t result;
alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result);
```
