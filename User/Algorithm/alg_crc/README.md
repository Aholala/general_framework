# 通用 CRC 算法

`alg_crc` 是无硬件依赖的统一 CRC 入口。协议只选择参数，不依赖 STM32 CRC 外设。

```c
uint32_t result;
alg_crc_calculate(&alg_crc8_0x8c_ff_config, data, data_size, &result);
```

内置配置：

- `alg_crc8_0x8c_ff_config`：USB、普通 UART、裁判系统 CRC8。
- `alg_crc16_ccitt_false_config`：nRF24 ACE 数据包。
- `alg_crc16_0x8408_ff_config`：裁判系统 CRC16。

也可以自行创建 `alg_crc_config_t`。支持 8/16/32 位、MSB-first/LSB-first、多项式、初值和输出异或配置。LSB-first 的多项式填写反射形式。
