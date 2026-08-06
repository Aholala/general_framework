# alg_crc — CRC 校验

统一 CRC8/CRC16/CRC32 计算。可配置多项式、初值、位序。

## 用法

```c
alg_crc_config_t cfg = {
    .width = 8,              // CRC8
    .polynomial = 0x8C,      // USB/视觉协议的多项式
    .initial_value = 0xFF,
    .final_xor = 0x00,
    .reflect_input = true,   // LSB first
    .reflect_output = true,
};
alg_crc_t crc;
alg_crc_init(&crc, &cfg);

// 单字节累加
alg_crc_update(&crc, data, length);

// 获取结果
uint32_t result = alg_crc_get_result(&crc);

// 重置
alg_crc_reset(&crc);
```

## 项目中用到的 CRC 参数

| 用途 | 宽度 | 多项式 | 初值 | 位序 |
|------|------|--------|------|------|
| USB 视觉协议 | 8 | 0x8C | 0xFF | LSB first |
| UART 协议 | 8 | 0x8C | 0xFF | LSB first |
| 裁判系统 | 8/16 | 取决于命令 | 取决于命令 | 取决于命令 |
