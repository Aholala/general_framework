# module_oled — OLED 显示屏

I2C 单色 128×64 页式 OLED。帧缓冲 + 整页刷新。

## 用法

```c
module_oled_t oled;
module_oled_config_t cfg = {
    .i2c = i2c_ptr, .i2c_address = 0x3C,
};
module_oled_init(&oled, &cfg);
module_oled_start(&oled);  // 清屏

// 写像素
module_oled_set_pixel(&oled, 64, 32, true);

// 写字符串（6×8 字体）
module_oled_draw_string(&oled, 0, 0, "Hello");

// 画线 / 矩形
module_oled_draw_line(&oled, 0, 0, 127, 63);

// 刷新
module_oled_flush(&oled);  // 整屏 I2C 写入

module_oled_stop(&oled);  // 清屏并关闭
```
