# module_ws2812 — WS2812 灯带

PWM+DMA 帧缓冲驱动。支持单色、彩虹、呼吸等内置效果。

## 用法

```c
module_ws2812_t leds;
module_ws2812_config_t cfg = {
    .timer = &htim3, .channel = 2, .led_count = 8,
};
module_ws2812_init(&leds, &cfg);

// 设置颜色（GRB 格式）
module_ws2812_color_t red = { .g = 0, .r = 255, .b = 0 };
module_ws2812_set_color(&leds, 0, &red);  // 第 0 颗

// 内置效果
module_ws2812_start_rainbow(&leds, 50);  // 50ms 周期
module_ws2812_start_breath(&leds, &red, 1000);  // 1s 呼吸

// 周期更新（推进效果状态机）
module_ws2812_update(&leds, elapsed_ms);

// 手动刷新
module_ws2812_flush(&leds);  // DMA 发送帧缓冲

module_ws2812_stop(&leds);  // 关闭所有灯
```
