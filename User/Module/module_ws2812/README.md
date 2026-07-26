# module_ws2812

基于 SPI 编码波形的 WS2812 灯带模块，支持像素缓冲、全局亮度、异步刷新以及闪烁、流水、
呼吸、彩虹和剧院追逐效果。

## 内存计算

调用者提供：

- `module_ws2812_color_t pixels[led_count]`；
- 编码发送缓冲区；
- SPI 基类。

缓冲区最小容量使用：

```c
MODULE_WS2812_REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)
```

当前编码每颗 LED 使用 9 字节，尾部增加复位低电平字节。两块缓冲均由调用者长期持有。

## SPI 条件

平台 SPI 频率必须与编码比特模式匹配，使每个 WS2812 bit 的高低时间落入芯片容差。不同
SPI 时钟不能直接复用同一编码常量，移植时必须用逻辑分析仪确认波形。

## 基本使用

```c
module_ws2812_init(&strip, &config);
module_ws2812_start(&strip);
module_ws2812_fill(&strip, module_ws2812_make_color(0U, 32U, 0U));
module_ws2812_show(&strip);
```

`set_pixel` 和 `fill` 只修改像素数组，`show` 编码并启动传输。

## 异步发送

`is_busy` 在发送期间保持真，禁止再次编码覆盖发送缓冲。平台 SPI 完成回调必须调用
`module_ws2812_notify_transmit_complete`。DMA 模式下需处理缓存一致性。

## 效果引擎

通过 `start_blink`、`start_color_wipe`、`start_breath`、`start_rainbow` 或
`start_theater_chase` 选择效果，周期调用 `module_ws2812_update(elapsed_time_ms)`。
效果状态保存在对象内，无阻塞延时。

`set_brightness` 是软件全局缩放，不改变原始像素颜色。频繁浮点缩放已避免，适合低优先级
状态灯任务。

## 赛场策略

灯带是非关键负载。SPI 忙或错误不应阻塞控制任务；效果刷新率应受限，避免抢占传感器 SPI
和 DMA 带宽。故障状态颜色优先级由 App 决定。

## 建议验证

- 1 颗、最大配置数量和缓冲边界；
- RGB 顺序和亮度 0/255；
- SPI 波形时序和 reset 时间；
- 同步/异步发送忙保护；
- 五种效果及大时间步；
- 发送错误恢复；
- 与其他 SPI 设备的总线仲裁。
