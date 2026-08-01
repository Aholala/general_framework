# WS2812 灯带驱动模块 (module_ws2812) —— 简要使用指南

## 1. 模块概述

`module_ws2812` 通过 SPI 以编码波形驱动 WS2812 智能 LED 灯带，支持像素缓冲、全局亮度、异步刷新以及多种灯光效果。它基于 `bsp_spi` 抽象层，与具体 MCU 解耦。

## 2. 内存与配置

调用者需提供：

- `pixels`：`module_ws2812_color_t` 数组，长度为 `led_count`
- `transmit_buffer`：编码后的 SPI 发送缓冲区，最小大小通过 `MODULE_WS2812_REQUIRED_BUFFER_SIZE` 计算

```c
MODULE_WS2812_REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)
```

## 3. 基本用法

```c
static module_ws2812_t strip;
static module_ws2812_color_t pixel_buf[16];
static uint8_t tx_buf[MODULE_WS2812_REQUIRED_BUFFER_SIZE(16, 1)];

const module_ws2812_config_t cfg = {
    .spi = spi_ptr,
    .pixels = pixel_buf,
    .led_count = 16,
    .transmit_buffer = tx_buf,
    .transmit_buffer_size = sizeof(tx_buf),
    .reset_byte_count = 1,
    .transmit_timeout_ms = 100,
    .transfer_mode = BSP_TRANSFER_MODE_DMA,
    .logical_name = "led_strip",
    .registration_key = 0,
};

module_ws2812_init(&strip, &cfg);
module_ws2812_start(&strip);
module_ws2812_fill(&strip, module_ws2812_make_color(0, 32, 0));
module_ws2812_show(&strip);
```

## 4. 效果引擎

通过 `start_blink`、`start_color_wipe`、`start_breath`、`start_rainbow` 或 `start_theater_chase` 启用效果，然后在主循环中周期调用 `module_ws2812_update(me, dt_ms)`。

```c
module_ws2812_start_breath(&strip, module_ws2812_make_color(255, 0, 0), 20);
// 在任务中：
module_ws2812_update(&strip, elapsed_ms);
```

## 5. SPI 注意事项

- SPI 频率需配合 WS2812 时序，每个 SPI bit 对应 WS2812 的一个 bit 周期。
- 异步发送时，SPI 完成回调必须调用 `module_ws2812_notify_transmit_complete` 清除忙标志。
- DMA 模式下需处理缓存一致性。

## 6. 建议验证

- 单颗 LED 和最大数量下的缓冲区边界
- RGB 顺序和亮度 0/255
- SPI 波形时序和复位低电平时间
- 同步/异步发送忙保护
- 五种效果及大步进时间
- 发送错误恢复
- 与其他 SPI 设备的总线仲裁

## 一页式接入顺序与可读信息

```c
/* 1. 先初始化 SPI BSP，并按 LED 数量静态分配像素和编码缓冲区。 */
static module_ws2812_t strip;
static module_ws2812_color_t pixels[LED_COUNT];
static uint8_t transmit_buffer[REQUIRED_BUFFER_SIZE];

/* 2. 配置 SPI、数组、复位字节数、发送模式和超时。 */
module_ws2812_status_t status = module_ws2812_init(&strip, &strip_config);

/* 3. start 后设置像素/填充颜色，再调用 show 真正发送。 */
status = module_ws2812_start(&strip);
module_ws2812_set_pixel(&strip, 0U, module_ws2812_make_color(255U, 0U, 0U));
status = module_ws2812_show(&strip);

/* 4. DMA/中断发送完成时，由平台回调 notify_transmit_complete。 */
module_ws2812_notify_transmit_complete(&strip, transfer_status);

/* 5. 使用效果时，启动一次效果并在任务中周期 update。 */
module_ws2812_start_breath(&strip, color, step_time_ms);
status = module_ws2812_update(&strip, elapsed_time_ms);

/* 6. stop_effect 只停效果；module_ws2812_stop 关闭整个灯带模块。 */
```

| 可读取信息 | 读取方式 | 说明 |
| --- | --- | --- |
| `module_ws2812_color_t[]` | 调用者持有的 `pixels` | 每颗 LED 当前待发送的 RGB 值 |
| `module_ws2812_effect_state_t` | `strip.effect`，仅调试读取 | 效果类型、相位、索引、亮度和累计时间 |
| 发送忙状态 | `module_ws2812_is_busy()` | DMA/中断发送是否仍未完成 |
| 全局亮度/启动状态 | `module_ws2812_t`，仅调试读取 | 当前亮度和模块运行状态 |

`pixels` 表示逻辑颜色，只有 `module_ws2812_show()` 成功发送后才会反映到灯带。
