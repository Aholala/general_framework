# OLED 显示模块 (module_oled)

## 1. 模块概述

`module_oled` 是基于 I2C 的单色页式 OLED 帧缓冲驱动，提供像素、直线、矩形、位图、清屏、对比度和整帧刷新。适用于常见 SSD1306 类控制器的基础图形显示（如 128x64、128x32 等）。

**核心功能**：

- 页式帧缓冲管理（宽度 × 高度/8 字节）
- 像素点绘制与擦除
- Bresenham 直线绘制
- 空心/填充矩形绘制
- 位图绘制（逐行高位在前格式）
- 全屏清空（全亮/全灭）
- 对比度调节
- 整帧刷新到屏幕

**设计哲学**：

- **零动态内存**：帧缓冲由调用者静态分配，Module 不分配任何内存。
- **页式寻址**：兼容 SSD1306 的页寻址模式，便于理解和使用。
- **非阻塞友好**：绘图操作仅修改本地帧缓冲，刷新操作可独立调度。

## 2. 设计边界

| **模块负责**         | **模块不负责**       |
| :------------------- | :------------------- |
| 帧缓冲管理和绘图原语 | 字体渲染和字符串排版 |
| I2C 通信与初始化序列 | UI 页面管理          |
| 整屏刷新             | 脏区域跟踪和增量刷新 |
| 对比度控制           | 动画引擎             |

**适用场景**：

- 调试信息显示（系统状态、传感器数据）
- 用户界面（菜单、参数设置）
- 简单图形显示（波形、图标）

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_oled_t                  (OLED 对象：I2C、帧缓冲、宽高)
```

`module_oled_t` 内部保存 I2C 基类、设备地址、屏幕尺寸和帧缓冲指针。通过 `module_device` 基类可接入统一设备管理框架。

## 4. 内存计算

帧缓冲区大小计算公式：

```c
frame_buffer_size = width_pixels * (height_pixels / 8)
```

例如：

- 128×64 显示：128 × 8 = 1024 字节
- 128×32 显示：128 × 4 = 512 字节
- 64×48 显示：64 × 6 = 384 字节

**约束**：`height_pixels` 必须是 8 的倍数，最大 64 像素。

## 5. 核心类型

### 5.1 配置结构 (`module_oled_config_t`)

```c
typedef struct {
    bsp_i2c_t *i2c;              // I2C BSP 基类
    uint16_t address_7bit;       // 7 位 I2C 地址（0x3C 或 0x3D）
    uint16_t width_pixels;       // 屏幕宽度（像素），最大 128
    uint16_t height_pixels;      // 屏幕高度（像素），最大 64，且为 8 的倍数
    uint8_t *frame_buffer;       // 帧缓冲区（调用者分配）
    size_t frame_buffer_size;    // 缓冲区大小
    uint32_t timeout_ms;         // I2C 超时（毫秒）
    const char *logical_name;    // 逻辑名称
    uint32_t registration_key;   // 注册键值
} module_oled_config_t;
```

## 6. API 参考

| 函数                         | 说明                        | 返回值                    |
| :--------------------------- | :-------------------------- | :------------------------ |
| `module_oled_init`           | 初始化 OLED 设备            | `OK` / `INVALID_ARGUMENT` |
| `module_oled_start`          | 启动 OLED（发送初始化序列） | `OK` / `TRANSPORT_ERROR`  |
| `module_oled_stop`           | 停止 OLED（关闭显示）       | `OK` / `TRANSPORT_ERROR`  |
| `module_oled_clear`          | 清屏（全亮或全灭）          | `OK` / `NOT_INITIALIZED`  |
| `module_oled_set_pixel`      | 设置单个像素                | `OK` / `INVALID_ARGUMENT` |
| `module_oled_draw_line`      | 绘制直线（Bresenham 算法）  | `OK` / `INVALID_ARGUMENT` |
| `module_oled_draw_rectangle` | 绘制矩形（空心或填充）      | `OK` / `INVALID_ARGUMENT` |
| `module_oled_draw_bitmap`    | 绘制位图                    | `OK` / `INVALID_ARGUMENT` |
| `module_oled_flush`          | 将帧缓冲区刷新到屏幕        | `OK` / `TRANSPORT_ERROR`  |
| `module_oled_set_contrast`   | 设置对比度（0~255）         | `OK` / `TRANSPORT_ERROR`  |

## 7. 使用示例

### 7.1 初始化与启动

```c
static module_oled_t s_oled;
static uint8_t oled_buffer[128 * 64 / 8];   // 128x64 显示

const module_oled_config_t cfg = {
    .i2c = board_i2c_ptr,                   // 已初始化的 I2C 基类
    .address_7bit = 0x3C,                   // SSD1306 默认地址
    .width_pixels = 128,
    .height_pixels = 64,
    .frame_buffer = oled_buffer,
    .frame_buffer_size = sizeof(oled_buffer),
    .timeout_ms = 10,
    .logical_name = "oled",
    .registration_key = 0,
};

module_oled_init(&s_oled, &cfg);
module_oled_start(&s_oled);   // 发送初始化序列并刷新一次
```

### 7.2 基本绘图

```c
// 清屏（全灭）
module_oled_clear(&s_oled, false);
module_oled_flush(&s_oled);

// 绘制像素
module_oled_set_pixel(&s_oled, 10, 10, true);

// 绘制对角线
module_oled_draw_line(&s_oled, 0, 0, 127, 63, true);

// 绘制填充矩形
module_oled_draw_rectangle(&s_oled, 20, 20, 40, 30, true, true);

// 绘制空心矩形（边框）
module_oled_draw_rectangle(&s_oled, 60, 20, 40, 30, false, true);

// 刷新显示
module_oled_flush(&s_oled);
```

### 7.3 绘制位图（如字体/图标）

```c
// 8x8 数字位图示例（1 表示亮，0 表示灭）
static const uint8_t digit_0[] = {
    0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C
};

module_oled_draw_bitmap(&s_oled, 50, 20, 8, 8, digit_0, sizeof(digit_0), true);
module_oled_flush(&s_oled);
```

### 7.4 对比度调节

```c
// 降低对比度（0~255）
module_oled_set_contrast(&s_oled, 0x40);
```

### 7.5 停止显示

```c
module_oled_stop(&s_oled);
```

## 8. 刷新机制

- `start()` 发送初始化序列后自动调用 `flush()` 显示初始内容。
- 绘图函数只修改本地帧缓冲，**不会**自动更新屏幕。
- 所有修改需要调用 `flush()` 才会发送到屏幕。
- 整屏刷新耗时较长（I2C 传输 1024 字节数据 + 命令开销）。

**推荐刷新策略**：

```c
// 低频率刷新（如 5Hz）
if (need_refresh) {
    module_oled_flush(&s_oled);
    need_refresh = false;
}
```

## 9. 实时性注意事项

- 整屏 I2C 刷新耗时可能达到 **10~50ms**，取决于 I2C 速率。
- **不要在**高优先级控制任务或 ISR 中调用 `flush()`。
- 可由低优先级任务或主循环调度。
- 若 I2C 总线被其他设备共享，需外部互斥（信号量或临界区）。

## 10. 故障处理

- OLED 是**非关键设备**，显示故障不应阻塞电机控制或安全逻辑。
- I2C 传输错误应返回 `TRANSPORT_ERROR`，调用者应统计并低频重试。
- 避免在故障路径无限同步刷新屏幕（可能导致系统卡死）。

## 11. 错误码速查

| 状态码             | 触发场景                                     |
| :----------------- | :------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、宽高非法、缓冲区太小、坐标超出边界 |
| `NOT_INITIALIZED`  | 对象未初始化                                 |
| `NOT_STARTED`      | 未调用 `start()` 或已调用 `stop()`           |
| `TRANSPORT_ERROR`  | I2C 通信失败（NACK、超时等）                 |

## 12. 建议验证测试项

- [ ] 不同宽高（128x64、128x32、64x48）初始化成功
- [ ] 四角像素正确点亮/熄灭
- [ ] 直线各种斜率（水平、垂直、45°、任意角度）
- [ ] 空心/填充矩形边界正确
- [ ] 位图绘制大小检查和裁剪
- [ ] 全亮/全灭清屏
- [ ] 对比度边界值（0 和 255）
- [ ] I2C NACK 和超时返回 `TRANSPORT_ERROR`
- [ ] 多次绘图后一次刷新显示正确
- [ ] 停止后拒绝操作（`NOT_STARTED`）

---

**总结**：`module_oled` 提供了轻量、可移植的 OLED 显示驱动，适用于需要简单图形显示的嵌入式系统。其页式帧缓冲设计兼容 SSD1306 类控制器，绘图原语覆盖了大多数基础图形需求。配合 `module_device` 基类，可接入统一设备管理框架。

## 一页式接入顺序与可读信息

```c
/* 1. 先初始化 I2C BSP；帧缓冲大小必须为 width * height / 8。 */
static module_oled_t oled;
static uint8_t frame_buffer[128U * 64U / 8U];

/* 2. 配置 I2C、7 位地址、尺寸和帧缓冲，然后初始化。 */
module_oled_status_t status = module_oled_init(&oled, &oled_config);

/* 3. start 发送控制器初始化命令并允许绘图。 */
status = module_oled_start(&oled);

/* 4. 绘图函数只修改 RAM 帧缓冲；完成一批绘图后统一 flush。 */
module_oled_clear(&oled, false);
module_oled_draw_rectangle(&oled, 0, 0, 20U, 10U, true, true);
status = module_oled_flush(&oled);

/* 5. 关闭显示或重新配置 I2C 前调用 stop。 */
```

本模块没有独立的传感数据 getter。可读取的信息主要是：

| 结构体/数据 | 读取方式 | 说明 |
| --- | --- | --- |
| `module_oled_config_t` | 调用者持有 | I2C 地址、屏幕尺寸和缓冲区配置 |
| `frame_buffer` | 调用者提供的数组 | 当前待刷新的单色像素页数据 |
| `module_oled_t` | 调试器只读查看 | 当前对比度、尺寸、帧缓冲引用和启动状态 |

屏幕当前内容应以帧缓冲为准；只有 `module_oled_flush()` 成功后，屏幕才与缓冲区同步。
