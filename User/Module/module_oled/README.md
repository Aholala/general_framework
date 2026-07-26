# module_oled

基于 I2C 的单色页式 OLED 帧缓冲驱动，提供像素、直线、矩形、位图、清屏、对比度和整帧
刷新。适用于常见 SSD1306 类控制器的基础图形显示。

## 配置与缓冲区

`module_oled_config_t` 包含 I2C、7 位地址、宽高、帧缓冲、缓冲区大小、超时、逻辑名称和
注册键。

所需帧缓冲区大小为：

```text
width_pixels * ceil(height_pixels / 8)
```

缓冲区由调用者静态持有。像素按页组织，Module 不使用动态内存。

## 生命周期

```c
module_oled_init(&oled, &config);
module_oled_start(&oled);
module_oled_clear(&oled, false);
module_oled_flush(&oled);
```

`start` 发送控制器初始化序列；`stop` 关闭显示。显示内容修改只作用于本地帧缓冲，调用
`flush` 才发送到屏幕。

## 绘图接口

- `set_pixel`；
- `draw_line`；
- `draw_rectangle`，支持填充/空心；
- `draw_bitmap`；
- `clear`；
- `set_contrast`。

坐标超出边界时接口按实现进行裁剪或返回参数错误，调用者不应依赖整数溢出。

## 位图格式

位图按本模块页式排列解释，大小必须覆盖指定宽高。字体渲染、字符串排版和 UI 页面管理
应作为更高层组件，不应把字体资源写进底层驱动。

## 实时性

整帧 I2C 刷新耗时较长。不要在高优先级控制任务或 ISR 中刷新；可按页面、脏区域或较低
刷新率由 App 调度。共享 I2C 时需要总线互斥。

## 故障策略

OLED 是非关键设备。显示离线不应阻塞电机控制，传输错误应统计并低频重试。比赛代码不要
在故障路径无限同步刷新屏幕。

## 建议验证

- 不同宽高和缓冲区边界；
- 四角像素、斜线和矩形裁剪；
- 位图大小检查；
- 全亮/全灭；
- 对比度边界；
- I2C NACK 和超时；
- 多次绘制后一次刷新。
