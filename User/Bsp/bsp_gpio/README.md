# bsp_gpio

通用数字 GPIO 输入输出抽象，提供读取、写入和翻转操作。模式、上下拉、速度和引脚复用由
平台端或 CubeMX 配置，通用层不保存端口号和引脚号。

## 对象模型

```text
bsp_device_t
└── bsp_gpio_t
    └── bsp_gpio_device_t
```

`bsp_gpio_device_t` 保存 `bsp_gpio_driver_ops_t`，平台句柄保存在设备基类中。多个 GPIO
实例可以共享同一套只读操作表，但必须使用不同句柄。

## 平台操作

平台端实现：

- `init`、`deinit`；
- `read`；
- `write`；
- `toggle`。

如果平台不能原子翻转，可在平台驱动中读写实现，但并发场景必须通过寄存器原子操作或临界
区避免竞争。

## 初始化与使用

```c
static bsp_gpio_device_t status_led;
static const bsp_gpio_config_t config = {
    .device_handle = &platform_status_led,
    .driver_ops = &platform_gpio_driver_ops,
};

bsp_gpio_init(&status_led, &config);
bsp_gpio_write(bsp_gpio_as_base(&status_led), true);
```

`bsp_gpio_read` 通过输出参数返回逻辑电平。高/低表示逻辑值，不自动处理 LED 低电平点亮；
板级装配或 Module 应通过清晰的 `active_level` 配置处理极性。

## 设计边界

适合 LED、蜂鸣器使能、片选、复位和状态输入。需要边沿回调时使用 `bsp_exti`；需要占空比
时使用 `bsp_pwm`。不要在通用 GPIO 内加入设备协议。

## 所有权与并发

对象不拥有平台句柄，无动态内存。写入和翻转如果可能由多个任务调用，应由外部互斥；
ISR 中只调用平台明确支持的非阻塞操作。

## 建议验证

- 输入高低读取；
- 输出高低和连续翻转；
- 未初始化对象；
- 缺失 `toggle` 时返回不支持；
- 两个引脚实例互不影响；
- 低有效器件由上层正确转换；
- 析构后拒绝访问。
