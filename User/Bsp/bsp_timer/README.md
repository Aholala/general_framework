# bsp_timer

通用基本定时器抽象，封装启动、停止、计数器、周期、时钟频率和周期到期通知。

## 接口范围

- `bsp_timer_start`、`bsp_timer_stop`；
- `bsp_timer_reset`；
- `bsp_timer_set_counter`、`get_counter`；
- `bsp_timer_set_period`、`get_period`；
- `bsp_timer_get_frequency`；
- `bsp_timer_set_callback`；
- `bsp_timer_notify_elapsed`。

预分频、时钟树、自动重装模式和 IRQ 优先级由平台端配置。

## 初始化

```c
static bsp_timer_device_t control_timer;
static const bsp_timer_config_t config = {
    .device_handle = &platform_control_timer,
    .driver_ops = &platform_timer_driver_ops,
    .callback = control_tick_callback,
    .user_context = &control_context,
};

bsp_timer_init(&control_timer, &config);
bsp_timer_set_period(bsp_timer_as_base(&control_timer), period_ticks);
bsp_timer_start(bsp_timer_as_base(&control_timer));
```

平台更新中断清除标志后调用 `bsp_timer_notify_elapsed`。

## 时间换算

定时周期由 `period_ticks` 和 `frequency_hz` 共同决定。是否包含自动重装寄存器的 `+1`
语义属于平台端，通用接口中的 period 必须表示一致的逻辑周期，平台驱动负责转换。

## 中断规则

到期回调通常运行在 ISR，只允许设置任务标志、增加饱和计数或释放同步量。复杂控制可以由
高优先级周期任务执行，而不是直接堆在硬件回调里。

## 与其他 BSP 的区别

- 连续 PWM 输出使用 `bsp_pwm`；
- 正交计数使用 `bsp_encoder`；
- 微秒时间测量和短延时使用 `bsp_timebase`；
- 本模块适合固定周期调度和通用计数。

## 生命周期与并发

反初始化前必须停止计数器并屏蔽中断。回调替换与 ISR 并发时需要临界区。对象不拥有平台
句柄，无动态内存。

## 建议验证

- 启停、计数读写和复位；
- 周期最小/最大值；
- 频率读取和时间换算；
- 多次到期通知；
- 空回调和回调上下文；
- 两个定时器实例；
- 停止与析构后的通知安全。
