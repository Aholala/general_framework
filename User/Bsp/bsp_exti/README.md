# bsp_exti

通用外部中断对象，封装启用、禁用、回调和用户上下文。触发边沿、输入引脚、NVIC 优先级和
硬件去抖由平台端配置。

## 接口

- `bsp_exti_init`：绑定句柄、驱动操作表和可选回调；
- `bsp_exti_set_callback`：运行时更换回调与上下文；
- `bsp_exti_enable`、`bsp_exti_disable`：控制中断源；
- `bsp_exti_notify`：平台 ISR 的通用通知入口；
- `bsp_device_deinit`：通过基类虚析构释放平台绑定。

## 初始化

```c
static bsp_exti_device_t imu_interrupt;
static const bsp_exti_config_t config = {
    .device_handle = &platform_imu_interrupt,
    .driver_ops = &platform_exti_driver_ops,
    .callback = imu_data_ready_callback,
    .user_context = &imu,
};

bsp_exti_init(&imu_interrupt, &config);
bsp_exti_enable(bsp_exti_as_base(&imu_interrupt));
```

平台 IRQ 确认并清除硬件标志后调用：

```c
bsp_exti_notify(bsp_exti_as_base(&imu_interrupt));
```

## 中断约束

回调运行在调用 `notify` 的上下文，通常是 ISR。回调只能记录时间戳、设置标志或释放
RTOS 同步量，不能阻塞、打印日志或执行 SPI/I2C 长事务。

若按键等输入需要软件去抖，回调只记录边沿，任务使用 `bsp_timebase` 延迟确认电平。

## 路由与多实例

平台端负责把具体 IRQ/引脚映射到对象，禁止通用 BSP 遍历全局对象查找句柄。多个 EXTI
对象可共享驱动操作表，每个实例独立保存回调和上下文。

## 生命周期

反初始化前先禁用中断，并确保不会再进入 `notify`。更换回调时若中断可能并发发生，需要
短临界区保护回调指针和上下文的一致更新。

## 建议验证

- 上升沿、下降沿和双边沿的平台路由；
- 启用/禁用后通知行为；
- 空回调；
- 回调上下文正确；
- 两个中断实例互不串扰；
- 高频边沿下不阻塞；
- 析构期间无悬空回调。
