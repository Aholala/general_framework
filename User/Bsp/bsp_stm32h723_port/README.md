# bsp_stm32h723_port

这是通用 BSP 与 STM32H723 HAL 之间唯一的芯片适配层。通用 `bsp_can_t`、`bsp_usart_t`、`bsp_spi_t` 等头文件仍然不包含 HAL；本目录集中保存 HAL 句柄映射、驱动操作表、静态板级对象和 HAL 回调路由。

## 初始化

CubeMX 完成 GPIO、FDCAN、SPI、TIM、UART 和 USB 初始化后，调用：

```c
const bsp_stm32h723_port_config_t port_config = {
    .initialize_watchdog = false,
};
bsp_stm32h723_port_init(&port_config);
```

随后通过 getter 获取基类指针并注入各模块。CAN 仍需由装配代码配置过滤器并调用 `bsp_can_start()`。BMI088 的两个片选 GPIO由模块配置，SPI 对象仅负责总线传输。

本端口不会修改 `Core` 生成代码。若项目需要 DR16 DMA Receive-to-Idle，必须先在 CubeMX 中给 UART5_RX 配置 DMA，并在项目自己的链接脚本/MPU 配置中放置不可缓存 DMA 缓冲区；否则应使用中断模式。端口检测到 HAL 句柄没有 DMA 通道时会返回 I/O 错误，不会静默假装 DMA 已启用。

## 看门狗

只有已经建立健康任务并能周期刷新时，才把 `initialize_watchdog` 设为 true。IWDG 启动后无法软件关闭。默认配置约 2 秒；实际超时受 LSI 容差影响，比赛前应测量。

## 共享资源

TIM3 四个 PWM 通道共用频率。修改任一通道频率会改变全部 TIM3 通道，模块装配应由一个所有者统一管理频率。中断回调只发布事件，不在 ISR 内解析协议或运行控制算法。
