# STM32H723VET6 板级引脚配置

本文档对应 `board_config.h`。该头文件只描述板级资源，不包含 STM32 HAL
类型；后续平台适配层负责把端口编号、引脚编号和复用编号转换为 HAL 配置。

## 时钟树

- 板载 HSE 晶振：24 MHz；
- CPU / SYSCLK：550 MHz；
- AHB：275 MHz；
- APB1 / APB2 / APB3 / APB4：137.5 MHz；
- FDCAN 内核时钟：直接使用 24 MHz HSE；
- USB 内核时钟：HSI48，48 MHz。

PLL1 使用 `M = 3`、`N = 68`、`FRACN = 6144`、`P = 1`。因此输入为
8 MHz，倍频系数为 68.75，最终 SYSCLK 为 550 MHz。`HSE_VALUE`、IOC
时钟树和生成代码必须始终保持为 24 MHz；任何一处仍写成 8 MHz 或 25 MHz
都会导致系统节拍、串口和定时器计算错误。

## 串口

| 逻辑用途 | 外设 | 发送 | 接收 | 复用 |
|---|---|---:|---:|---:|
| 调试串口 | USART2 | PA2 | PA3 | AF7 |
| 辅助串口 | USART6 | PC6 | PC7 | AF7 |
| UART7 | UART7 | PB4 | PB3 | AF11 |
| UART8 | UART8 | PE1 | PE0 | AF8 |
| DR16 D.BUS | UART5 | — | PD2 | AF8 |

DR16 使用 UART5 单线接收。具体波特率、校验位、停止位、反相方式以及 DMA
策略属于平台初始化参数，不在板级引脚表中写死。

## CAN 网络

| 逻辑网络 | STM32 外设 | 接收 | 发送 | 复用 |
|---|---|---:|---:|---:|
| CAN1 | FDCAN1 | PD0 | PD1 | AF9 |
| CAN2 | FDCAN2 | PB5 | PB6 | AF9 |
| CAN3 | FDCAN3 | PD12 | PD13 | AF5 |

框架对上层仍使用 CAN1、CAN2、CAN3 的网络名称，平台层使用 FDCAN 外设实现。
IOC 当前为三个网络统一选择 24 MHz HSE 内核时钟、Classic CAN、1 Mbit/s、
75% 采样点。位时序为预分频 3、`SEG1 = 5`、`SEG2 = 2`、`SJW = 2`。
每路配置 32 个 RX FIFO0 元素和 32 个 TX FIFO/Queue 元素。

三个 FDCAN 实例共享同一块 Message RAM，不能都从偏移 0 开始。当前每路占用
259 个 32 位字，FDCAN1、FDCAN2、FDCAN3 的起始偏移依次为 0、259、518。
过滤器的具体 ID 路由仍由板级装配和应用项目决定。

## USB 虚拟串口

| 信号 | 引脚 | 复用 |
|---|---:|---:|
| USB OTG HS ID | PA10 | AF10 |
| USB OTG HS DM | PA11 | AF10 |
| USB OTG HS DP | PA12 | AF10 |

该组引脚对应 OTG HS 控制器的内部 Full-Speed PHY。USB CDC 设备栈、时钟和
中断由 STM32 平台适配工程提供，通用 `bsp_usb_vcp` 不直接依赖 HAL。

`general_framework.ioc` 已选择 OTG HS 控制器、Embedded PHY 和 CDC Device
中间件。实际总线运行在内部 PHY 支持的 Full-Speed 模式。

## BMI088

BMI088 使用 SPI2：

| 信号 | 引脚 | 模式 |
|---|---:|---|
| SPI2 SCK | PB10 | AF5 |
| SPI2 MISO | PB14 | AF5 |
| SPI2 MOSI | PB15 | AF5 |
| 加速度计片选 | PD8 | GPIO，低有效 |
| 陀螺仪片选 | PE15 | GPIO，低有效 |
| 加速度计中断 | PE14 | EXTI14 |
| 陀螺仪中断 | PE13 | EXTI13 |

SPI2 使用 Mode 3、8 位数据、软件片选、32 分频，SPI 内核时钟约
183.333 MHz，SCK 约 5.729 MHz。该频率同时满足 BMI088 加速度计与陀螺仪
正常通信上限，并给长排线和比赛现场电磁环境留出裕量。两个片选在 GPIO
初始化阶段必须先置高，避免上电误选中。

## PWM 与蜂鸣器

| 逻辑输出 | 定时器通道 | 引脚 | 复用 |
|---|---|---:|---:|
| auxiliary_pwm_1 | TIM3_CH1 | PA6 | AF2 |
| auxiliary_pwm_2 | TIM3_CH2 | PA7 | AF2 |
| auxiliary_pwm_3 | TIM3_CH3 | PB0 | AF2 |
| auxiliary_pwm_4 | TIM3_CH4 | PB1 | AF2 |
| buzzer_pwm | TIM1_CH1 | PE9 | AF1 |

PE9 只能映射到 TIM1_CH1，不能映射到 TIM1_CH2。蜂鸣器已经拥有这个物理
通道，不能再把 PE9 注册为另一个独立 PWM 实例，否则两个对象会争用同一硬件。

## I²C 冲突说明

原始引脚表同时把 PC6、PC7 分配给 USART6 和 I²C，但这两个引脚在
STM32H723VET6 上没有硬件 I²C 复用功能。当前配置保留 USART6，并将 I²C
实例数设为 0。若项目需要硬件 I²C，必须重新选择一组未占用且支持 I²C
复用的 SCL/SDA 引脚；不建议在赛场主控制链路中用软件模拟 I²C 复用
USART6 引脚。

## 尚未在引脚表中分配的资源

编码器、ADC 和 DAC 当前实例数均为 0。PWM 频率、普通串口波特率以及 DR16
DMA 流仍需在具体项目确定控制周期、总线负载和任务优先级后统一配置，不能
仅凭引脚表猜测。

当前 IOC 已给 DR16 配置 100000 波特、9 位字长、偶校验和单停止位；BMI088
SPI2 配置为 Mode 3、32 分频；USB 选择 HSI48 内核时钟；三个 FDCAN 网络
配置为 Classic CAN 1 Mbit/s。其余普通串口保持 CubeMX 默认异步参数，PWM
工作频率仍需在具体项目中确定。

## 移植检查

1. 只在 `board_config.h` 修改逻辑设备与引脚映射。
2. 在目标平台端为每个逻辑设备装配 `device_handle + driver_ops`。
3. 确认片选默认拉高，电机输出和蜂鸣器在启动期间保持安全状态。
4. 确认两条 IMU 中断线没有 EXTI 线号冲突。
5. 校验 FDCAN 收发器使能脚、终端电阻和总线位时序。
6. 校验 USB 使用的控制器、PHY 和 48 MHz 时钟来源。
7. 生成代码后重新检查 CubeMX 是否改写了引脚复用或中断优先级。

## IOC 与生成代码

`general_framework.ioc` 是 STM32 平台资源的生成源，通用 BSP 不直接包含由它
生成的 HAL 句柄。修改 IOC 后必须通过 STM32CubeMX 重新生成 `Core`、HAL
外设驱动引用和 USB Device 文件，再由平台装配代码把这些句柄注入 BSP
`driver_ops`。在生成动作完成前，现有最小 `Core` 只能证明框架源码可构建，
不能代表新外设已经在目标板上完成运行时初始化。
