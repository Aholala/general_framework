# BSP 层

BSP 层提供可移植、可多实例、与芯片厂商无关的外设接口。通用 BSP 不包含 STM32 HAL
头文件，不保存 CubeMX 全局句柄，也不决定实际引脚和外设实例。

当前 STM32H723VET6 板级资源、引脚冲突修正和移植检查见
[BOARD_PINOUT.md](BOARD_PINOUT.md)，对应的机器可读常量集中在
[`board_config.h`](board_config.h)。

## 目录原则

一个外设能力对应一个目录，辅助职责作为该目录内的独立文件存在。例如：

```text
bsp_can/
├── bsp_can.h
├── bsp_can.c
├── bsp_can_dispatcher.h
├── bsp_can_dispatcher.c
└── README.md

bsp_fdcan/
├── bsp_fdcan.h
├── bsp_fdcan.c
├── bsp_fdcan_classic_adapter.h
├── bsp_fdcan_classic_adapter.c
└── README.md
```

这种布局保留了单一职责文件，又避免 `dispatcher`、`adapter` 等实现角色污染顶层目录。

## 外设索引

- [bsp_common](bsp_common/README.md)：设备基类、虚函数表、统一状态与事件；
- [bsp_gpio](bsp_gpio/README.md)：数字输入输出和电平控制；
- [bsp_exti](bsp_exti/README.md)：外部中断及去抖通知；
- [bsp_usart](bsp_usart/README.md)：同步、异步和 DMA 串口；
- [bsp_spi](bsp_spi/README.md)：全双工、发送、接收和片选无关 SPI；
- [bsp_i2c](bsp_i2c/README.md)：主机收发和存储器寄存器访问；
- [bsp_can](bsp_can/README.md)：Classic CAN 与任务上下文帧分发器；
- [bsp_fdcan](bsp_fdcan/README.md)：CAN FD 与 Classic CAN 适配器；
- [bsp_timer](bsp_timer/README.md)：基本定时、计数器和周期通知；
- [bsp_pwm](bsp_pwm/README.md)：PWM 启停、频率与占空比；
- [bsp_encoder](bsp_encoder/README.md)：增量编码器计数和速度采样；
- [bsp_adc](bsp_adc/README.md)：ADC 原始值、电压和序列采样；
- [bsp_dac](bsp_dac/README.md)：DAC 原始码和电压输出；
- [bsp_usb_vcp](bsp_usb_vcp/README.md)：USB CDC 虚拟串口异步收发；
- [bsp_watchdog](bsp_watchdog/README.md)：硬件看门狗刷新与复位来源；
- [bsp_timebase](bsp_timebase/README.md)：单调时钟、周期计数和微秒时间基准。
- [bsp_storage](bsp_storage/README.md)：Flash、QSPI/OSPI、EEPROM 与 SDMMC 的统一存储基类；
- [bsp_crc](bsp_crc/README.md)：硬件 CRC 计算接口；
- [bsp_rng](bsp_rng/README.md)：硬件随机数和缓冲区填充；
- [bsp_rtc](bsp_rtc/README.md)：结构化日期时间与 Unix 时间；
- [bsp_stm32h723_port](bsp_stm32h723_port/README.md)：现有 H723 HAL 句柄到通用 BSP 对象的可替换适配层。

`bsp_stm32h723_port` 不修改 `Core`，也不替 CubeMX 创建 DMA、缓存或中断配置；它只使用项目已经生成的句柄。

## C 面向对象模型

```text
bsp_device_t
└── bsp_xxx_t
    └── bsp_xxx_device_t
```

- 派生对象首成员统一命名为 `super`；
- 基类保存只读虚表指针 `vptr`；
- 操作表使用 `static const`；
- 派生实现通过 `BSP_CONTAINER_OF` 找回完整对象；
- 公共非虚接口负责状态与参数检查，再执行虚调用；
- 硬件访问由 `device_handle + driver_ops` 注入；
- 对象初始化后只通过指针使用，不按值复制。

## 平台端与板级装配

```text
Module / App
    ↓ 仅依赖 bsp_xxx_t *
通用 BSP
    ↓ device_handle + driver_ops
平台 Port
    ↓ HAL / LL / register
MCU peripheral
```

`board_config.h` 只保存逻辑设备、容量、通道和板级选择。平台 Port 负责 HAL 操作表、
CubeMX 句柄映射、IRQ 路由、DMA 和缓存一致性。业务模块不得直接访问平台句柄。

## 生命周期

1. 静态分配对象和缓冲区；
2. 准备只读配置与平台操作表；
3. 调用 `bsp_xxx_init`；
4. 注册回调或配置过滤器；
5. 调用 `start`；
6. 在任务中处理接收事件；
7. 故障时先停止输出，再停止外设；
8. 通过虚接口 `bsp_device_deinit` 反初始化。

构造失败必须留下确定的未初始化对象。销毁后禁止继续使用旧基类指针。

## 中断规则

- ISR 只清除硬件标志、保存最小事件并调用 `bsp_xxx_notify`；
- 不在 ISR 中解析协议、计算控制器或调用阻塞发送；
- DMA 接收应尽快重启；
- 用户回调与用户上下文成对存储在实例内；
- 共享 ISR/任务字段使用最小临界区或单生产者单消费者约束；
- RTOS 同时访问同一外设时由外部互斥机制串行化。

## 错误处理

全部 BSP 使用 `bsp_status_t`。公共接口检查：

- 空对象与空参数；
- 对象是否初始化；
- 长度、范围和枚举值；
- 必选或可选虚操作是否存在；
- 底层返回的忙、超时、I/O 和协议错误。

缺少可选操作返回 `BSP_STATUS_UNSUPPORTED`，不得伪造成功。

## 内存规则

- 不使用 `malloc`；
- 对象、路由表、DMA 缓冲区和工作区由调用者提供；
- 配置对象在初始化期间必须有效；
- 被对象长期保存的操作表、句柄和上下文必须覆盖对象生命周期；
- 大块数据不进行隐藏复制，异步接口必须在各自 README 中说明缓冲区所有权。

## 构建

工程只保留根目录一个 `CMakeLists.txt`。BSP 源文件、包含目录和依赖统一在根构建文件的
“BSP layer”区域维护，子目录不再保存局部 CMake 文件。

## 新增 BSP 的完成标准

- `.h`：类型、配置、操作表和公共接口；
- `.c`：参数检查、私有虚实现、`static const` 虚表和构造函数；
- `README.md`：边界、接口、初始化、所有权、中断、移植与验证；
- 无厂商 HAL 依赖；
- 支持至少两个独立实例；
- 所有状态保存在对象内；
- 加入根 `CMakeLists.txt` 并通过 Debug、Release 和严格告警构建。
