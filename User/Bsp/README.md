# BSP 层 —— 可移植外设抽象框架

## 1. 概述

BSP（Board Support Package）层提供了一套**与芯片厂商无关、可多实例、可移植**的外设接口抽象。它采用 C 语言模拟面向对象的设计，通过虚表和 `container_of` 宏实现多态，将硬件访问与业务逻辑彻底解耦。

**核心原则**：

- **无 HAL 依赖**：通用 BSP 头文件不包含任何厂商 HAL 头文件（如 `stm32h7xx_hal.h`）。
- **无全局句柄**：不保存 CubeMX 生成的全局外设句柄（如 `hfdcan1`）。
- **无引脚硬编码**：不决定实际引脚和外设实例，由板级配置文件描述。
- **静态内存**：所有对象由调用者静态分配，不使用 `malloc`。

当前 STM32H723VET6 板级资源、引脚冲突修正和移植检查见 [`BOARD_PINOUT.md`](BOARD_PINOUT.md)，对应的机器可读常量集中在 [`board_config.h`](board_config.h)。

## 2. 目录结构

每个外设能力对应一个目录，辅助职责作为该目录内的独立文件存在。这种布局保留了单一职责文件，又避免 `dispatcher`、`adapter` 等实现角色污染顶层目录。

```
bsp_can/                          # Classic CAN 外设
├── bsp_can.h                     # 公共接口
├── bsp_can.c                     # 实现
├── bsp_can_dispatcher.h          # 帧分发器（辅助）
├── bsp_can_dispatcher.c
└── README.md

bsp_fdcan/                        # CAN FD 外设
├── bsp_fdcan.h
├── bsp_fdcan.c
├── bsp_fdcan_classic_adapter.h   # Classic 适配器（辅助）
├── bsp_fdcan_classic_adapter.c
└── README.md
```

## 3. 外设索引

| 模块                                                 | 说明                                             |
| :--------------------------------------------------- | :----------------------------------------------- |
| [`bsp_common`](bsp_common/README.md)                 | 设备基类、虚函数表、统一状态与事件               |
| [`bsp_gpio`](bsp_gpio/README.md)                     | 数字输入输出和电平控制                           |
| [`bsp_exti`](bsp_exti/README.md)                     | 外部中断及去抖通知                               |
| [`bsp_usart`](bsp_usart/README.md)                   | 同步、异步和 DMA 串口                            |
| [`bsp_spi`](bsp_spi/README.md)                       | 全双工、发送、接收和片选无关 SPI                 |
| [`bsp_i2c`](bsp_i2c/README.md)                       | 主机收发和存储器寄存器访问                       |
| [`bsp_can`](bsp_can/README.md)                       | Classic CAN 与任务上下文帧分发器                 |
| [`bsp_fdcan`](bsp_fdcan/README.md)                   | CAN FD 与 Classic CAN 适配器                     |
| [`bsp_timer`](bsp_timer/README.md)                   | 基本定时、计数器和周期通知                       |
| [`bsp_pwm`](bsp_pwm/README.md)                       | PWM 启停、频率与占空比                           |
| [`bsp_encoder`](bsp_encoder/README.md)               | 增量编码器计数和速度采样                         |
| [`bsp_adc`](bsp_adc/README.md)                       | ADC 原始值、电压和序列采样                       |
| [`bsp_dac`](bsp_dac/README.md)                       | DAC 原始码和电压输出                             |
| [`bsp_usb_vcp`](bsp_usb_vcp/README.md)               | USB CDC 虚拟串口异步收发                         |
| [`bsp_watchdog`](bsp_watchdog/README.md)             | 硬件看门狗刷新与复位来源                         |
| [`bsp_timebase`](bsp_timebase/README.md)             | 单调时钟、周期计数和微秒时间基准                 |
| [`bsp_storage`](bsp_storage/README.md)               | Flash、QSPI/OSPI、EEPROM 与 SDMMC 的统一存储基类 |
| [`bsp_crc`](bsp_crc/README.md)                       | 硬件 CRC 计算接口                                |
| [`bsp_rng`](bsp_rng/README.md)                       | 硬件随机数和缓冲区填充                           |
| [`bsp_rtc`](bsp_rtc/README.md)                       | 结构化日期时间与 Unix 时间                       |
| [`bsp_stm32h723_port`](bsp_stm32h723_port/README.md) | H723 HAL 句柄到通用 BSP 对象的适配层             |

> `bsp_stm32h723_port` 不修改 `Core` 生成的代码，也不替 CubeMX 创建 DMA、缓存或中断配置；它只使用项目已经生成的 HAL 句柄。

## 4. C 面向对象模型

### 4.1 继承层次

```text
bsp_device_t
└── bsp_xxx_t          （基类：增加外设特有字段）
    └── bsp_xxx_device_t（派生类：持有 driver_ops 和通道信息）
```

### 4.2 设计规范

| 规则                                         | 说明                           |
| :------------------------------------------- | :----------------------------- |
| 派生对象首成员统一命名为 `super`             | 保证 `container_of` 宏的正确性 |
| 基类保存只读虚表指针 `vptr`                  | 实现运行时多态                 |
| 操作表使用 `static const`                    | 存储于只读区，所有实例共享     |
| 派生实现通过 `BSP_CONTAINER_OF` 找回完整对象 | 避免强制类型转换               |
| 公共非虚接口负责状态与参数检查               | 再执行虚调用，保证安全性       |
| 硬件访问由 `device_handle + driver_ops` 注入 | 实现硬件解耦                   |
| 对象初始化后只通过指针使用                   | 禁止按值复制                   |

### 4.3 `container_of` 宏

```c
#define BSP_CONTAINER_OF(pointer, type, member) \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))
```

**使用示例**（在虚函数实现中）：

```c
static bsp_status_t bsp_uart_send(bsp_uart_t *base, ...) {
    bsp_uart_device_t *dev = BSP_CONTAINER_OF(base, bsp_uart_device_t, super);
    return dev->driver_ops->send(dev->super.device_handle, ...);
}
```

## 5. 平台端与板级装配

### 5.1 分层架构

```text
┌─────────────────────────────────────────────────────────────┐
│                      Module / App                          │
│               （仅依赖 bsp_xxx_t * 基类指针）               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                     通用 BSP 层                            │
│       （bsp_xxx.h/.c：虚表、对象管理、参数校验）           │
└─────────────────────────────────────────────────────────────┘
                              ↓ device_handle + driver_ops
┌─────────────────────────────────────────────────────────────┐
│                   平台 Port 层                             │
│      （bsp_stm32h723_port：HAL 句柄映射、IRQ 路由）        │
└─────────────────────────────────────────────────────────────┘
                              ↓ HAL / LL / register
┌─────────────────────────────────────────────────────────────┐
│                    MCU 外设                                │
│              （FDCAN、USART、SPI、TIM...）                 │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 职责划分

| 层级               | 职责                                                    |
| :----------------- | :------------------------------------------------------ |
| **board_config.h** | 逻辑设备名称、容量、通道、引脚编码、板级选择            |
| **平台 Port**      | HAL 操作表、CubeMX 句柄映射、IRQ 路由、DMA 和缓存一致性 |
| **通用 BSP**       | 多态接口、参数校验、事件回调、对象生命周期              |
| **Module / App**   | 业务逻辑、协议解析、控制算法                            |

**关键约束**：业务模块不得直接访问平台 HAL 句柄。

## 6. 生命周期

### 6.1 标准流程

```text
1. 静态分配对象和缓冲区
   ↓
2. 准备只读配置与平台操作表
   ↓
3. 调用 bsp_xxx_init
   ↓
4. 注册回调或配置过滤器
   ↓
5. 调用 start
   ↓
6. 在任务中处理接收事件
   ↓
7. 故障时先停止输出，再停止外设
   ↓
8. 通过虚接口 bsp_device_deinit 反初始化
```

### 6.2 关键规则

- 构造失败必须留下**确定的未初始化对象**（`is_initialized = false`）。
- 销毁后禁止继续使用旧基类指针。
- 反初始化前应确保所有异步操作已停止或中止。

## 7. 中断规则

### 7.1 ISR 职责

- 清除硬件标志
- 保存最小事件数据
- 调用 `bsp_xxx_notify` 通知上层
- DMA 接收应尽快重启

### 7.2 ISR 禁止行为

- ❌ 解析协议（如 CAN 帧解析、UART 协议解码）
- ❌ 运行控制算法（如 PID 计算、电机控制）
- ❌ 调用阻塞发送（如 `HAL_Delay`）
- ❌ 打印日志（`printf`）

### 7.3 回调约束

- 用户回调与用户上下文成对存储在实例内
- 共享 ISR/任务字段使用最小临界区或单生产者单消费者约束
- RTOS 同时访问同一外设时由外部互斥机制串行化

## 8. 错误处理

### 8.1 统一状态码

全部 BSP 使用 `bsp_status_t` 返回错误码：

| 状态码                        | 含义                         |
| :---------------------------- | :--------------------------- |
| `BSP_STATUS_OK`               | 操作成功                     |
| `BSP_STATUS_INVALID_ARGUMENT` | 参数非法（空指针、超出范围） |
| `BSP_STATUS_OUT_OF_RANGE`     | 数值超出硬件/协议范围        |
| `BSP_STATUS_NOT_INITIALIZED`  | 对象未初始化                 |
| `BSP_STATUS_BUSY`             | 资源忙（异步操作进行中）     |
| `BSP_STATUS_TIMEOUT`          | 等待超时                     |
| `BSP_STATUS_IO_ERROR`         | I/O 错误（总线故障）         |
| `BSP_STATUS_NO_RESOURCE`      | 资源不足（队列/路由表满）    |
| `BSP_STATUS_UNSUPPORTED`      | 可选操作未实现               |

### 8.2 公共接口检查

所有公共接口必须检查：

- 空对象与空参数
- 对象是否初始化
- 长度、范围和枚举值合法性
- 必选或可选虚操作是否存在
- 底层返回的忙、超时、I/O 和协议错误

**重要**：缺少可选操作返回 `BSP_STATUS_UNSUPPORTED`，不得伪造成功。

## 9. 内存规则

- ❌ **禁止使用 `malloc` / `free`**：所有内存分配为静态。
- ✅ 对象、路由表、DMA 缓冲区和工作区由调用者提供。
- ✅ 配置对象在初始化期间必须保持有效。
- ✅ 被对象长期保存的操作表、句柄和上下文必须覆盖对象生命周期。
- ✅ 大块数据不进行隐藏复制，异步接口必须在各自 README 中说明缓冲区所有权。

## 10. 构建

工程只保留根目录一个 `CMakeLists.txt`。BSP 源文件、包含目录和依赖统一在根构建文件的 **"BSP layer"** 区域维护，子目录不再保存局部 CMake 文件。

## 11. 新增 BSP 模块的完成标准

新增一个 BSP 外设模块时，必须满足以下标准：

| 文件        | 内容要求                                                                   |
| :---------- | :------------------------------------------------------------------------- |
| `.h`        | 类型定义、配置结构、操作表、公共接口声明                                   |
| `.c`        | 参数校验、私有虚函数实现、`static const` 虚表、构造函数                    |
| `README.md` | 边界说明、接口文档、初始化示例、所有权约束、中断规则、移植要求、验证测试项 |

**质量要求**：

- ✅ 无厂商 HAL 头文件依赖
- ✅ 支持至少两个独立实例
- ✅ 所有状态保存在对象内（无全局变量）
- ✅ 加入根 `CMakeLists.txt` 并通过 Debug、Release 和严格告警构建

---

## 12. 快速参考

### 12.1 外设模块概览

| 模块类型        | 模块名称             | 主要功能                               |
| :-------------- | :------------------- | :------------------------------------- |
| **基础设施**    | `bsp_common`         | 设备基类、状态码、事件、`container_of` |
| **数字 I/O**    | `bsp_gpio`           | 读/写/翻转电平                         |
|                 | `bsp_exti`           | 外部中断回调                           |
| **模拟 I/O**    | `bsp_adc`            | ADC 采样（原始值/归一化/电压）         |
|                 | `bsp_dac`            | DAC 输出（原始值/归一化/电压）         |
|                 | `bsp_pwm`            | PWM 频率/占空比                        |
| **定时/计数**   | `bsp_timer`          | 基本定时器、周期中断                   |
|                 | `bsp_encoder`        | 增量编码器计数、增量计算               |
|                 | `bsp_timebase`       | 周期计数、微秒延时                     |
| **通信**        | `bsp_usart`          | UART 收发（固定长度/空闲线）           |
|                 | `bsp_spi`            | SPI 收发（全双工/半双工）              |
|                 | `bsp_i2c`            | I2C 收发（含寄存器访问）               |
|                 | `bsp_can`            | Classic CAN + 帧分发器                 |
|                 | `bsp_fdcan`          | CAN FD + Classic 适配器                |
|                 | `bsp_usb_vcp`        | USB 虚拟串口                           |
| **存储**        | `bsp_storage`        | Flash/QSPI/EEPROM/SDMMC 统一接口       |
| **安全/可靠性** | `bsp_watchdog`       | 看门狗刷新、复位检测                   |
|                 | `bsp_crc`            | 硬件 CRC 计算                          |
|                 | `bsp_rng`            | 硬件随机数                             |
| **时间**        | `bsp_rtc`            | 实时时钟（结构化时间/Unix 时间戳）     |
| **芯片适配**    | `bsp_stm32h723_port` | H723 HAL 到 BSP 的适配层               |

---

**总结**：BSP 层通过面向对象的 C 语言设计，将硬件访问与业务逻辑解耦，使得模块可以在不同 MCU 平台间移植。统一的接口风格、错误处理和生命周期管理，降低了学习成本和维护成本。新增 BSP 模块时，遵循本 README 中的标准规范，可保证整个 BSP 层的一致性和可维护性。
