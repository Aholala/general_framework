# BSP STM32H723 端口适配层 (bsp_stm32h723_port)

## 1. 模块概述

`bsp_stm32h723_port` 是**通用 BSP 与 STM32H723 HAL 之间唯一的芯片适配层**。它将 STM32H723 的 HAL 外设句柄（FDCAN、USART、SPI、TIM、EXTI 等）映射为通用 BSP 对象（`bsp_can_t`、`bsp_usart_t`、`bsp_spi_t` 等），使上层 Module 完全无需感知 HAL 或具体寄存器。

**核心职责**：

- 集中管理所有外设的静态设备对象和驱动操作表。
- 将 HAL 回调路由到通用 BSP 的 `notify` 函数。
- 提供外设基类指针的 getter 函数，供 Module 层注入。
- 初始化时间基准（DWT）、看门狗（可选）、USB VCP 等。

**设计原则**：

- **隔离 HAL**：通用 BSP 头文件（`bsp_can.h` 等）不包含任何 HAL 头文件。
- **零动态内存**：所有设备对象和缓冲区均为静态分配。
- **中断轻量化**：HAL 回调只发布事件，不解析协议或运行控制算法。
- **显式配置**：片选、过滤器等由上层 Module 配置，端口层不假设。

## 2. 外设清单

| 外设类型        | 实例数量 | 说明                                             |
| :-------------- | :------- | :----------------------------------------------- |
| **CAN (FDCAN)** | 3        | FDCAN1、FDCAN2、FDCAN3，支持 Classic CAN 帧      |
| **USART**       | 5        | USART2、USART6、UART7、UART8、UART5（DR16 专用） |
| **SPI**         | 1        | SPI2（BMI088 传感器）                            |
| **EXTI**        | 2        | BMI088 陀螺仪中断、加速度计中断                  |
| **PWM**         | 5        | TIM3（4 通道）+ TIM1（蜂鸣器）                   |
| **USB VCP**     | 1        | USB 虚拟串口（CDC）                              |
| **Timebase**    | 1        | DWT 周期计数器（高精度时间基准）                 |
| **Watchdog**    | 1        | 独立看门狗 IWDG1（可选）                         |

## 3. 对象模型

```text
┌─────────────────────────────────────────────────────────────┐
│                    bsp_stm32h723_port.c                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  静态设备对象 (bsp_xxx_device_t)                    │   │
│  │  ├── bsp_stm32h723_can_devices[3]                  │   │
│  │  ├── bsp_stm32h723_usart_devices[5]                │   │
│  │  ├── bsp_stm32h723_bmi088_spi_device               │   │
│  │  ├── bsp_stm32h723_exti_devices[2]                 │   │
│  │  ├── bsp_stm32h723_pwm_devices[5]                  │   │
│  │  ├── bsp_stm32h723_usb_device                      │   │
│  │  ├── bsp_stm32h723_timebase_device                 │   │
│  │  └── bsp_stm32h723_watchdog_device                 │   │
│  └─────────────────────────────────────────────────────┘   │
│                            │                                │
│                            ▼                                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  驱动操作表 (bsp_xxx_driver_ops_t)                  │   │
│  │  ├── bsp_stm32h723_can_driver_ops                  │   │
│  │  ├── bsp_stm32h723_usart_driver_ops                │   │
│  │  ├── bsp_stm32h723_spi_driver_ops                  │   │
│  │  ├── bsp_stm32h723_exti_driver_ops                 │   │
│  │  ├── bsp_stm32h723_pwm_driver_ops                  │   │
│  │  ├── bsp_stm32h723_timebase_driver_ops             │   │
│  │  ├── bsp_stm32h723_watchdog_driver_ops             │   │
│  │  └── bsp_stm32h723_usb_driver_ops                  │   │
│  └─────────────────────────────────────────────────────┘   │
│                            │                                │
│                            ▼                                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  HAL 回调路由 → bsp_xxx_notify()                    │   │
│  │  ├── HAL_FDCAN_RxFifo0Callback → bsp_can_notify    │   │
│  │  ├── HAL_UART_TxCpltCallback → bsp_usart_notify    │   │
│  │  ├── HAL_SPI_TxRxCpltCallback → bsp_spi_notify     │   │
│  │  ├── HAL_GPIO_EXTI_Callback → bsp_exti_notify      │   │
│  │  └── usb_cdc_receive_callback → bsp_usb_vcp_notify │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 4. 初始化与使用

### 4.1 前提条件

- CubeMX 已完成 GPIO、FDCAN、SPI、TIM、UART、USB 的初始化。
- `board_config.h` 定义了 `BOARD_CONFIG_APB_FREQUENCY_HZ`（APB 时钟频率）。
- 若使用 DMA，需在 CubeMX 中配置对应 DMA 通道。

### 4.2 端口初始化

```c
#include "bsp_stm32h723_port.h"

// 端口配置
const bsp_stm32h723_port_config_t port_config = {
    .initialize_watchdog = false,   // 暂不初始化看门狗
};

// 初始化端口
if (bsp_stm32h723_port_init(&port_config) != BSP_STATUS_OK) {
    // 初始化失败处理
}
```

### 4.3 获取外设基类指针

```c
// 获取 CAN1 基类指针
bsp_can_t *can1 = bsp_stm32h723_port_get_can(BSP_STM32H723_CAN_1);

// 获取 USART_DR16（UART5）基类指针
bsp_usart_t *dr16_usart = bsp_stm32h723_port_get_usart(BSP_STM32H723_UART_DR16);

// 获取 BMI088 SPI 基类指针
bsp_spi_t *bmi088_spi = bsp_stm32h723_port_get_bmi088_spi();

// 获取 BMI088 陀螺仪中断 EXTI
bsp_exti_t *gyro_exti = bsp_stm32h723_port_get_exti(BSP_STM32H723_EXTI_BMI088_GYROSCOPE);

// 获取 PWM（辅助通道1）
bsp_pwm_t *pwm_aux1 = bsp_stm32h723_port_get_pwm(BSP_STM32H723_PWM_AUXILIARY_1);

// 获取 USB VCP
bsp_usb_vcp_t *usb_vcp = bsp_stm32h723_port_get_usb_vcp();

// 获取 Timebase
bsp_timebase_t *timebase = bsp_stm32h723_port_get_timebase();

// 获取 Watchdog（需先启用）
bsp_watchdog_t *watchdog = bsp_stm32h723_port_get_watchdog();
```

### 4.4 典型应用初始化流程

```c
// 1. 初始化 BSP 端口
bsp_stm32h723_port_init(&port_config);

// 2. 获取外设基类指针
bsp_can_t *can = bsp_stm32h723_port_get_can(BSP_STM32H723_CAN_1);
bsp_spi_t *spi = bsp_stm32h723_port_get_bmi088_spi();
bsp_exti_t *exti = bsp_stm32h723_port_get_exti(BSP_STM32H723_EXTI_BMI088_GYROSCOPE);
bsp_timebase_t *tb = bsp_stm32h723_port_get_timebase();

// 3. 配置 CAN 过滤器
bsp_can_filter_t filter = { ... };
bsp_can_configure_filter(can, &filter);
bsp_can_start(can);

// 4. 配置 BMI088 模块（片选由模块控制）
// 模块内部会使用 spi 和 exti

// 5. 启动时间基准
bsp_timebase_reset(tb);
```

## 5. 外设配置详情

### 5.1 CAN (FDCAN)

| 实例  | HAL 句柄  | 说明   |
| :---- | :-------- | :----- |
| CAN_1 | `hfdcan1` | FDCAN1 |
| CAN_2 | `hfdcan2` | FDCAN2 |
| CAN_3 | `hfdcan3` | FDCAN3 |

- **帧格式**：仅支持 Classic CAN（FDFormat = FDCAN_CLASSIC_CAN）。
- **接收 FIFO**：FIFO0 和 FIFO1 均支持。
- **过滤器**：支持标准/扩展 ID 的掩码过滤。
- **中断**：`HAL_FDCAN_RxFifo0Callback`、`HAL_FDCAN_RxFifo1Callback`、`HAL_FDCAN_ErrorStatusCallback`。

### 5.2 USART

| 实例      | HAL 句柄 | 典型用途        |
| :-------- | :------- | :-------------- |
| USART_2   | `huart2` | 调试/通用       |
| USART_6   | `huart6` | 调试/通用       |
| UART_7    | `huart7` | 通用            |
| UART_8    | `huart8` | 通用            |
| UART_DR16 | `huart5` | DR16 遥控器接收 |

- **传输模式**：支持阻塞、中断、DMA（需 CubeMX 配置）。
- **空闲接收**：`receive_to_idle` 支持 DMA 空闲中断（DR16 推荐）。
- **中断**：`HAL_UART_TxCpltCallback`、`HAL_UART_RxCpltCallback`、`HAL_UARTEx_RxEventCallback`、`HAL_UART_ErrorCallback`。

### 5.3 SPI

| 实例       | HAL 句柄 | 说明                 |
| :--------- | :------- | :------------------- |
| BMI088 SPI | `hspi2`  | 仅用于 BMI088 传感器 |

- **传输模式**：支持阻塞、中断、DMA。
- **片选控制**：由 Module 层通过 GPIO 控制，SPI 驱动不管理片选。
- **中断**：`HAL_SPI_TxCpltCallback`、`HAL_SPI_RxCpltCallback`、`HAL_SPI_TxRxCpltCallback`。

### 5.4 PWM

| 实例       | HAL 句柄 | 通道 | 说明           |
| :--------- | :------- | :--- | :------------- |
| PWM_AUX_1  | `htim3`  | CH1  | TIM3 辅助通道1 |
| PWM_AUX_2  | `htim3`  | CH2  | TIM3 辅助通道2 |
| PWM_AUX_3  | `htim3`  | CH3  | TIM3 辅助通道3 |
| PWM_AUX_4  | `htim3`  | CH4  | TIM3 辅助通道4 |
| PWM_BUZZER | `htim1`  | CH1  | TIM1 蜂鸣器    |

- **频率共享**：TIM3 的四个 PWM 通道共享同一频率（ARR）。修改任一通道频率会影响全部四个通道。
- **频率范围**：基于 APB 时钟（2 倍频）计算，预分频器由 CubeMX 配置。
- **周期限制**：最大周期值 65536。

### 5.5 EXTI

| 实例          | 引脚                   | 中断号                       | 说明                 |
| :------------ | :--------------------- | :--------------------------- | :------------------- |
| GYROSCOPE     | `BMI088_GYRO_INT_Pin`  | `BMI088_GYRO_INT_EXTI_IRQn`  | 陀螺仪数据就绪中断   |
| ACCELEROMETER | `BMI088_ACCEL_INT_Pin` | `BMI088_ACCEL_INT_EXTI_IRQn` | 加速度计数据就绪中断 |

- **中断使能**：通过 `bsp_exti_enable` 使能 NVIC。
- **回调**：`HAL_GPIO_EXTI_Callback` 根据引脚号路由到对应 EXTI 对象。

### 5.6 Timebase (DWT)

- **原理**：使用 DWT 周期计数器（CYCCNT），精度为 CPU 周期级。
- **频率**：`SystemCoreClock`（CPU 主频）。
- **用途**：高精度时间测量、性能分析、微秒级延时。

### 5.7 Watchdog (IWDG1)

- **启动**：`initialize_watchdog = true` 时初始化。
- **超时**：默认约 2 秒（受 LSI 容差影响，建议实际测量）。
- **特性**：启动后无法软件关闭，仅硬件复位可停止。
- **复位检测**：通过 `__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)` 检测是否由看门狗复位。

### 5.8 USB VCP

- **协议**：USB CDC（虚拟串口）。
- **接收缓冲**：512 字节环形缓冲区（静态分配）。
- **回调**：`usb_cdc_receive_callback` 由 USB 协议栈调用，转发到 `bsp_usb_vcp_notify`。

## 6. 中断回调路由

所有 HAL 回调均由 `bsp_stm32h723_port.c` 中的路由函数拦截，并转发到对应的通用 BSP `notify` 函数：

| HAL 回调                        | 转发目标             | 事件                |
| :------------------------------ | :------------------- | :------------------ |
| `HAL_FDCAN_RxFifo0Callback`     | `bsp_can_notify`     | `RECEIVE_PENDING`   |
| `HAL_FDCAN_RxFifo1Callback`     | `bsp_can_notify`     | `RECEIVE_PENDING`   |
| `HAL_FDCAN_ErrorStatusCallback` | `bsp_can_notify`     | `ERROR`             |
| `HAL_UART_TxCpltCallback`       | `bsp_usart_notify`   | `TRANSMIT_COMPLETE` |
| `HAL_UART_RxCpltCallback`       | `bsp_usart_notify`   | `RECEIVE_COMPLETE`  |
| `HAL_UARTEx_RxEventCallback`    | `bsp_usart_notify`   | `RECEIVE_COMPLETE`  |
| `HAL_UART_ErrorCallback`        | `bsp_usart_notify`   | `ERROR`             |
| `HAL_SPI_TxRxCpltCallback`      | `bsp_spi_notify`     | `TRANSFER_COMPLETE` |
| `HAL_SPI_TxCpltCallback`        | `bsp_spi_notify`     | `TRANSMIT_COMPLETE` |
| `HAL_SPI_RxCpltCallback`        | `bsp_spi_notify`     | `RECEIVE_COMPLETE`  |
| `HAL_GPIO_EXTI_Callback`        | `bsp_exti_notify`    | （无事件参数）      |
| `usb_cdc_receive_callback`      | `bsp_usb_vcp_notify` | `RECEIVE_PENDING`   |

**重要原则**：所有回调在 ISR 中执行，只发布事件，不解析协议或运行控制算法。

## 7. 端口初始化状态机

```text
                    ┌──────────────────┐
                    │   uninitialized  │
                    └────────┬─────────┘
                             │ bsp_stm32h723_port_init()
                             ▼
                    ┌──────────────────┐
                    │  init_can()      │ ──失败──> 返回 IO_ERROR
                    │  init_usart()    │
                    │  init_spi()      │
                    │  init_exti()     │
                    │  init_pwm()      │
                    │  init_usb()      │
                    │  init_timebase() │
                    └────────┬─────────┘
                             │
                   ┌─────────▼─────────┐
                   │ initialize_watchdog│
                   │  ? true : false   │
                   └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │    initialized   │
                    └──────────────────┘
```

## 8. 错误码

| 错误码                        | 触发场景                 |
| :---------------------------- | :----------------------- |
| `BSP_STATUS_INVALID_ARGUMENT` | config 为 NULL           |
| `BSP_STATUS_BUSY`             | 端口已初始化（重复调用） |
| `BSP_STATUS_IO_ERROR`         | 任一外设初始化失败       |
| `BSP_STATUS_UNSUPPORTED`      | DWT 不支持（极少发生）   |

## 9. 移植与注意事项

### 9.1 CubeMX 配置要求

- **FDCAN**：启用所需的 FDCAN 实例，配置位时序。
- **USART**：启用并配置波特率，需要 DMA 的实例配置 DMA 通道。
- **SPI**：启用 SPI2，配置模式（CPOL/CPHA）与 BMI088 匹配。
- **TIM**：启用 TIM3（4 通道）、TIM1（蜂鸣器），配置预分频器。
- **EXTI**：配置 GPIO 引脚为外部中断模式（上升沿/下降沿）。
- **USB**：启用 USB_OTG_FS，配置为 CDC 类。

### 9.2 共享资源注意事项

- **TIM3 频率共享**：修改任一 TIM3 通道频率会改变全部四个通道。建议由单一 Module 统一管理 TIM3 的频率。
- **DMA 缓存**：DR16 若使用 DMA Receive-to-Idle，需在 MPU 中将 DMA 缓冲区设置为不可缓存。

### 9.3 看门狗注意事项

- 启动后无法软件关闭，仅硬件复位可禁用。
- 默认超时约 2 秒（受 LSI 影响，建议实际测量）。
- 只有健康任务能定期刷新时，才启用看门狗。

### 9.4 中断优先级

- 建议所有外设中断优先级不低于控制任务，避免影响实时性。
- EXTI 中断优先级应高于控制任务，确保传感器数据及时响应。

## 10. 建议验证测试项

- [ ] 端口初始化成功，所有 getter 返回非 NULL。
- [ ] CAN 发送/接收正常（可结合回环测试）。
- [ ] USART 三种模式（阻塞/中断/DMA）均正常工作。
- [ ] SPI 读写 BMI088 寄存器正常。
- [ ] EXTI 中断触发后回调正确执行。
- [ ] PWM 输出波形正确（频率、占空比）。
- [ ] USB VCP 收发正常。
- [ ] Timebase 读数准确（验证微秒级延时）。
- [ ] 看门狗复位检测功能正常。
- [ ] 重复调用端口初始化返回 `BUSY`。

---

**总结**：`bsp_stm32h723_port` 是通用 BSP 与 STM32H723 硬件之间的桥梁，通过集中管理设备对象、驱动操作表和 HAL 回调路由，实现了硬件与业务逻辑的彻底解耦。上层 Module 只需通过 getter 获取基类指针，无需关心 HAL 细节，极大地提高了代码的可移植性和可维护性。
