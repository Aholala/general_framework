# board_config — STM32H723 板级配置与初始化

## 职责

`board_config.h/.c` 是 **换芯片唯一需要改的文件**。它把 CubeMX 生成的 HAL 句柄注入 BSP 抽象层。

## 初始化流程

```c
#include "board_config.h"

board_config_init_t board_init = {
    .initialize_watchdog = false,  // 调试阶段先不启动 IWDG
};

if (board_config_init(&board_init) != BSP_STATUS_OK)
{
    // board_init.failed_step → "spi"        ← 哪个外设失败
    // board_init.last_error  → BSP_STATUS_TIMEOUT  ← 具体错误
    // bsp_error_read()       → 全局寄存器，任何任务可查
    Error_Handler();
}
```

**初始化顺序**：CAN(×3) → USART(DR16) → SPI(BMI088) → EXTI(×2) → PWM(蜂鸣器) → USB VCP → DWT → Watchdog(可选)

**失败回滚**：任一步失败 → 逆序停止已初始化的外设 → 记录错误 → 返回错误码。

## 获取外设

```c
bsp_can_t    *can1   = board_config_get_can(BOARD_CONFIG_CAN_1);
bsp_usart_t  *dr16   = board_config_get_usart(BOARD_CONFIG_UART_DR16);
bsp_spi_t    *bmi088 = board_config_get_bmi088_spi();
bsp_exti_t   *gyro   = board_config_get_exti(BOARD_CONFIG_EXTI_BMI088_GYROSCOPE);
bsp_pwm_t    *buzzer = board_config_get_pwm(BOARD_CONFIG_PWM_BUZZER);
bsp_usb_vcp_t *usb  = board_config_get_usb_vcp();
bsp_dwt_t    *dwt    = board_config_get_dwt();
bsp_watchdog_t *wdg = board_config_get_watchdog();
```

所有 getter 在 `board_config_initialized` 之前返回 `NULL`。

## 关键结构体

### board_config_init_t

| 字段 | 类型 | 说明 |
|------|------|------|
| `initialize_watchdog` | `bool` | 是否启动 IWDG |
| `last_error` | `bsp_status_t` | 失败时由 init 填充的具体错误码 |
| `failed_step` | `const char *` | 失败的外设名（`"can"`/`"usart"`/`"spi"`/...） |

### 外设索引枚举

```c
board_config_can_index_t   // BOARD_CONFIG_CAN_1 / _2 / _3
board_config_usart_index_t // BOARD_CONFIG_UART_DR16
board_config_exti_index_t  // BOARD_CONFIG_EXTI_BMI088_GYROSCOPE / _ACCELEROMETER
board_config_pwm_index_t   // BOARD_CONFIG_PWM_BUZZER（可扩展舵机槽位）
```

## 添加新外设

以添加舵机 PWM 为例：

**1.** `board_config.h` — 取消注释枚举槽位：
```c
BOARD_CONFIG_PWM_SERVO_1,  // 枚举值自动变为 1
// BOARD_CONFIG_PWM_COUNT 自动变为 2
```

**2.** `board_config.c` — 在 PWM entries 表中追加：
```c
[BOARD_CONFIG_PWM_SERVO_1] = {&htim2, 1U, BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL},
```

**3.** App 层获取：
```c
bsp_pwm_t *servo_pwm = board_config_get_pwm(BOARD_CONFIG_PWM_SERVO_1);
```

不需要改 BSP 层、Module 层、初始化流程。

## 板级常量

| 宏 | 值 | 说明 |
|----|-----|------|
| `BOARD_CONFIG_CPU_FREQUENCY_HZ` | 480MHz | CPU 主频 |
| `BOARD_CONFIG_APB_FREQUENCY_HZ` | 120MHz | APB 总线（PWM 时钟基准） |
| `BOARD_CONFIG_FDCAN_KERNEL_FREQUENCY_HZ` | 120MHz | FDCAN 内核时钟 |
| `BOARD_CONFIG_CAN_COUNT` | 3 | FDCAN1/2/3 |
| `BOARD_CONFIG_EXTI_COUNT` | 2 | BMI088 加速度+陀螺中断 |
| `BOARD_CONFIG_PWM_COUNT` | 1 | 蜂鸣器 |

引脚、DMA、NVIC 由 CubeMX 生成的 `Core/Src/*.c` 管理，不在本文件定义。

## 换 MCU 移植

只需要：
1. 替换 `Core/`（CubeMX 重新生成）
2. 替换本文件中的 HAL 句柄（`&hfdcan1`→新 MCU 的 CAN 句柄）
3. 替换 `driver_ops` 实现（HAL API 不同则改函数体）

`User/Bsp/`、`User/Module/`、`User/Algorithm/`、`User/App/` **一字不动**。
