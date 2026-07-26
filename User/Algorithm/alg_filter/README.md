# alg_filter

`alg_filter` 是 Algorithm 层的纯 C11 滤波库。文件、公共函数和类型统一使用
`alg_filter_` 前缀，宏和枚举常量使用 `ALG_FILTER_` 前缀。

## 功能范围

| 滤波器 | 对象类型 | 主要用途 |
|---|---|---|
| 一阶低通 | `alg_filter_low_pass_t` | 抑制高频噪声 |
| 一阶高通 | `alg_filter_high_pass_t` | 去除直流分量和缓慢漂移 |
| 指数平均 | `alg_filter_exponential_t` | 低成本平滑 |
| 滑动平均 | `alg_filter_moving_average_t` | 固定窗口均值 |
| 中值滤波 | `alg_filter_median_t` | 抑制脉冲噪声和离群点 |
| 通用 FIR | `alg_filter_fir_t` | 使用调用者提供的任意 FIR 系数 |
| Biquad | `alg_filter_biquad_t` | 二阶低通、高通、带通和陷波 |
| 互补滤波 | `alg_filter_complementary_t` | 融合测量值和变化率 |

卡尔曼滤波不属于本模块。它将在独立的 `alg_kalman` 算法模块中实现，避免滤波基础库承担矩阵、状态模型和观测模型等额外职责。

## 可移植性约束

- 不包含 STM32、HAL、CMSIS 或 RTOS 头文件。
- 不调用系统时间函数，采样周期由调用者显式传入。
- 不使用动态内存。
- 不使用可变全局状态。
- 每个对象拥有独立状态，支持任意数量的静态实例。
- 只依赖 C11 标准库中的 `<stdbool.h>`、`<stddef.h>` 和 `<math.h>`。
- 窗口滤波器与 FIR 使用调用者持有的缓冲区。
- 所有公共输入均检查空指针、范围、NaN 和无穷值。

## 低通滤波示例

```c
#include "alg_filter.h"

static alg_filter_low_pass_t s_velocity_filter;

void app_velocity_filter_init(void)
{
    (void)alg_filter_low_pass_init(&s_velocity_filter, 30.0F);
}

alg_filter_status_t app_velocity_filter_update(float raw_velocity,
                                               float delta_time_s,
                                               float *filtered_velocity)
{
    return alg_filter_low_pass_update(&s_velocity_filter,
                                   raw_velocity,
                                   delta_time_s,
                                   filtered_velocity);
}
```

## 窗口滤波器内存

```c
#define VELOCITY_WINDOW_SIZE (8U)

static float s_velocity_samples[VELOCITY_WINDOW_SIZE];
static alg_filter_moving_average_t s_velocity_filter;

void app_velocity_average_init(void)
{
    (void)alg_filter_moving_average_init(&s_velocity_filter,
                                      s_velocity_samples,
                                      VELOCITY_WINDOW_SIZE);
}
```

缓冲区的生命周期必须长于滤波器对象。不要把函数局部数组交给需要在函数返回后继续工作的滤波器。

中值滤波需要两个等长缓冲区：一个保存窗口样本，一个作为排序工作区。这避免了动态内存和变长栈数组。

## FIR 约定

`coefficients[0]` 对应当前输入，`coefficients[1]` 对应前一个输入，以此类推。系数数组和状态数组的长度必须等于 `tap_count`，并在滤波器生命周期内保持有效。

## Biquad 约定

内置系数生成支持低通、高通、带通和陷波：

```text
0 < center_frequency_hz < sample_frequency_hz / 2
quality_factor > 0
```

二阶 Butterworth 低通和高通常用 `quality_factor = 0.70710678F`。

`alg_filter_biquad_set_coefficients()` 接收已经归一化的系数，分母首项固定为 `a0 = 1`。该接口允许在不提供三角函数或使用离线设计工具的平台上直接加载预计算系数。

## 并发约束

滤波对象不是线程安全对象。同一实例如果会同时在中断和任务中访问，调用层必须使用临界区、消息队列或单一所有者策略进行同步。不同实例之间完全独立。

## 验证建议

集成到目标工程时至少验证：

- 所有公开滤波器的初始化和更新。
- 低通和高通的直流响应。
- 滑动窗口覆盖行为。
- 中值滤波的奇数和偶数样本。
- FIR 系数顺序。
- 四种 Biquad 类型。
- 互补滤波输出。
- 空指针、未初始化对象和非法参数。
