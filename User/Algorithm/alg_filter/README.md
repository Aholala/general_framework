# 通用滤波算法库 (alg_filter) —— 完整使用指南

## 1. 模块概述

`alg_filter` 是 Algorithm 层的纯 C11 滤波库，提供多种常用数字滤波器实现。所有滤波器都不包含 STM32、HAL、CMSIS 或 RTOS 头文件，不调用系统时间函数，采样周期由调用者显式传入。不使用动态内存，不使用可变全局状态，每个对象拥有独立状态，支持任意数量的静态实例。

**设计哲学**：

- **纯标准库依赖**：只依赖 C11 标准库中的 `<stdbool.h>`、`<stddef.h>` 和 `<math.h>`
- **零动态内存**：所有缓冲区由调用者提供，对象不分配内存
- **多实例支持**：每个对象独立状态，可创建任意多个实例
- **参数安全**：所有公共输入均检查空指针、范围、NaN 和无穷值

## 2. 滤波器列表

| 滤波器   | 对象类型                      | 主要用途                                 |
| :------- | :---------------------------- | :--------------------------------------- |
| 一阶低通 | `alg_filter_low_pass_t`       | 抑制高频噪声、平滑信号                   |
| 一阶高通 | `alg_filter_high_pass_t`      | 去除直流分量、缓慢漂移                   |
| 指数平均 | `alg_filter_exponential_t`    | 低成本平滑、资源受限场景                 |
| 滑动平均 | `alg_filter_moving_average_t` | 固定窗口均值、平滑周期信号               |
| 中值滤波 | `alg_filter_median_t`         | 抑制脉冲噪声和离群点                     |
| 通用 FIR | `alg_filter_fir_t`            | 任意 FIR 系数、定制频率响应              |
| Biquad   | `alg_filter_biquad_t`         | 二阶低通/高通/带通/陷波                  |
| 互补滤波 | `alg_filter_complementary_t`  | 融合测量值和变化率（IMU 姿态、GPS+速度） |

## 3. 各滤波器详解

### 3.1 一阶低通滤波器 (`alg_filter_low_pass_t`)

**公式**：`y = y + (dt / (tau + dt)) * (x - y)`，其中 `tau = 1 / (2*π*fc)`

**使用场景**：传感器噪声抑制、信号平滑

**示例**：

```c
static alg_filter_low_pass_t s_filter;

alg_filter_low_pass_init(&s_filter, 30.0F);  // 30Hz 截止频率
alg_filter_low_pass_reset(&s_filter, 0.0F);  // 可选：设置初始值

float filtered;
alg_filter_low_pass_update(&s_filter, raw_input, 0.001F, &filtered);
```

### 3.2 一阶高通滤波器 (`alg_filter_high_pass_t`)

**公式**：`y = (tau / (tau + dt)) * (y_prev + x - x_prev)`，其中 `tau = 1 / (2*π*fc)`

**使用场景**：去除直流偏移、趋势分离

**示例**：

```c
static alg_filter_high_pass_t s_filter;

alg_filter_high_pass_init(&s_filter, 0.5F);  // 0.5Hz 截止频率
alg_filter_high_pass_reset(&s_filter, initial_input);

float filtered;
alg_filter_high_pass_update(&s_filter, raw_input, 0.001F, &filtered);
```

### 3.3 指数平均滤波器 (`alg_filter_exponential_t`)

**公式**：`y = y + alpha * (x - y)`，`alpha ∈ (0, 1]`

**使用场景**：极低资源消耗的平滑、长时间常数滤波

**示例**：

```c
static alg_filter_exponential_t s_filter;

alg_filter_exponential_init(&s_filter, 0.1F);  // alpha = 0.1（强平滑）
float filtered;
alg_filter_exponential_update(&s_filter, raw_input, &filtered);
```

### 3.4 滑动平均滤波器 (`alg_filter_moving_average_t`)

**公式**：窗口内 N 个样本的算术平均

**使用场景**：周期信号平滑、消除高频噪声

**示例**：

```c
#define WINDOW_SIZE (10U)
static float s_samples[WINDOW_SIZE];
static alg_filter_moving_average_t s_filter;

alg_filter_moving_average_init(&s_filter, s_samples, WINDOW_SIZE);

float filtered;
alg_filter_moving_average_update(&s_filter, raw_input, &filtered);
```

### 3.5 中值滤波器 (`alg_filter_median_t`)

**原理**：排序窗口样本，取中位数

**使用场景**：脉冲噪声抑制、离群点剔除（如激光雷达测距异常值）

**示例**：

```c
#define WINDOW_SIZE (5U)
static float s_samples[WINDOW_SIZE];
static float s_sort_work[WINDOW_SIZE];
static alg_filter_median_t s_filter;

alg_filter_median_init(&s_filter, s_samples, s_sort_work, WINDOW_SIZE);

float filtered;
alg_filter_median_update(&s_filter, raw_input, &filtered);
```

### 3.6 FIR 滤波器 (`alg_filter_fir_t`)

**公式**：`y = Σ(h[i] * x[n-i])`，其中 h 为 FIR 系数

**使用场景**：自定义频率响应、带通/带阻设计

**示例**：

```c
#define TAP_COUNT (16U)
static const float s_coeffs[TAP_COUNT] = { ... };  // 预先设计
static float s_state[TAP_COUNT];
static alg_filter_fir_t s_filter;

alg_filter_fir_init(&s_filter, s_coeffs, s_state, TAP_COUNT);

float filtered;
alg_filter_fir_update(&s_filter, raw_input, &filtered);
```

### 3.7 Biquad 滤波器 (`alg_filter_biquad_t`)

**传递函数**：`H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)`

**内置类型**：低通、高通、带通、陷波

**示例**：

```c
static alg_filter_biquad_t s_filter;

// 50Hz 陷波滤波器，采样率 1kHz，Q=10
alg_filter_biquad_init(&s_filter, ALG_FILTER_BIQUAD_NOTCH, 1000.0F, 50.0F, 10.0F);

float filtered;
alg_filter_biquad_update(&s_filter, raw_input, &filtered);

// 也可以手动设置系数（离线设计）
alg_filter_biquad_set_coefficients(&s_filter, b0, b1, b2, a1, a2);
```

### 3.8 互补滤波器 (`alg_filter_complementary_t`)

**公式**：`output = weight * (output + rate*dt) + (1-weight) * measured`

**使用场景**：IMU 姿态融合（陀螺仪积分 + 加速度计测量）、速度积分 + GPS 位置

**示例**：

```c
static alg_filter_complementary_t s_filter;

// weight=0.98 表示 98% 信任积分，2% 信任测量
alg_filter_complementary_init(&s_filter, 0.98F, initial_value);

float filtered;
alg_filter_complementary_update(&s_filter, measured_value, rate_per_s, 0.001F, &filtered);
```

## 4. 内存管理

所有滤波器对象**不分配动态内存**，需要调用者提供存储：

| 滤波器类型          | 需要额外存储                                  |
| :------------------ | :-------------------------------------------- |
| 低通/高通/指数/互补 | 仅对象本身                                    |
| 滑动平均            | 1 个 `float` 数组（`capacity` 大小）          |
| 中值滤波            | 2 个 `float` 数组（样本 + 排序工作区）        |
| FIR                 | 1 个 `float` 系数数组 + 1 个 `float` 状态数组 |

**内存生命周期**：所有提供的缓冲区必须在滤波器对象生命周期内保持有效。

## 5. 并发约束

滤波器对象**不是线程安全**的。同一实例如果会同时在中断和任务中访问，调用层必须使用临界区、消息队列或单一所有者策略进行同步。不同实例之间完全独立，可安全并发。

## 6. 错误码速查

| 状态码             | 触发场景                               |
| :----------------- | :------------------------------------- |
| `OK`               | 操作成功                               |
| `INVALID_ARGUMENT` | 参数为空指针                           |
| `OUT_OF_RANGE`     | 参数超出有效范围（如负频率、零窗口等） |
| `NOT_INITIALIZED`  | 对象未初始化                           |
| `NUMERICAL_ERROR`  | 数值错误（溢出、非有限结果）           |

## 7. 建议验证测试项

- [ ] 所有滤波器初始化和更新正常
- [ ] 低通滤波器直流响应（DC 增益应为 1）
- [ ] 高通滤波器直流响应（DC 增益应为 0）
- [ ] 滑动平均窗口满后行为正确
- [ ] 中值滤波奇数和偶数样本
- [ ] FIR 系数顺序正确（coeff[0] 对应当前输入）
- [ ] 四种 Biquad 类型频率响应
- [ ] 互补滤波输出跟随测量值和积分值
- [ ] 空指针、未初始化对象和非法参数处理

---

**总结**：`alg_filter` 提供了完整的嵌入式数字滤波解决方案，涵盖从简单一阶 RC 到复杂 FIR/Biquad 的多种滤波器类型。所有滤波器均无动态内存依赖，适合资源受限的嵌入式系统。配合 BSP 和 Module 层的传感器数据，可构建完整的信号处理链路。
