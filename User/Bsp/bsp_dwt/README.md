# BSP DWT 周期计数器（bsp_dwt）

`bsp_dwt` 封装 Cortex-M7 内核唯一的 DWT `CYCCNT`。DWT 不是可枚举的板级外设，也不存在多个实例，因此本模块不使用 `bsp_device_t`、虚表、配置结构体或设备句柄。

## 使用顺序

```c
#include "bsp_dwt.h"

/* 1. 系统时钟配置完成后初始化一次。 */
if (bsp_dwt_init() != BSP_STATUS_OK)
{
    /* 当前内核或调试配置不支持周期计数器。 */
}

/* 2. 记录起点。 */
bsp_dwt_time_point_t start_time;
bsp_dwt_now(&start_time);

/* 3. 执行被测代码或启动一个非阻塞过程。 */
do_work();

/* 4. 读取耗时。32 位无符号减法可处理一次自然回绕。 */
uint32_t elapsed_cycles;
uint32_t elapsed_us;
bsp_dwt_elapsed_cycles(start_time, &elapsed_cycles);
bsp_dwt_cycles_to_us(elapsed_cycles, &elapsed_us);

/* 5. 极短硬件时序可以忙等待；任务级等待应使用 RTOS delay。 */
bsp_dwt_delay_us(10U);
```

非阻塞超时判断按以下顺序使用：

```c
bsp_dwt_time_point_t timeout_start;
bool has_elapsed;

bsp_dwt_now(&timeout_start);
do
{
    poll_device();
    bsp_dwt_has_elapsed_us(timeout_start, 1000U, &has_elapsed);
} while (!has_elapsed);
```

## 可读取信息

| 结构体/API | 信息 |
| --- | --- |
| `bsp_dwt_time_point_t.cycle_count` | 某一时刻的 32 位周期计数快照 |
| `bsp_dwt_get_cycle_count()` | 当前 `CYCCNT` 原始值 |
| `bsp_dwt_get_frequency_hz()` | 当前 `SystemCoreClock`，用于周期与时间换算 |
| `bsp_dwt_elapsed_cycles()` | 从起点到当前经过的周期数 |
| `bsp_dwt_is_initialized()` | DWT 跟踪和周期计数器是否均已使能 |

## 约束

- `bsp_dwt_reset()` 会改变全局周期计数，只应在启动阶段调用；运行期通常只记录时间点，不复位计数器。
- 单次耗时或超时区间不得超过 `UINT32_MAX / 2` 个周期，长时间等待使用 RTOS tick 或硬件定时器。
- 调试器暂停、低功耗和运行期修改系统主频都会影响测量结果。主频改变后，旧时间点不能继续用于时间换算。
- `bsp_dwt_delay_us()` 是忙等待，只适合芯片初始化和极短外设时序，不能代替任务延时。

## 建议验证

- 初始化前调用返回 `BSP_STATUS_NOT_INITIALIZED`；
- `NULL` 输出指针返回 `BSP_STATUS_INVALID_ARGUMENT`；
- 与已知硬件定时器比较周期换算误差；
- 验证 `CYCCNT` 回绕前后的差值；
- 验证超长忙等待和超时区间返回 `BSP_STATUS_OUT_OF_RANGE`。
