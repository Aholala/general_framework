# BSP DWT 周期计数器

`bsp_dwt` 保存跨 Cortex-M 平台共用的时间差、周期换算和微秒等待逻辑。
具体 DWT/CoreDebug 寄存器只允许出现在平台端：H723 端使用 Cortex-M7 实现，
F405 端使用 Cortex-M4 实现，两者注入相同的 `bsp_dwt_driver_ops_t`。

## 平台端需要实现

```c
static const bsp_dwt_driver_ops_t dwt_driver_ops = {
    .init = platform_dwt_init,
    .reset = platform_dwt_reset,
    .get_cycle_count = platform_dwt_get_cycle_count,
    .get_frequency_hz = platform_dwt_get_frequency_hz,
};
```

通用 `bsp_dwt.c` 不包含 STM32 型号头文件，也不持有全局单例。对象存储由
板级装配代码提供。

## 使用顺序

```c
static bsp_dwt_t dwt;

const bsp_dwt_config_t config = {
    .device_handle = platform_context,
    .driver_ops = &dwt_driver_ops,
};

/* 1. 系统时钟配置完成后初始化。 */
bsp_dwt_init(&dwt, &config);

/* 2. 记录起点。 */
bsp_dwt_time_point_t start_time;
bsp_dwt_now(&dwt, &start_time);

/* 3. 执行被测代码。 */
do_work();

/* 4. 读取时间差。 */
uint32_t elapsed_cycles;
uint32_t elapsed_us;
bsp_dwt_elapsed_cycles(&dwt, start_time, &elapsed_cycles);
bsp_dwt_cycles_to_us(&dwt, elapsed_cycles, &elapsed_us);

/* 5. 仅在极短硬件时序中使用忙等待。 */
bsp_dwt_delay_us(&dwt, 10U);
```

非阻塞超时：

```c
bool has_elapsed;
bsp_dwt_time_point_t timeout_start;

bsp_dwt_now(&dwt, &timeout_start);
do
{
    poll_device();
    bsp_dwt_has_elapsed_us(&dwt, timeout_start, 1000U, &has_elapsed);
} while (!has_elapsed);
```

## 可读信息

| 数据/API | 含义 |
| --- | --- |
| `bsp_dwt_t.is_initialized` | 对象是否已绑定平台驱动 |
| `bsp_dwt_time_point_t.cycle_count` | 32位周期计数快照 |
| `bsp_dwt_get_cycle_count()` | 当前原始周期计数 |
| `bsp_dwt_get_frequency_hz()` | 当前内核计数频率 |
| `bsp_dwt_elapsed_cycles()` | 处理32位自然回绕后的周期差 |

## 约束

- F405 与 H723 的 DWT 寄存器启用代码分别留在各自平台端，不能写回通用层。
- 单次区间不得超过 `UINT32_MAX / 2` 个周期。
- 调试器暂停、低功耗和运行期改变主频都会影响测量结果。
- `bsp_dwt_delay_us()` 是忙等待，不能代替 RTOS 任务延时。
