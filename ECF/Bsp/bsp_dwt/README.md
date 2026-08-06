# bsp_dwt — DWT 周期计数器

平铺结构体，通过 `driver_ops` 注入平台寄存器操作。用于微秒级延时和高精度时间差测量。

## 调用路径

```text
bsp_dwt_delay_us(dwt, 100)
  → 读 frequency → 换算 cycle_count = us * MHz
  → 循环读 CYCCNT 直到经过足够周期
```

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `bsp_dwt_t` | DWT 对象：`device_handle` + `driver_ops` + `is_initialized` |
| `bsp_dwt_time_point_t` | 时间快照：`cycle_count`（CYCCNT 读数） |
| `bsp_dwt_driver_ops_t` | 平台实现：`init/reset/get_cycle_count/get_frequency_hz` |

## 用法

```c
bsp_dwt_t *dwt = board_config_get_dwt();

// 微秒延时
bsp_dwt_delay_us(dwt, 100);  // 阻塞 100µs

// 超时检测（非阻塞）
bsp_dwt_time_point_t start;
bsp_dwt_now(dwt, &start);
// ... 做其他事 ...
bool elapsed;
bsp_dwt_has_elapsed_us(dwt, start, 5000, &elapsed);
if (elapsed) { /* 5ms 到了 */ }

// 测量代码段耗时
bsp_dwt_time_point_t t0, t1;
bsp_dwt_now(dwt, &t0);
do_work();
bsp_dwt_now(dwt, &t1);
uint32_t cycles, us;
bsp_dwt_elapsed_cycles(dwt, t0, &cycles);
bsp_dwt_cycles_to_us(dwt, cycles, &us);  // us = 耗时（微秒）
```

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_dwt_init(me, cfg)` | 初始化（使能 DWT 外设） |
| `bsp_dwt_reset(me)` | CYCCNT 清零 |
| `bsp_dwt_now(me, &tp)` | 拍时间快照 |
| `bsp_dwt_elapsed_cycles(me, start, &cycles)` | 计算经过的周期数（处理溢出） |
| `bsp_dwt_cycles_to_us(me, cycles, &us)` | 周期 → 微秒 |
| `bsp_dwt_us_to_cycles(me, us, &cycles)` | 微秒 → 周期 |
| `bsp_dwt_delay_us(me, us)` | 阻塞微秒延时 |
| `bsp_dwt_has_elapsed_us(me, start, us, &elapsed)` | 非阻塞超时检查 |
| `bsp_dwt_get_frequency_hz(me, &hz)` | 读计数器频率（= CPU 主频） |
| `bsp_dwt_is_initialized(me)` | 状态检查 |
