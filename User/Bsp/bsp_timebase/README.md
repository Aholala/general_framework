# bsp_timebase

基于自由运行周期计数器的单调时间基准，可由 Cortex DWT、通用 32 位定时器或其他硬件
实现。提供时间点、回绕安全的耗时计算、周期/微秒转换和短延时。

## 平台要求

`bsp_timebase_driver_ops_t` 提供：

- 初始化与反初始化；
- 计数器复位；
- 当前 32 位周期计数；
- 计数频率。

计数器必须单调递增并按 `uint32_t` 自然回绕。频率在对象使用期间应保持稳定；若系统动态
变频，平台端必须同步更新频率语义。

## 时间点和耗时

```c
bsp_timebase_time_point_t start_time;
bsp_timebase_now(timebase, &start_time);

uint32_t elapsed_cycles;
bsp_timebase_elapsed_cycles(timebase, start_time, &elapsed_cycles);
```

无符号减法允许正确处理一次 32 位回绕。若测量时间超过完整计数周期，将无法区分多次回绕。

## 微秒换算

- `bsp_timebase_cycles_to_us`；
- `bsp_timebase_us_to_cycles`；
- `bsp_timebase_has_elapsed_us`。

转换检查频率和整数溢出。长周期调度应使用系统 tick 或扩展 64 位时钟，本模块主要面向
驱动超时、性能测量和微秒级时序。

## 忙等待

`bsp_timebase_delay_us` 是同步忙等待，会占用 CPU。只适用于芯片上电、传感器复位或极短
硬件时序；不得在高优先级 ISR 中延迟，也不应代替 RTOS 延时。

## DWT 移植

DWT 平台端负责：

1. 检查内核是否支持周期计数器；
2. 使能跟踪和 `CYCCNT`；
3. 返回内核实际时钟频率；
4. 处理调试器或低功耗对 DWT 的影响；
5. 不把 Cortex 寄存器暴露给通用头文件。

## 所有权与并发

读取操作可在多个上下文使用，但复位计数器会破坏其他调用者的时间点。系统运行期建议只在
启动阶段复位一次。对象不使用动态内存。

## 建议验证

- 周期计数递增；
- 32 位回绕；
- 1 us、1 ms 和较大值转换；
- 换算溢出；
- `has_elapsed` 边界；
- 忙等待误差；
- 动态时钟和低功耗策略。
