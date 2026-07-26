# bsp_watchdog

硬件看门狗抽象，提供刷新、实际超时时间查询和看门狗复位来源检测。通用层不决定独立看门狗
或窗口看门狗，也不配置具体寄存器。

## 接口

- `bsp_watchdog_init`：绑定平台看门狗；
- `bsp_watchdog_refresh`：在允许的时间窗口内喂狗；
- `bsp_watchdog_get_timeout_ms`：取得平台实际超时；
- `bsp_watchdog_get_reset_detected`：读取上次复位是否由看门狗导致；
- `bsp_device_deinit`：在硬件允许时反初始化。

某些 MCU 看门狗启动后无法停止，平台 `deinit` 可以返回
`BSP_STATUS_UNSUPPORTED`，不能伪造已关闭。

## 正确喂狗策略

不要在定时器 ISR 中无条件喂狗，否则主控制、通信或安全任务死锁后看门狗仍不会复位。
推荐由健康监督任务检查关键心跳：

```text
control task heartbeat ─┐
communication heartbeat ├─> health supervisor ─> watchdog refresh
safety task heartbeat ──┘
```

只有所有必需任务在当前窗口内运行正常时才调用 `refresh`。

## 初始化

```c
static bsp_watchdog_device_t watchdog;
static const bsp_watchdog_config_t config = {
    .device_handle = &platform_watchdog,
    .driver_ops = &platform_watchdog_driver_ops,
};

bsp_watchdog_init(&watchdog, &config);
```

应用启动时先读取并记录复位原因，再按平台要求清除硬件复位标志。

## 超时配置

超时时间通常由低速时钟和分频决定，存在器差和温漂。`get_timeout_ms` 应返回平台计算后的
实际值。比赛项目应留出调度抖动和最坏执行时间裕量。

## 故障安全

复位前应尽可能由硬件保证电机使能和功率输出进入安全状态，不能依赖软件来得及执行退出
逻辑。复位后保持故障记录，并要求遥控和通信重新进入有效状态后再使能执行器。

## 建议验证

- 正常心跳持续刷新；
- 任一关键任务停跳后不再刷新；
- 超时复位与复位来源检测；
- 窗口过早刷新错误；
- 平台不支持反初始化；
- 低速时钟偏差；
- 上电后执行器保持安全。
