# bsp_watchdog — 看门狗抽象层（完整 OOP）

独立看门狗（IWDG）。一旦启动无法停止。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `bsp_watchdog_t` | 基类：`super`(bsp_device_t) |
| `bsp_watchdog_device_t` | 派生设备 |
| `bsp_watchdog_config_t` | 配置：`device_handle`, `driver_ops` |
| `bsp_watchdog_driver_ops_t` | 平台实现：`init/refresh/get_timeout/get_reset_detected` |

## 用法

```c
// board_config_init 中启用
board_config_init_t cfg = { .initialize_watchdog = true };
board_config_init(&cfg);

bsp_watchdog_t *wdg = board_config_get_watchdog();
if (wdg != NULL) {
    // 周期性喂狗
    bsp_watchdog_refresh(wdg);

    // 检查是否由看门狗复位启动
    bool reset_flag;
    bsp_watchdog_get_reset_detected(wdg, &reset_flag);
    if (reset_flag) { /* 上次是看门狗复位 */ }
}
```

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_watchdog_init(me, cfg)` | 初始化 + 启动 IWDG |
| `bsp_watchdog_refresh(me)` | 喂狗（重载计数器） |
| `bsp_watchdog_get_timeout_ms(me, &ms)` | 读超时时间 |
| `bsp_watchdog_get_reset_detected(me, &flag)` | 检查复位来源 |
| `bsp_watchdog_as_base(me)` | 向上转型 |

## 注意

- IWDG 用 LSI（~32kHz）作时钟，超时时间不精确，只用于最后防线
- 一旦 `bsp_watchdog_init` 启动，即使在回滚中也无法停止（硬件限制）
- 调试时把 `initialize_watchdog = false`，否则断点会触发复位
