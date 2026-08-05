# bsp_gpio

GPIO 使用全局 platform dispatcher，不使用继承、虚表或 `container_of`。

每个 `bsp_gpio_t` 只保存不透明硬件句柄和初始化标志；同一固件中的所有 GPIO 共享一份
`bsp_gpio_driver_ops_t`。首次 `bsp_gpio_init()` 会绑定平台操作表，后续实例必须使用同一张表。

运行调用路径：

```text
bsp_gpio_write(gpio, level)
  -> platform_ops->write(gpio->device_handle, level)
  -> HAL/寄存器
```

`init` 和 `deinit` 为可选平台操作；`read`、`write`、`toggle` 必须提供。Module 只持有
`bsp_gpio_t *`，不接触 HAL 类型。该单例设计不支持在一个固件中同时绑定两套 GPIO 平台实现。

