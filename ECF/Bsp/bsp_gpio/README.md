# bsp_gpio

GPIO 使用全局 platform dispatcher，不使用继承、虚表或 `container_of`。

每个 `bsp_gpio_t` 只保存不透明硬件句柄和初始化标志；同一固件中的所有 GPIO 共享一份
`bsp_gpio_driver_ops_t`。首次 `bsp_gpio_init()` 会绑定平台操作表，后续实例必须使用同一张表。

## 调用路径

```text
bsp_gpio_write(gpio, level)
  -> 判空 + is_initialized 检查
  -> platform_ops->write(gpio->device_handle, level)
  -> HAL/寄存器
```

一次函数指针跳转，不经过虚表。

## 关键结构体

| 结构体 | 字段 | 说明 |
|--------|------|------|
| `bsp_gpio_t` | `device_handle`, `is_initialized` | 轻量句柄，由调用者静态分配 |
| `bsp_gpio_config_t` | `device_handle`, `driver_ops` | 初始化时注入平台函数指针 |
| `bsp_gpio_driver_ops_t` | `init`, `deinit`, `read`, `write`, `toggle` | read/write/toggle 必须，init/deinit 可选 |

## 用法

```c
// 1. 启动时绑定平台（board_config.c 在 init 中调用一次）
bsp_gpio_bind_platform(&stm32_gpio_ops);

// 2. 初始化实例
bsp_gpio_t ce_pin;
bsp_gpio_config_t cfg = { .device_handle = &pin_ctx, .driver_ops = &stm32_gpio_ops };
bsp_gpio_init(&ce_pin, &cfg);

// 3. 读写
bsp_gpio_write(&ce_pin, true);   // 拉高
bsp_gpio_write(&ce_pin, false);  // 拉低
bool level;
bsp_gpio_read(&ce_pin, &level);

// 4. 反初始化
bsp_gpio_deinit(&ce_pin);
```

## 平台移植

换 MCU 时只需提供新的 `bsp_gpio_driver_ops_t`：

```c
static bsp_status_t my_gpio_write(void *handle, bool level) {
    my_pin_t *pin = (my_pin_t *)handle;
    MY_HAL_WritePin(pin->port, pin->num, level);
    return BSP_STATUS_OK;
}
```

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_gpio_bind_platform(ops)` | 注册平台 ops（仅一次） |
| `bsp_gpio_init(me, cfg)` | 初始化实例 |
| `bsp_gpio_deinit(me)` | 反初始化 |
| `bsp_gpio_read(me, level)` | 读电平 |
| `bsp_gpio_write(me, level)` | 写电平 |
| `bsp_gpio_toggle(me)` | 翻转 |
| `bsp_gpio_is_initialized(me)` | 检查是否已初始化 |
