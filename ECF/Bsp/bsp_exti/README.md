# bsp_exti

EXTI 使用全局 platform dispatcher。每个 `bsp_exti_t` 仅保存硬件句柄、业务回调、用户上下文
和初始化状态，不保存驱动虚表。

## 调用路径

```text
HAL_GPIO_EXTI_Callback(pin)
  -> board_config 按 pin 查找 EXTI 对象
  -> bsp_exti_notify(exti)
  -> callback(exti, user_context)   // 用户的 ISR 回调
```

回调运行在中断上下文，不能阻塞、不能调 FreeRTOS API。

## 关键结构体

| 结构体 | 字段 | 说明 |
|--------|------|------|
| `bsp_exti_t` | `device_handle`, `callback`, `user_context`, `is_initialized` | 轻量句柄 |
| `bsp_exti_config_t` | `device_handle`, `driver_ops`, `callback`, `user_context` | 初始化配置 |
| `bsp_exti_driver_ops_t` | `init`, `deinit`, `enable`, `disable` | enable/disable 必须 |
| `bsp_exti_callback_t` | `void (*)(bsp_exti_t *me, void *ctx)` | ISR 回调签名 |

## 用法

```c
// 1. 平台绑定
bsp_exti_bind_platform(&stm32_exti_ops);

// 2. 定义回调
static void on_imu_ready(bsp_exti_t *me, void *ctx) {
    imu_data_pending = true;  // 只置标志，ISR 中不可阻塞
}

// 3. 初始化
bsp_exti_t gyro_exti;
bsp_exti_config_t cfg = {
    .device_handle = &gyro_ctx,   // 平台句柄（pin + IRQn）
    .driver_ops    = &stm32_exti_ops,
    .callback      = on_imu_ready,
};
bsp_exti_init(&gyro_exti, &cfg);

// 4. 使能
bsp_exti_enable(&gyro_exti);

// 5. 运行时换回调
bsp_exti_set_callback(&gyro_exti, new_callback, NULL);

// 6. 反初始化（先 disable 再 deinit）
bsp_exti_disable(&gyro_exti);
bsp_exti_deinit(&gyro_exti);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `bsp_exti_bind_platform(ops)` | 注册平台 ops |
| `bsp_exti_init(me, cfg)` | 初始化 + 绑定回调 |
| `bsp_exti_deinit(me)` | 反初始化 |
| `bsp_exti_enable(me)` | NVIC 使能 |
| `bsp_exti_disable(me)` | NVIC 禁止 |
| `bsp_exti_set_callback(me, cb, ctx)` | 运行时换回调 |
| `bsp_exti_notify(me)` | ISR 入口（平台调用） |
| `bsp_exti_is_initialized(me)` | 状态检查 |
