# bsp_exti

EXTI 使用全局 platform dispatcher。每个 `bsp_exti_t` 仅保存硬件句柄、业务回调、用户上下文
和初始化状态，不保存驱动虚表。

平台操作表提供初始化、反初始化、启用和禁用中断。ISR 根据引脚找到轻量 EXTI 句柄后调用
`bsp_exti_notify()`，该函数直接执行已注册回调。

```text
HAL GPIO EXTI callback
  -> bsp_exti_notify(exti)
  -> callback(exti, user_context)
```

回调运行在中断上下文，不能阻塞。`bsp_exti_deinit()` 会先禁用中断，再调用可选的平台
`deinit`，成功后才清空句柄状态。

