# bsp_pwm

PWM 使用一份全局 platform dispatcher。每个 `bsp_pwm_t` 只保存定时器不透明句柄、通道和
初始化状态，不再继承 `bsp_device_t`，也没有第二层虚表。

```text
bsp_pwm_set_pulse(pwm, ticks)
  -> 边界检查
  -> platform_ops->set_pulse(handle, channel, ticks)
  -> HAL/寄存器
```

公共接口提供启停、频率、脉宽和归一化占空比。BSP 负责参数范围检查，平台端负责 ARR/CCR、
通道映射和安全输出电平。`bsp_pwm_deinit()` 会先停止输出，再调用可选的平台反初始化。

同一固件只能绑定一套 PWM 平台实现；不同定时器和通道通过轻量句柄区分。

