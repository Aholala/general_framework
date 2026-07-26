# bsp_pwm

通用 PWM 通道抽象，支持启停、频率、脉冲宽度和归一化占空比。定时器实例、通道号和引脚
复用通过平台句柄与配置注入。

## 数据模型

平台操作使用：

- `frequency_hz`：逻辑 PWM 频率；
- `pulse_ticks`：高电平比较值；
- `period_ticks`：完整周期计数。

公共接口 `set_duty_cycle`、`get_duty_cycle` 在 `[0.0F, 1.0F]` 与 tick 之间转换。
平台端负责硬件 ARR/CCR 的边界和是否包含 `+1` 的细节。

## 初始化与输出

```c
static bsp_pwm_device_t servo_pwm;
static const bsp_pwm_config_t config = {
    .device_handle = &platform_timer,
    .driver_ops = &platform_pwm_driver_ops,
    .channel = board_servo_channel,
};

bsp_pwm_init(&servo_pwm, &config);
bsp_pwm_set_frequency(bsp_pwm_as_base(&servo_pwm), 50U);
bsp_pwm_set_duty_cycle(bsp_pwm_as_base(&servo_pwm), 0.075F);
bsp_pwm_start(bsp_pwm_as_base(&servo_pwm));
```

对舵机更推荐 Module 通过微秒脉宽映射角度，而不是在 App 中散落占空比常数。

## 动态修改

平台驱动应明确频率变化是否会重置计数器，以及更新比较值是否使用预装载。对电机和舵机，
建议在安全边界更新，避免产生短脉冲。

## 安全状态

停止 PWM 后引脚电平由平台配置决定。控制执行器时，板级实现必须定义停止后的安全电平；
通用 BSP 不假设低电平一定安全。

## 所有权与并发

多个通道可以共享同一定时器句柄，但修改公共频率可能影响同一计时器的所有通道。平台端
或板级配置必须显式记录这种资源耦合。单通道对象不使用动态内存。

## 建议验证

- 0%、50%、100% 占空比；
- 频率和周期换算；
- 脉冲超过周期；
- 动态更新无异常窄脉冲；
- 同一定时器多通道资源关系；
- 启停后的安全电平；
- 未初始化和缺失操作。
