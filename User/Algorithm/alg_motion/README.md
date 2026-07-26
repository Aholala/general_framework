# alg_motion

运动控制中的基础状态处理：非对称速率限制器和周期量连续化器。模块不依赖具体执行器，
可用于遥控命令、速度目标、舵角和编码器角度。

## 速率限制器

`alg_motion_rate_limiter_t` 对目标变化率进行限制：

- `rising_rate_per_s`：目标上升时的最大变化率；
- `falling_rate_per_s`：目标下降时的最大变化率；
- `output_min`、`output_max`：最终输出边界。

```c
alg_motion_rate_limiter_init(&limiter, &config, initial_output);
alg_motion_rate_limiter_update(&limiter, target, delta_time_s, &limited_output);
```

上升和下降速率独立，适用于底盘加速/减速、摩擦轮启动和电流软启动。限幅发生在每次状态
更新后。重新切换模式时可用 `reset` 对齐当前输出，避免阶跃。

## 周期量连续化

`alg_motion_unwrapper_t` 将 `[0, period)` 或等价周期范围内的测量转换为连续量：

```c
alg_motion_unwrapper_init(&unwrapper, 2.0F * pi);
alg_motion_unwrapper_reset(&unwrapper, wrapped_angle, initial_continuous_angle);
alg_motion_unwrapper_update(&unwrapper, new_wrapped_angle, &continuous_angle);
```

它按相邻样本最短周期差累计，适用于单圈编码器和航向角。前提是相邻采样之间的真实变化
小于半个周期；高速运动或低采样率违反此前提时必须结合转速或圈数信息。

## 返回状态

区分参数错误、未初始化、范围错误和数值错误。输入目标、周期、时间步长必须为有限数，
`delta_time_s` 必须为正。

## 内存与并发

对象只保存少量状态，无动态内存。每个信号使用独立实例。更新函数会修改状态，同一实例
只能由一个任务更新，读取与复位需要外部同步。

## 建议验证

- 上升、下降、越过零点和输出限幅；
- 控制周期抖动；
- 目标瞬时反向；
- 周期边界正向和反向跨越；
- 多圈累计；
- 相邻变化恰好接近半周期；
- NaN、无穷、零周期和非正时间步长。
