# module_swerve

单个舵轮执行模块，把 `alg_swerve_module_target_t` 转换成驱动电机速度目标和舵向电机位置
目标。底盘几何解算留在 `alg_swerve`，本模块只处理一个物理舵轮。

## 配置

- 驱动电机基类；
- 舵向电机基类；
- 轮半径；
- 驱动减速比；
- 舵向零位偏移；
- 驱动和舵向方向符号。

两个电机必须完成注册并具有有效反馈。

## 生命周期

```c
module_swerve_init(&wheel_module, &config);
module_swerve_enable(&wheel_module);
module_swerve_apply_target(
    &wheel_module, &target, delta_time_s);
```

启用会按顺序使能两台电机；任何一步失败应保持安全状态。禁用将两个执行器目标归零并关闭。

## 目标转换

驱动轮目标线速度按轮半径和减速比转换为电机角速度。舵向目标角度叠加机械零位，并应用方向
符号。

调用 `apply_target` 前，App/底盘控制器通常先用 `alg_swerve_optimize_target` 根据当前舵角
选择最短转向和轮速反向。

## 舵角反馈

`module_swerve_get_steering_angle` 从舵向电机连续位置中扣除零位并转换方向，返回物理舵角。
编码器跨圈处理由具体电机 Module 完成。

## 故障

任一电机未注册、离线、故障或未使能时返回 NOT_READY/MOTOR_ERROR。上层将该舵轮标记
不可用并交给 `alg_swerve` 降级，而不是继续发送单边输出。

## 边界

本模块不拥有电机，不计算整车运动学，不做零位标定，也不自动清故障。对象由一个控制任务
周期调用。

## 建议验证

- 轮速单位、减速比和方向；
- 舵向零位和反向安装；
- 两电机启停失败回滚；
- 舵角跨圈；
- 驱动或舵向离线；
- 优化后的反轮速目标；
- 四个独立舵轮实例。
