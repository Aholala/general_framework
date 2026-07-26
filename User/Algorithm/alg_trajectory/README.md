# alg_trajectory

在线一维轨迹生成器，支持梯形速度和限加加速度 S 曲线，可跟踪位置目标或速度目标，并在
运行中平滑切换目标。

## 状态与约束

`alg_trajectory_state_t` 包含位置、速度和加速度。配置包含：

- 最大速度；
- 最大加速度；
- 最大减速度；
- 最大加加速度；
- 位置和速度完成容差。

这些量没有固定物理单位，但必须一致。云台可使用 `rad`，直线机构可使用 `m`。

## 轨迹类型

- `ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL`：直接限制加减速度，响应快；
- `ALG_TRAJECTORY_PROFILE_S_CURVE`：额外限制 jerk，机械冲击和电流尖峰更小。

S 曲线需要 `maximum_jerk_per_s3 > 0`。梯形模式仍会遵守速度和加减速度约束。

## 使用流程

```c
alg_trajectory_init(&trajectory, &config,
                    ALG_TRAJECTORY_PROFILE_S_CURVE,
                    &initial_state);
alg_trajectory_set_position_target(&trajectory, target_position, 0.0F);

alg_trajectory_status_t status =
    alg_trajectory_update(&trajectory, delta_time_s, &command_state);
```

位置模式会向 `target_position` 运动，并允许指定终端速度。速度模式持续逼近目标速度，不以
到达某个位置为结束条件。

## 在线目标切换

运行中可再次调用 `set_position_target` 或 `set_velocity_target`。生成器从当前位置、速度和
加速度继续规划，不清零状态。控制模式切换或重新定位时使用 `reset` 明确同步当前状态。

## 停止距离

`alg_trajectory_calculate_stopping_distance` 给出恒定减速度近似停止距离，可用于限位预判。
S 曲线还存在 jerk 过渡距离，安全限位应额外保留裕量。

## 完成判定

位置误差和速度误差同时进入配置容差后返回 `ALG_TRAJECTORY_STATUS_FINISHED`，同时
`alg_trajectory_is_finished` 为真。上层仍应持续保持最终闭环，不能因轨迹完成而关闭电机。

## 内存与调用约束

无动态内存。每个受控自由度使用独立对象。`delta_time_s` 必须来自单调时基且为有限正数。
同一对象不得由多个任务并发更新。

## 建议验证

- 正向/反向位置阶跃；
- 非零初速度和终端速度；
- 运行中反向切换目标；
- 梯形与 S 曲线的速度、加速度和 jerk 上限；
- 速度模式切换；
- 控制周期抖动；
- 极短距离、零距离及非法参数。
