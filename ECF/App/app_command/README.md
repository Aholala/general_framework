# app_command — 遥控命令映射

把 DR16/板间遥控数据映射为云台/底盘/发射机构命令。

## 用法

```c
app_command_config_t cfg = {
    .dr16 = &dr16,
    .board_comm = &link,
    .dr16_is_local = true,
};
app_command_init(&cfg);

// 在任务中周期调用
app_command_update(dt);
// 自动通过 app_exchange 发布 chassis/gimbal/shooter 命令
```

## 遥控映射

| 遥控输入 | 映射 |
|---------|------|
| `channel[0]` (右水平) | 云台偏航角速度 |
| `channel[1]` (右垂直) | 云台俯仰角速度 |
| `channel[2]` (左水平) | 底盘 Y 方向速度 |
| `channel[3]` (左垂直) | 底盘 X 方向速度 |
| `left_switch UP` | 小陀螺模式 |
| `left_switch DOWN` | 无力模式 |
| `right_switch UP` | LQR 控制 + 视觉跟随 |
| `right_switch MIDDLE` | IMU 反馈 |
| `mouse_right` | 自瞄模式（自动开火） |
| `dial` | 摩擦轮速度 + 手动开火 |
