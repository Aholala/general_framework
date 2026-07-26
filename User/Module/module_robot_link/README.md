# module_robot_link

云台板与底盘板之间的 Classic CAN 数据协议，传输 DR16、云台、底盘、发射机构和心跳关键
数据。模块只负责协议与在线快照，不决定 DR16 或拨弹盘实际安装位置。

## 消息布局

从 `base_identifier` 开始连续分配：

- 遥控主通道；
- 遥控辅助通道；
- 遥控按键/鼠标；
- 云台主数据；
- 云台辅助状态；
- 底盘数据；
- 发射机构数据；
- 心跳。

所有帧均为 8 字节 Classic CAN，适合直接注册到 `bsp_can_dispatcher`。

## 原子分片提交

DR16 和云台数据无法装入单帧。发送同一数据组时所有分片共享序号；接收端先写入 staging
快照，只有收到相同序号的全部分片后才一次性提交到公开数据。

新序号到达会丢弃未完成旧事务，避免把不同时刻的帧拼成一组。这比逐帧直接修改公开结构
更适合控制系统。

## 初始化

```c
module_robot_link_init(&robot_link, &config);
```

配置包含 CAN 基类、起始 ID、发送超时和数据离线超时。不同机器人或总线上的 ID 区域不得
重叠。

## 发送

- `module_robot_link_send_remote`；
- `send_gimbal`；
- `send_chassis`；
- `send_shooter`；
- `send_heartbeat`。

发送函数同步生成协议帧并调用 CAN 发送。调度频率由 App 决定，关键控制数据应优先于低
价值遥测，并限制总线占用率。

## 接收

CAN 路由回调调用 `module_robot_link_handle_frame`。完整数据通过 `get_remote`、
`get_gimbal`、`get_chassis` 和 `get_shooter` 返回只读内部快照。

指针只在对象生命周期内有效，调用方不能修改或长期跨并发周期持有。

## 在线状态

`module_robot_link_update_time` 推进每个数据组独立超时。App 必须周期调用一次，且只能有
一个时间所有者。数据离线时应回退到本板本地安全状态，不能继续使用最后控制目标。

## 角色切换

DR16 和拨弹盘安装位置由 App 编译配置决定：拥有设备的板采集/控制，另一块板只消费
Robot Link 快照。Module 不使用 `#define` 隐藏硬件归属。

## 建议验证

- 每种消息的字节往返；
- 分片乱序、丢帧、重复帧和新序号抢占；
- 序号回绕；
- 各数据组独立离线；
- 基础 ID 边界和错误帧；
- CAN 发送失败；
- 两块板不同发送周期下的数据一致性。
