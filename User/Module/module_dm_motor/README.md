# module_dm_motor

达妙电机的通用 CAN 协议类，支持 MIT、速度和位置速度三种模式，以及使能、失能、保存零位
和清故障命令。DM4310 通过派生配置复用本模块。

## 模式多态

`module_dm_motor_t` 内部保存 `module_dm_mode_ops_t`，不同控制模式以不同编码方式实现同名
目标更新：

- `MODULE_DM_MODE_MIT`；
- `MODULE_DM_MODE_VELOCITY`；
- `MODULE_DM_MODE_POSITION_VELOCITY`。

模式在构造时确定。切换模式通常还要求驱动器端参数/命令配合，因此运行中不直接修改枚举。

## 限制参数

`module_dm_limits_t` 显式定义位置、速度、转矩、Kp 和 Kd 的最小/最大值。MIT 浮点量按该
范围量化到协议字段。配置必须与具体固件协议一致，不能把不同型号范围混用。

## 初始化和注册

```c
module_dm_motor_init(&motor, &config);
module_dm_motor_register(&motor, &registry);
```

配置包含 CAN 基类、主机标识符、反馈标识符、发送超时、控制模式和限制。逻辑名称与注册键
用于统一设备管理。

## 命令接口

- `module_dm_motor_command_mit`：立即编码并发送 MIT 命令；
- `command_velocity`：立即发送速度命令；
- `command_position_velocity`：立即发送位置速度命令；
- `set_*_target`：只更新目标，由统一 `module_motor_update` 调度发送；
- `send_state_command`：使能、失能、保存零位或清故障。

框架运行时推荐“设置目标 + 周期 update”，避免不同任务直接抢占 CAN。

## 反馈与故障

`module_dm_motor_handle_feedback` 校验标识符并解码位置、速度、转矩、MOS 温度和驱动器故障。
故障枚举覆盖过压、欠压、过流、MOS/电机过温、通信丢失和过载。

故障时电机基类进入安全状态。清除驱动器故障后仍需确认反馈在线、清除软件锁存并显式重新
使能。

## 零位安全

`SAVE_ZERO` 会修改驱动器持久状态，只能在机械位置确认、输出关闭和人工授权后调用。不得
把它放进普通启动流程或自动重试。

## 建议验证

- 三种模式的协议字节；
- 最小、中心、最大值量化；
- 超范围拒绝/限幅策略；
- 状态命令固定帧；
- 所有故障码和温度；
- 主机/反馈 ID 错误；
- 离线、恢复和显式使能；
- 两个 DM 实例共享 CAN。
