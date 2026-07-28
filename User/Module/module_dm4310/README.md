# 达妙 DM4310 电机模块

## 职责

`module_dm4310` 是 `module_dm_motor_t` 的型号专用派生类，按
《DM-J4310-2EC V1.2 User Manual V1.2》的 CAN 协议实现 MIT、位置速度、
速度和力位混合四种控制模式，同时保留达妙协议参数可配置能力。

```text
module_motor_t
    └── module_dm_motor_t
            └── module_dm4310_t
```

## 为什么不写死协议范围

DM4310 的 `PMAX`、`VMAX`、`TMAX`、反馈 ID、接收 ID 和控制模式可通过调试工具或寄存器修改。
MIT 编码和反馈解码必须使用电机当前实际参数。因此 `module_dm4310_config_t` 要求显式传入
`protocol_limits`，模块不会提供可能与固件不一致的危险默认值。

必须配置：

- 位置最小/最大值，单位 `rad`；
- 速度最小/最大值，单位 `rad/s`；
- 力矩最小/最大值，单位 `N·m`；
- Kp 最小/最大值；
- Kd 最小/最大值。

## CAN 标识符

配置中的 `base_command_identifier` 是 MIT 模式命令 ID：

| 模式 | 实际发送 ID |
|---|---:|
| MIT | `base_command_identifier` |
| 位置速度 | `base_command_identifier + 0x100` |
| 速度 | `base_command_identifier + 0x200` |
| 力位混合 | `base_command_identifier + 0x300` |

`feedback_identifier` 必须与电机参数中的主机反馈 ID（MST_ID）一致。
多个电机可以共用 MST_ID；总线模块会结合反馈 D0 低 4 位中的电机命令 ID 路由。
协议使用标准 CAN 帧；手册默认波特率为 1 Mbps。速度模式控制帧 DLC 为 4，
其他控制模式以及状态命令的 DLC 为 8。

## 初始化

```c
const module_dm4310_config_t config = {
    .logical_name = "pitch_motor",
    .registration_key = 20U,
    .can = gimbal_can,
    .control_mode = MODULE_DM4310_CONTROL_MIT,
    .base_command_identifier = 0x01U,
    .feedback_identifier = 0x11U,
    .transmit_timeout_ms = 2U,
    .protocol_limits = dm4310_protocol_limits_from_tool,
};

(void)module_dm4310_init(&pitch_motor, &config);
(void)module_dm4310_register(&pitch_motor, &motor_registry);
(void)module_dm4310_enable(&pitch_motor);
```

对象必须注册后才能发送使能、失能、设置零位和清除故障命令。

## 控制接口

### MIT 模式

```c
const module_dm_mit_command_t command = {
    .position_rad = target_position_rad,
    .velocity_rad_per_s = target_velocity_rad_per_s,
    .proportional_gain = proportional_gain,
    .derivative_gain = derivative_gain,
    .torque_nm = feedforward_torque_nm,
};

(void)module_dm4310_command_mit(&pitch_motor, &command);
```

### 速度和位置速度

```c
(void)module_dm4310_command_velocity(&pitch_motor, target_velocity_rad_per_s);
(void)module_dm4310_command_position_velocity(
    &pitch_motor, target_position_rad, target_velocity_rad_per_s);
```

### 力位混合模式

```c
const module_dm_force_position_command_t command = {
    .position_rad = target_position_rad,
    .velocity_limit_rad_per_s = 20.0F,
    .current_limit_per_unit = 0.5F,
};

(void)module_dm4310_command_force_position(&pitch_motor, &command);
```

力位混合帧使用 `0x300 + ID`：

- D0-D3：目标位置 `float`，小端序；
- D4-D5：速度限制乘以 100，`uint16_t` 小端序，范围 0~100 rad/s；
- D6-D7：相电流限制标幺值乘以 10000，`uint16_t` 小端序，范围 0~1。

调用与初始化模式不匹配的接口会返回 `MODULE_MOTOR_STATUS_UNSUPPORTED`。

## 状态和反馈

- `module_dm4310_enable` / `module_dm4310_disable`；
- `module_dm4310_set_zero_position`：仅允许在 Disabled 状态调用；
- `module_dm4310_clear_fault`；
- `module_dm4310_handle_feedback`；
- `module_dm4310_get_feedback`；
- `module_dm4310_get_fault`；
- `module_dm4310_get_mos_temperature_c`。

统一反馈包含位置、速度、力矩和电机温度；MOS 温度保存在达妙派生对象中，通过专用 getter
读取。反馈状态码 `0` 会同步为 Disabled，`1` 同步为 Enabled，`8~E` 同步为 Fault。

## 安全要求

- 首次使用必须从调试工具读取并核对协议范围和 CAN ID；
- 设置零位会改变输出轴位置参考，只能在明确的标定流程中执行；
- 使能前必须确保机械机构不会因当前位置误差突然运动；
- 检测到过压、欠压、过流、过温、通信丢失或过载后必须停止运动命令；
- App 应实现反馈超时、软限位、急停和重新使能条件。
