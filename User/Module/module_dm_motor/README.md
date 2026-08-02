# 达妙电机驱动模块 (module_dm_motor) —— 完整使用指南

## 1. 模块概述

`module_dm_motor` 是达妙（DM）电机的通用 CAN 协议驱动类，支持 MIT、位置速度、速度和力位混合四种控制模式，以及使能、失能、设置零位和清故障命令。DM4310 等型号通过派生配置复用本模块。

本目录同时保存 DM4310 型号封装：

| 文件 | 职责 | 详细文档 |
|---|---|---|
| `module_dm_motor.c/.h` | 达妙通用协议和电机对象 | 本文档 |
| `module_dm_motor_bus.c/.h` | 总线路由与反馈分发 | 本文档 |
| `module_dm4310.c/.h` | DM-J4310-2EC 型号限制和默认配置 | [README_DM4310.md](README_DM4310.md) |

## 2. 设计边界

| **模块负责**                               | **模块不负责**                         |
| :----------------------------------------- | :------------------------------------- |
| CAN 命令编码（四种模式）                   | CAN 硬件初始化和过滤器配置             |
| 反馈帧解码（位置、速度、扭矩、温度、故障） | 电机参数的具体语义（由派生型号补充）   |
| 浮点量量化到协议字段                       | 电机注册表管理（由 module_motor 提供） |
| 状态命令发送（使能/禁用/设置零位/清故障）  | 具体型号的 limits 参数配置             |

## 3. 对象关系

```text
module_device_t
└── module_motor_t
    └── module_dm_motor_t
        ├── module_dm4310_t (派生)
        └── ...
```

## 4. 核心概念

### 4.1 控制模式

| 模式     | CAN ID 偏移 | 命令格式                                     |
| :------- | :---------- | :------------------------------------------- |
| MIT      | +0x000      | 紧凑位域（位置16位 + 速度/Kp/Kd/扭矩各12位） |
| 速度     | +0x200      | 4字节 float 速度（小端，DLC=4）              |
| 位置速度 | +0x100      | 前4字节 float 位置 + 后4字节 float 速度      |
| 力位混合 | +0x300      | float 位置 + uint16 速度限制 + uint16 电流限制 |

### 4.2 限制参数（limits）

所有浮点值按 `limits` 范围量化到协议字段，必须与固件协议一致。

```c
typedef struct {
    float position_min_rad;
    float position_max_rad;
    float velocity_min_rad_per_s;
    float velocity_max_rad_per_s;
    float torque_min_nm;
    float torque_max_nm;
    float proportional_gain_min;
    float proportional_gain_max;
    float derivative_gain_min;
    float derivative_gain_max;
} module_dm_limits_t;
```

## 5. 使用示例

### 5.1 初始化单个电机

```c
static module_dm_motor_t s_motor;

module_dm_limits_t limits = {
    .position_min_rad = -12.56F,
    .position_max_rad = 12.56F,
    .velocity_min_rad_per_s = -30.0F,
    .velocity_max_rad_per_s = 30.0F,
    .torque_min_nm = -10.0F,
    .torque_max_nm = 10.0F,
    .proportional_gain_min = 0.0F,
    .proportional_gain_max = 500.0F,
    .derivative_gain_min = 0.0F,
    .derivative_gain_max = 5.0F,
};

module_dm_motor_config_t cfg = {
    .motor_name = "dm_motor_1",
    .registration_key = 1,
    .can = can_ptr,
    .control_mode = MODULE_DM_MODE_MIT,
    .master_identifier = 0x01,
    .feedback_identifier = 0x00,
    .transmit_timeout_ms = 10,
    .limits = limits,
};

module_dm_motor_init(&s_motor, &cfg);
module_dm_motor_register(&s_motor, &motor_registry);
```

### 5.2 初始化总线（多电机）

```c
static module_dm_motor_bus_t s_bus;
static module_dm_motor_t *s_motor_storage[8];

module_dm_motor_bus_init(&s_bus, can_ptr, s_motor_storage, 8, 4);
module_dm_motor_bus_register(&s_bus, &s_motor);
```

### 5.3 控制电机

```c
// 方式1：使用统一 update 调度
module_motor_enable(module_dm_motor_as_base(&s_motor));

module_dm_mit_command_t cmd = {
    .position_rad = 1.57F,
    .velocity_rad_per_s = 0.0F,
    .proportional_gain = 100.0F,
    .derivative_gain = 1.0F,
    .torque_nm = 0.0F,
};
module_dm_motor_set_mit_target(&s_motor, &cmd);

// 周期更新（10ms）
void control_loop(void) {
    module_dm_motor_bus_update(&s_bus, 0.01F);
}

// 方式2：立即发送
module_dm_motor_command_mit(&s_motor, &cmd);
```

### 5.4 处理反馈（在 CAN 回调中）

```c
void can_rx_callback(const bsp_can_frame_t *frame) {
    module_dm_motor_bus_handle_feedback(&s_bus, frame);
}

// 或直接处理
module_dm_motor_handle_feedback(&s_motor, frame);
```

多个电机可以共用同一个反馈 `MST_ID`。总线使用“反馈 CAN ID + D0 低 4 位电机 ID”
联合路由，并拒绝会产生相同联合键的配置。

### 5.5 状态命令

```c
// 使能
module_dm_motor_send_state_command(&s_motor, MODULE_DM_COMMAND_ENABLE);

// 禁用
module_dm_motor_send_state_command(&s_motor, MODULE_DM_COMMAND_DISABLE);

// 清除故障
module_dm_motor_send_state_command(&s_motor, MODULE_DM_COMMAND_CLEAR_FAULT);

// 将当前输出轴位置设为零位
module_dm_motor_send_state_command(&s_motor, MODULE_DM_COMMAND_SET_ZERO);
```

### 5.6 参数寄存器和通信丢失保护

参数服务使用标准帧 `0x7FF`：读操作码 `0x33`、写操作码 `0x55`、保存操作码
`0xAA`。响应从配置的 `MST_ID` 返回，CAN 接收帧仍交给
`module_dm_motor_bus_handle_feedback()` 或 `module_dm_motor_handle_feedback()`。

```c
/* 已 init + register，且当前保持 DISABLED。 */
module_dm_motor_set_communication_timeout(&s_motor, 0U); /* 0 = 关闭通信丢失保护 */
module_dm_motor_save_parameters(&s_motor);               /* 只在参数变化时保存 */

/* 保存后留出手册要求的最长 30 ms，再主动回读。 */
module_dm_motor_read_parameter(&s_motor, MODULE_DM_REGISTER_COMMUNICATION_TIMEOUT);
const module_dm_parameter_response_t *response =
    module_dm_motor_get_parameter_response(&s_motor);
```

`TIMEOUT` 单位为 `50 us/count`。写参数立即生效但默认不掉电保存；保存命令仅在
Disabled 状态有效，并会一次保存全部参数。电机 Flash 约 10000 次擦写寿命，严禁把
`module_dm_motor_save_parameters()` 放进控制循环或每次启动流程。

## 6. 故障处理

- 故障码通过 `module_dm_motor_get_fault()` 获取。
- 故障时电机基类进入 `MODULE_MOTOR_STATE_FAULT` 状态。
- 清除驱动器故障后需：
  1. 确认反馈在线
  2. 清除软件故障锁存（`module_motor_clear_fault`）
  3. 显式重新使能

## 7. 总线轮询机制

`module_dm_motor_bus_update` 每周期最多发送 `maximum_transmits_per_cycle` 帧，避免 CAN 邮箱堵塞。

```c
// 控制频率 1kHz，每周期发送 4 帧 → 4kbps 总线负载
module_dm_motor_bus_init(&bus, can, storage, 8, 4);
```

## 8. 错误码速查

| 状态码                 | 触发场景                          |
| :--------------------- | :-------------------------------- |
| `INVALID_ARGUMENT`     | 参数为空、limits 非法、标识符越界 |
| `NOT_REGISTERED`       | 未注册到注册表                    |
| `DUPLICATE_KEY`        | 反馈 ID 或发送 ID 重复            |
| `OUT_OF_RANGE`         | 目标值超出 limits 范围            |
| `UNSUPPORTED`          | 设置目标时控制模式不匹配          |
| `TRANSPORT_ERROR`      | CAN 发送失败                      |
| `FEEDBACK_UNAVAILABLE` | 反馈帧未匹配到任何电机            |

## 9. 建议验证测试项

- [ ] 四种模式的协议字节编码正确
- [ ] 速度模式 DLC=4，其余控制模式 DLC=8
- [ ] 状态命令使用当前控制模式对应的 CAN ID 偏移
- [ ] 最小、中心、最大值量化
- [ ] 超范围拒绝/限幅策略
- [ ] 状态命令固定帧（0xFF 前缀）
- [ ] 所有故障码和温度解码
- [ ] 主机/反馈 ID 错误处理
- [ ] 离线、恢复和显式使能
- [ ] 两个 DM 实例共享 CAN
- [ ] 总线轮询发送预算限制

---

**总结**：`module_dm_motor` 提供了完整的达妙电机 CAN 协议驱动，通过 `mode_vptr` 多态实现四种控制模式，浮点量按 `limits` 范围量化到协议字段。`module_dm_motor_bus` 提供多电机管理、反馈路由和轮询发送预算，适用于多电机系统的统一调度。

## 一页式总线接入顺序与可读信息

```c
/* 1. 初始化 CAN BSP、电机注册表；为 DM 总线准备电机指针存储。 */
module_dm_motor_bus_init(&dm_bus, can, dm_motor_storage,
                         dm_motor_capacity, maximum_transmits_per_cycle);

/* 2. 使用 module_dm4310_init 或通用 init，协议 limits 必须来自实际固件配置。 */
module_dm4310_init(&motor, &motor_config);
module_dm4310_register(&motor, &registry);
module_dm_motor_bus_register(&dm_bus, module_dm4310_as_dm_motor(&motor));

/* 3. CAN 接收路由统一交给总线解析反馈。 */
module_dm_motor_bus_handle_feedback(&dm_bus, frame);

/* 4. 确认反馈和安全条件后 enable，并只调用当前 control_mode 对应的命令接口。 */
module_dm4310_enable(&motor);
module_dm4310_command_velocity(&motor, target_velocity_rad_per_s);

/* 5. 周期调用 bus_update；它按发送预算轮询电机。 */
module_dm_motor_bus_update(&dm_bus, delta_time_s);

/* 6. 停机 disable；从总线和注册表分别 unregister。 */
```

| 可读取信息 | API/结构体 | 说明 |
| --- | --- | --- |
| 通用反馈 | `module_motor_feedback_t` / `get_feedback()` | 位置、速度、扭矩、电流、温度、在线状态 |
| 驱动故障 | `module_dm_motor_get_fault()` | 达妙协议故障枚举 |
| MOS 温度 | `module_dm_motor_get_mos_temperature_c()` | 驱动器功率器件温度 |
| 协议范围 | `module_dm_limits_t` | 位置、速度、力矩、Kp、Kd 的量化边界 |
| 命令结构 | `module_dm_mit_command_t`、`module_dm_force_position_command_t` | MIT 与力位混合模式的完整输入 |
| 参数响应 | `module_dm_parameter_response_t` | 最近读/写/保存响应、寄存器地址、raw/float 值 |
| 通信超时 | `requested_communication_timeout_counts` / `confirmed_communication_timeout_counts` | 请求值、响应确认值和确认标志 |
| 总线状态 | `module_dm_motor_bus_t`，仅调试读取 | 电机数量、轮询位置和每周期发送预算 |

`control_mode` 与命令 API 不匹配时应视为调用错误；不要在运行中通过直接修改对象字段切换模式。
