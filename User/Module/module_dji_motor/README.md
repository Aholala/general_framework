# 大疆 DJI 电机驱动模块 (module_dji_motor) —— 完整使用指南

## 1. 模块概述

`module_dji_motor` 是大疆 M2006、M3508 和 GM6020 电机的 CAN 协议驱动基类，负责总线注册、反馈解码、编码器多圈累计、控制模式和分组电流帧发送。具体型号派生模块（`module_m2006`、`module_m3508`、`module_gm6020`）只补充型号参数与专用语义。

**核心功能**：

- CAN 总线管理（3 个发送组 × 4 个电机槽位）
- 13 位编码器反馈解码与多圈累计
- 三种控制模式：直通、速度、位置
- 速度 PID 和位置串级 PID 控制
- 过温保护与故障状态管理

## 2. 设计边界

| **模块负责**                               | **模块不负责**                                         |
| :----------------------------------------- | :----------------------------------------------------- |
| CAN 反馈帧解码（编码器、速度、电流、温度） | CAN 硬件初始化和过滤器配置                             |
| 编码器回绕处理和多圈累计                   | 电机参数的具体语义（由派生型号模块补充）               |
| 速度/位置 PID 控制                         | 电机注册表管理（由 module_motor 基类提供）             |
| 分组电流帧打包发送                         | 具体型号的 max_command 和 gear_ratio（由派生模块配置） |

## 3. 对象关系

```text
module_device_t
└── module_motor_t
    └── module_dji_motor_t
        ├── module_m2006_t
        ├── module_m3508_t
        └── module_gm6020_t
```

## 4. 总线管理

### 4.1 发送组

| 组索引 | CAN ID | 槽位 0                                   | 槽位 1 | 槽位 2 | 槽位 3 |
| :----- | :----- | :--------------------------------------- | :----- | :----- | :----- |
| 0      | 0x1FF  | ID 1~4（M2006/M3508）或 ID 1~4（GM6020） |        |        |        |
| 1      | 0x200  | ID 5~8（M2006/M3508）                    |        |        |        |
| 2      | 0x2FF  | ID 5~7（GM6020）                         |        |        |        |

### 4.2 协议映射

| 型号        | 标识符范围 | 接收 ID     | 发送组        |
| :---------- | :--------- | :---------- | :------------ |
| M2006/M3508 | 1~4        | 0x201~0x204 | 组 1（0x200） |
| M2006/M3508 | 5~8        | 0x205~0x208 | 组 0（0x1FF） |
| GM6020      | 1~4        | 0x205~0x208 | 组 0（0x1FF） |
| GM6020      | 5~7        | 0x209~0x20B | 组 2（0x2FF） |

## 5. 使用示例

### 5.1 初始化总线

```c
static module_dji_motor_bus_t s_can1_motor_bus;

module_dji_motor_bus_init(&s_can1_motor_bus, can1_ptr, 10);
```

### 5.2 初始化并注册电机

```c
// 配置 M3508 电机
module_dji_motor_config_t cfg = {
    .logical_name = "left_wheel",
    .registration_key = 1,
    .motor_bus = &s_can1_motor_bus,
    .motor_model = MODULE_DJI_MOTOR_M3508,
    .control_mode = MODULE_DJI_CONTROL_POSITION,
    .motor_identifier = 1,
    .direction_sign = 1.0F,
    .maximum_temperature_c = 70.0F,
    .current_scale_a_per_count = 0.001F,
    .velocity_pid_config = { ... },
    .position_pid_config = { ... },
};

module_dji_motor_t motor;
module_dji_motor_init(&motor, &cfg);

// 注册到总线槽位和注册表
module_dji_motor_register(&motor, &motor_registry);
```

### 5.3 控制电机

```c
// 获取基类指针
module_motor_t *base = module_dji_motor_as_base(&motor);

// 使能电机
module_motor_enable(base);

// 设置目标位置（弧度）
module_motor_set_target(base, 1.57F);  // 90°

// 周期更新
module_motor_update(base, 0.01F);      // 10ms

// 发送命令到总线
module_dji_motor_bus_flush(&s_can1_motor_bus);
```

### 5.4 处理反馈帧（在 CAN 回调中）

```c
void can_rx_callback(const bsp_can_frame_t *frame) {
    module_dji_motor_bus_handle_feedback(&s_can1_motor_bus, frame);
}
```

## 6. 反馈数据

| 字段                  | 单位  | 说明                   |
| :-------------------- | :---- | :--------------------- |
| `position_rad`        | rad   | 累积位置（多圈）       |
| `velocity_rad_per_s`  | rad/s | 速度                   |
| `current_a`           | A     | 电流（换算后）         |
| `motor_temperature_c` | °C    | 电机温度               |
| `raw_position`        | count | 原始编码器值（0~8191） |

## 7. 控制模式

| 模式       | 目标值含义           | 控制结构          |
| :--------- | :------------------- | :---------------- |
| `DIRECT`   | 电流命令（直接输出） | 无                |
| `VELOCITY` | 速度（rad/s）        | 速度 PID          |
| `POSITION` | 位置（rad）          | 位置/速度串级 PID |

## 8. 安全与故障

- 未收到有效反馈不能使能。
- 反馈超时会通过基类 `module_motor` 处理（锁定输出）。
- 温度超限时自动进入故障状态（`state = FAULT`），命令清零。
- 故障恢复后需清除故障锁存并显式使能。

## 9. 建议验证测试项

- [ ] 三种型号及所有合法 ID（1~8）的协议映射
- [ ] 发送组和槽位映射正确
- [ ] 编码器正反向多圈回绕
- [ ] 三种控制模式输出正确
- [ ] 四电机成组发送和未用槽位归零
- [ ] 反馈离线、过温、使能和故障恢复
- [ ] 两条 CAN 总线实例互不影响

---

**总结**：`module_dji_motor` 提供了完整的 DJI 电机 CAN 协议驱动，涵盖总线管理、反馈解码、编码器多圈累计、PID 控制和分组帧发送。其设计将与具体型号相关的参数（减速比、最大命令值）与通用逻辑分离，便于扩展新型号。配合 `module_motor` 基类和 `module_motor_registry` 注册表，可实现多电机系统的统一管理。
