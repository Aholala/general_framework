# module_dji_motor

大疆 M2006、M3508 和 GM6020 的共享 CAN 协议基类，负责总线注册、反馈解码、编码器多圈
累计、控制模式和分组电流帧发送。具体型号派生模块只补充型号参数与专用语义。

## 对象关系

```text
module_device_t
└── module_motor_t
    └── module_dji_motor_t
        ├── module_m2006_t
        ├── module_m3508_t
        └── module_gm6020_t
```

Module/App 通过 `module_motor_t *` 使用统一使能、目标、更新和反馈接口。

## 总线对象

每条 CAN 网络创建一个 `module_dji_motor_bus_t`：

```c
module_dji_motor_bus_init(&can1_motor_bus, can1, transmit_timeout_ms);
```

总线对象维护三个发送组、每组四个电机槽位。新电机初始化时按型号和
`motor_identifier` 占用唯一槽位；重复占用返回资源错误。

## 注册后使用

```c
module_m3508_init(&wheel_motor, &config);
module_dji_motor_register(&wheel_motor.super, &motor_registry);
module_motor_enable(module_dji_motor_as_base(&wheel_motor.super));
```

注册表用于按逻辑键管理实例。初始化、总线槽位注册和设备注册是不同概念，失败路径必须
成对注销。

## 控制模式

- `MODULE_DJI_CONTROL_DIRECT`：目标直接映射电流命令；
- `MODULE_DJI_CONTROL_VELOCITY`：内部速度 PID；
- `MODULE_DJI_CONTROL_POSITION`：位置/速度串级 PID。

周期调用统一 `module_motor_update` 计算 `command_value`，然后每条总线调用一次
`module_dji_motor_bus_flush`，把四个槽位打包成同一 8 字节发送帧。

## 反馈

CAN 分发器把帧交给 `module_dji_motor_bus_handle_feedback`。模块解码编码器、速度、电流和
温度，并处理 13 位编码器回绕形成连续位置。反馈成功会刷新基类在线计时。

## 安全

- 未收到有效反馈不能使能；
- 反馈超时会虚调用关闭输出并锁存故障；
- 温度超限由健康管理或具体型号策略处理；
- 组内未注册/禁用电机命令保持零；
- 故障恢复后先清锁存，再显式使能。

## 单位和配置

方向、减速比、电流换算、最大温度和 PID 参数均显式配置或由具体型号构造补全。外部接口
使用 `rad`、`rad/s`、A 和 °C，CAN 原始计数只保留在协议对象内部。

## 建议验证

- 三种型号及所有合法 ID；
- 发送组和槽位映射；
- 编码器正反向多圈回绕；
- 三种控制模式；
- 四电机成组发送和未用槽归零；
- 反馈离线、过温、使能和故障恢复；
- 两条 CAN 总线实例互不影响。
