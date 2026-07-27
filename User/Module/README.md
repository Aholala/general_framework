# Module 层

Module 把设备协议、数据解析和功能状态机封装成可注册、多实例对象。硬件通过 BSP 基类注入，
控制算法通过 Algorithm 对象注入；Module 不访问 STM32 HAL 全局句柄，也不决定项目模式。

## 基类与健康管理

- [module_device](module_device/README.md)：设备生命周期、虚表和注册表；
- [module_motor](module_motor/README.md)：电机多态基类、反馈、故障锁存和注册表；
- [module_motor_health](module_motor_health/README.md)：电机组健康与可用性。

## 电机

- [module_dji_motor](module_dji_motor/README.md)：DJI 共享 CAN 协议和分组发送；
- [module_m2006](module_m2006/README.md)：M2006 派生配置；
- [module_m3508](module_m3508/README.md)：M3508 派生配置；
- [module_gm6020](module_gm6020/README.md)：GM6020 派生配置；
- [module_dm_motor](module_dm_motor/README.md)：DM 多模式协议；
- [module_dm4310](module_dm4310/README.md)：DM4310 型号封装；
- [module_swerve](module_swerve/README.md)：单个舵轮的驱动/舵向执行组件。

## 传感器与输入

- [module_bmi088](module_bmi088/README.md)：六轴 IMU 驱动；
- [module_dr16](module_dr16/README.md)：遥控器流解析、双缓冲和在线检测；
- [module_referee](module_referee/README.md)：裁判系统组帧、CRC、路由和统计。
- [module_referee_ui](module_referee_ui/README.md)：客户端图形队列、批量编码和发送限频；

## 通信

- [module_robot_link](module_robot_link/README.md)：云台板/底盘板 CAN 数据协议；
- [module_vision](module_vision/README.md)：USB VCP 视觉协议；
- [module_bluetooth](module_bluetooth/README.md)：串口蓝牙透明链路；
- [module_nrf24l01](module_nrf24l01/README.md)：2.4 GHz 收发器。

## 功能设备

- [module_shooter](module_shooter/README.md)：摩擦轮、拨弹和堵转回退；
- [module_servo](module_servo/README.md)：PWM 舵机；
- [module_buzzer](module_buzzer/README.md)：非阻塞音调序列；
- [module_oled](module_oled/README.md)：单色 OLED 帧缓冲图形；
- [module_ws2812](module_ws2812/README.md)：RGB 灯带和效果引擎。
- [module_diagnostic](module_diagnostic/README.md)：故障确认、恢复、严重度和锁存诊断；

## 生命周期

1. 初始化并启动依赖 BSP；
2. 静态准备对象、缓冲区和只读配置；
3. 调用 `module_xxx_init`；
4. 对可注册设备执行注册；
5. 注册 BSP/CAN 接收路由；
6. 启动 Module；
7. 在任务周期调用 `update/process`；
8. 故障时先归零和禁用，再停止通信；
9. 注销设备并虚析构。

`module_device` 使用两阶段构造，派生初始化完整后才提交基类有效状态。失败路径必须回滚，
禁止半初始化对象参与虚调用。

## ISR 与任务

ISR 只复制最小数据、记录长度/标志并立即重启接收。协议解析、用户回调、状态机和控制更新
位于任务上下文。高吞吐模块使用调用者持有的双缓冲或 stream buffer，覆盖与重启错误都有
统计。

## 安全规则

- 执行器没有有效反馈不能使能；
- 在线超时触发安全输出和故障锁存；
- 故障恢复后要求显式清除并重新使能；
- 配置和输入做范围、单位和有限数检查；
- 危险持久命令不能放入自动重试；
- 非关键显示/灯光/无线错误不得阻塞控制周期。

## 所有权

Module 不分配内存。对象、注册表、DMA 缓冲、路由、像素、帧缓冲和用户上下文均由调用者
提供。被对象长期保存的指针必须覆盖对象生命周期；异步发送缓冲不得提前复用。

## 新增 Module 完成标准

- 通过 BSP 基类注入硬件；
- 支持多个独立实例；
- 接入 `module_device` 或适当功能基类；
- 使用 `me`、`super`、`vptr` 和 `static const ops`；
- 提供完整构造、运行、停止和错误路径；
- 明确 ISR/任务边界与缓冲所有权；
- README 包含初始化、流程、安全、移植和验证；
- 加入根 `CMakeLists.txt` 并通过严格构建。
