# App 层

`User/App` 是机器人项目层，负责业务控制、整车装配、板间数据交换以及 FreeRTOS 调度适配。
业务 App 不创建任务、不包含永久循环，也不直接调用 `osDelay()`。

## 目录结构

```text
User/App/
├── app_chassis/       舵轮底盘模式、坐标变换和自锁控制
├── app_command/       DR16 输入映射、模式选择和命令仲裁
├── app_gimbal/        Pitch/Yaw 位置闭环和目标源选择
├── app_imu/           BMI088 采样与姿态角发布
├── app_safety/        心跳、离线检测和安全状态管理
├── app_shooter/       摩擦轮、拨弹及堵转回退控制
├── app_exchange/      App 之间的强类型数据快照
├── app_vision/        USB 虚拟串口视觉协议
├── Task/              FreeRTOS 周期任务适配器（每个任务一个子目录）
├── app_robot.c/.h     板级角色和整车对象装配入口
├── app_config.h       编译期角色、安装位置和周期配置
└── app_types.h        App 层共享命令与反馈类型
```

目录名和业务源文件统一使用 `app_<功能>`；只有 `Task` 中的调度入口使用
`task_<功能>`。例如：

```text
task_gimbal_run() -> app_gimbal_update() -> Module / Algorithm / BSP
```

依赖方向固定为 `Task -> App -> Module/Algorithm/BSP`，App 不反向依赖 Task。

## 当前项目配置

- 主控：两块 STM32H723VET6，分别作为云台板和底盘板。
- DR16：安装在底盘板，通过 CAN2 向云台板共享。
- 拨弹盘：下供弹，安装在底盘板 CAN1。
- 云台：DM4310 Pitch 位于 CAN1，GM6020 Yaw 位于 CAN2。
- 底盘：四个 M3508 行走电机位于 CAN1，四个 GM6020 舵向电机位于 CAN3。
- 视觉：固定 12 字节帧，帧头 `0xA5 0x5A`，包含模式、Pitch、Yaw 和 CRC8；角度来自 BMI088 姿态解算，单位 rad。

## 各部分职责

| 目录 | 作用 | 不负责 |
|---|---|---|
| `app_command` | 遥控/视觉目标仲裁，发布控制命令 | UART、CAN 字节解析 |
| `app_imu` | 读取 BMI088 数据并生成姿态快照 | BMI088 寄存器访问 |
| `app_gimbal` | PID/LQR、IMU/编码器锁定、目标跟随 | 电机 CAN 协议 |
| `app_chassis` | 无力、普通、小陀螺、跟随、自锁 | 电调协议和通用运动学公式 |
| `app_shooter` | 摩擦轮开关、射击请求和堵转处理 | M3508 帧编码 |
| `app_safety` | 心跳和失联状态 | 外设底层错误处理 |
| `app_exchange` | 并发安全的命令/反馈快照 | 跨板线协议 |
| `app_vision` | 视觉帧收发、校验和超时 | USB HAL 实现 |
| `Task` | 周期、延时和 App 更新调用 | 业务控制逻辑 |

## 调度原则

CubeMX 创建的任务入口位于 `Core/Src/freertos.c`，入口只调用对应的
`task_*_run()`。周期循环集中在 `User/App/Task`，并使用 `osDelayUntil()` 避免周期漂移。
调整优先级和栈空间时修改 CubeMX/FreeRTOS 配置；调整控制策略时只修改对应 App。

## 参数边界

本仓库是考核模板，PID/LQR 参数可以使用占位值。电机 ID、方向、舵向零点、机械尺寸和
DM4310 控制范围属于整车装配参数，应集中在 `app_robot` 或项目配置中，不散落到任务入口。
