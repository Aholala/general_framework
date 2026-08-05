# general_framework 架构

## 分层结论

保留四个顶层即可，不建议再增加 `Service`、`Component` 或 `Middleware`
顶层。它们与 App、Module 的职责容易重叠，最终会让同一种代码被不同开发者
放到不同目录。

```text
App（项目策略、模式与调度；当前留空）
 ├── Module（可独立注册和复用的物理设备/稳定功能）
 ├── Algorithm（纯计算、状态估计和控制）
 └── Bsp（芯片无关外设对象）
      └── 平台适配器 / HAL Port（具体项目提供，不属于通用库）
```

依赖方向固定为：

- Algorithm 不依赖其他用户层；
- BSP 不依赖 Algorithm、Module 或 App；
- Module 可以依赖 BSP 和 Algorithm；
- App 可以装配前三层；
- 任何下层都不能反向调用 App。

## 每层放什么

### Algorithm

只接收数值、状态、配置和调用者工作区。不识别 CAN、UART、电机型号或线程。
PID、LQR、滤波、KF/EKF、IMU 姿态、运动约束和底盘运动学都属于这里。

### BSP

提供厂商无关的外设接口。CAN、USART、SPI 等复杂通信外设使用基类和虚表；
GPIO、EXTI、PWM 使用每类一份全局 platform ops，并由轻量句柄区分资源。
具体 HAL 句柄保持不透明，通用 BSP 不包含 STM32 头文件，也不直接绑定
`hcan1`、`huart3` 等全局对象。

平台适配器负责把 STM32 HAL 回调路由到 `bsp_xxx_notify`。它是最终项目的
硬件胶水，不应让每个上层模块重新适配 HAL。

### Module

表示一个可注册、可多实例使用的真实设备或稳定功能单元，例如一只电机、
BMI088、DR16、板间链路或发射机构。模块保存自身状态，通过 BSP 基类访问
硬件，不查找散落的全局句柄。

### App

只在具体机器人项目中填写：遥控映射、云台/底盘模式、状态机、安全联锁、
任务周期、参数选择、模块间数据流和编译期装配选项。云台跟随、小陀螺、
无力模式等都属于 App，不应固化进通用 Module。

## 旋转中心约定

底盘坐标统一为 `+x` 向前、`+y` 向左、`+z` 向上，逆时针角速度为正。
旋转中心坐标相对底盘原点给出。指定旋转中心处的速度为零时，底盘绕该点
纯旋转。

- 舵轮、麦克纳姆轮、全向轮：支持任意二维旋转中心；
- 差速底盘：仅支持横向瞬时转心；
- Ackermann：瞬时转心由车辆曲率和转角决定。

## 对象和生命周期

- 对象由调用者分配，不使用动态内存；
- `init` 初始化全部字段，失败后保持未初始化状态；
- 多态基类首成员名为 `super`，基类虚表指针名为 `vptr`；
- 每个派生实现使用 `MODULE_STATIC_ASSERT_SUPER_FIRST`、
  `MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST` 或 `BSP_STATIC_ASSERT_SUPER_FIRST`
  在编译期验证对象布局；
- 虚表和驱动操作表为 `static const`；
- 必须操作在构造时校验，可选操作缺失时由公共接口返回 `UNSUPPORTED`；
- 注册成功后才能使用设备，注销时同时解除总线路由；
- ISR 只通知或置位，解析和控制在任务上下文执行。

## 多态的使用边界

只有在同一个公共接口确实存在两种以上可替换实现时才使用
`super + vptr + container_of`。典型场景是不同型号电机、不同外设平台实现，
以及需要通过统一基类管理的设备生命周期。

以下代码保持普通结构体和普通函数，不增加虚表：

- PID、LQR、滤波、运动学等纯算法；
- 只存在一种实现、没有替换需求的协议工具；
- 组合多个对象完成固定流程的功能组件，例如发射机构和单舵轮执行组件；
- 仅仅转发另一个函数、没有独立状态或契约的薄包装。

多态对象遵循以下固定结构：调用者静态分配对象，`init` 绑定只读虚表和外部
依赖，公共非虚接口完成校验和分发，私有虚实现通过 `container_of` 恢复派生
对象。GPIO、EXTI、PWM 是明确例外：平台操作表为初始化后不再改变的文件级
单例，运行状态仍保存在各自的轻量资源句柄中。

## 项目板级装配

代码分成三个明确边界：

```text
User/Bsp/                 厂商无关的外设对象和公共接口
User/board_config.h/.c    当前 H723 工程的引脚和 HAL 适配
Core/、USB_DEVICE/、IOC   CubeMX 生成内容，保持生成位置
```

`User/board_config.h` 描述实例容量、逻辑名称、引脚、通道和总线映射；
`User/board_config.c` 实现当前 H723 的 HAL 操作表，持有 BSP 对象存储，并将
CubeMX 句柄注入对象，同时负责 HAL 回调路由。它们是具体工程适配代码，不属于
通用 BSP。更换开发板或 MCU 时替换这两个文件，Algorithm、Module 和
`User/Bsp` 均不重写，也不移动 CubeMX 生成目录。

Classic CAN 是 F405 与 H723 的共同边界：F405 bxCAN 端口和 H723 FDCAN
Classic 端口都实现 `bsp_can_driver_ops_t`。只有明确使用 CAN FD 长帧、BRS
或协议状态时，上层才依赖可选的 `bsp_fdcan_t` 扩展接口。DWT 同样通过
`bsp_dwt_driver_ops_t` 注入寄存器操作，通用 DWT 代码不包含具体 MCU 头文件。
