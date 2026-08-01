# Module 层

Module 层把设备协议、数据解析和功能状态机封装成可注册、多实例对象。硬件通过 BSP 基类注入，控制算法通过 Algorithm 对象注入；Module 不访问 STM32 HAL 全局句柄，也不决定项目模式。

## 1. 概述

Module 层位于 BSP 层之上，是业务逻辑与硬件抽象之间的桥梁。它利用 BSP 提供的多态外设接口（`bsp_can_t`、`bsp_usart_t`、`bsp_spi_t`、`bsp_pwm_t` 等），将具体的设备协议、传感器数据解析、执行器状态机和通信协议封装为可复用的软件组件。

**核心职责**：

- 封装设备协议（如 DJI 电机 CAN 协议、裁判系统串口协议、DR16 遥控器协议）
- 实现功能状态机（如发射机构堵转恢复、舵轮运动控制）
- 管理设备生命周期（初始化、启动、停止、注册、注销）
- 提供统一的数据接口（反馈、状态、统计）
- 对需要统一生命周期的设备，通过 `module_device` 基类实现多态调用

- **硬件解耦**：通过 BSP 基类指针注入硬件依赖，不直接访问 HAL 或寄存器
- **静态内存**：所有对象、缓冲区由调用者静态分配，不使用 `malloc`
- **多实例**：支持同一类型的多个独立实例（如多个电机、多个传感器）
- **可测试**：通过依赖注入，可在单元测试中替换为模拟对象

## 2. 架构分层

```text
┌─────────────────────────────────────────────────────────────┐
│                      Application / App                      │
│               （控制策略、模式管理、用户交互）               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      Module Layer                          │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────┐   │
│  │  电机模块   │ │  传感器模块 │ │   功能设备模块      │   │
│  │ M3508/M2006 │ │ BMI088/DR16 │ │ 发射机构/舵轮/灯带  │   │
│  │ DM4310/GM6020│ │ 裁判系统/   │ │ 蜂鸣器/OLED/舵机   │   │
│  └─────────────┘ └─────────────┘ └─────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              通信模块 (Robot Link/Vision/NRF24L01) │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           基础框架 (module_device/motor/health)    │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                       BSP Layer                            │
│     CAN / USART / SPI / I2C / PWM / GPIO / USB VCP         │
└─────────────────────────────────────────────────────────────┘
```

## 3. 模块索引

### 3.1 基础框架

| 模块                                                   | 说明                                                      |
| :----------------------------------------------------- | :-------------------------------------------------------- |
| [`module_device`](module_device/README.md)             | 设备基类：生命周期、虚表、两阶段构造、逻辑名称和注册键    |
| [`module_motor`](module_motor/README.md)               | 电机基类：统一注册、使能/禁用、反馈超时、故障锁存和注册表 |
| [`module_motor_health`](module_motor/README_health.md) | 电机健康聚合：与电机基类共同位于 `module_motor/`          |

### 3.2 电机模块

| 模块                                                 | 说明                                                         |
| :--------------------------------------------------- | :----------------------------------------------------------- |
| [`module_dji_motor`](module_dji_motor/README.md)     | 大疆电机共享 CAN 协议：M2006/M3508/GM6020 统一基类，分组发送 |
| [`module_m2006`](module_dji_motor/README_M2006.md)   | M2006 + C610，与 DJI 公共协议位于同一目录                    |
| [`module_m3508`](module_dji_motor/README_M3508.md)   | M3508 + C620，与 DJI 公共协议位于同一目录                    |
| [`module_gm6020`](module_dji_motor/README_GM6020.md) | GM6020，与 DJI 公共协议位于同一目录                          |
| [`module_dm_motor`](module_dm_motor/README.md)       | 达妙电机通用协议：MIT/速度/位置速度三种模式，多态编码        |
| [`module_dm4310`](module_dm_motor/README_DM4310.md)  | 达妙 DM4310 型号封装，与达妙公共协议位于同一目录             |
| [`module_swerve`](module_swerve/README.md)           | 舵轮执行组件：驱动电机速度 + 舵向电机位置，运动学目标转换    |

### 3.3 传感器与输入

| 模块                                               | 说明                                                        |
| :------------------------------------------------- | :---------------------------------------------------------- |
| [`module_bmi088`](module_bmi088/README.md)         | BMI088 六轴 IMU：加速度/陀螺仪、轴映射、自检、零偏校准      |
| [`module_dr16`](module_dr16/README.md)             | DR16 遥控器：18 字节帧解析、摇杆归一化、双缓冲、在线检测    |
| [`module_referee`](module_referee/README.md)       | 裁判系统协议：帧同步、CRC8/16、命令路由、统计               |
| [`module_referee_ui`](module_referee/README_UI.md) | 裁判系统 UI，与协议解析和数据仓库共同位于 `module_referee/` |

### 3.4 通信模块

| 模块                                                 | 说明                                                             |
| :--------------------------------------------------- | :--------------------------------------------------------------- |
| [`module_board_comm`](module_board_comm/README.md)   | 云台板/底盘板 CAN 数据协议：DR16/云台/底盘/发射机构/心跳         |
| [`module_vision_comm`](module_vision_comm/README.md) | 视觉通信协议：5 字节固定帧（0xA5 0x5A + 2 数据 + CRC8），USB VCP |
| [`module_bluetooth`](module_bluetooth/README.md)     | 蓝牙串口透传：双缓冲接收、在线超时、AT 命令发送                  |
| [`module_nrf24l01`](module_nrf24l01/README.md)       | nRF24L01+ 2.4GHz 收发器：点对点协议、CRC16-CCITT                 |

### 3.5 功能设备

| 模块                                                      | 说明                                                            |
| :-------------------------------------------------------- | :-------------------------------------------------------------- |
| [`module_shooter`](module_shooter/README.md)              | 发射机构状态机：摩擦轮 + 拨弹电机、堵转检测、回退重试、故障锁存 |
| [`module_servo`](module_servo/README.md)                  | PWM 舵机：角度/脉宽映射、归一化控制                             |
| [`module_buzzer`](module_buzzer/README.md)                | 蜂鸣器：非阻塞音调序列、循环播放                                |
| [`module_oled`](module_oled/README.md)                    | 单色 OLED：I2C 帧缓冲、像素/直线/矩形/位图绘制                  |
| [`module_ws2812`](module_ws2812/README.md)                | RGB 灯带：SPI 编码、全局亮度、闪烁/流水/呼吸/彩虹）             |
| [`module_diagnostic`](module_device/README_diagnostic.md) | 通用诊断基础设施，与设备基类共同位于 `module_device/`           |

### 3.6 快速接入和数据读取索引

每个子 README 末尾都有“**一页式接入顺序与可读信息**”。第一次使用模块时按下面的顺序进入对应文档，不要只复制某个控制函数：

| 模块                                                      | 必须遵守的接入顺序                                                                             | 主要可读结构体或状态                                           |
| --------------------------------------------------------- | ---------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| [`module_device`](module_device/README.md)                | `init_base → 派生资源初始化 → complete_init → start/update/stop`                               | 初始化状态、逻辑名称、注册键                                   |
| [`module_diagnostic`](module_device/README_diagnostic.md) | 定义条目和状态存储 → init → start → 周期 update                                                | `module_diagnostic_state_t`、活动数量、最高等级                |
| [`module_motor`](module_motor/README.md)                  | 注册表初始化 → 具体电机 init/register → 反馈 → enable → set_target/update → disable/unregister | 名称、协议 ID、dt、运行时间、状态及完整反馈                    |
| [`module_motor_health`](module_motor/README_health.md)    | 电机就绪 → 阈值/状态数组配置 → init → 周期 update → 查询可用性                                 | `module_motor_health_state_t`、原因位掩码                      |
| [`module_dji_motor`](module_dji_motor/README.md)          | CAN/总线/注册表 → 配置三级 PID → 型号 init/register → 反馈 → enable/target/update → bus_flush  | 三级 PID、各级目标、通用反馈、原始命令及编码器状态             |
| [`M2006`](module_dji_motor/README_M2006.md)               | 选择控制模式/PID 形式 → init/register → 对应目标接口 → update/flush                            | 名称、ID、运行信息、输出轴反馈、三级 PID及原始命令             |
| [`M3508`](module_dji_motor/README_M3508.md)               | 选择控制模式/PID 形式 → init/register → 对应目标接口 → update/flush                            | 名称、ID、运行信息、输出轴反馈、三级 PID及原始命令             |
| [`GM6020`](module_dji_motor/README_GM6020.md)             | 选择固定控制模式 → init/register → 对应目标接口 → update/flush                                 | 多圈反馈、原始电压命令                                         |
| [`module_dm_motor`](module_dm_motor/README.md)            | CAN/DM 总线 → limits 和 ID → init/register → 反馈 → 模式命令 → bus_update                      | 通用反馈、`module_dm_fault_t`、MOS 温度                        |
| [`DM4310`](module_dm_motor/README_DM4310.md)              | 从实际电机读取协议参数 → init/register → 反馈 → enable → 匹配模式命令                          | 通用反馈、故障、MOS 温度、协议 limits                          |
| [`module_bmi088`](module_bmi088/README.md)                | SPI/片选/延时 → init/start → 可选校准 → 周期 read                                              | `module_bmi088_process_data_t`、`module_bmi088_raw_data_t`     |
| [`module_dr16`](module_dr16/README.md)                    | DBUS USART/DMA 双缓冲 → init/start → process/update_time                                       | `module_dr16_process_data_t`                                   |
| [`module_nrf24l01`](module_nrf24l01/README.md)            | 两端统一无线参数 → init/start → RX 或 TX → 周期接收/发送轮询                                   | `module_nrf24l01_packet_t`、重发/丢包计数                      |
| [`module_referee`](module_referee/README.md)              | USART/四类缓冲区/路由 → init/start → 周期 update → 消费 update_mask                            | `module_referee_process_data_t`、`module_referee_statistics_t` |
| [`module_referee_ui`](module_referee/README_UI.md)        | 裁判模块在线 → 队列配置 → init/start → enqueue → 周期 update                                   | `module_referee_ui_graphic_t`、队列和丢弃计数                  |
| [`module_board_comm`](module_board_comm/README.md)        | 两板统一 CAN ID → init/路由 → 发送/接收 → update_time                                          | 遥控、云台、底盘和发射机构数据结构体                           |
| [`module_vision_comm`](module_vision_comm/README.md)      | USB VCP → init → 接收 feed/发送 send → get_data                                                | `module_vision_comm_process_data_t`                            |
| [`module_bluetooth`](module_bluetooth/README.md)          | USART/双缓冲 → init/start → 周期 update → transmit/stop                                        | 接收回调、在线状态、接收错误计数                               |
| [`module_shooter`](module_shooter/README.md)              | 三电机就绪 → init/enable → friction → request_shots → 周期 update                              | 状态、待发数量、卡弹重试次数                                   |
| [`module_swerve`](module_swerve/README.md)                | 两电机就绪 → init/enable → 运动学 target → apply_target                                        | 舵向角和两个电机反馈                                           |
| [`module_servo`](module_servo/README.md)                  | PWM → init/start → 选择一种单位设置目标 → stop                                                 | 当前命令角度（无机械反馈）                                     |
| [`module_buzzer`](module_buzzer/README.md)                | PWM → init/start → tone/sequence → 周期 update                                                 | 播放状态和当前序列进度                                         |
| [`module_oled`](module_oled/README.md)                    | I2C/帧缓冲 → init/start → 绘图 → flush                                                         | 调用者帧缓冲和启动状态                                         |
| [`module_ws2812`](module_ws2812/README.md)                | SPI/像素和编码缓冲 → init/start → 修改像素 → show/notify                                       | 像素数组、效果状态和发送忙状态                                 |

通用读取规则：优先使用 `get_*()`、`is_*()` 或只读回调；getter 返回内部指针时只读且不要长期缓存。没有 getter 的公开对象字段只用于调试监控，App 不得借此修改模块状态机。

## 4. 对象模型

### 4.1 继承层次

```text
module_device_t                    (通用设备生命周期基类)
    ├── module_bmi088_t
    ├── module_dr16_t
    ├── module_referee_t
    ├── module_nrf24l01_t
    ├── module_oled_t
    └── 其他需要统一 start/stop/update 的设备

module_motor_t                     (独立的电机基类，不继承 module_device_t)
    ├── module_dji_motor_t
    │       ├── module_m2006_t
    │       ├── module_m3508_t
    │       └── module_gm6020_t
    └── module_dm_motor_t
            └── module_dm4310_t

普通组合对象（不使用虚表）
    ├── module_shooter_t
    └── module_swerve_t
```

### 4.2 设计规范

- 派生对象首成员统一命名为 `super`
- 使用 `MODULE_STATIC_ASSERT_SUPER_FIRST` 或
  `MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST` 在编译期验证首成员布局
- 基类保存只读虚表指针 `vptr`
- 操作表使用 `static const`
- 派生实现通过 `MODULE_CONTAINER_OF` 找回完整对象
- 公共非虚接口负责状态与参数检查，再执行虚调用
- 硬件访问通过 BSP 基类指针注入

只有存在多种可替换实现或确实需要统一生命周期调度时才接入基类。纯算法、
单一实现和固定组合组件使用普通结构体函数，不为形式上的“面向对象”增加虚表。

### 4.3 两阶段构造

所有继承 `module_device_t` 的模块必须使用两阶段构造：

```c
// 1. 第一阶段：初始化基类（填写基类字段，is_initialized = false）
status = module_device_init_base(&me->super, &s_ops, logical_name, registration_key);

// 2. 初始化派生类资源（如硬件配置、注册回调等）
//    若失败，调用 abort_init 回滚

// 3. 第二阶段：完成构造（设置 is_initialized = true）
status = module_device_complete_init(&me->super);
```

**约束**：禁止派生模块直接写入 `super.is_initialized`、`super.vptr` 或 `super.object_magic`。

## 5. 生命周期

### 5.1 标准流程

```text
1. 初始化并启动依赖的 BSP 外设
   ↓
2. 静态分配对象、缓冲区和只读配置
   ↓
3. 调用 module_xxx_init（两阶段构造）
   ↓
4. 对可注册设备执行注册（如电机注册到 module_motor_registry）
   ↓
5. 注册 BSP/CAN 接收路由（如 bsp_can_dispatcher）
   ↓
6. 调用 module_xxx_start 启动设备
   ↓
7. 在任务周期调用 update/process
   ↓
8. 故障时先归零和禁用输出，再停止通信
   ↓
9. 注销设备；停止异步操作后，按具体模块 API 释放注册或绑定关系
```

### 5.2 关键规则

- 构造失败必须留下确定的未初始化对象（`is_initialized = false`）
- 停止或注销后禁止继续使用失效的注册关系
- 解除回调或总线绑定前应确保所有异步操作已停止或中止
- 未注册对象不能执行控制操作

## 6. ISR 与任务边界

### 6.1 ISR 职责

- 只复制最小数据到处理缓冲区
- 记录数据长度和标志
- 立即重启 DMA/中断接收
- 设置 `receive_pending` 标志
- **禁止**解析协议、执行用户回调或运行控制算法

### 6.2 任务上下文职责

- 协议解析和帧路由
- 用户回调执行
- 状态机推进和控制器更新
- 在线超时计时
- 统计信息更新

### 6.3 缓冲区策略

高吞吐模块使用调用者持有的双缓冲或流缓冲：

```text
receive_buffer (DMA)  →  pending_buffer (ISR拷贝)  →  任务解析
```

- 接收覆盖和重启错误均有统计计数
- 缓冲区由调用者静态分配，Module 不持有所有权

## 7. 安全规则

| 规则           | 说明                                             |
| :------------- | :----------------------------------------------- |
| **反馈检查**   | 执行器没有有效反馈不能使能                       |
| **在线超时**   | 触发安全输出和故障锁存，输出归零                 |
| **故障恢复**   | 要求显式清除故障并重新使能，不允许自动恢复       |
| **参数校验**   | 配置和输入做范围、单位和有限数检查（`isfinite`） |
| **危险命令**   | 保存零点等修改持久存储的命令不能放入自动重试     |
| **优先级隔离** | 非关键设备（显示/灯光/无线）错误不得阻塞控制周期 |

## 8. 内存与所有权

- Module 不分配内存（不使用 `malloc`/`free`）
- 对象、注册表、DMA 缓冲、路由表、像素、帧缓冲和用户上下文均由调用者提供
- 被对象长期保存的指针必须覆盖对象生命周期
- 异步发送缓冲不得提前复用（发送完成前保持有效）
- 大块数据不进行隐藏复制，异步接口必须在各自 README 中说明缓冲区所有权

## 9. 新增 Module 的完成标准

新增一个 Module 模块时，必须满足以下标准：

| 要求            | 说明                                                              |
| :-------------- | :---------------------------------------------------------------- |
| **BSP 注入**    | 通过 BSP 基类指针注入硬件依赖，不直接访问 HAL                     |
| **多实例**      | 支持至少两个独立实例同时工作                                      |
| **基类接入**    | 接入 `module_device` 或适当功能基类（如 `module_motor`）          |
| **命名规范**    | 使用 `me`、`super`、`vptr` 和 `static const ops`                  |
| **两阶段构造**  | 完整实现 `init_base` → 派生初始化 → `complete_init`，失败路径回滚 |
| **错误路径**    | 提供完整的构造、运行、停止和错误处理路径                          |
| **文档**        | README 包含边界、接口、初始化、流程、安全、移植与验证             |
| **构建**        | 加入根 `CMakeLists.txt` 并通过 Debug、Release 和严格告警构建      |
| **无 HAL 依赖** | 不包含任何厂商 HAL 头文件                                         |
