# general_framework

面向 RoboMaster 和通用运动控制项目的 C11 静态模板库。框架强调可移植、多实例、显式依赖、
调用者持有内存和 C 语言面向对象，不在通用层绑定 STM32H723 的引脚、CAN 实例或 CubeMX
全局句柄。

## 四层结构

```text
User/
├── Algorithm/  纯数学、估计、控制和运动学
├── App/        项目模式、安全策略和模块编排
├── Bsp/        厂商无关的外设对象接口
└── Module/     电机、传感器、通信和功能设备
```

依赖方向：

```text
App
├── Module ──> BSP
└── Algorithm

Module ──> Algorithm（仅确有算法需求时）
BSP ──> platform driver_ops
```

Algorithm、BSP 和 Module 不得反向依赖 App。通用 BSP 不包含厂商 HAL；具体芯片访问通过
`device_handle + static const driver_ops` 注入。

## C 面向对象

- 基类对象首成员为 `super`；
- 基类持有只读虚函数表 `vptr`；
- 派生操作表通过首成员 `super` 继承；
- 公共非虚函数先校验，再进行虚调用；
- 派生实现使用 `container_of` 恢复完整对象；
- 多个实例共享操作表，运行状态保存在对象内；
- 初始化后对象只通过指针使用，不按值复制；
- 不使用动态内存。

## 现有能力

算法层包含数学、滤波、KF、六轴 IMU 四元数 EKF、PID 家族、LQR/LQI、轨迹规划、单轴
控制器以及差速、麦轮、全向轮、舵轮和 Ackermann 运动学。

BSP 层包含 GPIO、EXTI、USART、SPI、I2C、Classic CAN、CAN FD、Timer、PWM、
Encoder、ADC、DAC、USB VCP、Watchdog 和单调 Timebase。

Module 层包含 M2006、M3508、GM6020、DM 电机/DM4310、BMI088、DR16、舵轮、板间
通信、视觉、裁判系统、发射机构、舵机、蜂鸣器、蓝牙、OLED、WS2812 和 nRF24L01。

## 目录组织

一个能力域对应一个目录，辅助职责作为目录内独立文件。例如 CAN 分发器放在 `bsp_can/`，
FDCAN Classic 适配器放在 `bsp_fdcan/`。这样保留文件单一职责，同时避免顶层目录被
`dispatcher`、`adapter` 等实现角色打碎。

每个组件目录平铺 `.h`、`.c` 和 `README.md`，不再分 `Inc/Src`。

## 构建

工程只维护根目录一个 `CMakeLists.txt`。其中按 STM32、Algorithm、BSP、Module 四个区域
集中声明源文件、包含目录和依赖，子目录不保存局部构建文件。

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

构建系统将目标文件、依赖文件和 CMake 缓存放入被忽略的 `.build/` 目录。
Algorithm、BSP、Module、FreeRTOS 和 USB 使用对象目标参与最终链接，不会在工程
根目录生成独立的 `.a` 静态库。每次构建对外只输出：

```text
firmware/general_framework.elf
```

Debug 与 Release 使用同一个交付路径，后执行的构建覆盖前一个固件。ELF 同时包含
可烧录映像和调试符号信息，STM32CubeProgrammer、OpenOCD 和调试器均可直接使用。
修改目录或增加源文件时，只更新根构建文件对应区域。

## 移植步骤

1. 保留 Algorithm 和 Module 源码；
2. 为目标平台实现所需 BSP `driver_ops`；
3. 在板级装配中绑定句柄、引脚、DMA、IRQ 和回调；
4. 在 `board_config.h` 定义逻辑设备和容量；
5. 静态创建对象、缓冲区和注册表；
6. 按 BSP → Module → App 顺序初始化；
7. 设置安全初态后再启动中断和执行器；
8. 完成目标平台的严格告警构建与实机验证。

## 文档入口

- [架构与依赖规则](ARCHITECTURE.md)
- [RoboMaster 能力路线图](ROBOMASTER_ROADMAP.md)
- [Algorithm 层](User/Algorithm/README.md)
- [App 层](User/App/README.md)
- [BSP 层](User/Bsp/README.md)
- [STM32H723VET6 板级引脚](User/Bsp/BOARD_PINOUT.md)
- [Module 层](User/Module/README.md)

每个具体组件 README 都包含职责边界、数据模型、初始化、运行流程、内存/并发约束、移植
要求和建议验证项目。
