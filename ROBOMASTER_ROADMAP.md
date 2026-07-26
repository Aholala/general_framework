# RoboMaster 电控库补充路线

现有库已经覆盖控制/滤波/姿态、常见底盘运动学、CAN 电机、BMI088、DR16、
板间通信、视觉和发射机构。下面按实战优先级补充，不建议一次创建只有空接口
的目录。

## 已完成的第一批通用能力

1. `module_referee`：裁判系统流解析、CRC8/CRC16、命令路由、在线检测和版本
   隔离核心；
2. `bsp_gpio`、`bsp_watchdog`；
3. `alg_trajectory`、`module_buzzer`、`module_bluetooth`、`module_oled`、
   `module_ws2812`、`module_nrf24l01`、`module_servo`。

## 下一优先级

1. `module_power_manager`：裁判功率数据、母线电压/电流、缓冲能量和超级电容
   状态的统一输入；策略计算放 Algorithm，模式选择放 App。
2. `alg_power_limit`：底盘总功率估计、轮电机功率分配、饱和回分配与掉线轮
   剔除。
3. `module_parameter_store`：参数版本、CRC、双备份和断电安全提交；底层依赖
   通用非易失存储 BSP。

## 第二优先级

1. `module_supercapacitor`：不同厂商电容协议派生类，统一电压、电流、能量、
   故障和使能接口。
2. `module_referee_ui`：图形对象缓存、增删改批处理和发送限频。
3. `alg_trajectory`：梯形/S 曲线、位置速度加速度约束、在线重规划。
4. `alg_chassis_estimator`：轮速、IMU yaw/角速度和可选外部定位的平面状态融合。
5. `module_servo`、`module_indicator`：使用 BSP 基类注入，
   不直接操作 HAL。
6. `bsp_flash`、`bsp_qspi`/`bsp_ospi`、`bsp_sdmmc`：按具体机器人是否需要
   日志、参数和离线数据选择实现。

## 第三优先级

- `module_armor_hit`、`module_heat_manager`、`module_ammunition_manager`；
- `alg_ballistic`：弹道补偿和飞行时间，但目标选择仍属于 App；
- `module_diagnostic`：设备健康快照、故障码和分层日志；
- `bsp_rtc`、`bsp_crc`、`bsp_rng`、`bsp_eth`：仅在项目确有需求时加入。

DMA 不应单独包装成供 Module 使用的“外设类”。它是 USART/SPI/ADC 等传输
实现的一部分，由平台适配器和对应 BSP 的传输模式管理。
