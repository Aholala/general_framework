# Algorithm 层

Algorithm 只处理数值、状态和几何关系，不访问设备、寄存器、HAL、RTOS 或 App。算法使用
调用者提供的对象和工作区，不进行动态内存分配。

## 数学、运动与轨迹

- [alg_math](alg_math/README.md)：标量、统计、向量、矩阵和四元数；
- [alg_trajectory](alg_trajectory/README.md)：梯形速度、S 曲线和在线目标切换。
- [alg_attitude](alg_attitude/README.md)：Mahony/Madgwick 六轴姿态备份与外部 yaw 修正；

## 滤波与状态估计

- [alg_filter](alg_filter/README.md)：基础、窗口、FIR、Biquad 和互补滤波；
- [alg_kalman](alg_kalman/README.md)：矩阵、标量、线性 KF 和通用 EKF；
- [alg_imu_ekf](alg_imu_ekf/README.md)：六轴 IMU 四元数姿态 EKF。

## 控制

- [alg_pid](alg_pid/README.md)：位置式、增量式、串级、模糊和增益调度 PID；
- [alg_lqr](alg_lqr/README.md)：LQR、LQI、Riccati 与常用模型；
- [alg_angle_controller](alg_angle_controller/README.md)：PID/LQR 多态单轴接口。

## 底盘运动学

- [alg_chassis](alg_chassis/README.md)：底盘运动计算和车轮状态监测；
- [alg_mecanum](alg_mecanum/README.md)：X/O 型四麦克纳姆轮；
- [alg_omni](alg_omni/README.md)：三轮、四轮和任意非对称全向轮；
- [alg_swerve](alg_swerve/README.md)：任意舵轮布局。

## 单位规范

- 长度：`m`；
- 线速度：`m/s`；
- 角度：`rad`；
- 角速度：`rad/s`；
- 时间：`s`，模块调度接口明确要求时使用 `ms`；
- 温度：`degC` 或字段后缀 `_c`；
- 所有量的单位写入变量名。

## 状态和工作区

固定规模算法把状态存入实例；变长矩阵、轮组约束和残差使用调用者工作区。对象初始化后
不得按值复制。有状态更新对象由单一周期任务拥有，跨任务访问由外部同步。

## 错误处理

算法接口区分无效参数、未初始化、范围错误、数值错误、欠约束、奇异和降级。App 必须检查
返回值，不能把 `DEGRADED` 或 `UNDERDETERMINED` 当作正常满输出。

## 新增算法完成标准

- 公开头文件只包含完整类型与接口；
- 私有助手、内部常量和内部状态尽量留在 `.c`/`_internal.h`；
- 明确坐标、符号、单位和数值前提；
- 不访问硬件或全局设备；
- 不使用动态内存；
- 对 NaN、无穷、零维度和边界输入有确定结果；
- README 记录复杂度、工作区和建议验证；
- 加入根 `CMakeLists.txt` 并通过严格构建。
