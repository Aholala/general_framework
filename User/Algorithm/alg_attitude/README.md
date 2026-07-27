# alg_attitude

面向六轴 IMU 的轻量姿态备份算法，可运行 Mahony 或 Madgwick。它用于 EKF 尚未完成初始化、算力降级或赛场故障恢复，不替代 `alg_imu_ekf` 的协方差估计。

加速度模长超出配置窗口时自动退化为纯陀螺积分；外部磁航向、视觉航向或双轴机构约束可通过 `alg_attitude_correct_yaw()` 注入。六轴 IMU 无法仅靠重力长期观测 yaw，这是物理限制。
