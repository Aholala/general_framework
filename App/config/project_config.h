#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/**
 * @file    project_config.h
 * @brief   集中式项目参数配置文件（项目特性化层）
 * @note    这是整个项目唯一需要"按机器人调整"的配置文件。
 *          所有 PID、CAN ID、机械尺寸、电机限幅、功能开关都在这里。
 *          换机器人 = 改这个文件 + board_config.h/.c。
 *
 *          参考模式: SteerHeroGimbalUC2026/App/Gimbal/task/gimbal_config.h
 */

/* 引入框架默认配置 (任务周期、板级角色等通用常量) */
#include "app_config.h"

/* ==========================================================================
   1. 功能裁剪开关（注释掉 = 不编译 = 零开销）
   ========================================================================== */
// #define PROJECT_HAS_FIREDISH             // 是否有拨弹盘
// #define PROJECT_HAS_POWER_CTRL           // 是否有功率控制 (RLS)
// #define PROJECT_HAS_WS2812               // 是否有 RGB LED 灯效
// #define PROJECT_HAS_SOLENOID             // 是否有电磁阀
// #define PROJECT_HAS_TOF                  // 是否有 TOF 激光测距
// #define PROJECT_HAS_SLOPE                // 是否有斜坡模式
// #define PROJECT_HAS_CRAB_MODE            // 是否有蟹行模式
// #define PROJECT_HAS_VISION_USB           // 是否有 USB 视觉通信
// #define PROJECT_HAS_REFEREE              // 是否有裁判系统
// #define PROJECT_HAS_DEBUG_LIGHT          // 是否有调试灯

/* ==========================================================================
   2. 任务周期 & 板级角色
      继承自 ECF/App/app_config.h (APP_COMMAND_PERIOD_MS, APP_CHASSIS_PERIOD_MS 等)
      项目如需覆盖，在此 #define 即可。
   ========================================================================== */

/* ==========================================================================
   4. CAN / FDCAN 总线 → 外设映射
      这些宏把"逻辑 CAN 通道"映射到 board_config.h 中定义的物理 CAN 实例。
      BOARD_CONFIG_CAN_1 → FDCAN1, BOARD_CONFIG_CAN_2 → FDCAN2, ...
   ========================================================================== */
#define PROJECT_CAN_CHASSIS_WHEEL    BOARD_CONFIG_CAN_1    // 底盘轮电机 CAN 总线
#define PROJECT_CAN_GIMBAL_YAW       BOARD_CONFIG_CAN_2    // 云台 yaw 电机 CAN 总线
#define PROJECT_CAN_AUXILIARY        BOARD_CONFIG_CAN_3    // 辅助 CAN 总线

#ifdef PROJECT_HAS_FIREDISH
#define PROJECT_CAN_FIREDISH         PROJECT_CAN_AUXILIARY // 拨弹盘使用辅助 CAN
#endif

/* ==========================================================================
   5. 电机 CAN ID（DJI 协议 TX 标识符, DM 协议 RX/TX 标识符）
      CAN ID 在项目中必须唯一，不能和裁判系统、板间通信冲突。
   ========================================================================== */
/* 底盘: 4x M3508 麦轮电机 (CAN1) */
#define PROJECT_M3508_RF_TX_ID  0x201    // 右前轮 CAN TX ID
#define PROJECT_M3508_LF_TX_ID  0x202    // 左前轮 CAN TX ID
#define PROJECT_M3508_LB_TX_ID  0x203    // 左后轮 CAN TX ID
#define PROJECT_M3508_RB_TX_ID  0x204    // 右后轮 CAN TX ID

/* 云台: DM4310 yaw 轴电机 (CAN2) */
#define PROJECT_DM4310_YAW_TX_ID  0x4C   // yaw 电机 MIT 命令 TX ID
#define PROJECT_DM4310_YAW_RX_ID  0x4D   // yaw 电机反馈 RX ID

/* 云台: DM4340 pitch 轴电机 (CAN1 或 CAN3) */
#define PROJECT_DM4340_PITCH_TX_ID  0x5C  // pitch 电机 MIT 命令 TX ID
#define PROJECT_DM4340_PITCH_RX_ID  0x5D  // pitch 电机反馈 RX ID

#ifdef PROJECT_HAS_FIREDISH
#define PROJECT_DM4310_FIREDISH_TX_ID  0x3C  // 拨弹盘电机 MIT 命令 TX ID
#define PROJECT_DM4310_FIREDISH_RX_ID  0x3D  // 拨弹盘电机反馈 RX ID
#endif

/* ==========================================================================
   6. 电机方向反转标志
      MOTOR_DIRECTION_NORMAL  = 0  — 不反转
      MOTOR_DIRECTION_REVERSE = 1  — 反转
      麦轮底盘: 左前和左后通常需要反转（镜像安装）
   ========================================================================== */
#define PROJECT_WHEEL_RF_DIRECTION  MOTOR_DIRECTION_NORMAL    // 右前轮不反转
#define PROJECT_WHEEL_LF_DIRECTION  MOTOR_DIRECTION_REVERSE   // 左前轮反转
#define PROJECT_WHEEL_LB_DIRECTION  MOTOR_DIRECTION_REVERSE   // 左后轮反转
#define PROJECT_WHEEL_RB_DIRECTION  MOTOR_DIRECTION_NORMAL    // 右后轮不反转

#define PROJECT_YAW_DIRECTION    MOTOR_DIRECTION_NORMAL       // yaw 电机不反转
#define PROJECT_PITCH_DIRECTION  MOTOR_DIRECTION_NORMAL       // pitch 电机不反转

/* ==========================================================================
   7. PID 参数 — 底盘轮电机（M3508, 速度环）
      ⚠ 所有 PID 值目前为占位值 1.0f，实际使用时需要调参。
      PID 模式位掩码:
        OUTPUT_LIMIT   = 输出限幅
        INTEGRAL_LIMIT = 积分限幅
        STEP_IN        = 阶跃输入处理
        DEADZONE       = 死区
   ========================================================================== */
#define PROJECT_WHEEL_SPEED_KP            1.0f    // 速度环 Kp (占位)
#define PROJECT_WHEEL_SPEED_KI            1.0f    // 速度环 Ki (占位)
#define PROJECT_WHEEL_SPEED_KD            1.0f    // 速度环 Kd (占位)
#define PROJECT_WHEEL_SPEED_MAX_IOUT      1.0f    // 速度环积分限幅 (占位)
#define PROJECT_WHEEL_SPEED_MAX_OUT       1.0f    // 速度环输出限幅 (占位)
#define PROJECT_WHEEL_SPEED_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT)

#define PROJECT_WHEEL_ANGLE_KP            1.0f    // 角度环 Kp (占位)
#define PROJECT_WHEEL_ANGLE_KI            1.0f    // 角度环 Ki (占位)
#define PROJECT_WHEEL_ANGLE_KD            1.0f    // 角度环 Kd (占位)
#define PROJECT_WHEEL_ANGLE_MAX_IOUT      1.0f    // 角度环积分限幅 (占位)
#define PROJECT_WHEEL_ANGLE_MAX_OUT       1.0f    // 角度环输出限幅 (占位)
#define PROJECT_WHEEL_ANGLE_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT | STEP_IN)
#define PROJECT_WHEEL_ANGLE_STEP_IN       1.0f    // 角度环阶跃步长 (占位)

/* ==========================================================================
   8. PID 参数 — Yaw 轴电机（DM4310, 角度环）
   ========================================================================== */
#define PROJECT_YAW_ANGLE_KP            1.0f    // yaw 角度环 Kp (占位)
#define PROJECT_YAW_ANGLE_KI            1.0f    // yaw 角度环 Ki (占位)
#define PROJECT_YAW_ANGLE_KD            1.0f    // yaw 角度环 Kd (占位)
#define PROJECT_YAW_ANGLE_MAX_IOUT      1.0f    // yaw 角度环积分限幅 (占位)
#define PROJECT_YAW_ANGLE_MAX_OUT       1.0f    // yaw 角度环输出限幅 (占位)
#define PROJECT_YAW_ANGLE_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT | STEP_IN)
#define PROJECT_YAW_ANGLE_STEP_IN       1.0f    // yaw 角度环阶跃步长 (占位)

#define PROJECT_YAW_SPEED_KP            1.0f    // yaw 速度环 Kp (占位)
#define PROJECT_YAW_SPEED_KI            1.0f    // yaw 速度环 Ki (占位)
#define PROJECT_YAW_SPEED_KD            1.0f    // yaw 速度环 Kd (占位)
#define PROJECT_YAW_SPEED_MAX_IOUT      1.0f    // yaw 速度环积分限幅 (占位)
#define PROJECT_YAW_SPEED_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT)

/* ==========================================================================
   9. PID 参数 — Pitch 轴电机（DM4340, 角度环）
   ========================================================================== */
#define PROJECT_PITCH_ANGLE_KP            1.0f  // pitch 角度环 Kp (占位)
#define PROJECT_PITCH_ANGLE_KI            1.0f  // pitch 角度环 Ki (占位)
#define PROJECT_PITCH_ANGLE_KD            1.0f  // pitch 角度环 Kd (占位)
#define PROJECT_PITCH_ANGLE_MAX_IOUT      1.0f  // pitch 角度环积分限幅 (占位)
#define PROJECT_PITCH_ANGLE_MAX_OUT       1.0f  // pitch 角度环输出限幅 (占位)
#define PROJECT_PITCH_ANGLE_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT)

#define PROJECT_PITCH_SPEED_KP            1.0f  // pitch 速度环 Kp (占位)
#define PROJECT_PITCH_SPEED_KI            1.0f  // pitch 速度环 Ki (占位)
#define PROJECT_PITCH_SPEED_KD            1.0f  // pitch 速度环 Kd (占位)
#define PROJECT_PITCH_SPEED_MAX_IOUT      1.0f  // pitch 速度环积分限幅 (占位)
#define PROJECT_PITCH_SPEED_MAX_OUT       1.0f  // pitch 速度环输出限幅 (占位)
#define PROJECT_PITCH_SPEED_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT)

/* pitch 轴前馈系数 (重力补偿) */
#define PROJECT_PITCH_FF_KFA              1.0f  // 前馈加速度系数 (占位)
#define PROJECT_PITCH_FF_KFB              1.0f  // 前馈速度系数 (占位)

/* ==========================================================================
   10. PID 参数 — Z 轴跟随（底盘→云台旋转闭环）
       "双头龙"PID: 云台角度差 → 底盘 Z 轴角速度输出
   ========================================================================== */
#define PROJECT_Z_FOLLOW_KP            1.0f    // Z跟随 Kp (占位)
#define PROJECT_Z_FOLLOW_KI            1.0f    // Z跟随 Ki (占位)
#define PROJECT_Z_FOLLOW_KD            1.0f    // Z跟随 Kd (占位)
#define PROJECT_Z_FOLLOW_MAX_IOUT      1.0f    // Z跟随积分限幅 (占位)
#define PROJECT_Z_FOLLOW_MAX_OUT       1.0f    // Z跟随输出限幅 (占位)
#define PROJECT_Z_FOLLOW_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT | DEADZONE)

/* ==========================================================================
   11. PID 参数 — 发射机构摩擦轮 (M3508 或 M2006, 速度环)
   ========================================================================== */
#ifdef PROJECT_HAS_FIREDISH
#define PROJECT_FRICTION_SPEED_KP            1.0f  // 摩擦轮速度环 Kp (占位)
#define PROJECT_FRICTION_SPEED_KI            1.0f  // 摩擦轮速度环 Ki (占位)
#define PROJECT_FRICTION_SPEED_KD            1.0f  // 摩擦轮速度环 Kd (占位)
#define PROJECT_FRICTION_SPEED_MAX_IOUT      1.0f  // 摩擦轮速度环积分限幅 (占位)
#define PROJECT_FRICTION_SPEED_MAX_OUT       1.0f  // 摩擦轮速度环输出限幅 (占位)
#define PROJECT_FRICTION_SPEED_PID_MODE      (OUTPUT_LIMIT | INTEGRAL_LIMIT)
#endif

/* ==========================================================================
   12. DM 电机控制限幅（根据 DM-J4310-2EC / DM4340 用户手册）
       ⚠ KP/KD/V/P/T 上下限必须与电机手册严格一致，超限会导致电机拒绝命令。
       =====================================================================
       DM-J4310-2EC (yaw, 拨弹盘):
         手册 V1.2 2026-04-09
         - KP 范围:  0.0 ~ 500.0
         - KD 范围:  0.0 ~ 5.0
         - V  范围:  -30.0 ~ 30.0  rad/s
         - P  范围:  -12.56637 ~ 12.56637  rad (±2π, 即 ±720°)
         - T  范围:  -10.0 ~ 10.0  Nm (峰值扭矩)
       =====================================================================
       DM4340 (pitch):
         - KP 范围:  0.0 ~ 500.0
         - KD 范围:  0.0 ~ 5.0
         - V  范围:  -30.0 ~ 30.0  rad/s
         - P  范围:  -12.56 ~ 12.56  rad
         - T  范围:  -28.0 ~ 28.0  Nm (峰值扭矩，比 4310 大)
       =====================================================================
       DJI M3508 (轮电机, 通过 C620 电调):
         手册: 最大持续电流 2.8A, C620 峰值输出 ±10A
         - 电流分辨率: 0.00061035 A/LSB (1/16384 * 10A)
         - 电流原始值范围: -16384 ~ +16384 (int16_t)
         - 速度范围: 0 ~ 469 RPM (空载)
       =====================================================================
   ========================================================================== */

/* DM4310 (yaw + 拨弹盘电机) 限幅 */
#define PROJECT_DM4310_KP_MIN   0.0f        // KP 最小 (手册: 0)
#define PROJECT_DM4310_KP_MAX   500.0f      // KP 最大 (手册: 500)
#define PROJECT_DM4310_KD_MIN   0.0f        // KD 最小 (手册: 0)
#define PROJECT_DM4310_KD_MAX   5.0f        // KD 最大 (手册: 5)
#define PROJECT_DM4310_V_MIN   -30.0f       // 速度最小 rad/s (手册: -30)
#define PROJECT_DM4310_V_MAX    30.0f       // 速度最大 rad/s (手册: 30)
#define PROJECT_DM4310_P_MIN   -12.56637f   // 位置最小 rad (手册: -4π ≈ -12.56637)
#define PROJECT_DM4310_P_MAX    12.56637f   // 位置最大 rad (手册: +4π ≈ 12.56637)
#define PROJECT_DM4310_T_MIN   -10.0f       // 扭矩最小 Nm (手册: -10, 峰值)
#define PROJECT_DM4310_T_MAX    10.0f       // 扭矩最大 Nm (手册: 10, 峰值)

/* DM4340 (pitch 电机) 限幅 */
#define PROJECT_DM4340_KP_MIN   0.0f        // KP 最小
#define PROJECT_DM4340_KP_MAX   500.0f      // KP 最大
#define PROJECT_DM4340_KD_MIN   0.0f        // KD 最小
#define PROJECT_DM4340_KD_MAX   5.0f        // KD 最大
#define PROJECT_DM4340_V_MIN   -30.0f       // 速度最小 rad/s
#define PROJECT_DM4340_V_MAX    30.0f       // 速度最大 rad/s
#define PROJECT_DM4340_P_MIN   -12.56f      // 位置最小 rad
#define PROJECT_DM4340_P_MAX    12.56f      // 位置最大 rad
#define PROJECT_DM4340_T_MIN   -28.0f       // 扭矩最小 Nm (手册: -28, 峰值比4310大)
#define PROJECT_DM4340_T_MAX    28.0f       // 扭矩最大 Nm (手册: 28)

/* ==========================================================================
   13. DJI M3508 电机限幅（C620 电调）
       C620 使用 int16_t 电流值，范围 -16384 ~ +16384
       换算: 电流(A) = 原始值 × 0.00061035 A/LSB
       -16384 → -10A (峰值反向), +16384 → +10A (峰值正向)
   ========================================================================== */
#define PROJECT_M3508_CURRENT_RAW_MIN    -16384    // M3508 电流原始值下限 (-10A)
#define PROJECT_M3508_CURRENT_RAW_MAX     16384    // M3508 电流原始值上限 (+10A)
#define PROJECT_M3508_CURRENT_RESOLUTION  0.00061035f  // 电流分辨率 A/LSB
#define PROJECT_M3508_MAX_SPEED_RPM       469U     // M3508 空载最大转速 RPM

/* ==========================================================================
   14. 机械参数
   ========================================================================== */
/* 底盘几何 */
#define PROJECT_WHEEL_RADIUS_M              1.0f    // 轮半径 m (占位)
#define PROJECT_WHEEL_GEAR_RATIO            1.0f    // 减速比 (占位)
#define PROJECT_CHASSIS_HALF_TRACK_M        1.0f    // 半轮距 m (占位)
#define PROJECT_CHASSIS_HALF_WHEELBASE_M    1.0f    // 半轴距 m (占位)
#define PROJECT_MECANUM_ROTATE_ARM          1.0f    // 麦轮旋转臂系数 (占位)
#define PROJECT_MECANUM_WHEEL_SPEED_MAX     1.0f    // 麦轮最大速度 (占位)

/* 运动限幅 */
#define PROJECT_CHASSIS_MAX_VELOCITY_MPS    1.0f    // 底盘最大线速度 m/s (占位)
#define PROJECT_CHASSIS_MAX_SPIN_RAD_S      1.0f    // 底盘最大旋转角速度 rad/s (占位)
#define PROJECT_CHASSIS_FOLLOW_GAIN         1.0f    // 底盘跟随增益 (占位)
#define PROJECT_CHASSIS_STOP_DEADBAND       1.0f    // 底盘停止死区 (占位)

/* 云台限幅 */
#define PROJECT_GIMBAL_MAX_YAW_RATE_RAD_S   1.0f    // yaw 最大角速度 rad/s (占位)
#define PROJECT_GIMBAL_MAX_PITCH_RATE_RAD_S 1.0f    // pitch 最大角速度 rad/s (占位)
#define PROJECT_GIMBAL_MIN_PITCH_RAD       -1.0f    // pitch 最小角度 rad (占位)
#define PROJECT_GIMBAL_MAX_PITCH_RAD        1.0f    // pitch 最大角度 rad (占位)

/* pitch 重力补偿参数 */
#define PROJECT_PITCH_LEVER_ARM_M           1.0f    // pitch 杠杆臂长 m (占位)
#define PROJECT_PITCH_GRAVITY_FORCE_N       1.0f    // pitch 重力 N (占位)
#define PROJECT_PITCH_ENCODER_OFFSET        1.0f    // pitch 编码器零偏 rad (占位)

/* ==========================================================================
   15. 拨弹盘参数 (仅当 PROJECT_HAS_FIREDISH 启用时编译)
   ========================================================================== */
#ifdef PROJECT_HAS_FIREDISH
#define PROJECT_FIREDISH_KP                      1.0f  // 拨弹盘 MIT KP (占位)
#define PROJECT_FIREDISH_KD                      1.0f  // 拨弹盘 MIT KD (占位)
#define PROJECT_FIREDISH_TOOTH_DEG               1.0f  // 每齿角度 ° (占位)
#define PROJECT_FIREDISH_FEED_DONE_TOLERANCE_DEG 1.0f  // 拨弹完成角度容差 ° (占位)
#define PROJECT_FIREDISH_FEED_DONE_SPEED         1.0f  // 拨弹完成速度阈值 RPM (占位)
#define PROJECT_FIREDISH_RECOVER_OFFSET_DEG      1.0f  // 堵转恢复后退角度 ° (占位)
#define PROJECT_FIREDISH_RECOVER_TOLERANCE_DEG   1.0f  // 堵转恢复到位容差 ° (占位)
#define PROJECT_FIREDISH_ZERO_SYNC_TOLERANCE_DEG 1.0f  // 归零同步角度容差 ° (占位)
#define PROJECT_FIREDISH_ZERO_SPEED_THRESHOLD    1.0f  // 归零速度阈值 RPM (占位)
#define PROJECT_FIREDISH_ZERO_WAIT_KD            1.0f  // 归零等待 KD (占位)
#define PROJECT_FIREDISH_ZERO_RETRY_CYCLES       1U    // 归零重试周期数 (占位)
#define PROJECT_FIREDISH_ENABLE_BURST_CYCLES     1U    // 使能连发周期数 (占位)
#define PROJECT_FIREDISH_POSITION_MIN_RAD        PROJECT_DM4310_P_MIN // 拨弹盘最小位置
#define PROJECT_FIREDISH_POSITION_MAX_RAD        PROJECT_DM4310_P_MAX // 拨弹盘最大位置
#endif

/* ==========================================================================
   16. 功率模型系数 (仅当 PROJECT_HAS_POWER_CTRL 启用时编译)
       模型: P = k0 + k1*T + k2*w + k3*T*w + k4*T² + k5*w²
       T = 扭矩 (Nm), w = 角速度 (rad/s), P = 功率 (W)
   ========================================================================== */
#ifdef PROJECT_HAS_POWER_CTRL
#define PROJECT_POWER_MODEL_K \
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}  // 功率模型系数 (占位)
#endif

/* ==========================================================================
   17. 板间通信参数
   ========================================================================== */
#define PROJECT_BOARD_COMM_BASE_ID        (0x500U)  // 板间 CAN 基 ID
#define PROJECT_BOARD_COMM_LINK_CAN       BOARD_CONFIG_CAN_2   // 板间通信 CAN 总线
#define PROJECT_BOARD_COMM_TX_TIMEOUT_MS  (2U)     // 发送超时 ms
#define PROJECT_BOARD_COMM_OFFLINE_MS     (100U)   // 离线判定时间 ms

/* ==========================================================================
   18. 安全 & 看门狗参数
   ========================================================================== */
#define PROJECT_DR16_OFFLINE_TIMEOUT_MS       (100U)   // DR16 遥控器离线超时 ms
#define PROJECT_MOTOR_FEEDBACK_TIMEOUT_MS     (50U)    // 电机反馈超时 ms
#define PROJECT_SAFETY_DISCONNECTION_MS       (500U)   // 安全任务失联阈值 ms

/* ==========================================================================
   19. 编译期校验
       校验继承自 app_config.h 的板级角色和设备位置配置。
       错误的配置在编译时报错，防止运行时才发现。
   ========================================================================== */
#if (APP_DR16_LOCATION != APP_DEVICE_LOCATION_GIMBAL) && \
    (APP_DR16_LOCATION != APP_DEVICE_LOCATION_CHASSIS)
#error "APP_DR16_LOCATION must be GIMBAL or CHASSIS"
#endif

#if (APP_FEEDER_LOCATION != APP_DEVICE_LOCATION_GIMBAL) && \
    (APP_FEEDER_LOCATION != APP_DEVICE_LOCATION_CHASSIS)
#error "APP_FEEDER_LOCATION must be GIMBAL or CHASSIS"
#endif

#endif /* PROJECT_CONFIG_H */
