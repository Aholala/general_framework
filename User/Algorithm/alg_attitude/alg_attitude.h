/**
 * @file alg_attitude.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 六轴 IMU 姿态估计算法头文件
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持 Mahony 互补滤波和 Madgwick 梯度下降滤波两种算法。
 *       加速度模长超出配置窗口时自动退化为纯陀螺积分。
 *       外部磁航向、视觉航向或机构约束可通过 alg_attitude_correct_yaw() 注入。
 */

#ifndef ALG_ATTITUDE_H
#define ALG_ATTITUDE_H

#include <stdbool.h> // bool 类型

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 姿态算法状态码
     */
    typedef enum
    {
        ALG_ATTITUDE_STATUS_OK = 0,           // 操作成功（使用了加速度修正）
        ALG_ATTITUDE_STATUS_GYRO_ONLY,        // 仅陀螺仪积分（加速度超出有效范围）
        ALG_ATTITUDE_STATUS_INVALID_ARGUMENT, // 参数非法
        ALG_ATTITUDE_STATUS_NOT_INITIALIZED,  // 对象未初始化
        ALG_ATTITUDE_STATUS_NUMERICAL_ERROR   // 数值错误（如四元数归一化失败）
    } alg_attitude_status_t;

    /* ======================== 算法类型枚举 ======================== */

    /**
     * @brief 姿态估计算法类型
     */
    typedef enum
    {
        ALG_ATTITUDE_METHOD_MAHONY = 0, // Mahony 互补滤波（PI 反馈修正陀螺仪）
        ALG_ATTITUDE_METHOD_MADGWICK    // Madgwick 梯度下降滤波
    } alg_attitude_method_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 姿态算法配置
     */
    typedef struct
    {
        alg_attitude_method_t method;    // 算法选择
        float proportional_gain;         // Mahony 比例增益（Kp），控制修正速度
        float integral_gain;             // Mahony 积分增益（Ki），补偿陀螺零偏
        float madgwick_beta;             // Madgwick 梯度下降步长（β）
        float acceleration_min_m_per_s2; // 加速度有效下限（m/s²）
        float acceleration_max_m_per_s2; // 加速度有效上限（m/s²）
    } alg_attitude_config_t;

    /* ======================== 四元数结构体 ======================== */

    /**
     * @brief 四元数结构体（q = q0 + q1*i + q2*j + q3*k）
     * @note q0 为标量分量（w），q1/q2/q3 为矢量分量（x/y/z）
     *       始终保证归一化（|q| = 1.0）
     */
    typedef struct
    {
        float q0; // 标量分量（w）
        float q1; // X 轴分量
        float q2; // Y 轴分量
        float q3; // Z 轴分量
    } alg_attitude_quaternion_t;

    /* ======================== 旋转矩阵结构体 ======================== */

    /**
     * @brief 3×3 旋转矩阵
     * @note 从机体坐标系到世界坐标系的旋转
     */
    typedef struct
    {
        float element[3][3];
    } alg_attitude_rotation_matrix_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 姿态估计器对象
     */
    typedef struct
    {
        alg_attitude_config_t config;         // 配置参数
        alg_attitude_quaternion_t quaternion; // 当前姿态四元数
        float integral_error_x;               // Mahony X 轴积分误差
        float integral_error_y;               // Mahony Y 轴积分误差
        float integral_error_z;               // Mahony Z 轴积分误差
        bool is_initialized;                  // 是否已初始化
    } alg_attitude_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化姿态估计器
     * @param me 姿态估计器对象
     * @param config 配置参数
     * @param initial_quaternion 初始四元数（NULL 则使用单位四元数）
     * @return 执行状态
     */
    alg_attitude_status_t alg_attitude_init(alg_attitude_t *me,
                                            const alg_attitude_config_t *config,
                                            const alg_attitude_quaternion_t *initial_quaternion);

    /**
     * @brief 重置姿态到指定四元数
     * @param me 姿态估计器对象
     * @param quaternion 目标四元数
     * @return 执行状态
     */
    alg_attitude_status_t alg_attitude_reset(alg_attitude_t *me,
                                             const alg_attitude_quaternion_t *quaternion);

    /**
     * @brief 更新姿态估计
     * @param me 姿态估计器对象
     * @param gyro_x_rad_per_s X 轴陀螺仪（rad/s）
     * @param gyro_y_rad_per_s Y 轴陀螺仪（rad/s）
     * @param gyro_z_rad_per_s Z 轴陀螺仪（rad/s）
     * @param acceleration_x_m_per_s2 X 轴加速度（m/s²）
     * @param acceleration_y_m_per_s2 Y 轴加速度（m/s²）
     * @param acceleration_z_m_per_s2 Z 轴加速度（m/s²）
     * @param delta_time_s 时间步长（秒）
     * @return OK=使用加速度修正，GYRO_ONLY=仅陀螺仪积分
     */
    alg_attitude_status_t alg_attitude_update(alg_attitude_t *me,
                                              float gyro_x_rad_per_s,
                                              float gyro_y_rad_per_s, float gyro_z_rad_per_s,
                                              float acceleration_x_m_per_s2,
                                              float acceleration_y_m_per_s2,
                                              float acceleration_z_m_per_s2, float delta_time_s);

    /**
     * @brief 外部航向修正（注入磁航向、视觉航向或机构约束）
     * @param me 姿态估计器对象
     * @param measured_yaw_rad 测量的航向角（弧度）
     * @param correction_gain 修正增益（0~1）
     * @return 执行状态
     * @note 六轴 IMU 无法仅靠重力长期观测 yaw，需定期注入外部航向
     */
    alg_attitude_status_t alg_attitude_correct_yaw(alg_attitude_t *me,
                                                   float measured_yaw_rad,
                                                   float correction_gain);

    /**
     * @brief 获取欧拉角（roll/pitch/yaw）
     * @param me 姿态估计器对象
     * @param roll_rad 输出滚转角（弧度）
     * @param pitch_rad 输出俯仰角（弧度）
     * @param yaw_rad 输出偏航角（弧度）
     * @return 执行状态
     */
    alg_attitude_status_t alg_attitude_get_euler(const alg_attitude_t *me,
                                                 float *roll_rad,
                                                 float *pitch_rad, float *yaw_rad);

    /**
     * @brief 获取旋转矩阵
     * @param me 姿态估计器对象
     * @param rotation_matrix 输出旋转矩阵
     * @return 执行状态
     */
    alg_attitude_status_t
    alg_attitude_get_rotation_matrix(const alg_attitude_t *me,
                                     alg_attitude_rotation_matrix_t *rotation_matrix);

#ifdef __cplusplus
}
#endif

#endif