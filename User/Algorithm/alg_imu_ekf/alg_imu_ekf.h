/**
 * @file alg_imu_ekf.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 六轴 IMU 四元数扩展卡尔曼滤波器头文件
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 状态为 6 维：[qw, qx, qy, qz, bias_x, bias_y]
 *       陀螺仪是预测输入，加速度计是观测。
 *       六轴系统只能长期校正 Roll/Pitch，Yaw 依赖 Z 轴陀螺仪积分。
 */

#ifndef ALG_IMU_EKF_H
#define ALG_IMU_EKF_H

#include <stdbool.h> // bool 类型

#include "alg_filter.h" // 低通滤波器（用于加速度计预滤波）
#include "alg_kalman.h" // 通用 EKF 框架

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 维度常量 ======================== */

/** @brief 状态维度：4 四元数 + 2 陀螺仪零偏 = 6 */
#define ALG_IMU_EKF_STATE_DIMENSION (6U)
/** @brief 观测维度：3 轴加速度方向 = 3 */
#define ALG_IMU_EKF_MEASUREMENT_DIMENSION (3U)
/** @brief 控制维度：3 轴陀螺仪 = 3 */
#define ALG_IMU_EKF_CONTROL_DIMENSION (3U)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief IMU EKF 状态码
     * @note ACCELEROMETER_REJECTED 表示本次观测被拒绝（模长异常或 NIS 超阈值）
     */
    typedef enum
    {
        ALG_IMU_EKF_STATUS_OK = 0,                 // 操作成功
        ALG_IMU_EKF_STATUS_INVALID_ARGUMENT,       // 参数非法
        ALG_IMU_EKF_STATUS_OUT_OF_RANGE,           // 参数超出范围
        ALG_IMU_EKF_STATUS_NOT_INITIALIZED,        // 对象未初始化
        ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED, // 加速度观测被拒绝
        ALG_IMU_EKF_STATUS_NUMERICAL_ERROR,        // 数值错误
        ALG_IMU_EKF_STATUS_KALMAN_ERROR            // 底层卡尔曼错误
    } alg_imu_ekf_status_t;

    /* ======================== 四元数结构体 ======================== */

    /**
     * @brief 四元数（q = w + x*i + y*j + z*k）
     * @note 表示从机体系到世界系的旋转
     *       世界系 +Z 向上
     */
    typedef struct
    {
        float w; // 标量分量
        float x; // X 轴分量
        float y; // Y 轴分量
        float z; // Z 轴分量
    } alg_imu_ekf_quaternion_t;

    /* ======================== 欧拉角结构体 ======================== */

    /**
     * @brief 欧拉角（ZYX 旋转顺序）
     * @note roll（绕 X）、pitch（绕 Y）、yaw（绕 Z）
     *       单位均为弧度
     */
    typedef struct
    {
        float roll_rad;  // 滚转角（-π ~ π）
        float pitch_rad; // 俯仰角（-π/2 ~ π/2）
        float yaw_rad;   // 偏航角（-π ~ π）
    } alg_imu_ekf_euler_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief IMU EKF 配置参数
     * @note 所有参数均需现场标定或根据传感器规格书设置
     */
    typedef struct
    {
        float gravity_m_s2;                        // 标准重力加速度（m/s²），默认 9.80665
        float gyro_noise_std_rad_s;                // 陀螺仪白噪声标准差（rad/s）
        float gyro_bias_random_walk_std_rad_s2;    // X/Y 零偏随机游走标准差（rad/s²）
        float accelerometer_direction_noise_std;   // 单位重力方向观测噪声标准差
        float accelerometer_lpf_cutoff_hz;         // 加速度计低通滤波截止频率（Hz）
        float accelerometer_rejection_threshold_g; // 模长偏离 1g 的硬拒绝阈值（G）
        float chi_square_adaptation_threshold;     // 开始自适应增大噪声的 NIS 阈值
        float chi_square_rejection_threshold;      // 完全拒绝观测的 NIS 阈值
        float maximum_measurement_noise_scale;     // 自适应测量噪声最大倍率
        float gyro_bias_fading_factor;             // 零偏协方差渐消因子（≥1）
        float initial_attitude_variance;           // 初始四元数状态方差
        float initial_gyro_bias_variance;          // 初始 X/Y 零偏方差
    } alg_imu_ekf_config_t;

    /* ======================== 诊断数据结构体 ======================== */

    /**
     * @brief IMU EKF 诊断信息
     * @note 用于调试和状态监控
     */
    typedef struct
    {
        float filtered_accelerometer_m_s2[3]; // 低通滤波后的加速度（m/s²）
        float innovation[3];                  // 三维创新残差
        float accelerometer_norm_m_s2;        // 原始加速度模长（m/s²）
        float accelerometer_deviation_g;      // 相对 1g 的偏差（G）
        float normalized_innovation_squared;  // 归一化创新平方（NIS）
        float measurement_noise_scale;        // 当前测量噪声倍率
        bool was_accelerometer_used;          // 最近一次是否使用了加速度观测
    } alg_imu_ekf_diagnostics_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief IMU EKF 对象
     * @note 保存所有状态、协方差、滤波器实例和诊断信息
     *       初始化后不能按值复制或移动
     */
    typedef struct
    {
        alg_imu_ekf_config_t config;              // 配置参数
        alg_kalman_extended_t kalman;             // 通用 EKF 实例
        float state[ALG_IMU_EKF_STATE_DIMENSION]; // 状态向量
        float covariance[ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION]; // 协方差矩阵
        float process_noise[ALG_IMU_EKF_STATE_DIMENSION *
                            ALG_IMU_EKF_STATE_DIMENSION]; // 过程噪声矩阵
        float measurement_noise[ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                                ALG_IMU_EKF_MEASUREMENT_DIMENSION]; // 测量噪声矩阵
        float kalman_workspace[ALG_KALMAN_WORKSPACE_SIZE(           // EKF 工作区
            ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION)];
        float normalization_workspace[2U * ALG_IMU_EKF_STATE_DIMENSION *
                                      ALG_IMU_EKF_STATE_DIMENSION]; // 四元数归一化工作区
        float innovation_workspace[60U];                            // 创新计算工作区
        alg_filter_low_pass_t accelerometer_filter[3];              // 三轴加速度低通滤波器
        float filtered_accelerometer_m_s2[3];                       // 滤波后加速度
        float innovation[3];                                        // 创新残差
        float last_accelerometer_norm_m_s2;                         // 上次加速度模长
        float last_accelerometer_deviation_g;                       // 上次模长偏差
        float last_normalized_innovation_squared;                   // 上次 NIS
        float last_measurement_noise_scale;                         // 上次测量噪声倍率
        bool was_accelerometer_used;                                // 上次是否使用加速度
        bool is_initialized;                                        // 是否已初始化
    } alg_imu_ekf_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化配置为默认值
     * @param config 配置结构体指针
     * @return 执行状态
     * @note 默认值适用于大多数消费级 IMU，实际使用需根据传感器特性调整
     */
    alg_imu_ekf_status_t alg_imu_ekf_config_init(alg_imu_ekf_config_t *config);

    /**
     * @brief 初始化 IMU EKF
     * @param me EKF 对象
     * @param config 配置参数
     * @return 执行状态
     */
    alg_imu_ekf_status_t alg_imu_ekf_init(alg_imu_ekf_t *me, const alg_imu_ekf_config_t *config);

    /**
     * @brief 重置 EKF 到指定姿态和零偏
     * @param me EKF 对象
     * @param quaternion 初始四元数
     * @param gyro_bias_rad_s X/Y 轴零偏（rad/s），长度 2
     * @return 执行状态
     */
    alg_imu_ekf_status_t alg_imu_ekf_reset(alg_imu_ekf_t *me,
                                           const alg_imu_ekf_quaternion_t *quaternion,
                                           const float gyro_bias_rad_s[2]);

    /**
     * @brief 从加速度计读取重置姿态（Roll/Pitch 根据重力对齐，Yaw 置 0）
     * @param me EKF 对象
     * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
     * @return 执行状态
     * @note 应在设备静止时调用
     */
    alg_imu_ekf_status_t alg_imu_ekf_reset_from_accelerometer(alg_imu_ekf_t *me,
                                                              const float accelerometer_m_s2[3]);

    /**
     * @brief EKF 预测步骤（仅陀螺仪积分）
     * @param me EKF 对象
     * @param gyroscope_rad_s 陀螺仪读数（rad/s），长度 3
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     */
    alg_imu_ekf_status_t alg_imu_ekf_predict(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                             float delta_time_s);

    /**
     * @brief EKF 校正步骤（加速度计观测）
     * @param me EKF 对象
     * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 观测被拒绝时返回 ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED
     */
    alg_imu_ekf_status_t alg_imu_ekf_correct_accelerometer(alg_imu_ekf_t *me,
                                                           const float accelerometer_m_s2[3],
                                                           float delta_time_s);

    /**
     * @brief 完整 EKF 更新（预测 + 校正）
     * @param me EKF 对象
     * @param gyroscope_rad_s 陀螺仪读数（rad/s），长度 3
     * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
     * @param delta_time_s 时间步长（秒）
     * @param accelerometer_used 输出本次是否使用了加速度观测
     * @return 执行状态
     * @note 即使加速度被拒绝，预测步骤仍会执行，返回 OK
     */
    alg_imu_ekf_status_t alg_imu_ekf_update(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                            const float accelerometer_m_s2[3], float delta_time_s,
                                            bool *accelerometer_used);

    /**
     * @brief 获取当前四元数
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_quaternion(const alg_imu_ekf_t *me,
                                                    alg_imu_ekf_quaternion_t *quaternion);

    /**
     * @brief 获取当前欧拉角（ZYX 顺序）
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_euler(const alg_imu_ekf_t *me, alg_imu_ekf_euler_t *euler);

    /**
     * @brief 获取陀螺仪零偏（X/Y 轴）
     * @note Z 轴零偏恒为 0（六轴 IMU 无法观测）
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_gyro_bias(const alg_imu_ekf_t *me,
                                                   float gyro_bias_rad_s[3]);

    /**
     * @brief 获取校正后的陀螺仪读数（原始读数减去零偏）
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_corrected_gyroscope(const alg_imu_ekf_t *me,
                                                             const float gyroscope_rad_s[3],
                                                             float corrected_gyroscope_rad_s[3]);

    /**
     * @brief 获取诊断信息
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_diagnostics(const alg_imu_ekf_t *me,
                                                     alg_imu_ekf_diagnostics_t *diagnostics);

    /**
     * @brief 获取机体系中的重力向量
     */
    alg_imu_ekf_status_t alg_imu_ekf_get_gravity_body(const alg_imu_ekf_t *me,
                                                      float gravity_body_m_s2[3]);

    /**
     * @brief 获取机体系中的线性加速度（测量值减去重力）
     */
    alg_imu_ekf_status_t
    alg_imu_ekf_get_linear_acceleration_body(const alg_imu_ekf_t *me,
                                             const float accelerometer_m_s2[3],
                                             float linear_acceleration_body_m_s2[3]);

    /**
     * @brief 获取世界系中的线性加速度（旋转到世界系后减去重力）
     */
    alg_imu_ekf_status_t
    alg_imu_ekf_get_linear_acceleration_world(const alg_imu_ekf_t *me,
                                              const float accelerometer_m_s2[3],
                                              float linear_acceleration_world_m_s2[3]);

#ifdef __cplusplus
}
#endif

#endif /* ALG_IMU_EKF_H */