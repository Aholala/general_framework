/**
 * @file module_bmi088.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BMI088 六轴 IMU 传感器模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 bsp_spi_t 抽象层驱动 BMI088 加速度计和陀螺仪，
 *       支持轴重映射、量程配置、自检、陀螺仪零偏标定等。
 */

#ifndef MODULE_BMI088_H
#define MODULE_BMI088_H

#include "bsp_spi.h"       // SPI BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct module_bmi088 module_bmi088_t;

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief BMI088 模块状态码
     */
    typedef enum
    {
        MODULE_BMI088_STATUS_OK = 0,                 // 操作成功
        MODULE_BMI088_STATUS_INVALID_ARGUMENT,       // 参数非法
        MODULE_BMI088_STATUS_NOT_INITIALIZED,        // 对象未初始化
        MODULE_BMI088_STATUS_TRANSPORT_ERROR,        // SPI 传输错误
        MODULE_BMI088_STATUS_ACCEL_NOT_FOUND,        // 加速度计不存在（Chip ID 错误）
        MODULE_BMI088_STATUS_GYRO_NOT_FOUND,         // 陀螺仪不存在（Chip ID 错误）
        MODULE_BMI088_STATUS_REGISTER_VERIFY_FAILED, // 寄存器回读校验失败
        MODULE_BMI088_STATUS_SELF_TEST_FAILED,       // 自检失败
        MODULE_BMI088_STATUS_CALIBRATION_MOTION,     // 校准时传感器运动过大
        MODULE_BMI088_STATUS_OUT_OF_RANGE            // 参数超出范围
    } module_bmi088_status_t;

    /* ======================== 量程枚举 ======================== */

    /**
     * @brief 加速度计量程
     */
    typedef enum
    {
        MODULE_BMI088_ACCEL_RANGE_3G = 0, // ±3G
        MODULE_BMI088_ACCEL_RANGE_6G,     // ±6G
        MODULE_BMI088_ACCEL_RANGE_12G,    // ±12G
        MODULE_BMI088_ACCEL_RANGE_24G     // ±24G
    } module_bmi088_accel_range_t;

    /**
     * @brief 陀螺仪量程
     */
    typedef enum
    {
        MODULE_BMI088_GYRO_RANGE_2000_DPS = 0, // ±2000°/s
        MODULE_BMI088_GYRO_RANGE_1000_DPS,     // ±1000°/s
        MODULE_BMI088_GYRO_RANGE_500_DPS,      // ±500°/s
        MODULE_BMI088_GYRO_RANGE_250_DPS,      // ±250°/s
        MODULE_BMI088_GYRO_RANGE_125_DPS       // ±125°/s
    } module_bmi088_gyro_range_t;

    /**
     * @brief 传感器选择（加速度计或陀螺仪）
     */
    typedef enum
    {
        MODULE_BMI088_SENSOR_ACCEL = 0, // 加速度计
        MODULE_BMI088_SENSOR_GYRO       // 陀螺仪
    } module_bmi088_sensor_t;

    /* ======================== 轴映射结构体 ======================== */

    /**
     * @brief 轴映射表项
     * @note 将传感器原始轴映射到逻辑轴，并校正方向
     */
    typedef struct
    {
        uint8_t source_axis;  // 传感器原始轴索引（0=X, 1=Y, 2=Z），必须唯一
        float direction_sign; // 方向符号：+1.0 或 -1.0
    } module_bmi088_axis_map_t;

    /* ======================== 数据结构体 ======================== */

    /**
     * @brief 原始 AD 计数值（便于调试）
     */
    typedef struct
    {
        int16_t acceleration[3];     // 加速度计原始值（带符号）
        int16_t angular_velocity[3]; // 陀螺仪原始值（带符号）
        int16_t temperature;         // 温度原始值（带符号，10 位有符号扩展）
    } module_bmi088_raw_data_t;

    /**
     * @brief 物理量数据（供算法使用）
     */
    typedef struct
    {
        float acceleration_m_per_s2[3];      // 加速度（m/s²）
        float angular_velocity_rad_per_s[3]; // 角速度（rad/s）
        float temperature_c;                 // 温度（°C）
        uint32_t timestamp_us;               // 采样时间戳（微秒），由 get_time_us 提供
        uint32_t sample_interval_us;         // 与上次采样的间隔（微秒）
        uint32_t sample_count;               // 成功采样总数
        uint32_t failed_sample_count;        // 失败采样总数
        bool is_valid;                       // 当前数据是否有效（上次读取成功）
    } module_bmi088_data_t;

    /* ======================== 回调函数类型 ======================== */

    /**
     * @brief 片选控制回调
     * @param user_context 用户上下文
     * @param sensor 选择哪个传感器
     * @param is_selected true=选中，false=取消选中
     * @note 片选为低电平有效，选中时应输出低电平
     */
    typedef void (*module_bmi088_chip_select_t)(void *user_context, module_bmi088_sensor_t sensor,
                                                bool is_selected);

    /**
     * @brief 毫秒级延时回调
     * @param user_context 用户上下文
     * @param delay_ms 延时毫秒数
     */
    typedef void (*module_bmi088_delay_ms_t)(void *user_context, uint32_t delay_ms);

    /**
     * @brief 微秒级时间戳获取回调（可选）
     * @param user_context 用户上下文
     * @return 当前微秒时间戳
     * @note 若为 NULL，则不提供时间戳
     */
    typedef uint32_t (*module_bmi088_get_time_us_t)(void *user_context);

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief BMI088 初始化配置
     */
    typedef struct
    {
        const char *logical_name;                          // 设备逻辑名称
        uint32_t registration_key;                         // 模块注册键值
        bsp_spi_t *spi;                                    // SPI BSP 基类（必须已初始化）
        module_bmi088_chip_select_t set_chip_select;       // 片选控制回调（必须）
        module_bmi088_delay_ms_t delay_ms;                 // 毫秒延时回调（必须）
        module_bmi088_get_time_us_t get_time_us;           // 微秒时间戳回调（可选，可为 NULL）
        void *user_context;                                // 回调用户上下文
        module_bmi088_accel_range_t acceleration_range;    // 加速度量程
        module_bmi088_gyro_range_t angular_velocity_range; // 陀螺仪量程
        module_bmi088_axis_map_t axis_map[3];              // 轴映射表（三个逻辑轴）
        uint32_t transfer_timeout_ms;                      // SPI 传输超时（毫秒）
    } module_bmi088_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief BMI088 设备对象
     */
    struct module_bmi088
    {
        module_device_t super;                             // 设备基类
        bsp_spi_t *spi;                                    // SPI BSP 基类
        module_bmi088_chip_select_t set_chip_select;       // 片选控制回调
        module_bmi088_delay_ms_t delay_ms;                 // 毫秒延时回调
        module_bmi088_get_time_us_t get_time_us;           // 微秒时间戳回调（可选）
        void *user_context;                                // 回调用户上下文
        module_bmi088_accel_range_t acceleration_range;    // 当前加速度量程
        module_bmi088_gyro_range_t angular_velocity_range; // 当前陀螺仪量程
        module_bmi088_axis_map_t axis_map[3];              // 轴映射表
        module_bmi088_raw_data_t raw_data;                 // 原始数据缓存
        module_bmi088_data_t data;                         // 物理量数据缓存
        float acceleration_scale_m_per_s2;                 // 加速度换算因子（LSB → m/s²）
        float angular_velocity_scale_rad_per_s;            // 角速度换算因子（LSB → rad/s）
        float angular_velocity_bias_rad_per_s[3];          // 陀螺仪零偏（rad/s）
        uint32_t transfer_timeout_ms;                      // SPI 传输超时
        bool has_timestamp;                                // 是否已有有效时间戳（用于计算间隔）
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 BMI088 设备
     * @param me 设备对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_bmi088_status_t module_bmi088_init(module_bmi088_t *const me,
                                              const module_bmi088_config_t *const config);

    /**
     * @brief 重新配置量程（无需重新初始化）
     * @param me 设备对象
     * @param acceleration_range 加速度量程
     * @param angular_velocity_range 陀螺仪量程
     * @return 执行状态
     */
    module_bmi088_status_t
    module_bmi088_configure(module_bmi088_t *const me,
                            module_bmi088_accel_range_t acceleration_range,
                            module_bmi088_gyro_range_t angular_velocity_range);

    /**
     * @brief 读取一次传感器数据（加速度、角速度、温度）
     * @param me 设备对象
     * @return 执行状态
     * @note 读取成功后数据存放在对象内部，通过 get_data / get_raw_data 获取
     */
    module_bmi088_status_t module_bmi088_read(module_bmi088_t *const me);

    /**
     * @brief 执行自检（加速度计和陀螺仪）
     * @param me 设备对象
     * @return 执行状态
     * @note 自检会临时修改量程配置，完成后恢复
     */
    module_bmi088_status_t module_bmi088_run_self_test(module_bmi088_t *const me);

    /**
     * @brief 陀螺仪零偏标定（静止状态下）
     * @param me 设备对象
     * @param sample_count 采样次数
     * @param sample_interval_ms 采样间隔（毫秒）
     * @param maximum_stationary_deviation 最大允许偏差（rad/s），超过则认为有运动
     * @return 执行状态
     * @note 标定成功后零偏自动补偿到 data.angular_velocity_rad_per_s
     */
    module_bmi088_status_t module_bmi088_calibrate_gyroscope(module_bmi088_t *const me,
                                                             uint32_t sample_count,
                                                             uint32_t sample_interval_ms,
                                                             float maximum_stationary_deviation);

    /**
     * @brief 手动设置陀螺仪零偏
     * @param me 设备对象
     * @param angular_velocity_bias_rad_per_s 三维零偏数组
     * @return 执行状态
     */
    module_bmi088_status_t
    module_bmi088_set_gyroscope_bias(module_bmi088_t *const me,
                                     const float angular_velocity_bias_rad_per_s[3]);

    /**
     * @brief 获取物理量数据指针
     * @param me 设备对象
     * @return 数据指针（若未初始化或对象为空则返回 NULL）
     */
    const module_bmi088_data_t *module_bmi088_get_data(const module_bmi088_t *const me);

    /**
     * @brief 获取原始 AD 计数值指针
     * @param me 设备对象
     * @return 原始数据指针（若未初始化或对象为空则返回 NULL）
     */
    const module_bmi088_raw_data_t *module_bmi088_get_raw_data(const module_bmi088_t *const me);

    /**
     * @brief 获取 module_device_t 基类指针
     * @param me 设备对象
     * @return 基类指针
     */
    module_device_t *module_bmi088_as_device(module_bmi088_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BMI088_H */