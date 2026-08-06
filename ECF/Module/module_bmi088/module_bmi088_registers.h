/**
 * @file module_bmi088_registers.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BMI088 加速度计和陀螺仪的寄存器地址及常量定义
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 */

#ifndef MODULE_BMI088_REGISTERS_H
#define MODULE_BMI088_REGISTERS_H

/* ==================== 加速度计寄存器 ==================== */

/** @brief 加速度计 Chip ID 寄存器地址 */
#define MODULE_BMI088_ACCEL_CHIP_ID_REGISTER (0x00U)
/** @brief 加速度计 Chip ID 期望值（0x1E） */
#define MODULE_BMI088_ACCEL_CHIP_ID_VALUE (0x1EU)
/** @brief 加速度计数据寄存器起始地址（X/Y/Z 各 2 字节，共 6 字节） */
#define MODULE_BMI088_ACCEL_DATA_REGISTER (0x12U)
/** @brief 加速度计温度寄存器地址（2 字节） */
#define MODULE_BMI088_ACCEL_TEMPERATURE_REGISTER (0x22U)
/** @brief 加速度计配置寄存器（输出数据速率、滤波器带宽） */
#define MODULE_BMI088_ACCEL_CONFIG_REGISTER (0x40U)
/** @brief 加速度计量程寄存器 */
#define MODULE_BMI088_ACCEL_RANGE_REGISTER (0x41U)
/** @brief 加速度计中断 IO 配置寄存器 */
#define MODULE_BMI088_ACCEL_INTERRUPT_IO_REGISTER (0x53U)
/** @brief 加速度计中断映射寄存器 */
#define MODULE_BMI088_ACCEL_INTERRUPT_MAP_REGISTER (0x58U)
/** @brief 加速度计自检寄存器 */
#define MODULE_BMI088_ACCEL_SELF_TEST_REGISTER (0x6DU)
/** @brief 加速度计电源配置寄存器（低功耗/普通模式） */
#define MODULE_BMI088_ACCEL_POWER_CONFIG_REGISTER (0x7CU)
/** @brief 加速度计电源控制寄存器（开启/关闭） */
#define MODULE_BMI088_ACCEL_POWER_CONTROL_REGISTER (0x7DU)
/** @brief 加速度计软复位寄存器 */
#define MODULE_BMI088_ACCEL_SOFT_RESET_REGISTER (0x7EU)

/* ==================== 加速度计常量 ==================== */

/** @brief 加速度计软复位命令（0xB6） */
#define MODULE_BMI088_ACCEL_SOFT_RESET_COMMAND (0xB6U)
/** @brief 加速度计电源开启值（0x04，使能所有轴） */
#define MODULE_BMI088_ACCEL_POWER_ON (0x04U)
/** @brief 加速度计普通模式（非低功耗） */
#define MODULE_BMI088_ACCEL_ACTIVE_MODE (0x00U)
/** @brief 加速度计配置：普通模式，800Hz ODR，滤波器带宽 0.5x ODR */
#define MODULE_BMI088_ACCEL_CONFIG_NORMAL_800_HZ (0xAAU)
/** @brief 加速度计中断输出使能（推挽输出） */
#define MODULE_BMI088_ACCEL_INTERRUPT_OUTPUT_ENABLE (0x08U)
/** @brief 加速度计中断映射：数据就绪映射到 INT1 */
#define MODULE_BMI088_ACCEL_INTERRUPT_MAP_DATA_READY (0x04U)
/** @brief 加速度计自检关闭值 */
#define MODULE_BMI088_ACCEL_SELF_TEST_OFF (0x00U)
/** @brief 加速度计自检正方向（正电压激励） */
#define MODULE_BMI088_ACCEL_SELF_TEST_POSITIVE (0x0DU)
/** @brief 加速度计自检负方向（负电压激励） */
#define MODULE_BMI088_ACCEL_SELF_TEST_NEGATIVE (0x09U)

/* ==================== 陀螺仪寄存器 ==================== */

/** @brief 陀螺仪 Chip ID 寄存器地址 */
#define MODULE_BMI088_GYRO_CHIP_ID_REGISTER (0x00U)
/** @brief 陀螺仪 Chip ID 期望值（0x0F） */
#define MODULE_BMI088_GYRO_CHIP_ID_VALUE (0x0FU)
/** @brief 陀螺仪数据寄存器起始地址（X/Y/Z 各 2 字节，共 6 字节） */
#define MODULE_BMI088_GYRO_DATA_REGISTER (0x02U)
/** @brief 陀螺仪量程寄存器 */
#define MODULE_BMI088_GYRO_RANGE_REGISTER (0x0FU)
/** @brief 陀螺仪带宽寄存器 */
#define MODULE_BMI088_GYRO_BANDWIDTH_REGISTER (0x10U)
/** @brief 陀螺仪电源寄存器 */
#define MODULE_BMI088_GYRO_POWER_REGISTER (0x11U)
/** @brief 陀螺仪软复位寄存器 */
#define MODULE_BMI088_GYRO_SOFT_RESET_REGISTER (0x14U)
/** @brief 陀螺仪中断控制寄存器 */
#define MODULE_BMI088_GYRO_INTERRUPT_CONTROL_REGISTER (0x15U)
/** @brief 陀螺仪中断 IO 配置寄存器 */
#define MODULE_BMI088_GYRO_INTERRUPT_IO_REGISTER (0x16U)
/** @brief 陀螺仪中断映射寄存器 */
#define MODULE_BMI088_GYRO_INTERRUPT_MAP_REGISTER (0x18U)
/** @brief 陀螺仪自检寄存器 */
#define MODULE_BMI088_GYRO_SELF_TEST_REGISTER (0x3CU)

/* ==================== 陀螺仪常量 ==================== */

/** @brief 陀螺仪软复位命令（0xB6） */
#define MODULE_BMI088_GYRO_SOFT_RESET_COMMAND (0xB6U)
/** @brief 陀螺仪普通模式（非睡眠） */
#define MODULE_BMI088_GYRO_NORMAL_MODE (0x00U)
/** @brief 陀螺仪带宽配置：2000Hz ODR，230Hz 滤波器（0x81） */
#define MODULE_BMI088_GYRO_BANDWIDTH_2000_230_HZ (0x81U)
/** @brief 陀螺仪数据就绪中断使能（0x80） */
#define MODULE_BMI088_GYRO_DATA_READY_ENABLE (0x80U)
/** @brief 陀螺仪中断推挽输出，低电平有效（0x00） */
#define MODULE_BMI088_GYRO_INTERRUPT_PUSH_PULL_ACTIVE_LOW (0x00U)
/** @brief 陀螺仪中断映射：数据就绪映射到 INT3（0x01） */
#define MODULE_BMI088_GYRO_INTERRUPT_MAP_DATA_READY_INT3 (0x01U)
/** @brief 陀螺仪自检触发命令（0x01） */
#define MODULE_BMI088_GYRO_SELF_TEST_TRIGGER (0x01U)
/** @brief 陀螺仪自检就绪标志位 */
#define MODULE_BMI088_GYRO_SELF_TEST_READY (0x02U)
/** @brief 陀螺仪自检失败标志位 */
#define MODULE_BMI088_GYRO_SELF_TEST_FAILED (0x04U)

/* ==================== SPI 通用常量 ==================== */

/** @brief SPI 读操作位（寄存器地址最高位为 1 表示读） */
#define MODULE_BMI088_SPI_READ_BIT (0x80U)

#endif /* MODULE_BMI088_REGISTERS_H */