/**
 * @file    task_imu.c
 * @brief   IMU 采样与姿态解算任务 — FreeRTOS 调度适配器
 * @note    IMU 需要最高频率 (1kHz) 以保证姿态解算的低延迟和高精度。
 *          BMI088 SPI 读取 + Mahony/Madgwick/EKF 解算在此周期内完成。
 *          解算结果通过 app_exchange 发布给云台和底盘任务使用。
 */

#include "task_imu.h"           // 任务入口声明 (给 freertos.c 用)

#include "app_imu.h"            // ECF 框架: IMU 读取 + 姿态估计算法
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  IMU 任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_IMU_PERIOD_MS (默认 1ms = 1kHz)
 *         这是所有任务中频率最高的，保证姿态数据的实时性。
 */
void task_imu_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        /* 调用 ECF 框架的 IMU 更新:
         *   BMI088 SPI 读取 → 滤波 → 姿态解算 (Mahony/EKF) → 发布到数据交换层 */
        app_imu_update((float)APP_IMU_PERIOD_MS * 0.001F);

        wake_tick += APP_IMU_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
