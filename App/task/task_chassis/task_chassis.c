/**
 * @file    task_chassis.c
 * @brief   底盘控制任务 — FreeRTOS 调度适配器
 * @note    这是一个纯调度层文件。它只做三件事:
 *          1. 按固定周期唤醒
 *          2. 调用 ECF/App/app_chassis 的 app_chassis_update()
 *          3. 不包含任何控制逻辑、PID、运动学 — 那些在 ECF/App/ 层
 */

#include "task_chassis.h"       // 任务入口声明 (给 freertos.c 用)

#include "app_chassis.h"        // ECF 框架: 底盘控制更新函数
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2: osDelayUntil 等 API

/**
 * @brief  底盘任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   使用 osDelayUntil 保证周期精确，不会因执行时间累积漂移。
 *         周期由 APP_CHASSIS_PERIOD_MS 定义 (默认 5ms = 200Hz)。
 */
void task_chassis_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick 作为基准
    (void)argument;                                     // 参数未使用，避免编译警告

    for (;;)
    {
        /* 将毫秒转为秒, 调用 ECF 框架的底盘更新函数 */
        app_chassis_update((float)APP_CHASSIS_PERIOD_MS * 0.001F);

        /* 绝对延时: 保证周期精确, 不受本周期执行时间影响 */
        wake_tick += APP_CHASSIS_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
