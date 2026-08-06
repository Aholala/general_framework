/**
 * @file    task_shooter.c
 * @brief   发射机构任务 — FreeRTOS 调度适配器
 * @note    负责: 摩擦轮速度闭环 (PID) + 拨弹盘位置步进 (MIT 控制)
 *          + 卡弹检测与自动恢复 + 手动退弹保险
 */

#include "task_shooter.h"       // 任务入口声明 (给 freertos.c 用)

#include "app_shooter.h"        // ECF 框架: 发射机构状态机 + 摩擦轮 PID
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  发射机构任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_SHOOTER_PERIOD_MS (默认 5ms = 200Hz)
 */
void task_shooter_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        /* 调用 ECF 框架的发射机构更新:
         *   摩擦轮速度闭环 → 拨弹盘位置步进控制 → 卡弹检测/恢复 */
        app_shooter_update((float)APP_SHOOTER_PERIOD_MS * 0.001F);

        wake_tick += APP_SHOOTER_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
