/**
 * @file    task_safety.c
 * @brief   安全监控任务 — FreeRTOS 调度适配器
 * @note    低频率轮询 (100Hz): 看门狗喂狗, 遥控失联检测, 电机健康检查,
 *          裁判系统在线检测, SafeTask_c 链表轮询并触发失联回调。
 */

#include "task_safety.h"        // 任务入口声明 (给 freertos.c 用)

#include "app_safety.h"         // ECF 框架: 安全监控逻辑 (链表轮询 + 回调)
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  安全监控任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_SAFETY_DEFAULT_TASK_PERIOD_MS (默认 10ms = 100Hz)
 *         安全任务频率不需要高，但必须保证在失联阈值内至少轮询一次。
 */
void task_safety_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        /* 调用 ECF 框架的安全监控:
         *   链表轮询 → 失联时间累计 → 触发失联回调 (如: 电机失能) */
        app_safety_process();

        wake_tick += APP_SAFETY_DEFAULT_TASK_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
