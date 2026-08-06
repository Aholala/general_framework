/**
 * @file    task_command.c
 * @brief   命令与通信任务 — FreeRTOS 调度适配器
 * @note    每个周期内按顺序调用三个更新:
 *          1. app_robot_communication_update() — DR16 解析 + 板间 CAN 收发
 *          2. app_command_update()           — 遥控器 → 机器人命令映射
 *          3. app_vision_update()           — USB 视觉数据收发
 */

#include "task_command.h"       // 任务入口声明 (给 freertos.c 用)

#include "app_command.h"        // ECF 框架: 命令映射 (摇杆/键鼠 → 底盘/云台目标值)
#include "app_robot.h"          // ECF 框架: 顶层通信装配 (DR16 + 板间 CAN)
#include "app_vision.h"         // ECF 框架: USB CDC 视觉通信
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  命令任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_COMMAND_PERIOD_MS (默认 5ms = 200Hz)
 */
void task_command_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        /* 1. 通信更新: DR16 遥控器解析 + 板间 CAN 帧接收 */
        app_robot_communication_update(APP_COMMAND_PERIOD_MS);

        /* 2. 命令映射: 遥控器原始值 → 机器人目标速度/角度/发射 */
        app_command_update((float)APP_COMMAND_PERIOD_MS * 0.001F);

        /* 3. 视觉通信: USB CDC 收发 (mode/ID 协议) */
        app_vision_update(APP_COMMAND_PERIOD_MS);

        wake_tick += APP_COMMAND_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
