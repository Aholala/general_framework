#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include "bsp_common.h"

#include <stdint.h>

/**
 * @brief 机器人顶层初始化（装配 DR16、板间通信、遥控命令）
 * @return BSP_STATUS_OK 成功，其他值表示失败
 * @note 失败时已初始化的模块会被回滚
 */
bsp_status_t app_robot_init(void);

/**
 * @brief 通信周期更新（DR16 解析 + 板间 CAN 收发）
 * @param elapsed_time_ms 距上次调用的时间（毫秒）
 */
void app_robot_communication_update(uint32_t elapsed_time_ms);

#endif
