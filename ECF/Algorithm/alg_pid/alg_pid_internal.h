/**
 * @file alg_pid_internal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief PID 库内部工具函数头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 仅供本模块内部使用，不对外暴露。
 */

#ifndef ALG_PID_INTERNAL_H
#define ALG_PID_INTERNAL_H

#include "alg_pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 检查浮点数是否有限
     * @param value 要检查的值
     * @return true 表示有限
     */
    bool alg_pid_internal_is_finite(float value);

    /**
     * @brief 限幅函数
     * @param value     输入值
     * @param minimum   下限
     * @param maximum   上限
     * @return 限幅后的值
     */
    float alg_pid_internal_clamp(float value, float minimum, float maximum);

    /**
     * @brief 应用死区
     * @param value    输入值
     * @param deadband 死区宽度（>=0）
     * @return 若 |value| <= deadband 返回 0，否则返回原值
     */
    float alg_pid_internal_apply_deadband(float value, float deadband);

    /**
     * @brief 验证位置式 PID 配置的有效性
     * @param config 配置指针
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_internal_validate_config(const alg_pid_config_t *config);

#ifdef __cplusplus
}
#endif

#endif