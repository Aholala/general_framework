#ifndef ALG_PID_INTERNAL_H
#define ALG_PID_INTERNAL_H

#include "alg_pid.h"

bool alg_pid_internal_is_finite(float value);
float alg_pid_internal_clamp(float value, float minimum, float maximum);
float alg_pid_internal_apply_deadband(float value, float deadband);
alg_pid_status_t alg_pid_internal_validate_config(const alg_pid_config_t *config);

#endif /* ALG_PID_INTERNAL_H */
