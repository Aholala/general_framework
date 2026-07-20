#ifndef ALG_PID_INTERNAL_H
#define ALG_PID_INTERNAL_H

#include "alg_pid.h"

bool AlgPidInternal_IsFinite(float value);
float AlgPidInternal_Clamp(float value, float minimum, float maximum);
float AlgPidInternal_ApplyDeadband(float value, float deadband);
AlgPidStatus_t AlgPidInternal_ValidateConfig(const AlgPidConfig_t *config);

#endif /* ALG_PID_INTERNAL_H */
