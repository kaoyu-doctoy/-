#include "pid.h"
#include "../app/app_config.h"

/* 初始化 PID 参数、限幅范围和运行状态。 */
void PID_Init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->integral = 0;
    pid->lastError = 0;
    pid->integralLimit = APP_PID_INTEGRAL_LIMIT;
    pid->outputMin = APP_PID_OUTPUT_MIN;
    pid->outputMax = APP_PID_OUTPUT_MAX;
}

/* 设置新的 PID 参数，并清空历史积分和误差。 */
void PID_SetParam(pid_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_Reset(pid);
}

/* 更新 PID 目标值。 */
void PID_SetTarget(pid_t *pid, int32_t target)
{
    if (pid != 0)
    {
        pid->target = target;
    }
}

/* 清除 PID 的积分项和上一次误差。 */
void PID_Reset(pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0;
    pid->lastError = 0;
}

/* 执行一次 PID 计算，并在输出未饱和时更新积分项。 */
int32_t PID_Update(pid_t *pid, int32_t feedback)
{
    int32_t error;
    int32_t derivative;
    int32_t integralCandidate;
    int32_t output;

    if (pid == 0)
    {
        return 0;
    }

    error = pid->target - feedback;
    derivative = error - pid->lastError;
    integralCandidate = pid->integral + error;

    if (integralCandidate > pid->integralLimit)
    {
        integralCandidate = pid->integralLimit;
    }
    else if (integralCandidate < -pid->integralLimit)
    {
        integralCandidate = -pid->integralLimit;
    }

    output = ((pid->kp * error) + (pid->ki * integralCandidate) + (pid->kd * derivative)) / APP_PID_GAIN_DEN;

    if (output > pid->outputMax)
    {
        output = pid->outputMax;
    }
    else if (output < pid->outputMin)
    {
        output = pid->outputMin;
    }
    else
    {
        pid->integral = integralCandidate;
    }

    pid->lastError = error;
    return output;
}
