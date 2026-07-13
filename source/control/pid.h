#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t target;
    int32_t integral;
    int32_t lastError;
    int32_t integralLimit;
    int32_t outputMin;
    int32_t outputMax;
} pid_t;

/* 初始化 PID 控制器参数和运行状态。 */
void PID_Init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd);

/* 修改 PID 的 kp、ki、kd 参数并保留限幅设置。 */
void PID_SetParam(pid_t *pid, int32_t kp, int32_t ki, int32_t kd);

/* 设置 PID 控制目标值。 */
void PID_SetTarget(pid_t *pid, int32_t target);

/* 清空 PID 积分项和上一次误差。 */
void PID_Reset(pid_t *pid);

/* 根据反馈值计算一次 PID 输出。 */
int32_t PID_Update(pid_t *pid, int32_t feedback);

#endif
