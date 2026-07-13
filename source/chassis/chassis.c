#include "chassis.h"
#include "../app/app_config.h"
#include "../bsp/bsp_encoder.h"
#include "../control/pid.h"

#define APP_PI_F (3.1415926535f)
#define APP_DEG_TO_RAD (APP_PI_F / 180.0f)

static pid_t s_wheelPid[MOTOR_COUNT];
static int32_t s_targetCps[MOTOR_COUNT];
static int32_t s_activeTargetCps[MOTOR_COUNT];
static int32_t s_actualCps[MOTOR_COUNT];
static int16_t s_outputDuty[MOTOR_COUNT];
static bool s_speedControlEnabled[MOTOR_COUNT];

typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
} wheel_pid_default_t;

static const wheel_pid_default_t s_wheelPidDefault[MOTOR_COUNT] = {
    {APP_PID_LF_KP_DEFAULT, APP_PID_LF_KI_DEFAULT, APP_PID_LF_KD_DEFAULT},
    {APP_PID_RF_KP_DEFAULT, APP_PID_RF_KI_DEFAULT, APP_PID_RF_KD_DEFAULT},
    {APP_PID_LB_KP_DEFAULT, APP_PID_LB_KI_DEFAULT, APP_PID_LB_KD_DEFAULT},
    {APP_PID_RB_KP_DEFAULT, APP_PID_RB_KI_DEFAULT, APP_PID_RB_KD_DEFAULT},
};

/* 将浮点数限制在指定范围内。 */
static float Chassis_ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

/* 返回 int32_t 的绝对值。 */
static int32_t Chassis_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 将轮子线速度 m/s 换算为编码器速度 count/s。 */
static int32_t Chassis_MpsToCps(float wheelSpeedMps)
{
    float cps;

    cps = (APP_MOTOR_GEAR_RATIO * APP_ENCODER_COUNTS_PER_REV * wheelSpeedMps) /
          (APP_PI_F * APP_WHEEL_DIAMETER_M);

    if (cps >= 0.0f)
    {
        return (int32_t)(cps + 0.5f);
    }
    return (int32_t)(cps - 0.5f);
}

/* 将轮子走过的实际距离换算成编码器累计计数。 */
int32_t Chassis_DistanceMToEncoderCounts(float distanceM)
{
    float counts;

    counts = (APP_MOTOR_GEAR_RATIO * APP_ENCODER_COUNTS_PER_REV * distanceM) /
             (APP_PI_F * APP_WHEEL_DIAMETER_M);

    if (counts >= 0.0f)
    {
        return (int32_t)(counts + 0.5f);
    }
    return (int32_t)(counts - 0.5f);
}

/* 读取四路编码器的平均绝对累计计数。 */
int32_t Chassis_GetAverageAbsEncoderCounts(void)
{
    int32_t total = 0;
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        total += Chassis_AbsInt32(BSP_EncoderGetCount((motor_id_t)index));
    }

    return total / (int32_t)MOTOR_COUNT;
}

/* 根据目标速度估算前馈占空比，帮助 PID 更快进入有效输出区间。 */
static int32_t Chassis_GetFeedforwardDuty(uint16_t targetSpeedCps)
{
    int32_t duty;

    if (targetSpeedCps == 0U)
    {
        return 0U;
    }

    duty = (((int32_t)targetSpeedCps * APP_PWM_DUTY_MAX) + (APP_SPEED_FEEDFORWARD_MAX_CPS / 2L)) /
           APP_SPEED_FEEDFORWARD_MAX_CPS;

    if (duty < (int32_t)APP_SPEED_MIN_ACTIVE_DUTY)
    {
        duty = APP_SPEED_MIN_ACTIVE_DUTY;
    }
    else if (duty > APP_PWM_DUTY_MAX)
    {
        duty = APP_PWM_DUTY_MAX;
    }

    return duty;
}

/* 初始化四轮目标、反馈、输出缓存和 PID 参数。 */
void Chassis_Init(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        PID_Init(&s_wheelPid[index],
                 s_wheelPidDefault[index].kp,
                 s_wheelPidDefault[index].ki,
                 s_wheelPidDefault[index].kd);
        s_targetCps[index] = 0;
        s_activeTargetCps[index] = 0;
        s_actualCps[index] = 0;
        s_outputDuty[index] = 0;
        s_speedControlEnabled[index] = false;
    }
}

/* 将底盘 vx/vy/wz 速度目标解算为四个麦轮目标速度。 */
void Chassis_SetVelocity(float vx, float vy, float wz)
{
    float halfBaseTrack;
    float rotateSpeed;
    float wheelSpeed[MOTOR_COUNT];
    uint8_t index;

    vx = Chassis_ClampFloat(vx, -APP_MAX_SPEED_MPS, APP_MAX_SPEED_MPS);
    vy = Chassis_ClampFloat(vy, -APP_MAX_SPEED_MPS, APP_MAX_SPEED_MPS);
    wz = Chassis_ClampFloat(wz, -APP_MAX_YAW_RATE_RADPS, APP_MAX_YAW_RATE_RADPS);

    halfBaseTrack = (APP_WHEEL_BASE_M + APP_TRACK_WIDTH_M) * 0.5f;
    rotateSpeed = wz * halfBaseTrack;

    wheelSpeed[MOTOR_LF] = vx - vy - rotateSpeed;
    wheelSpeed[MOTOR_RF] = vx + vy + rotateSpeed;
    wheelSpeed[MOTOR_LB] = vx + vy - rotateSpeed;
    wheelSpeed[MOTOR_RB] = vx - vy + rotateSpeed;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        Chassis_SetWheelTarget((motor_id_t)index, Chassis_MpsToCps(wheelSpeed[index]));
    }
}

/* 把旧车模“速度 + 转向角”命令转换为麦轮底盘运动目标。 */
void Chassis_SetLegacyCar(float speedMps, float steerDeg)
{
    float wz;

    steerDeg = Chassis_ClampFloat(steerDeg, -APP_LEGACY_MAX_STEER_DEG, APP_LEGACY_MAX_STEER_DEG);
    wz = (steerDeg / APP_LEGACY_MAX_STEER_DEG) * APP_MAX_YAW_RATE_RADPS;
    Chassis_SetVelocity(speedMps, 0.0f, wz);
}

/* 直接指定单个轮子的闭环速度目标。 */
void Chassis_SetWheelTarget(motor_id_t motor, int32_t targetCps)
{
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }

    if (targetCps > APP_WHEEL_MAX_TARGET_CPS)
    {
        targetCps = APP_WHEEL_MAX_TARGET_CPS;
    }
    else if (targetCps < -APP_WHEEL_MAX_TARGET_CPS)
    {
        targetCps = -APP_WHEEL_MAX_TARGET_CPS;
    }

    s_targetCps[motor] = targetCps;
    s_speedControlEnabled[motor] = true;
}

/* 进入单轮手动 PWM 模式，关闭该轮速度闭环。 */
void Chassis_SetManualDuty(motor_id_t motor, int16_t signedDuty)
{
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }

    s_speedControlEnabled[motor] = false;
    s_targetCps[motor] = 0;
    s_activeTargetCps[motor] = 0;
    PID_Reset(&s_wheelPid[motor]);
    BSP_MotorSetSignedDuty(motor, signedDuty);
}

/* 单独修改电机方向脚，主要用于接线和方向调试。 */
void Chassis_SetManualDirection(motor_id_t motor, int8_t direction)
{
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }

    BSP_MotorSetDirection(motor, direction);
}

/* 周期更新四轮速度闭环，并把计算得到的占空比输出到电机。 */
void Chassis_UpdateControl(void)
{
    int32_t currentTarget;
    int32_t deltaTarget;
    int32_t targetAbs;
    int32_t pidOutput;
    int32_t signedDuty;
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        s_actualCps[index] = BSP_EncoderGetSpeedCps((motor_id_t)index);

        if (!s_speedControlEnabled[index])
        {
            continue;
        }

        currentTarget = s_activeTargetCps[index];
        deltaTarget = s_targetCps[index] - currentTarget;
        if (deltaTarget > APP_SPEED_TARGET_RAMP_CPS_PER_TICK)
        {
            deltaTarget = APP_SPEED_TARGET_RAMP_CPS_PER_TICK;
        }
        else if (deltaTarget < -APP_SPEED_TARGET_RAMP_CPS_PER_TICK)
        {
            deltaTarget = -APP_SPEED_TARGET_RAMP_CPS_PER_TICK;
        }
        currentTarget += deltaTarget;
        s_activeTargetCps[index] = currentTarget;

        targetAbs = Chassis_AbsInt32(currentTarget);
        PID_SetTarget(&s_wheelPid[index], targetAbs);
        pidOutput = PID_Update(&s_wheelPid[index], Chassis_AbsInt32(s_actualCps[index]));
        signedDuty = (int32_t)Chassis_GetFeedforwardDuty((uint16_t)targetAbs) + pidOutput;

        if (signedDuty > APP_PWM_DUTY_MAX)
        {
            signedDuty = APP_PWM_DUTY_MAX;
        }
        else if (signedDuty < 0)
        {
            signedDuty = 0;
        }

        if (currentTarget < 0)
        {
            signedDuty = -signedDuty;
        }

        s_outputDuty[index] = (int16_t)signedDuty;
        BSP_MotorSetSignedDuty((motor_id_t)index, (int16_t)signedDuty);
    }
}

/* 停止所有闭环目标并关闭电机输出。 */
void Chassis_Stop(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        s_speedControlEnabled[index] = false;
        s_targetCps[index] = 0;
        s_activeTargetCps[index] = 0;
        PID_Reset(&s_wheelPid[index]);
    }
    BSP_MotorStopAll();
}

/* 修改指定轮子的速度 PID 参数。 */
void Chassis_SetPid(motor_id_t motor, int32_t kp, int32_t ki, int32_t kd)
{
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }

    PID_SetParam(&s_wheelPid[motor], kp, ki, kd);
}

/* 同时修改四个轮子的速度 PID 参数。 */
void Chassis_SetAllPid(int32_t kp, int32_t ki, int32_t kd)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        Chassis_SetPid((motor_id_t)index, kp, ki, kd);
    }
}

/* 读取底盘当前目标速度、实际速度和输出占空比。 */
void Chassis_GetStatus(chassis_status_t *status)
{
    uint8_t index;

    if (status == 0)
    {
        return;
    }

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        status->targetCps[index] = s_targetCps[index];
        status->actualCps[index] = s_actualCps[index];
        status->outputDuty[index] = s_outputDuty[index];
    }
}
