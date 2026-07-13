#include "bsp_encoder.h"
#include "../app/app_config.h"

#include "fsl_qtmr.h"

#define ENCODER_COUNTER_CHANNEL (kQTMR_Channel_0)
#define ENCODER_PRIMARY_SOURCE  (kQTMR_ClockCounter0InputPin)
#define ENCODER_SECONDARY_SOURCE (kQTMR_Counter1InputPin)
#define ENCODER_COUNT_MODE (kQTMR_PriSrcRiseEdgeSecDir)

static TMR_Type *const s_encoderBase[MOTOR_COUNT] = {TMR1, TMR2, TMR3, TMR4};
static uint16_t s_lastCount[MOTOR_COUNT];
static int32_t s_lastDelta[MOTOR_COUNT];
static int32_t s_speedCps[MOTOR_COUNT];
static int32_t s_totalCount[MOTOR_COUNT];

/* 检查电机编号是否在四轮范围内。 */
static bool Encoder_IsValid(motor_id_t motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT);
}

/* 初始化单路 QTIMER 为“脉冲 + 方向”编码器计数模式。 */
static void Encoder_InitOne(TMR_Type *base)
{
    qtmr_config_t config;

    QTMR_GetDefaultConfig(&config);
    config.primarySource = ENCODER_PRIMARY_SOURCE;
    config.secondarySource = ENCODER_SECONDARY_SOURCE;

    QTMR_Init(base, ENCODER_COUNTER_CHANNEL, &config);
    QTMR_SetLoadValue(base, ENCODER_COUNTER_CHANNEL, 0U);
    base->CHANNEL[ENCODER_COUNTER_CHANNEL].CNTR = 0U;
    QTMR_ClearStatusFlags(base, ENCODER_COUNTER_CHANNEL,
                          kQTMR_CompareFlag | kQTMR_Compare1Flag | kQTMR_Compare2Flag |
                              kQTMR_OverflowFlag | kQTMR_EdgeFlag);
    QTMR_StartTimer(base, ENCODER_COUNTER_CHANNEL, ENCODER_COUNT_MODE);
}

/* 初始化四个轮子的编码器计数器和速度缓存。 */
void BSP_EncoderInit(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        Encoder_InitOne(s_encoderBase[index]);
        s_lastCount[index] = (uint16_t)QTMR_GetCurrentTimerCount(s_encoderBase[index], ENCODER_COUNTER_CHANNEL);
        s_lastDelta[index] = 0;
        s_speedCps[index] = 0;
        s_totalCount[index] = 0;
    }
}

/* 返回指定编码器的软件累计计数。 */
int32_t BSP_EncoderGetCount(motor_id_t motor)
{
    if (!Encoder_IsValid(motor))
    {
        return 0;
    }

    return s_totalCount[motor];
}

/* 计算指定编码器从上次读取到现在的增量。 */
int32_t BSP_EncoderGetDelta(motor_id_t motor)
{
    uint16_t currentCount;
    int32_t delta;

    if (!Encoder_IsValid(motor))
    {
        return 0;
    }

    currentCount = (uint16_t)QTMR_GetCurrentTimerCount(s_encoderBase[motor], ENCODER_COUNTER_CHANNEL);
    delta = (int16_t)(currentCount - s_lastCount[motor]);
    s_lastCount[motor] = currentCount;
    s_lastDelta[motor] = delta;
    s_totalCount[motor] += delta;

    return delta;
}

/* 根据控制周期内的计数增量估算并滤波速度，单位 count/s。 */
int32_t BSP_EncoderGetSpeedCps(motor_id_t motor)
{
    int32_t delta;
    int32_t instantSpeed;

    if (!Encoder_IsValid(motor))
    {
        return 0;
    }

    delta = BSP_EncoderGetDelta(motor);
    instantSpeed = (delta * 1000L) / (int32_t)APP_CONTROL_PERIOD_MS;
    s_speedCps[motor] = (int32_t)((((int64_t)s_speedCps[motor] *
                                    (APP_SPEED_FILTER_DEN - APP_SPEED_FILTER_NEW_WEIGHT)) +
                                   ((int64_t)instantSpeed * APP_SPEED_FILTER_NEW_WEIGHT) +
                                   (APP_SPEED_FILTER_DEN / 2L)) /
                                  APP_SPEED_FILTER_DEN);
    return s_speedCps[motor];
}

/* 清零单个编码器的硬件计数和软件缓存。 */
void BSP_EncoderClear(motor_id_t motor)
{
    if (!Encoder_IsValid(motor))
    {
        return;
    }

    s_encoderBase[motor]->CHANNEL[ENCODER_COUNTER_CHANNEL].CNTR = 0U;
    s_lastCount[motor] = 0U;
    s_lastDelta[motor] = 0;
    s_speedCps[motor] = 0;
    s_totalCount[motor] = 0;
}

/* 清零全部编码器。 */
void BSP_EncoderClearAll(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        BSP_EncoderClear((motor_id_t)index);
    }
}
