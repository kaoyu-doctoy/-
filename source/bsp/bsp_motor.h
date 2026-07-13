#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    /* 左前轮电机。 */
    MOTOR_LF = 0,
    /* 右前轮电机。 */
    MOTOR_RF = 1,
    /* 左后轮电机。 */
    MOTOR_LB = 2,
    /* 右后轮电机。 */
    MOTOR_RB = 3,
    /* 电机总数，用于数组长度和边界检查。 */
    MOTOR_COUNT = 4
} motor_id_t;

/* 初始化四路 DRV8701E PH/EN 电机驱动相关 GPIO 和 PWM。 */
void BSP_MotorInit(void);

/* 统一使能或关闭电机驱动的 SLEEP/ENABLE 控制。 */
void BSP_MotorEnable(bool enable);

/* 设置指定电机的 PWM 占空比，范围 0 到 100。 */
void BSP_MotorSetDuty(motor_id_t motor, uint16_t duty);

/* 设置指定电机方向，direction 大于等于 0 为正向，小于 0 为反向。 */
void BSP_MotorSetDirection(motor_id_t motor, int8_t direction);

/* 用带符号占空比同时设置电机方向和 PWM 输出。 */
void BSP_MotorSetSignedDuty(motor_id_t motor, int16_t signedDuty);

/* 停止指定电机输出。 */
void BSP_MotorStop(motor_id_t motor);

/* 停止全部电机输出。 */
void BSP_MotorStopAll(void);

#endif
