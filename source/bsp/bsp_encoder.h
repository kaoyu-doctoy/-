#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include "bsp_motor.h"

/* 初始化四路带方向增量式编码器的 QTIMER 计数功能。 */
void BSP_EncoderInit(void);

/* 读取指定电机对应编码器的累计计数值。 */
int32_t BSP_EncoderGetCount(motor_id_t motor);

/* 读取指定编码器自上次调用以来的计数变化量。 */
int32_t BSP_EncoderGetDelta(motor_id_t motor);

/* 读取指定编码器的滤波后速度，单位为 count/s。 */
int32_t BSP_EncoderGetSpeedCps(motor_id_t motor);

/* 清零指定编码器的计数和速度缓存。 */
void BSP_EncoderClear(motor_id_t motor);

/* 清零全部编码器的计数和速度缓存。 */
void BSP_EncoderClearAll(void);

#endif
