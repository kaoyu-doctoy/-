#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "../bsp/bsp_motor.h"

typedef struct
{
    /* 底盘 X 方向速度，单位 m/s。 */
    float vx;
    /* 底盘 Y 方向速度，单位 m/s。 */
    float vy;
    /* 底盘角速度，单位 rad/s。 */
    float wz;
} chassis_velocity_t;

typedef struct
{
    /* 四个轮子的目标速度，单位 count/s。 */
    int32_t targetCps[MOTOR_COUNT];
    /* 四个轮子的实际速度，单位 count/s。 */
    int32_t actualCps[MOTOR_COUNT];
    /* 四个轮子的当前输出占空比，带符号表示方向。 */
    int16_t outputDuty[MOTOR_COUNT];
} chassis_status_t;

/* 初始化底盘控制状态和四路速度 PID。 */
void Chassis_Init(void);

/* 设置麦轮底盘速度目标，输入为 vx/vy/wz。 */
void Chassis_SetVelocity(float vx, float vy, float wz);

/* 兼容旧车模命令：用速度和舵角近似生成底盘速度目标。 */
void Chassis_SetLegacyCar(float speedMps, float steerDeg);

/* 直接设置单个轮子的目标速度，单位 count/s。 */
void Chassis_SetWheelTarget(motor_id_t motor, int32_t targetCps);

/* 直接设置单个电机的手动带符号占空比。 */
void Chassis_SetManualDuty(motor_id_t motor, int16_t signedDuty);

/* 手动设置单个电机方向，供调试或协议命令使用。 */
void Chassis_SetManualDirection(motor_id_t motor, int8_t direction);

/* 周期执行四轮速度闭环控制，并刷新电机输出。 */
void Chassis_UpdateControl(void);

/* 停止底盘运动并清零四轮目标。 */
void Chassis_Stop(void);

/* 设置单个轮子的 PID 参数。 */
void Chassis_SetPid(motor_id_t motor, int32_t kp, int32_t ki, int32_t kd);

/* 设置全部轮子的 PID 参数。 */
void Chassis_SetAllPid(int32_t kp, int32_t ki, int32_t kd);

/* 获取当前底盘目标、反馈和输出状态。 */
void Chassis_GetStatus(chassis_status_t *status);

/* 将轮子行驶距离换算为编码器计数，用于按格子距离自动行驶。 */
int32_t Chassis_DistanceMToEncoderCounts(float distanceM);

/* 读取四个轮子的平均绝对累计计数，用于判断一段直线是否完成。 */
int32_t Chassis_GetAverageAbsEncoderCounts(void);

#endif
