#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t angleCentiDeg[3];
    int16_t relativeYawCentiDeg;
    bool initialized;
    bool calibrated;
} imu_status_t;

/* 初始化软件 I2C 和 IMU 芯片，成功返回 true。 */
bool IMU_Init(void);

/* 周期读取陀螺仪数据并积分更新角度。 */
bool IMU_Update(void);

/* 静止采样陀螺仪零偏，用于减小角度漂移。 */
void IMU_Calibrate(void);

/* 将当前偏航角作为零点，清零相对角度。 */
void IMU_ResetYaw(void);

/* 开关 IMU 状态串口打印。 */
void IMU_SetPrintEnabled(bool enable);

/* 查询 IMU 状态串口打印是否开启。 */
bool IMU_IsPrintEnabled(void);

/* 获取相对偏航角，单位为 0.01 度。 */
int16_t IMU_GetRelativeYawCentiDeg(void);

/* 计算目标偏航角与当前偏航角的最短误差，单位为 0.01 度。 */
int16_t IMU_GetYawErrorCentiDeg(int16_t targetCentiDeg);

/* 复制当前 IMU 状态到调用者提供的结构体。 */
void IMU_GetStatus(imu_status_t *status);

/* 通过调试串口打印当前 IMU 角度状态。 */
void IMU_PrintStatus(void);

#endif
