#include "imu.h"
#include "../app/app_config.h"
#include "../bsp/bsp_soft_i2c.h"

#include "fsl_debug_console.h"

static volatile int16_t s_angleCentiDeg[3];
static volatile int16_t s_yawZeroCentiDeg;
static volatile bool s_initialized;
static volatile bool s_calibrated;
static volatile bool s_printEnabled;
static int32_t s_offsetSum[3];
static int16_t s_offsetRaw[3];
static uint8_t s_calCount;

/* 将 IMU 连续两个高低字节转换为有符号原始数据。 */
static int16_t IMU_DecodeRaw(const uint8_t *buffer)
{
    return (int16_t)((((uint16_t)buffer[0]) << 8U) | (uint16_t)buffer[1]);
}

/* 将角度规范到 -180.00 度到 180.00 度范围内。 */
static int16_t IMU_NormalizeCentiDeg(int32_t angleCentiDeg)
{
    while (angleCentiDeg > 18000)
    {
        angleCentiDeg -= 36000;
    }
    while (angleCentiDeg < -18000)
    {
        angleCentiDeg += 36000;
    }

    return (int16_t)angleCentiDeg;
}

/* 计算两个角度之间的最短角度差。 */
static int16_t IMU_AngleDiffCentiDeg(int16_t targetCentiDeg, int16_t currentCentiDeg)
{
    return IMU_NormalizeCentiDeg((int32_t)targetCentiDeg - (int32_t)currentCentiDeg);
}

/* 以整数度形式输出 0.01 度单位的角度值。 */
static void IMU_PrintDeg(int16_t value)
{
    int32_t temp = value;

    if (temp < 0)
    {
        PUTCHAR('-');
        temp = -temp;
    }

    PRINTF("%u", (uint32_t)(temp / 100L));
}

/* 初始化软件 I2C、启动 IMU 并开始零偏校准流程。 */
bool IMU_Init(void)
{
    BSP_SoftI2C_Init();
    IMU_Calibrate();
    s_initialized = BSP_SoftI2C_WriteReg(APP_IMU_I2C_ADDR_7BIT, APP_IMU_REG_PWR_MGMT0, 0x0FU);
    return s_initialized;
}

/* 读取陀螺仪原始数据，完成校准采样或积分更新三轴角度。 */
bool IMU_Update(void)
{
    uint8_t buffer[6];
    uint8_t axis;
    int32_t gyro;

    if (!s_initialized)
    {
        s_initialized = BSP_SoftI2C_WriteReg(APP_IMU_I2C_ADDR_7BIT, APP_IMU_REG_PWR_MGMT0, 0x0FU);
        return false;
    }

    if (!BSP_SoftI2C_ReadRegs(APP_IMU_I2C_ADDR_7BIT, APP_IMU_REG_GYRO_DATA_X1, buffer, sizeof(buffer)))
    {
        return false;
    }

    if (s_calCount < APP_IMU_CALIBRATION_SAMPLES)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            s_offsetSum[axis] += (int32_t)IMU_DecodeRaw(&buffer[axis * 2U]);
        }

        s_calCount++;
        if (s_calCount >= APP_IMU_CALIBRATION_SAMPLES)
        {
            for (axis = 0U; axis < 3U; axis++)
            {
                s_offsetRaw[axis] = (int16_t)(s_offsetSum[axis] / (int32_t)APP_IMU_CALIBRATION_SAMPLES);
                s_angleCentiDeg[axis] = 0;
            }
            s_yawZeroCentiDeg = 0;
            s_calibrated = true;
        }
        return true;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        gyro = (int32_t)IMU_DecodeRaw(&buffer[axis * 2U]) - (int32_t)s_offsetRaw[axis];
        s_angleCentiDeg[axis] = IMU_NormalizeCentiDeg((int32_t)s_angleCentiDeg[axis] +
                                                      ((gyro * (int32_t)APP_IMU_PERIOD_MS) / 164L));
    }

    return true;
}

/* 重启零偏校准流程，并清空当前角度积分。 */
void IMU_Calibrate(void)
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++)
    {
        s_offsetSum[axis] = 0;
        s_offsetRaw[axis] = 0;
        s_angleCentiDeg[axis] = 0;
    }

    s_calCount = 0U;
    s_yawZeroCentiDeg = 0;
    s_calibrated = false;
}

/* 将当前 Z 轴角度记录为偏航零点。 */
void IMU_ResetYaw(void)
{
    s_yawZeroCentiDeg = s_angleCentiDeg[2];
}

/* 设置是否允许 IMU 任务周期打印角度。 */
void IMU_SetPrintEnabled(bool enable)
{
    s_printEnabled = enable;
}

/* 返回 IMU 周期打印开关状态。 */
bool IMU_IsPrintEnabled(void)
{
    return s_printEnabled;
}

/* 返回相对零点的偏航角，单位 0.01 度。 */
int16_t IMU_GetRelativeYawCentiDeg(void)
{
    return IMU_NormalizeCentiDeg((int32_t)s_angleCentiDeg[2] - (int32_t)s_yawZeroCentiDeg);
}

/* 返回目标偏航角与当前偏航角之间的误差。 */
int16_t IMU_GetYawErrorCentiDeg(int16_t targetCentiDeg)
{
    return IMU_AngleDiffCentiDeg(targetCentiDeg, IMU_GetRelativeYawCentiDeg());
}

/* 复制 IMU 当前角度、初始化状态和校准状态。 */
void IMU_GetStatus(imu_status_t *status)
{
    uint8_t axis;

    if (status == 0)
    {
        return;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        status->angleCentiDeg[axis] = s_angleCentiDeg[axis];
    }

    status->relativeYawCentiDeg = IMU_GetRelativeYawCentiDeg();
    status->initialized = s_initialized;
    status->calibrated = s_calibrated;
}

/* 按旧协议格式通过调试串口输出三轴角度。 */
void IMU_PrintStatus(void)
{
    PRINTF("gyro:");
    IMU_PrintDeg(s_angleCentiDeg[0]);
    PUTCHAR(',');
    IMU_PrintDeg(s_angleCentiDeg[1]);
    PUTCHAR(',');
    IMU_PrintDeg(IMU_GetRelativeYawCentiDeg());
    PUTCHAR('\n');
}
