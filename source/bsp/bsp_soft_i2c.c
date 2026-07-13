#include "bsp_soft_i2c.h"
#include "../app/app_config.h"

#include "fsl_common.h"
#include "fsl_gpio.h"

#define SOFT_I2C_SCL_GPIO GPIO1
#define SOFT_I2C_SCL_PIN  (23U)
#define SOFT_I2C_SDA_GPIO GPIO1
#define SOFT_I2C_SDA_PIN  (22U)

/* 软件 I2C 时序延时，具体速度由 APP_SOFT_I2C_DELAY_COUNT 控制。 */
static void SoftI2C_Delay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < APP_SOFT_I2C_DELAY_COUNT; count++)
    {
        __NOP();
    }
}

/* 释放 SCL 线为高电平，模拟开漏输出。 */
static void SoftI2C_SCL_High(void)
{
    SOFT_I2C_SCL_GPIO->GDIR &= ~(1UL << SOFT_I2C_SCL_PIN);
}

/* 主动拉低 SCL 线。 */
static void SoftI2C_SCL_Low(void)
{
    GPIO_PinWrite(SOFT_I2C_SCL_GPIO, SOFT_I2C_SCL_PIN, 0U);
    SOFT_I2C_SCL_GPIO->GDIR |= (1UL << SOFT_I2C_SCL_PIN);
}

/* 释放 SDA 线为高电平，模拟开漏输出。 */
static void SoftI2C_SDA_High(void)
{
    SOFT_I2C_SDA_GPIO->GDIR &= ~(1UL << SOFT_I2C_SDA_PIN);
}

/* 主动拉低 SDA 线。 */
static void SoftI2C_SDA_Low(void)
{
    GPIO_PinWrite(SOFT_I2C_SDA_GPIO, SOFT_I2C_SDA_PIN, 0U);
    SOFT_I2C_SDA_GPIO->GDIR |= (1UL << SOFT_I2C_SDA_PIN);
}

/* 读取 SDA 当前电平。 */
static bool SoftI2C_ReadSDA(void)
{
    return (GPIO_PinReadPadStatus(SOFT_I2C_SDA_GPIO, SOFT_I2C_SDA_PIN) != 0U);
}

/* 产生 I2C 起始条件。 */
static void SoftI2C_Start(void)
{
    SoftI2C_SDA_High();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
    SoftI2C_Delay();
}

/* 产生 I2C 停止条件。 */
static void SoftI2C_Stop(void)
{
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_High();
    SoftI2C_Delay();
}

/* 写入 1 字节并读取从机 ACK，返回 true 表示收到 ACK。 */
static bool SoftI2C_WriteByte(uint8_t data)
{
    uint8_t bit;
    bool ack;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((data & 0x80U) != 0U)
        {
            SoftI2C_SDA_High();
        }
        else
        {
            SoftI2C_SDA_Low();
        }

        SoftI2C_Delay();
        SoftI2C_SCL_High();
        SoftI2C_Delay();
        SoftI2C_SCL_Low();
        data <<= 1U;
    }

    SoftI2C_SDA_High();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    ack = !SoftI2C_ReadSDA();
    SoftI2C_SCL_Low();
    SoftI2C_Delay();

    return ack;
}

/* 读取 1 字节，ack 为 true 时读取后向从机回应 ACK。 */
static uint8_t SoftI2C_ReadByte(bool ack)
{
    uint8_t bit;
    uint8_t data = 0U;

    SoftI2C_SDA_High();
    for (bit = 0U; bit < 8U; bit++)
    {
        data <<= 1U;
        SoftI2C_SCL_High();
        SoftI2C_Delay();
        if (SoftI2C_ReadSDA())
        {
            data |= 1U;
        }
        SoftI2C_SCL_Low();
        SoftI2C_Delay();
    }

    if (ack)
    {
        SoftI2C_SDA_Low();
    }
    else
    {
        SoftI2C_SDA_High();
    }

    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
    SoftI2C_SDA_High();
    SoftI2C_Delay();

    return data;
}

/* 初始化软件 I2C 的 GPIO，并释放总线到空闲高电平。 */
void BSP_SoftI2C_Init(void)
{
    gpio_pin_config_t inputConfig = {kGPIO_DigitalInput, 0U, kGPIO_NoIntmode};

    GPIO_PinInit(SOFT_I2C_SCL_GPIO, SOFT_I2C_SCL_PIN, &inputConfig);
    GPIO_PinInit(SOFT_I2C_SDA_GPIO, SOFT_I2C_SDA_PIN, &inputConfig);
    SoftI2C_SCL_High();
    SoftI2C_SDA_High();
}

/* 对指定 I2C 设备执行寄存器写操作。 */
bool BSP_SoftI2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    bool ok;

    SoftI2C_Start();
    ok = SoftI2C_WriteByte((uint8_t)(devAddr << 1U)) && SoftI2C_WriteByte(regAddr) &&
         SoftI2C_WriteByte(data);
    SoftI2C_Stop();

    return ok;
}

/* 从指定 I2C 设备读取单个寄存器。 */
bool BSP_SoftI2C_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
    return BSP_SoftI2C_ReadRegs(devAddr, regAddr, data, 1U);
}

/* 从指定 I2C 设备连续读取多个寄存器。 */
bool BSP_SoftI2C_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buffer, uint8_t len)
{
    uint8_t index;

    if ((buffer == 0) || (len == 0U))
    {
        return false;
    }

    SoftI2C_Start();
    if (!SoftI2C_WriteByte((uint8_t)(devAddr << 1U)) || !SoftI2C_WriteByte(regAddr))
    {
        SoftI2C_Stop();
        return false;
    }

    SoftI2C_Start();
    if (!SoftI2C_WriteByte((uint8_t)((devAddr << 1U) | 1U)))
    {
        SoftI2C_Stop();
        return false;
    }

    for (index = 0U; index < len; index++)
    {
        buffer[index] = SoftI2C_ReadByte(index < (uint8_t)(len - 1U));
    }

    SoftI2C_Stop();
    return true;
}
