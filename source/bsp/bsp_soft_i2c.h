#ifndef BSP_SOFT_I2C_H
#define BSP_SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>

/* 初始化软件 I2C 使用的 SCL/SDA GPIO。 */
void BSP_SoftI2C_Init(void);

/* 向指定 I2C 设备的单个寄存器写入 1 字节。 */
bool BSP_SoftI2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data);

/* 从指定 I2C 设备的单个寄存器读取 1 字节。 */
bool BSP_SoftI2C_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data);

/* 从指定 I2C 设备的连续寄存器读取多个字节。 */
bool BSP_SoftI2C_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buffer, uint8_t len);

#endif
