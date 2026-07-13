#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stdint.h>
#include "../bsp/bsp_uart.h"

/* 初始化视觉模块的软件解析状态。 */
void Vision_Init(void);

/* 周期轮询视觉串口，并解析已经收到的文本帧。 */
void Vision_Poll(void);

/* 查询指定视觉串口当前可读取的原始字节数。 */
uint16_t Vision_Available(bsp_vision_port_t port);

/* 从指定视觉串口读取原始数据，主要用于临时调试。 */
uint16_t Vision_Read(bsp_vision_port_t port, uint8_t *buffer, uint16_t maxLen);

/* 获取指定视觉串口的接收溢出次数。 */
uint32_t Vision_GetOverflowCount(bsp_vision_port_t port);

/* 获取最近一次视觉角度，单位 0.01 度；数据超时或未收到时返回 false。 */
bool Vision_GetYawCentiDeg(int16_t *yawCentiDeg);

/* 解析指定视觉串口缓冲中的一批文本帧。 */
bool Vision_ParseFrame(bsp_vision_port_t port);

#endif
