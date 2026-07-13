#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* 初始化调参/控制串口协议接收状态。 */
void Protocol_Init(void);

/* 输入调试串口收到的 1 字节，并在帧完整时触发解析。 */
void Protocol_InputByte(uint8_t data);

/* 检查协议接收超时，超时则丢弃未完成帧。 */
void Protocol_CheckTimeout(void);
void Protocol_PeriodicSend(void);

/* 解析并执行一帧完整的上位机/调试串口命令。 */
void Protocol_ProcessFrame(const char *frame);

#endif
