#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    /* 视觉模块串口 1。 */
    BSP_VISION_PORT_1 = 0,
    /* 视觉模块串口 2。 */
    BSP_VISION_PORT_2 = 1,
    /* 视觉串口数量，用于数组长度和边界检查。 */
    BSP_VISION_PORT_COUNT = 2
} bsp_vision_port_t;

/* 初始化视觉串口，调试串口沿用 BOARD_InitDebugConsole 的配置。 */
void BSP_UartInit(void);

/* 从调试串口读取 1 字节，成功返回 true，无数据返回 false。 */
bool BSP_DebugUartReadByte(uint8_t *data);

/* 轮询两个视觉串口，把收到的字节放入各自环形缓冲区。 */
void BSP_VisionUartPoll(void);

/* 查询指定视觉串口缓冲区内当前可读取的字节数。 */
uint16_t BSP_VisionUartAvailable(bsp_vision_port_t port);

/* 从指定视觉串口缓冲区读取数据，返回实际读取字节数。 */
uint16_t BSP_VisionUartRead(bsp_vision_port_t port, uint8_t *buffer, uint16_t maxLen);

/* 获取指定视觉串口因缓冲区满导致丢字节的累计次数。 */
uint32_t BSP_VisionUartGetOverflowCount(bsp_vision_port_t port);

#endif
