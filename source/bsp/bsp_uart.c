#include "bsp_uart.h"
#include "../app/app_config.h"

#include "fsl_lpuart.h"
#include "board.h"

static LPUART_Type *const s_visionUartBase[BSP_VISION_PORT_COUNT] = {LPUART2, LPUART8};
static uint8_t s_visionRxBuffer[BSP_VISION_PORT_COUNT][APP_VISION_UART_RX_BUF_SIZE];
static volatile uint16_t s_visionRxHead[BSP_VISION_PORT_COUNT];
static volatile uint16_t s_visionRxTail[BSP_VISION_PORT_COUNT];
static volatile uint32_t s_visionOverflowCount[BSP_VISION_PORT_COUNT];

/* 判断指定 LPUART 是否已有可读取的接收数据。 */
static bool Uart_HasRxData(LPUART_Type *base)
{
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    return (((base->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT) != 0U);
#else
    return ((LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag) != 0U);
#endif
}

/* 检查视觉串口编号是否有效。 */
static bool Vision_IsValidPort(bsp_vision_port_t port)
{
    return ((uint32_t)port < (uint32_t)BSP_VISION_PORT_COUNT);
}

/* 把视觉串口收到的字节写入对应环形缓冲区。 */
static void Vision_StoreByte(bsp_vision_port_t port, uint8_t data)
{
    uint16_t nextHead;

    if (!Vision_IsValidPort(port))
    {
        return;
    }

    nextHead = (uint16_t)((s_visionRxHead[port] + 1U) % APP_VISION_UART_RX_BUF_SIZE);
    if (nextHead == s_visionRxTail[port])
    {
        s_visionOverflowCount[port]++;
        return;
    }

    s_visionRxBuffer[port][s_visionRxHead[port]] = data;
    s_visionRxHead[port] = nextHead;
}

/* 初始化两个视觉 LPUART，并清空对应接收缓冲。 */
void BSP_UartInit(void)
{
    lpuart_config_t uartConfig;
    uint8_t index;

    for (index = 0U; index < (uint8_t)BSP_VISION_PORT_COUNT; index++)
    {
        LPUART_GetDefaultConfig(&uartConfig);
        uartConfig.baudRate_Bps = APP_VISION_UART_BAUDRATE;
        uartConfig.enableTx = true;
        uartConfig.enableRx = true;

        if (LPUART_Init(s_visionUartBase[index], &uartConfig, BOARD_DebugConsoleSrcFreq()) != kStatus_Success)
        {
            while (1)
            {
            }
        }

        s_visionRxHead[index] = 0U;
        s_visionRxTail[index] = 0U;
        s_visionOverflowCount[index] = 0U;
    }
}

/* 从调试串口读取一个字节，处理溢出标志并以非阻塞方式返回。 */
bool BSP_DebugUartReadByte(uint8_t *data)
{
    LPUART_Type *debugUart = (LPUART_Type *)BOARD_DEBUG_UART_BASEADDR;

    if (data == 0)
    {
        return false;
    }

    if ((LPUART_GetStatusFlags(debugUart) & kLPUART_RxOverrunFlag) != 0U)
    {
        (void)LPUART_ClearStatusFlags(debugUart, kLPUART_RxOverrunFlag);
    }

    if (!Uart_HasRxData(debugUart))
    {
        return false;
    }

    *data = LPUART_ReadByte(debugUart);
    return true;
}

/* 轮询两个视觉串口，把硬件接收 FIFO 中的数据搬到软件环形缓冲。 */
void BSP_VisionUartPoll(void)
{
    LPUART_Type *uartBase;
    uint8_t port;

    for (port = 0U; port < (uint8_t)BSP_VISION_PORT_COUNT; port++)
    {
        uartBase = s_visionUartBase[port];
        if ((LPUART_GetStatusFlags(uartBase) & kLPUART_RxOverrunFlag) != 0U)
        {
            (void)LPUART_ClearStatusFlags(uartBase, kLPUART_RxOverrunFlag);
        }

        while (Uart_HasRxData(uartBase))
        {
            Vision_StoreByte((bsp_vision_port_t)port, LPUART_ReadByte(uartBase));
        }
    }
}

/* 计算指定视觉串口缓冲区当前可读字节数。 */
uint16_t BSP_VisionUartAvailable(bsp_vision_port_t port)
{
    uint16_t head;
    uint16_t tail;

    if (!Vision_IsValidPort(port))
    {
        return 0U;
    }

    head = s_visionRxHead[port];
    tail = s_visionRxTail[port];
    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(APP_VISION_UART_RX_BUF_SIZE - tail + head);
}

/* 从指定视觉串口环形缓冲读取最多 maxLen 字节。 */
uint16_t BSP_VisionUartRead(bsp_vision_port_t port, uint8_t *buffer, uint16_t maxLen)
{
    uint16_t readCount = 0U;

    if (!Vision_IsValidPort(port) || (buffer == 0))
    {
        return 0U;
    }

    while ((readCount < maxLen) && (s_visionRxTail[port] != s_visionRxHead[port]))
    {
        buffer[readCount] = s_visionRxBuffer[port][s_visionRxTail[port]];
        s_visionRxTail[port] = (uint16_t)((s_visionRxTail[port] + 1U) % APP_VISION_UART_RX_BUF_SIZE);
        readCount++;
    }

    return readCount;
}

/* 通过指定视觉串口阻塞发送一段数据。 */
bool BSP_VisionUartWrite(bsp_vision_port_t port, const uint8_t *buffer, uint16_t length)
{
    if (!Vision_IsValidPort(port) || (buffer == 0) || (length == 0U))
    {
        return false;
    }

    return (LPUART_WriteBlocking(s_visionUartBase[port], buffer, (size_t)length) ==
            kStatus_Success);
}

/* 返回指定视觉串口缓冲满时丢弃字节的累计次数。 */
uint32_t BSP_VisionUartGetOverflowCount(bsp_vision_port_t port)
{
    if (!Vision_IsValidPort(port))
    {
        return 0U;
    }

    return s_visionOverflowCount[port];
}
