#include "vision.h"
#include "../app/app_config.h"
#include "../path/path_planner.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

static char s_lineBuffer[BSP_VISION_PORT_COUNT][APP_VISION_FRAME_MAX_LEN];
static uint16_t s_lineLength[BSP_VISION_PORT_COUNT];
static int16_t s_yawCentiDeg;
static bool s_yawValid;
static TickType_t s_yawTick;

/* 判断视觉串口编号是否有效。 */
static bool Vision_IsValidPort(bsp_vision_port_t port)
{
    return ((uint32_t)port < (uint32_t)BSP_VISION_PORT_COUNT);
}

/* 判断字符是否为空白。 */
static bool Vision_IsSpace(char ch)
{
    return ((ch == ' ') || (ch == '\t'));
}

/* 判断字符是否为数字。 */
static bool Vision_IsDigit(char ch)
{
    return ((ch >= '0') && (ch <= '9'));
}

/* 大小写不敏感地判断一段前缀。 */
static bool Vision_MatchPrefix(const char *line, const char *prefix)
{
    char a;
    char b;

    while (*prefix != '\0')
    {
        a = *line;
        b = *prefix;
        if ((a >= 'A') && (a <= 'Z'))
        {
            a = (char)(a + ('a' - 'A'));
        }
        if ((b >= 'A') && (b <= 'Z'))
        {
            b = (char)(b + ('a' - 'A'));
        }
        if (a != b)
        {
            return false;
        }
        line++;
        prefix++;
    }

    return true;
}

/* 解析无符号整数，遇到指定分隔符或字符串结束停止。 */
static bool Vision_ParseUint(const char **text, uint16_t *value)
{
    uint32_t parsed = 0U;
    bool hasDigit = false;

    while (Vision_IsSpace(**text))
    {
        (*text)++;
    }

    while (Vision_IsDigit(**text))
    {
        parsed = (parsed * 10U) + (uint32_t)(**text - '0');
        if (parsed > 65535U)
        {
            return false;
        }
        hasDigit = true;
        (*text)++;
    }

    if (!hasDigit)
    {
        return false;
    }

    *value = (uint16_t)parsed;
    return true;
}

/* 解析带小数的角度值，结果转换为 0.01 度。 */
static bool Vision_ParseCentiDeg(const char *text, int16_t *value)
{
    bool negative = false;
    bool hasDigit = false;
    int32_t integerPart = 0;
    int32_t decimalPart = 0;
    uint8_t decimalDigits = 0U;
    int32_t centiDeg;

    while (Vision_IsSpace(*text))
    {
        text++;
    }

    if ((*text == '+') || (*text == '-'))
    {
        negative = (*text == '-');
        text++;
    }

    while (Vision_IsDigit(*text))
    {
        integerPart = (integerPart * 10L) + (int32_t)(*text - '0');
        hasDigit = true;
        text++;
    }

    if (*text == '.')
    {
        text++;
        while (Vision_IsDigit(*text) && (decimalDigits < 2U))
        {
            decimalPart = (decimalPart * 10L) + (int32_t)(*text - '0');
            decimalDigits++;
            hasDigit = true;
            text++;
        }

        while (Vision_IsDigit(*text))
        {
            text++;
        }
    }

    if (!hasDigit)
    {
        return false;
    }

    if (decimalDigits == 0U)
    {
        decimalPart = 0;
    }
    else if (decimalDigits == 1U)
    {
        decimalPart *= 10L;
    }

    centiDeg = (integerPart * 100L) + decimalPart;
    if (negative)
    {
        centiDeg = -centiDeg;
    }

    while (centiDeg > 18000L)
    {
        centiDeg -= 36000L;
    }
    while (centiDeg < -18000L)
    {
        centiDeg += 36000L;
    }

    *value = (int16_t)centiDeg;
    return true;
}

/* 保存视觉端发来的车体 yaw 角。 */
static bool Vision_HandleYawLine(const char *line)
{
    const char *valueText = line;
    int16_t yaw;

    if (Vision_MatchPrefix(line, "yaw:"))
    {
        valueText = line + 4;
    }
    else if (Vision_MatchPrefix(line, "z:"))
    {
        valueText = line + 2;
    }
    else if (Vision_MatchPrefix(line, "angle:"))
    {
        valueText = line + 6;
    }
    else if (Vision_MatchPrefix(line, "yaw,"))
    {
        valueText = line + 4;
    }
    else if (Vision_MatchPrefix(line, "z,"))
    {
        valueText = line + 2;
    }
    else if (!Vision_IsDigit(*line) && (*line != '-') && (*line != '+'))
    {
        return false;
    }

    if (!Vision_ParseCentiDeg(valueText, &yaw))
    {
        return false;
    }

    s_yawCentiDeg = yaw;
    s_yawValid = true;
    s_yawTick = xTaskGetTickCount();
    return true;
}

/* 正式比赛地图帧：map:seq,level,mode,w,h,rows；mode 0=纯推箱，1=标签匹配。 */
static bool Vision_HandleMapLine(const char *line)
{
    const char *text;
    uint16_t sequence;
    uint16_t level;
    uint16_t mode;
    uint16_t width;
    uint16_t height;
    bool mapOk;

    if (Vision_MatchPrefix(line, "map:"))
    {
        text = line + 4;
    }
    else if (Vision_MatchPrefix(line, "map,"))
    {
        text = line + 4;
    }
    else
    {
        return false;
    }

    if (!Vision_ParseUint(&text, &sequence) || (*text != ','))
    {
        (void)Vision_SendMapAck(0U, true);
        return true;
    }
    text++;
    if (!Vision_ParseUint(&text, &level) || (*text != ',') ||
        level == 0U || level > APP_MISSION_LEVEL_COUNT)
    {
        (void)Vision_SendMapAck(sequence, true);
        return true;
    }
    text++;
    if (!Vision_ParseUint(&text, &mode) || (*text != ',') || mode > 1U)
    {
        (void)Vision_SendMapAck(sequence, true);
        return true;
    }
    text++;
    if (!Vision_ParseUint(&text, &width) || (*text != ','))
    {
        (void)Vision_SendMapAck(sequence, true);
        return true;
    }
    text++;
    if (!Vision_ParseUint(&text, &height) || ((*text != ',') && (*text != ':')))
    {
        (void)Vision_SendMapAck(sequence, true);
        return true;
    }
    text++;

    mapOk = PathPlanner_SetMapRows((uint8_t)width, (uint8_t)height, text);
    (void)Vision_SendMapAck(sequence, !mapOk);
    if (mapOk)
    {
        PathPlanner_MissionOnMapReceived(sequence, (uint8_t)level, mode == 1U);
    }
    return true;
}

/* 发车区观测：zone:1 表示在发车区，zone:0 表示已进入比赛场地。 */
static bool Vision_HandleZoneLine(const char *line)
{
    const char *text;
    uint16_t inStartArea;

    if (Vision_MatchPrefix(line, "zone:"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "zone,"))
    {
        text = line + 5;
    }
    else
    {
        return false;
    }
    if (!Vision_ParseUint(&text, &inStartArea) || *text != '\0' || inStartArea > 1U)
    {
        return true;
    }
    (void)PathPlanner_SetStartArea(inStartArea == 1U);
    return true;
}
/* 解析 pose:x,y,heading，用于告诉 MCU 当前车在哪个格子。 */
static bool Vision_HandlePoseLine(const char *line)
{
    const char *text;
    uint16_t x;
    uint16_t y;
    uint16_t heading;

    if (Vision_MatchPrefix(line, "pose:"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "pose,"))
    {
        text = line + 5;
    }
    else
    {
        return false;
    }

    if (!Vision_ParseUint(&text, &x) || (*text != ','))
    {
        return false;
    }
    text++;
    if (!Vision_ParseUint(&text, &y) || (*text != ','))
    {
        return false;
    }
    text++;
    if (!Vision_ParseUint(&text, &heading) || (heading > (uint16_t)PATH_DIR_Y_NEG))
    {
        return false;
    }

    return PathPlanner_SetPose((uint8_t)x, (uint8_t)y, (path_dir_t)heading);
}

/* 解析 goal:x,y 或 goto:x,y，收到后立即规划并开始执行。 */
static bool Vision_HandleGoalLine(const char *line)
{
    const char *text;
    uint16_t x;
    uint16_t y;

    if (Vision_MatchPrefix(line, "goal:"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "goal,"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "goto:"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "goto,"))
    {
        text = line + 5;
    }
    else
    {
        return false;
    }

    if (!Vision_ParseUint(&text, &x) || (*text != ','))
    {
        return false;
    }
    text++;
    if (!Vision_ParseUint(&text, &y))
    {
        return false;
    }

    return PathPlanner_Goto((uint8_t)x, (uint8_t)y);
}

/* 解析 cell:mm，用于临时调整格子尺寸。 */
static bool Vision_HandleCellLine(const char *line)
{
    const char *text;
    uint16_t cellSizeMm;

    if (Vision_MatchPrefix(line, "cell:"))
    {
        text = line + 5;
    }
    else if (Vision_MatchPrefix(line, "cell,"))
    {
        text = line + 5;
    }
    else
    {
        return false;
    }

    if (!Vision_ParseUint(&text, &cellSizeMm))
    {
        return false;
    }

    PathPlanner_SetCellSizeMm(cellSizeMm);
    return true;
}

/* 解析 result:seq,code；也兼容视觉端只发送0～20的裸数字。 */
static bool Vision_HandleRecognitionLine(const char *line)
{
    const char *text = line;
    uint16_t sequence = 0U;
    uint16_t code;
    bool accepted;
    mission_status_t missionStatus;

    if (Vision_MatchPrefix(line, "result:"))
    {
        text = line + 7;
        if (!Vision_ParseUint(&text, &sequence) || (*text != ','))
        {
            return true;
        }
        text++;
    }
    else if (Vision_MatchPrefix(line, "result,"))
    {
        text = line + 7;
        if (!Vision_ParseUint(&text, &sequence) || (*text != ','))
        {
            return true;
        }
        text++;
    }
    else if (!Vision_IsDigit(*line))
    {
        return false;
    }

    if (!Vision_ParseUint(&text, &code) || (*text != '\0') || (code > 20U))
    {
        return true;
    }

    accepted = PathPlanner_MissionOnRecognitionResult(sequence, (uint8_t)code);
    if (!accepted)
    {
        (void)Vision_SendRecognitionAck(sequence, "ignored");
    }
    else
    {
        PathPlanner_GetMissionStatus(&missionStatus);
        (void)Vision_SendRecognitionAck(sequence,
                (missionStatus.state == MISSION_STATE_RETRY_RECOGNITION) ? "retry" : "ok");
    }
    return true;
}
/* 根据端口分发一行视觉文本。 */
static bool Vision_HandleLine(bsp_vision_port_t port, const char *line)
{
    if (port == BSP_VISION_PORT_1)
    {
        if (Vision_HandleMapLine(line))
        {
            return true;
        }
        if (Vision_HandleYawLine(line))
        {
            return true;
        }
        if (Vision_HandlePoseLine(line))
        {
            return true;
        }
        if (Vision_HandleZoneLine(line))
        {
            return true;
        }
        return Vision_HandleCellLine(line);
    }

    if (Vision_HandleRecognitionLine(line))
    {
        return true;
    }

    /* 保留旧视觉协议，方便已有模块继续发送这些命令。 */
    if (Vision_HandleMapLine(line))
    {
        return true;
    }
    if (Vision_HandlePoseLine(line))
    {
        return true;
    }
    if (Vision_HandleGoalLine(line))
    {
        return true;
    }
    return Vision_HandleCellLine(line);
}
/* 初始化视觉模块的软件解析状态。 */
void Vision_Init(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)BSP_VISION_PORT_COUNT; index++)
    {
        s_lineLength[index] = 0U;
        s_lineBuffer[index][0] = '\0';
    }

    s_yawCentiDeg = 0;
    s_yawValid = false;
    s_yawTick = xTaskGetTickCount();
}

/* 周期轮询视觉串口，并解析已经收到的文本帧。 */
void Vision_Poll(void)
{
    uint8_t port;

    BSP_VisionUartPoll();
    for (port = 0U; port < (uint8_t)BSP_VISION_PORT_COUNT; port++)
    {
        (void)Vision_ParseFrame((bsp_vision_port_t)port);
    }
}

/* 查询指定视觉串口当前可读取的原始字节数。 */
uint16_t Vision_Available(bsp_vision_port_t port)
{
    return BSP_VisionUartAvailable(port);
}

/* 从指定视觉串口读取原始数据，主要用于临时调试。 */
uint16_t Vision_Read(bsp_vision_port_t port, uint8_t *buffer, uint16_t maxLen)
{
    return BSP_VisionUartRead(port, buffer, maxLen);
}

/* 获取指定视觉串口的接收溢出次数。 */
uint32_t Vision_GetOverflowCount(bsp_vision_port_t port)
{
    return BSP_VisionUartGetOverflowCount(port);
}

/* 获取最近一次视觉角度，单位 0.01 度；数据超时或未收到时返回 false。 */
bool Vision_GetYawCentiDeg(int16_t *yawCentiDeg)
{
    if ((yawCentiDeg == 0) || !s_yawValid)
    {
        return false;
    }

    if ((xTaskGetTickCount() - s_yawTick) > pdMS_TO_TICKS(APP_VISION_YAW_TIMEOUT_MS))
    {
        return false;
    }

    *yawCentiDeg = s_yawCentiDeg;
    return true;
}

/* 发送一段以NUL结尾的视觉协议文本。 */
static bool Vision_SendText(bsp_vision_port_t port, const char *text)
{
    uint16_t length = 0U;

    if (text == 0)
    {
        return false;
    }
    while ((length < APP_VISION_TX_FRAME_MAX_LEN) && (text[length] != '\0'))
    {
        length++;
    }
    if ((length == 0U) || (length >= APP_VISION_TX_FRAME_MAX_LEN))
    {
        return false;
    }
    return BSP_VisionUartWrite(port, (const uint8_t *)text, length);
}

bool Vision_RequestMap(uint16_t sequence, uint8_t level)
{
    char frame[APP_VISION_TX_FRAME_MAX_LEN];
    int length;

    length = snprintf(frame, sizeof(frame), "map_req:%u,%u\n",
                      (unsigned int)sequence, (unsigned int)level);
    return ((length > 0) && ((uint32_t)length < sizeof(frame)) &&
            Vision_SendText(BSP_VISION_PORT_1, frame));
}

bool Vision_SendMapAck(uint16_t sequence, bool retry)
{
    char frame[APP_VISION_TX_FRAME_MAX_LEN];
    int length;

    length = snprintf(frame, sizeof(frame), "map_ack:%u,%s\n", (unsigned int)sequence,
                      retry ? "retry" : "ok");
    return ((length > 0) && ((uint32_t)length < sizeof(frame)) &&
            Vision_SendText(BSP_VISION_PORT_1, frame));
}

bool Vision_RequestRecognition(uint16_t sequence, uint16_t pointIndex)
{
    char frame[APP_VISION_TX_FRAME_MAX_LEN];
    int length;

    length = snprintf(frame, sizeof(frame), "recognize:%u,%u\n", (unsigned int)sequence,
                      (unsigned int)pointIndex);
    return ((length > 0) && ((uint32_t)length < sizeof(frame)) &&
            Vision_SendText(BSP_VISION_PORT_2, frame));
}

bool Vision_SendRecognitionAck(uint16_t sequence, const char *status)
{
    char frame[APP_VISION_TX_FRAME_MAX_LEN];
    int length;

    if (status == 0)
    {
        return false;
    }
    length = snprintf(frame, sizeof(frame), "result_ack:%u,%s\n", (unsigned int)sequence, status);
    return ((length > 0) && ((uint32_t)length < sizeof(frame)) &&
            Vision_SendText(BSP_VISION_PORT_2, frame));
}
/* 解析指定视觉串口缓冲中的一批文本帧。 */
bool Vision_ParseFrame(bsp_vision_port_t port)
{
    uint8_t data;
    bool parsedAny = false;

    if (!Vision_IsValidPort(port))
    {
        return false;
    }

    while (BSP_VisionUartRead(port, &data, 1U) == 1U)
    {
        if ((data == '\n') || (data == '\r') || (data == ';'))
        {
            if (s_lineLength[port] > 0U)
            {
                s_lineBuffer[port][s_lineLength[port]] = '\0';
                if (Vision_HandleLine(port, s_lineBuffer[port]))
                {
                    parsedAny = true;
                }
                s_lineLength[port] = 0U;
            }
            continue;
        }

        if (s_lineLength[port] >= (APP_VISION_FRAME_MAX_LEN - 1U))
        {
            s_lineLength[port] = 0U;
            continue;
        }

        s_lineBuffer[port][s_lineLength[port]] = (char)data;
        s_lineLength[port]++;
    }

    return parsedAny;
}
