#include "protocol.h"
#include "../app/app_config.h"
#include "../bsp/bsp_encoder.h"
#include "../chassis/chassis.h"
#include "../path/path_planner.h"
#include "../sensor/imu.h"

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_debug_console.h"

#define PROTOCOL_FRAME_START ('@')
#define PROTOCOL_FRAME_END   (';')

typedef enum
{
    PROTOCOL_STREAM_NONE = 0,
    PROTOCOL_STREAM_STATUS,
    PROTOCOL_STREAM_ENCODER
} protocol_stream_t;

static char s_frame[APP_DEBUG_UART_FRAME_MAX_LEN];
static uint16_t s_frameLength;
static bool s_frameActive;
static TickType_t s_lastRxTick;
static protocol_stream_t s_streamMode;
static TickType_t s_lastStreamTick;

static bool Protocol_ParseUint(const char *frame, uint16_t length, uint16_t *pos, uint16_t *value)
{
    bool hasDigit = false;
    uint16_t parsed = 0U;

    while (*pos < length)
    {
        char ch = frame[*pos];

        if (ch == ',')
        {
            break;
        }
        if ((ch < '0') || (ch > '9'))
        {
            return false;
        }
        if ((parsed > 6553U) || ((parsed == 6553U) && ((uint16_t)(ch - '0') > 5U)))
        {
            return false;
        }

        parsed = (uint16_t)((parsed * 10U) + (uint16_t)(ch - '0'));
        hasDigit = true;
        (*pos)++;
    }

    if (!hasDigit)
    {
        return false;
    }

    *value = parsed;
    return true;
}

static bool Protocol_ParseInt(const char *frame, uint16_t length, uint16_t *pos, int32_t *value)
{
    bool negative = false;
    uint16_t parsed;

    if (*pos >= length)
    {
        return false;
    }

    if ((frame[*pos] == '+') || (frame[*pos] == '-'))
    {
        negative = (frame[*pos] == '-');
        (*pos)++;
    }

    if (!Protocol_ParseUint(frame, length, pos, &parsed))
    {
        return false;
    }

    *value = negative ? -(int32_t)parsed : (int32_t)parsed;
    return true;
}

static bool Protocol_ParseFloat(const char *frame, uint16_t length, uint16_t *pos, float *value)
{
    bool negative = false;
    bool hasDigit = false;
    float parsed = 0.0f;
    float decimalScale = 0.1f;

    if (*pos >= length)
    {
        return false;
    }

    if ((frame[*pos] == '+') || (frame[*pos] == '-'))
    {
        negative = (frame[*pos] == '-');
        (*pos)++;
    }

    while (*pos < length)
    {
        char ch = frame[*pos];

        if (ch == ',')
        {
            break;
        }
        if (ch == '.')
        {
            (*pos)++;
            break;
        }
        if ((ch < '0') || (ch > '9'))
        {
            return false;
        }

        parsed = (parsed * 10.0f) + (float)(ch - '0');
        hasDigit = true;
        (*pos)++;
    }

    while (*pos < length)
    {
        char ch = frame[*pos];

        if (ch == ',')
        {
            break;
        }
        if ((ch < '0') || (ch > '9'))
        {
            return false;
        }

        parsed += (float)(ch - '0') * decimalScale;
        decimalScale *= 0.1f;
        hasDigit = true;
        (*pos)++;
    }

    if (!hasDigit)
    {
        return false;
    }

    *value = negative ? -parsed : parsed;
    return true;
}

static bool Protocol_FrameTailEquals(const char *frame, uint16_t length, uint16_t start, const char *word)
{
    uint16_t index = 0U;

    while (word[index] != '\0')
    {
        if (((uint16_t)(start + index) >= length) || (frame[start + index] != word[index]))
        {
            return false;
        }
        index++;
    }

    return ((uint16_t)(start + index) == length);
}

static void Protocol_ResetFrame(void)
{
    s_frameActive = false;
    s_frameLength = 0U;
    s_frame[0] = '\0';
}

static void Protocol_PrintInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        PUTCHAR('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    PRINTF("%u", magnitude);
}

static void Protocol_PrintInt32List(const int32_t *values, uint8_t count)
{
    uint8_t index;

    for (index = 0U; index < count; index++)
    {
        if (index != 0U)
        {
            PUTCHAR(',');
        }
        Protocol_PrintInt32(values[index]);
    }
}

static void Protocol_PrintText(const char *text)
{
    while ((text != 0) && (*text != '\0'))
    {
        PUTCHAR(*text);
        text++;
    }
}

static void Protocol_PrintOk(const char *command)
{
    Protocol_PrintText("ok:");
    Protocol_PrintText(command);
    PUTCHAR('\n');
}

static void Protocol_PrintOkChannelValue(const char *command, uint16_t channel, int32_t value)
{
    Protocol_PrintText("ok:");
    Protocol_PrintText(command);
    PUTCHAR(',');
    Protocol_PrintInt32((int32_t)channel);
    PUTCHAR(',');
    Protocol_PrintInt32(value);
    PUTCHAR('\n');
}

static void Protocol_PrintOkValue(const char *command, int32_t value)
{
    Protocol_PrintText("ok:");
    Protocol_PrintText(command);
    PUTCHAR(',');
    Protocol_PrintInt32(value);
    PUTCHAR('\n');
}

static void Protocol_PrintOkPid(uint16_t channel, int32_t kp, int32_t ki, int32_t kd)
{
    Protocol_PrintText("ok:pid,");
    Protocol_PrintInt32((int32_t)channel);
    PUTCHAR(',');
    Protocol_PrintInt32(kp);
    PUTCHAR(',');
    Protocol_PrintInt32(ki);
    PUTCHAR(',');
    Protocol_PrintInt32(kd);
    PUTCHAR('\n');
}

static void Protocol_PrintStatus(void)
{
    chassis_status_t status;
    int32_t values[8];

    Chassis_GetStatus(&status);
    values[0] = status.actualCps[MOTOR_LF];
    values[1] = status.actualCps[MOTOR_RF];
    values[2] = status.actualCps[MOTOR_LB];
    values[3] = status.actualCps[MOTOR_RB];
    values[4] = status.outputDuty[MOTOR_LF];
    values[5] = status.outputDuty[MOTOR_RF];
    values[6] = status.outputDuty[MOTOR_LB];
    values[7] = status.outputDuty[MOTOR_RB];

    PRINTF("status:");
    Protocol_PrintInt32List(values, 8U);
    PUTCHAR('\n');
}

static void Protocol_PrintEncoderStatus(void)
{
    chassis_status_t status;
    int32_t values[8];

    Chassis_GetStatus(&status);
    values[0] = BSP_EncoderGetCount(MOTOR_LF);
    values[1] = BSP_EncoderGetCount(MOTOR_RF);
    values[2] = BSP_EncoderGetCount(MOTOR_LB);
    values[3] = BSP_EncoderGetCount(MOTOR_RB);
    values[4] = status.actualCps[MOTOR_LF];
    values[5] = status.actualCps[MOTOR_RF];
    values[6] = status.actualCps[MOTOR_LB];
    values[7] = status.actualCps[MOTOR_RB];

    PRINTF("enc:");
    Protocol_PrintInt32List(values, 8U);
    PUTCHAR('\n');
}

static void Protocol_PrintPathStatus(void)
{
    path_status_t status;
    int32_t values[11];

    PathPlanner_GetStatus(&status);
    values[0] = (int32_t)status.state;
    values[1] = status.mapReady ? 1 : 0;
    values[2] = status.poseReady ? 1 : 0;
    values[3] = (int32_t)status.x;
    values[4] = (int32_t)status.y;
    values[5] = (int32_t)status.heading;
    values[6] = (int32_t)status.stepIndex;
    values[7] = (int32_t)status.stepCount;
    values[8] = status.travelledCounts;
    values[9] = status.targetCounts;
    values[10] = (int32_t)status.cellSizeMm;

    PRINTF("path:");
    Protocol_PrintInt32List(values, 11U);
    PUTCHAR('\n');
}

static void Protocol_PrintMissionStatus(void)
{
    mission_status_t status;
    int32_t values[8];

    PathPlanner_GetMissionStatus(&status);
    values[0] = (int32_t)status.state;
    values[1] = (int32_t)status.currentPoint;
    values[2] = (int32_t)status.pointCount;
    values[3] = (int32_t)status.recognizedCount;
    values[4] = (int32_t)status.requestSequence;
    values[5] = (int32_t)status.retryCount;
    values[6] = status.mapReady ? 1 : 0;
    values[7] = status.poseReady ? 1 : 0;

    PRINTF("mission:");
    Protocol_PrintInt32List(values, 8U);
    PUTCHAR('\n');
}

static void Protocol_PrintRecognitionResults(void)
{
    mission_status_t status;
    recognition_point_t point;
    uint16_t index;

    PathPlanner_GetMissionStatus(&status);
    PRINTF("objects:%u,%u\n", (unsigned int)status.recognizedCount,
           (unsigned int)status.pointCount);
    for (index = 0U; index < status.pointCount; index++)
    {
        if (!PathPlanner_GetRecognitionPoint(index, &point))
        {
            continue;
        }
        PRINTF("object:%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)index,
               (unsigned int)point.objectX,
               (unsigned int)point.objectY,
               (unsigned int)point.observeX,
               (unsigned int)point.observeY,
               (unsigned int)point.observeDir,
               (unsigned int)point.resultCode,
               point.recognized ? 1U : 0U);
    }
}
static void Protocol_StartStream(protocol_stream_t mode)
{
    s_streamMode = mode;
    s_lastStreamTick = xTaskGetTickCount();

    if (mode == PROTOCOL_STREAM_STATUS)
    {
        Protocol_PrintStatus();
    }
    else if (mode == PROTOCOL_STREAM_ENCODER)
    {
        Protocol_PrintEncoderStatus();
    }
}

void Protocol_Init(void)
{
    Protocol_ResetFrame();
    s_streamMode = PROTOCOL_STREAM_NONE;
    s_lastStreamTick = xTaskGetTickCount();
}

void Protocol_InputByte(uint8_t data)
{
    if (data == PROTOCOL_FRAME_START)
    {
        s_frameActive = true;
        s_frameLength = 0U;
        s_lastRxTick = xTaskGetTickCount();
        return;
    }

    if (!s_frameActive)
    {
        return;
    }

    if ((data == PROTOCOL_FRAME_END) || (data == '\r') || (data == '\n'))
    {
        Protocol_ProcessFrame(s_frame);
        Protocol_ResetFrame();
        return;
    }

    if (s_frameLength >= (APP_DEBUG_UART_FRAME_MAX_LEN - 1U))
    {
        Protocol_ResetFrame();
        return;
    }

    s_frame[s_frameLength] = (char)data;
    s_frameLength++;
    s_frame[s_frameLength] = '\0';
    s_lastRxTick = xTaskGetTickCount();
}

void Protocol_CheckTimeout(void)
{
    if (s_frameActive && ((xTaskGetTickCount() - s_lastRxTick) > pdMS_TO_TICKS(APP_DEBUG_UART_FRAME_TIMEOUT_MS)))
    {
        Protocol_ResetFrame();
    }
}

void Protocol_PeriodicSend(void)
{
    TickType_t nowTick;

    if (s_streamMode == PROTOCOL_STREAM_NONE)
    {
        return;
    }

    nowTick = xTaskGetTickCount();
    if ((nowTick - s_lastStreamTick) < pdMS_TO_TICKS(APP_DEBUG_UART_STREAM_PERIOD_MS))
    {
        return;
    }

    s_lastStreamTick = nowTick;
    if (s_streamMode == PROTOCOL_STREAM_STATUS)
    {
        Protocol_PrintStatus();
    }
    else if (s_streamMode == PROTOCOL_STREAM_ENCODER)
    {
        Protocol_PrintEncoderStatus();
    }
}

void Protocol_ProcessFrame(const char *frame)
{
    uint16_t pos = 0U;
    uint16_t length = 0U;
    uint16_t channel;
    uint16_t value;
    uint16_t x;
    uint16_t y;
    uint16_t direction;
    uint16_t cells;
    int32_t signedValue;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    float vx;
    float vy;
    float wz;
    float steerDeg;

    if (frame == 0)
    {
        return;
    }

    while ((length < APP_DEBUG_UART_FRAME_MAX_LEN) && (frame[length] != '\0'))
    {
        length++;
    }

    if (length < 3U)
    {
        return;
    }

    if ((length == 6U) && (frame[0] == 'S') && (frame[1] == 'T') && (frame[2] == 'A') &&
        (frame[3] == 'T') && (frame[4] == 'U') && (frame[5] == 'S'))
    {
        Protocol_StartStream(PROTOCOL_STREAM_STATUS);
        return;
    }

    if ((length >= 3U) && (frame[0] == 'E') && (frame[1] == 'N') && (frame[2] == 'C'))
    {
        if (length == 3U)
        {
            Protocol_StartStream(PROTOCOL_STREAM_ENCODER);
            return;
        }

        if ((length >= 5U) && (frame[3] == ','))
        {
            pos = 4U;
            if (!Protocol_ParseUint(frame, length, &pos, &channel) || (pos != length) ||
                (channel > (uint16_t)MOTOR_COUNT))
            {
                return;
            }

            if (channel == 0U)
            {
                BSP_EncoderClearAll();
            }
            else
            {
                BSP_EncoderClear((motor_id_t)(channel - 1U));
            }

            Protocol_StartStream(PROTOCOL_STREAM_ENCODER);
        }
        return;
    }

    if ((length >= 7U) && (frame[0] == 'M') && (frame[1] == 'I') && (frame[2] == 'S') &&
        (frame[3] == 'S') && (frame[4] == 'I') && (frame[5] == 'O') && (frame[6] == 'N'))
    {
        if ((length == 7U) || ((length > 8U) && (frame[7] == ',') &&
            Protocol_FrameTailEquals(frame, length, 8U, "STATUS")))
        {
            Protocol_PrintMissionStatus();
        }
        else if ((frame[7] == ',') && Protocol_FrameTailEquals(frame, length, 8U, "START"))
        {
            if (!PathPlanner_MissionStart())
            {
                PRINTF("error:mission_start\n");
            }
            Protocol_PrintMissionStatus();
        }
        else if ((frame[7] == ',') && Protocol_FrameTailEquals(frame, length, 8U, "STOP"))
        {
            PathPlanner_MissionStop();
            Protocol_PrintMissionStatus();
        }
        else if ((frame[7] == ',') && Protocol_FrameTailEquals(frame, length, 8U, "RESET"))
        {
            PathPlanner_MissionReset();
            Protocol_PrintMissionStatus();
        }
        else
        {
            PRINTF("error:mission_command\n");
        }
        return;
    }

    if ((length == 13U) && (frame[0] == 'R') && (frame[1] == 'E') && (frame[2] == 'C') &&
        (frame[3] == 'O') && (frame[4] == 'G') && (frame[5] == ',') &&
        Protocol_FrameTailEquals(frame, length, 6U, "RESULTS"))
    {
        Protocol_PrintRecognitionResults();
        return;
    }
    if ((length >= 4U) && (frame[0] == 'M') && (frame[1] == 'A') && (frame[2] == 'P') &&
        (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseUint(frame, length, &pos, &x) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &y) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;

        (void)PathPlanner_SetMapRows((uint8_t)x, (uint8_t)y, &frame[pos]);
        Protocol_PrintPathStatus();
        return;
    }

    if ((length >= 4U) && (frame[0] == 'P') && (frame[1] == 'A') && (frame[2] == 'T') &&
        (frame[3] == 'H'))
    {
        if (length == 4U)
        {
            Protocol_PrintPathStatus();
            return;
        }

        if (frame[4] != ',')
        {
            return;
        }

        if (Protocol_FrameTailEquals(frame, length, 5U, "STATUS"))
        {
            Protocol_PrintPathStatus();
        }
        else if (Protocol_FrameTailEquals(frame, length, 5U, "START"))
        {
            PathPlanner_Start();
            Protocol_PrintPathStatus();
        }
        else if (Protocol_FrameTailEquals(frame, length, 5U, "STOP"))
        {
            PathPlanner_Stop();
            Protocol_PrintPathStatus();
        }
        return;
    }

    if ((length >= 5U) && (frame[0] == 'P') && (frame[1] == 'O') && (frame[2] == 'S') &&
        (frame[3] == 'E') && (frame[4] == ','))
    {
        pos = 5U;
        if (!Protocol_ParseUint(frame, length, &pos, &x) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &y) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &direction) || (pos != length) ||
            (direction > (uint16_t)PATH_DIR_Y_NEG))
        {
            return;
        }

        (void)PathPlanner_SetPose((uint8_t)x, (uint8_t)y, (path_dir_t)direction);
        Protocol_PrintPathStatus();
        return;
    }

    if ((length >= 3U) && (frame[0] == 'G') && (frame[1] == 'O') && (frame[2] == ','))
    {
        pos = 3U;
        if (!Protocol_ParseUint(frame, length, &pos, &x) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &y) || (pos != length))
        {
            return;
        }

        (void)PathPlanner_Goto((uint8_t)x, (uint8_t)y);
        Protocol_PrintPathStatus();
        return;
    }

    if ((length >= 5U) && (frame[0] == 'S') && (frame[1] == 'T') && (frame[2] == 'E') &&
        (frame[3] == 'P') && (frame[4] == ','))
    {
        pos = 5U;
        if (!Protocol_ParseUint(frame, length, &pos, &direction) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &cells) || (pos != length) ||
            (direction > (uint16_t)PATH_DIR_Y_NEG) || (cells == 0U))
        {
            return;
        }

        (void)PathPlanner_MoveCells((path_dir_t)direction, (uint8_t)cells);
        Protocol_PrintPathStatus();
        return;
    }

    if ((length >= 5U) && (frame[0] == 'C') && (frame[1] == 'E') && (frame[2] == 'L') &&
        (frame[3] == 'L') && (frame[4] == ','))
    {
        pos = 5U;
        if (!Protocol_ParseUint(frame, length, &pos, &value) || (pos != length))
        {
            return;
        }

        PathPlanner_SetCellSizeMm(value);
        Protocol_PrintPathStatus();
        return;
    }

    if ((length >= 5U) && (frame[0] == 'G') && (frame[1] == 'Y') && (frame[2] == 'R') &&
        (frame[3] == 'O') && (frame[4] == ','))
    {
        pos = 5U;
        if (!Protocol_ParseUint(frame, length, &pos, &value) || (pos != length) || (value > 2U))
        {
            return;
        }

        if (value == 0U)
        {
            IMU_SetPrintEnabled(false);
        }
        else if (value == 1U)
        {
            IMU_SetPrintEnabled(true);
        }
        else
        {
            IMU_Calibrate();
            IMU_ResetYaw();
        }
        Protocol_PrintOkValue("gyro", (int32_t)value);
        return;
    }

    if ((length >= 4U) && (frame[0] == 'V') && (frame[1] == 'E') && (frame[2] == 'L') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseFloat(frame, length, &pos, &vx) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseFloat(frame, length, &pos, &vy) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseFloat(frame, length, &pos, &wz) || (pos != length))
        {
            return;
        }

        Chassis_SetVelocity(vx, vy, wz);
        Protocol_PrintOk("vel");
        return;
    }

    if ((length >= 4U) && (frame[0] == 'P') && (frame[1] == 'I') && (frame[2] == 'D') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseUint(frame, length, &pos, &channel) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseInt(frame, length, &pos, &kp) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseInt(frame, length, &pos, &ki) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseInt(frame, length, &pos, &kd) || (pos != length) || (channel > (uint16_t)MOTOR_COUNT))
        {
            return;
        }

        if (channel == 0U)
        {
            Chassis_SetAllPid(kp, ki, kd);
        }
        else
        {
            Chassis_SetPid((motor_id_t)(channel - 1U), kp, ki, kd);
        }
        Protocol_PrintOkPid(channel, kp, ki, kd);
        return;
    }

    if ((length >= 4U) && (frame[0] == 'C') && (frame[1] == 'A') && (frame[2] == 'R') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseFloat(frame, length, &pos, &vx) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseFloat(frame, length, &pos, &steerDeg) || (pos != length))
        {
            return;
        }
        Chassis_SetLegacyCar(vx, steerDeg);
        Protocol_PrintOk("car");
        return;
    }

    if ((length >= 5U) && (frame[0] == 'G') && (frame[1] == 'C') && (frame[2] == 'A') &&
        (frame[3] == 'R') && (frame[4] == ','))
    {
        pos = 5U;
        if (!Protocol_ParseFloat(frame, length, &pos, &vx) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseFloat(frame, length, &pos, &steerDeg) || (pos != length))
        {
            return;
        }
        Chassis_SetLegacyCar(vx, steerDeg);
        Protocol_PrintOk("gcar");
        return;
    }

    if ((length >= 4U) && (frame[0] == 'P') && (frame[1] == 'W') && (frame[2] == 'M') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseUint(frame, length, &pos, &channel) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseInt(frame, length, &pos, &signedValue) || (pos != length) ||
            (channel < 1U) || (channel > (uint16_t)MOTOR_COUNT))
        {
            return;
        }
        Chassis_SetManualDuty((motor_id_t)(channel - 1U), (int16_t)signedValue);
        Protocol_PrintOkChannelValue("pwm", channel, signedValue);
        return;
    }

    if ((length >= 4U) && (frame[0] == 'S') && (frame[1] == 'P') && (frame[2] == 'D') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseUint(frame, length, &pos, &channel) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseInt(frame, length, &pos, &signedValue) || (pos != length) ||
            (channel < 1U) || (channel > (uint16_t)MOTOR_COUNT))
        {
            return;
        }
        Chassis_SetWheelTarget((motor_id_t)(channel - 1U), signedValue);
        Protocol_PrintOkChannelValue("spd", channel, signedValue);
        return;
    }

    if ((length >= 4U) && (frame[0] == 'D') && (frame[1] == 'I') && (frame[2] == 'R') && (frame[3] == ','))
    {
        pos = 4U;
        if (!Protocol_ParseUint(frame, length, &pos, &channel) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }
        pos++;
        if (!Protocol_ParseUint(frame, length, &pos, &value) || (pos != length) ||
            (channel < 1U) || (channel > (uint16_t)MOTOR_COUNT) || (value > 2U))
        {
            return;
        }

        Chassis_SetManualDirection((motor_id_t)(channel - 1U), (value == 2U) ? -1 : ((value == 1U) ? 1 : 0));
        Protocol_PrintOkChannelValue("dir", channel, (int32_t)value);
        return;
    }
}
