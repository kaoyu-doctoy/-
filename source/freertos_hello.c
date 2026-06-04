/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "fsl_pwm.h"
#include "fsl_qtmr.h"
#include "board.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef APP_DEFAULT_PWM_FREQUENCY
#define APP_DEFAULT_PWM_FREQUENCY (1000UL)
#endif

#define APP_PWM_DEFAULT_DUTY  (0U)
#define APP_PWM_CHANNEL_COUNT (2U)
#define APP_PWM1_MODULE       (kPWM_Module_0)
#define APP_PWM2_MODULE       (kPWM_Module_1)
#define APP_SERVO_MODULE      (kPWM_Module_2)
#define APP_PWM_CHANNEL       (kPWM_PwmA)
#define APP_PWM_CONTROL       (kPWM_Control_Module_0 | kPWM_Control_Module_1)
#define APP_SERVO_PWM_CONTROL (kPWM_Control_Module_2)
#define APP_PWM_ALIGN_MODE    (kPWM_SignedCenterAligned)
#define APP_SERVO_PWM_FREQUENCY (50UL)
#define APP_SERVO_PERIOD_US     (20000UL)
#define APP_SERVO_CENTER_US     (1500U)
#define APP_SERVO_MIN_US        (500U)
#define APP_SERVO_MAX_US        (2500U)
#define APP_SERVO_MAX_STEER_DEG (90.0f)
#define APP_SERVO_RIGHT_ANGLE_DEG   (60.0f)
#define APP_SERVO_NEUTRAL_ANGLE_DEG (102.0f)
#define APP_SERVO_LEFT_ANGLE_DEG    (180.0f)
#define APP_SERVO_MAX_ANGLE_DEG     (180.0f)
#define APP_SERVO_US_PER_STEER_DEG \
    (((float)(APP_SERVO_MAX_US - APP_SERVO_CENTER_US)) / APP_SERVO_MAX_STEER_DEG)

#define APP_MOTOR1_IN1_GPIO GPIO1
#define APP_MOTOR1_IN1_PIN  (0U)
#define APP_MOTOR1_IN2_GPIO GPIO1
#define APP_MOTOR1_IN2_PIN  (1U)
#define APP_MOTOR2_IN1_GPIO GPIO1
#define APP_MOTOR2_IN1_PIN  (2U)
#define APP_MOTOR2_IN2_GPIO GPIO1
#define APP_MOTOR2_IN2_PIN  (3U)

#define APP_ENCODER_COUNT          (2U)
#define APP_ENCODER1_BASE          TMR3
#define APP_ENCODER2_BASE          TMR4
#define APP_ENCODER_COUNTER        (kQTMR_Channel_0)
#define APP_ENCODER_PRIMARY_PIN    (kQTMR_ClockCounter0InputPin)
#define APP_ENCODER_SECONDARY_PIN  (kQTMR_Counter1InputPin)

#define APP_PID_PERIOD_MS      (50U)
#define APP_STATUS_PERIOD_MS   (50U)
#define APP_PID_GAIN_DEN       (1000L)
#define APP_PID_KP_NUM         (20L)
#define APP_PID_KI_NUM         (2L)
#define APP_PID_KD_NUM         (0L)
#define APP_PID_INTEGRAL_LIMIT (40000L)
#define APP_SPEED_FEEDFORWARD_MAX_CPS (12000L)
#define APP_SPEED_MIN_ACTIVE_DUTY     (16U)
#define APP_SPEED_FILTER_NEW_WEIGHT   (1L)
#define APP_SPEED_FILTER_DEN          (8L)
#define APP_SPEED_TARGET_RAMP_CPS_PER_PERIOD (600L)
#define APP_WHEEL_MAX_TARGET_CPS             (7000L)

#define APP_PI_F                          (3.1415926535f)
#define APP_CAR_WHEEL_DIAMETER_M         (0.065f)
#define APP_CAR_WHEEL_BASE_M             (0.1435f)
#define APP_CAR_REAR_TRACK_M             (0.167f)
#define APP_CAR_FRONT_KINGPIN_TRACK_M    (0.104f)
#define APP_CAR_GEAR_RATIO               (30.0f)
#define APP_ENCODER_COUNTS_PER_MOTOR_REV (60.0f)
#define APP_CAR_SMALL_STEER_DEG          (3.0f)
#define APP_CAR_MAX_SPEED_MPS            (0.80f)
#define APP_CAR_MAX_STEER_DEG            (37.0f)
#define APP_DEG_TO_RAD                   (APP_PI_F / 180.0f)

#define APP_GYRO_SCL_GPIO GPIO1
#define APP_GYRO_SCL_PIN  (23U)
#define APP_GYRO_SDA_GPIO GPIO1
#define APP_GYRO_SDA_PIN  (22U)
#define APP_GYRO_I2C_ADDRESS_7BIT (0x68U)
#define APP_GYRO_REG_PWR_MGMT0    (0x4EU)
#define APP_GYRO_REG_GYRO_DATA_X1 (0x25U)
#define APP_GYRO_CALIBRATION_SAMPLES (40U)
#define APP_GYRO_I2C_DELAY_COUNT  (1200U)
#define APP_GYRO_READ_PERIOD_MS   (50U)
#define APP_GYRO_PRINT_PERIOD_MS  (50U)
#define APP_GYRO_ASSIST_GAIN_NUM  (1L)
#define APP_GYRO_ASSIST_GAIN_DEN  (10L)

#define led_task_PRIORITY     (configMAX_PRIORITIES - 3)
#define uart_rx_task_PRIORITY (configMAX_PRIORITIES - 4)
#define pid_task_PRIORITY     (configMAX_PRIORITIES - 2)

/* UART protocol: @PWM,ch,duty; @SPD,ch,cps; @DIR,ch,mode; @SER,servo0-180; @CAR,mps,steer; @GCAR,mps,steer; @GYRO,0/1/2; */
#define APP_UART_FRAME_START      ('@')
#define APP_UART_FRAME_END        (';')
#define APP_UART_FRAME_TIMEOUT_MS (1000U)
#define APP_UART_RX_FRAME_MAX_LEN (64U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void led_task(void *pvParameters);
static void uart_rx_task(void *pvParameters);
static void pid_task(void *pvParameters);
static void APP_InitPwm(void);
static void APP_InitEncoders(void);
static void APP_InitMotorDirectionPins(void);
static void APP_InitGyroI2cPins(void);
static void APP_UpdatePwmDuty(uint8_t index, uint8_t dutyCyclePercent);
static void APP_ApplyPwmDuty(uint8_t index, uint8_t dutyCyclePercent);
static void APP_SetSignedSpeedTarget(uint8_t index, int32_t targetSpeedCps);
static void APP_SetMotorDirection(uint8_t index, int32_t targetSpeedCps);
static uint8_t APP_RunPid(uint8_t index, int32_t actualSpeedCps);
static uint8_t APP_GetFeedforwardDuty(uint16_t targetSpeedCps);
static void APP_SetServoPulseUs(uint16_t pulseUs);
static void APP_SetServoAngleDeg(float servoAngleDeg);
static void APP_SetFrontSteeringAngleDeg(float frontSteerDeg);
static void APP_ApplyAckermannControl(float speedMps, float steerDeg);
static void APP_ApplyAckermannControlInternal(float speedMps, float steerDeg);
static void APP_ApplyGyroAssistedControl(float speedMps, float steerDeg);
static void APP_UpdateGyroAssistedServo(void);
static float APP_CalcAckermannQ(float steerDeg);
static float APP_CalcAckermannFrontSteerDeg(float steerDeg);
static int32_t APP_MpsToEncoderCps(float wheelSpeedMps);
static float APP_TanApprox(float radians);
static float APP_AtanApprox(float value);
static float APP_AbsFloat(float value);
static float APP_ClampFloat(float value, float minValue, float maxValue);
static uint16_t APP_ReadEncoderCounter(uint8_t index);
static int32_t APP_AbsInt32(int32_t value);
static void APP_PrintSpeedStatus(void);
static bool APP_IsAnySpeedControlEnabled(void);
static void APP_GyroI2cDelay(void);
static void APP_GyroReleaseScl(void);
static void APP_GyroReleaseSda(void);
static void APP_GyroDriveSclLow(void);
static void APP_GyroDriveSdaLow(void);
static bool APP_GyroReadSda(void);
static void APP_GyroI2cStart(void);
static void APP_GyroI2cStop(void);
static bool APP_GyroI2cWriteByte(uint8_t value);
static uint8_t APP_GyroI2cReadByte(bool ack);
static bool APP_GyroWriteRegister(uint8_t reg, uint8_t value);
static bool APP_GyroReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
static bool APP_GyroReadAngles(void);
static int16_t APP_GyroDecodeAngle(const uint8_t *buffer);
static int16_t APP_GyroNormalizeCentiDeg(int32_t angleCentiDeg);
static int16_t APP_GyroAngleDiffCentiDeg(int16_t targetCentiDeg, int16_t currentCentiDeg);
static int16_t APP_GyroGetRelativeYawCentiDeg(void);
static void APP_PrintDeg(int16_t value);
static void APP_PrintGyroStatus(void);
static void APP_HandleUartByte(uint8_t data);
static void APP_ResetUartFrame(void);
static bool APP_UartHasRxData(LPUART_Type *base);
static void APP_ProcessUartFrame(const char *frame, uint8_t length);
static bool APP_ParseUintField(const char *frame, uint8_t length, uint8_t *pos, uint16_t *value);
static bool APP_ParseFloatField(const char *frame, uint8_t length, uint8_t *pos, float *value);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile uint8_t s_pwmDuty[APP_PWM_CHANNEL_COUNT] = {APP_PWM_DEFAULT_DUTY, APP_PWM_DEFAULT_DUTY};
static char s_uartRxFrame[APP_UART_RX_FRAME_MAX_LEN];
static uint8_t s_uartRxFrameLength;
static bool s_uartFrameActive;
static TickType_t s_uartLastRxTick;
static volatile uint16_t s_targetSpeedCps[APP_PWM_CHANNEL_COUNT];
static volatile int32_t s_signedTargetSpeedCps[APP_PWM_CHANNEL_COUNT];
static volatile int32_t s_activeSignedTargetSpeedCps[APP_PWM_CHANNEL_COUNT];
static volatile int32_t s_actualSpeedCps[APP_PWM_CHANNEL_COUNT];
static volatile bool s_speedControlEnabled[APP_PWM_CHANNEL_COUNT];
static int32_t s_pidIntegral[APP_PWM_CHANNEL_COUNT];
static int32_t s_pidLastError[APP_PWM_CHANNEL_COUNT];
static volatile uint16_t s_servoPulseUs = APP_SERVO_CENTER_US;
static volatile float s_servoAngleDeg = APP_SERVO_NEUTRAL_ANGLE_DEG;
static volatile float s_steerAngleDeg;
#define s_gyroXAngleCentiDeg (s_gyroAngleCentiDeg[0])
#define s_gyroYAngleCentiDeg (s_gyroAngleCentiDeg[1])
#define s_gyroYawCentiDeg    (s_gyroAngleCentiDeg[2])
static volatile int16_t s_gyroAngleCentiDeg[3];
static volatile int16_t s_gyroYawZeroCentiDeg;
static volatile int16_t s_gyroYawErrorCentiDeg;
static volatile bool s_gyroInitialized;
static int32_t s_gyroOffsetSum[3];
static int16_t s_gyroOffsetRaw[3];
static uint8_t s_gyroCalCount;
static volatile bool s_gyroPrintEnabled;
static volatile bool s_gyroAssistEnabled;
static volatile float s_gyroCommandSpeedMps;
static volatile float s_gyroCommandSteerDeg;
static volatile int16_t s_gyroTargetYawCentiDeg;

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t APP_SetupPwmChannel(pwm_submodule_t module, uint8_t dutyCyclePercent)
{
    pwm_signal_param_t pwmSignal;

    pwmSignal.pwmChannel       = APP_PWM_CHANNEL;
    pwmSignal.level            = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent = dutyCyclePercent;
    pwmSignal.deadtimeValue    = 0U;
    pwmSignal.faultState       = kPWM_PwmFaultState0;
    pwmSignal.pwmchannelenable = true;

    return PWM_SetupPwm(BOARD_PWM_BASEADDR, module, &pwmSignal, 1U, APP_PWM_ALIGN_MODE, APP_DEFAULT_PWM_FREQUENCY,
                        PWM_SRC_CLK_FREQ);
}

static void APP_InitPwm(void)
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t servoSignal;

    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.reloadLogic     = kPWM_ReloadPwmFullCycle;
    pwmConfig.pairOperation   = kPWM_Independent;
    pwmConfig.prescale        = kPWM_Prescale_Divide_8;
    pwmConfig.enableDebugMode = true;

    if ((PWM_Init(BOARD_PWM_BASEADDR, APP_PWM1_MODULE, &pwmConfig) == kStatus_Fail) ||
        (PWM_Init(BOARD_PWM_BASEADDR, APP_PWM2_MODULE, &pwmConfig) == kStatus_Fail))
    {
        while (1)
        {
        }
    }

    pwmConfig.prescale = kPWM_Prescale_Divide_128;
    if (PWM_Init(BOARD_PWM_BASEADDR, APP_SERVO_MODULE, &pwmConfig) == kStatus_Fail)
    {
        while (1)
        {
        }
    }

    if ((APP_SetupPwmChannel(APP_PWM1_MODULE, s_pwmDuty[0]) != kStatus_Success) ||
        (APP_SetupPwmChannel(APP_PWM2_MODULE, s_pwmDuty[1]) != kStatus_Success))
    {
        while (1)
        {
        }
    }

    servoSignal.pwmChannel       = APP_PWM_CHANNEL;
    servoSignal.level            = kPWM_HighTrue;
    servoSignal.dutyCyclePercent = 8U;
    servoSignal.deadtimeValue    = 0U;
    servoSignal.faultState       = kPWM_PwmFaultState0;
    servoSignal.pwmchannelenable = true;
    if (PWM_SetupPwm(BOARD_PWM_BASEADDR, APP_SERVO_MODULE, &servoSignal, 1U, APP_PWM_ALIGN_MODE,
                     APP_SERVO_PWM_FREQUENCY, PWM_SRC_CLK_FREQ) != kStatus_Success)
    {
        while (1)
        {
        }
    }

    PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, APP_PWM1_MODULE, APP_PWM_CHANNEL, kPWM_faultchannel_0, 0U);
    PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, APP_PWM2_MODULE, APP_PWM_CHANNEL, kPWM_faultchannel_0, 0U);
    PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, APP_SERVO_MODULE, APP_PWM_CHANNEL, kPWM_faultchannel_0, 0U);

    APP_SetServoAngleDeg(APP_SERVO_NEUTRAL_ANGLE_DEG);

    PWM_SetPwmLdok(BOARD_PWM_BASEADDR, APP_PWM_CONTROL | APP_SERVO_PWM_CONTROL, true);
    PWM_StartTimer(BOARD_PWM_BASEADDR, APP_PWM_CONTROL | APP_SERVO_PWM_CONTROL);
}

static void APP_InitEncoder(TMR_Type *base)
{
    qtmr_config_t qtmrConfig;

    QTMR_GetDefaultConfig(&qtmrConfig);
    qtmrConfig.primarySource   = APP_ENCODER_PRIMARY_PIN;
    qtmrConfig.secondarySource = APP_ENCODER_SECONDARY_PIN;

    QTMR_Init(base, APP_ENCODER_COUNTER, &qtmrConfig);
    QTMR_SetLoadValue(base, APP_ENCODER_COUNTER, 0U);
    base->CHANNEL[APP_ENCODER_COUNTER].CNTR = 0U;
    QTMR_ClearStatusFlags(base, APP_ENCODER_COUNTER,
                          kQTMR_CompareFlag | kQTMR_Compare1Flag | kQTMR_Compare2Flag | kQTMR_OverflowFlag |
                              kQTMR_EdgeFlag);
    QTMR_StartTimer(base, APP_ENCODER_COUNTER, kQTMR_QuadCountMode);
}

static void APP_InitEncoders(void)
{
    APP_InitEncoder(APP_ENCODER1_BASE);
    APP_InitEncoder(APP_ENCODER2_BASE);
}

static void APP_InitMotorDirectionPins(void)
{
    gpio_pin_config_t outputConfig = {
        kGPIO_DigitalOutput,
        0U,
        kGPIO_NoIntmode,
    };

    GPIO_PinInit(APP_MOTOR1_IN1_GPIO, APP_MOTOR1_IN1_PIN, &outputConfig);
    GPIO_PinInit(APP_MOTOR1_IN2_GPIO, APP_MOTOR1_IN2_PIN, &outputConfig);
    GPIO_PinInit(APP_MOTOR2_IN1_GPIO, APP_MOTOR2_IN1_PIN, &outputConfig);
    GPIO_PinInit(APP_MOTOR2_IN2_GPIO, APP_MOTOR2_IN2_PIN, &outputConfig);
}

static void APP_InitGyroI2cPins(void)
{
    gpio_pin_config_t inputConfig = {
        kGPIO_DigitalInput,
        0U,
        kGPIO_NoIntmode,
    };

    GPIO_PinInit(APP_GYRO_SCL_GPIO, APP_GYRO_SCL_PIN, &inputConfig);
    GPIO_PinInit(APP_GYRO_SDA_GPIO, APP_GYRO_SDA_PIN, &inputConfig);
    APP_GyroReleaseScl();
    APP_GyroReleaseSda();
}

static void APP_UpdatePwmDuty(uint8_t index, uint8_t dutyCyclePercent)
{
    if (index < APP_PWM_CHANNEL_COUNT)
    {
        s_speedControlEnabled[index] = false;
        s_targetSpeedCps[index]      = 0U;
        s_signedTargetSpeedCps[index] = 0;
        s_activeSignedTargetSpeedCps[index] = 0;
        s_pidIntegral[index]         = 0;
        s_pidLastError[index]        = 0;
    }

    APP_ApplyPwmDuty(index, dutyCyclePercent);
}

static void APP_ApplyPwmDuty(uint8_t index, uint8_t dutyCyclePercent)
{
    pwm_submodule_t module = APP_PWM1_MODULE;

    if (index >= APP_PWM_CHANNEL_COUNT)
    {
        return;
    }

    if (dutyCyclePercent > 100U)
    {
        dutyCyclePercent = 100U;
    }

    if (s_pwmDuty[index] == dutyCyclePercent)
    {
        return;
    }

    if (index == 1U)
    {
        module = APP_PWM2_MODULE;
    }

    PWM_UpdatePwmDutycycle(BOARD_PWM_BASEADDR, module, APP_PWM_CHANNEL, APP_PWM_ALIGN_MODE, dutyCyclePercent);
    PWM_SetPwmLdok(BOARD_PWM_BASEADDR, (index == 0U) ? kPWM_Control_Module_0 : kPWM_Control_Module_1, true);
    s_pwmDuty[index] = dutyCyclePercent;

}

static void APP_SetSignedSpeedTarget(uint8_t index, int32_t targetSpeedCps)
{
    bool wasDisabled;

    if (targetSpeedCps > APP_WHEEL_MAX_TARGET_CPS)
    {
        targetSpeedCps = APP_WHEEL_MAX_TARGET_CPS;
    }
    else if (targetSpeedCps < -APP_WHEEL_MAX_TARGET_CPS)
    {
        targetSpeedCps = -APP_WHEEL_MAX_TARGET_CPS;
    }
    wasDisabled = !s_speedControlEnabled[index];
    s_signedTargetSpeedCps[index] = targetSpeedCps;
    s_speedControlEnabled[index] = true;

    if (wasDisabled)
    {
        s_activeSignedTargetSpeedCps[index] = 0;
        s_targetSpeedCps[index]             = 0U;
        s_pidIntegral[index]                = 0;
        s_pidLastError[index]               = 0;
        APP_SetMotorDirection(index, targetSpeedCps);
    }

}

static void APP_SetMotorDirection(uint8_t index, int32_t targetSpeedCps)
{
    GPIO_Type *in1Gpio;
    GPIO_Type *in2Gpio;
    uint32_t in1Pin;
    uint32_t in2Pin;
    uint8_t forwardLevel = 0U;
    uint8_t reverseLevel = 0U;

    if (index == 0U)
    {
        in1Gpio = APP_MOTOR1_IN1_GPIO;
        in2Gpio = APP_MOTOR1_IN2_GPIO;
        in1Pin  = APP_MOTOR1_IN1_PIN;
        in2Pin  = APP_MOTOR1_IN2_PIN;
    }
    else if (index == 1U)
    {
        in1Gpio = APP_MOTOR2_IN1_GPIO;
        in2Gpio = APP_MOTOR2_IN2_GPIO;
        in1Pin  = APP_MOTOR2_IN1_PIN;
        in2Pin  = APP_MOTOR2_IN2_PIN;
    }
    else
    {
        return;
    }

    if (targetSpeedCps > 0)
    {
        forwardLevel = 1U;
    }
    else if (targetSpeedCps < 0)
    {
        reverseLevel = 1U;
    }

    GPIO_PinWrite(in1Gpio, in1Pin, forwardLevel);
    GPIO_PinWrite(in2Gpio, in2Pin, reverseLevel);
}

static uint8_t APP_RunPid(uint8_t index, int32_t actualSpeedCps)
{
    int32_t error;
    int32_t derivative;
    int32_t integralCandidate;
    int32_t feedforwardDuty;
    int32_t correction;
    int32_t output;
    int32_t clampedOutput;

    if (index >= APP_PWM_CHANNEL_COUNT)
    {
        return 0U;
    }

    if (s_targetSpeedCps[index] == 0U)
    {
        s_pidIntegral[index]  = 0;
        s_pidLastError[index] = 0;
        return 0U;
    }

    feedforwardDuty = APP_GetFeedforwardDuty(s_targetSpeedCps[index]);
    error      = (int32_t)s_targetSpeedCps[index] - actualSpeedCps;
    derivative = ((error - s_pidLastError[index]) * 1000L) / (int32_t)APP_PID_PERIOD_MS;

    integralCandidate = s_pidIntegral[index] + error;
    if (integralCandidate > APP_PID_INTEGRAL_LIMIT)
    {
        integralCandidate = APP_PID_INTEGRAL_LIMIT;
    }
    else if (integralCandidate < -APP_PID_INTEGRAL_LIMIT)
    {
        integralCandidate = -APP_PID_INTEGRAL_LIMIT;
    }

    correction = (APP_PID_KP_NUM * error) + (APP_PID_KI_NUM * integralCandidate) + (APP_PID_KD_NUM * derivative);
    correction /= APP_PID_GAIN_DEN;
    output = feedforwardDuty + correction;
    clampedOutput = output;

    if (clampedOutput < 0)
    {
        clampedOutput = 0;
    }
    else if (clampedOutput > 100)
    {
        clampedOutput = 100;
    }

    if ((output == clampedOutput) || ((output > 100) && (error < 0)) || ((output < 0) && (error > 0)))
    {
        s_pidIntegral[index] = integralCandidate;
    }

    s_pidLastError[index] = error;

    return (uint8_t)clampedOutput;
}

static uint8_t APP_GetFeedforwardDuty(uint16_t targetSpeedCps)
{
    int32_t duty;

    if (targetSpeedCps == 0U)
    {
        return 0U;
    }

    duty = (((int32_t)targetSpeedCps * 100L) + (APP_SPEED_FEEDFORWARD_MAX_CPS / 2L)) /
           APP_SPEED_FEEDFORWARD_MAX_CPS;

    if (duty < (int32_t)APP_SPEED_MIN_ACTIVE_DUTY)
    {
        duty = APP_SPEED_MIN_ACTIVE_DUTY;
    }
    else if (duty > 100)
    {
        duty = 100;
    }

    return (uint8_t)duty;
}

static void APP_SetServoPulseUs(uint16_t pulseUs)
{
    uint32_t dutyCycle;

    if (pulseUs < APP_SERVO_MIN_US)
    {
        pulseUs = APP_SERVO_MIN_US;
    }
    else if (pulseUs > APP_SERVO_MAX_US)
    {
        pulseUs = APP_SERVO_MAX_US;
    }

    if (pulseUs == s_servoPulseUs)
    {
        return;
    }

    dutyCycle = (((uint32_t)pulseUs * 65535UL) + (APP_SERVO_PERIOD_US / 2UL)) / APP_SERVO_PERIOD_US;
    if (dutyCycle > 65535UL)
    {
        dutyCycle = 65535UL;
    }

    PWM_UpdatePwmDutycycleHighAccuracy(BOARD_PWM_BASEADDR, APP_SERVO_MODULE, APP_PWM_CHANNEL, APP_PWM_ALIGN_MODE,
                                       (uint16_t)dutyCycle);
    PWM_SetPwmLdok(BOARD_PWM_BASEADDR, APP_SERVO_PWM_CONTROL, true);
    s_servoPulseUs = pulseUs;
}

static void APP_SetServoAngleDeg(float servoAngleDeg)
{
    float pulseUs;

    servoAngleDeg = APP_ClampFloat(servoAngleDeg, 0.0f, APP_SERVO_MAX_ANGLE_DEG);
    pulseUs = (float)APP_SERVO_MIN_US +
              ((servoAngleDeg * (float)(APP_SERVO_MAX_US - APP_SERVO_MIN_US)) / APP_SERVO_MAX_ANGLE_DEG);

    APP_SetServoPulseUs((uint16_t)(pulseUs + 0.5f));
    s_servoAngleDeg = servoAngleDeg;
}

static void APP_SetFrontSteeringAngleDeg(float frontSteerDeg)
{
    float servoAngleDeg;

    frontSteerDeg = APP_ClampFloat(frontSteerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);

    if (frontSteerDeg >= 0.0f)
    {
        servoAngleDeg = APP_SERVO_NEUTRAL_ANGLE_DEG +
                        ((frontSteerDeg * (APP_SERVO_LEFT_ANGLE_DEG - APP_SERVO_NEUTRAL_ANGLE_DEG)) /
                         APP_CAR_MAX_STEER_DEG);
    }
    else
    {
        servoAngleDeg = APP_SERVO_NEUTRAL_ANGLE_DEG +
                        ((frontSteerDeg * (APP_SERVO_NEUTRAL_ANGLE_DEG - APP_SERVO_RIGHT_ANGLE_DEG)) /
                         APP_CAR_MAX_STEER_DEG);
    }

    APP_SetServoAngleDeg(servoAngleDeg);
    s_steerAngleDeg = frontSteerDeg;
}

static void APP_ApplyAckermannControl(float speedMps, float steerDeg)
{
    s_gyroAssistEnabled = false;
    APP_ApplyAckermannControlInternal(speedMps, steerDeg);
}

static void APP_ApplyAckermannControlInternal(float speedMps, float steerDeg)
{
    float q;
    float frontSteerDeg;
    float leftWheelSpeed;
    float rightWheelSpeed;
    int32_t leftTargetCps;
    int32_t rightTargetCps;

    speedMps = APP_ClampFloat(speedMps, -APP_CAR_MAX_SPEED_MPS, APP_CAR_MAX_SPEED_MPS);
    steerDeg = APP_ClampFloat(steerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);

    if (APP_AbsFloat(steerDeg) < APP_CAR_SMALL_STEER_DEG)
    {
        steerDeg = 0.0f;
    }

    q             = APP_CalcAckermannQ(steerDeg);
    frontSteerDeg = APP_CalcAckermannFrontSteerDeg(steerDeg);
    leftWheelSpeed  = speedMps * (1.0f - q);
    rightWheelSpeed = speedMps * (1.0f + q);
    leftTargetCps   = APP_MpsToEncoderCps(leftWheelSpeed);
    rightTargetCps  = APP_MpsToEncoderCps(rightWheelSpeed);

    APP_SetFrontSteeringAngleDeg(frontSteerDeg);
    APP_SetSignedSpeedTarget(0U, leftTargetCps);
    APP_SetSignedSpeedTarget(1U, rightTargetCps);
}

static void APP_ApplyGyroAssistedControl(float speedMps, float steerDeg)
{
    speedMps = APP_ClampFloat(speedMps, -APP_CAR_MAX_SPEED_MPS, APP_CAR_MAX_SPEED_MPS);
    steerDeg = APP_ClampFloat(steerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);

    s_gyroCommandSpeedMps = speedMps;
    s_gyroCommandSteerDeg = steerDeg;
    s_gyroTargetYawCentiDeg = APP_GyroGetRelativeYawCentiDeg();
    s_gyroYawErrorCentiDeg = 0;
    s_gyroAssistEnabled = true;

    APP_ApplyAckermannControlInternal(speedMps, steerDeg);
    APP_UpdateGyroAssistedServo();
}

static void APP_UpdateGyroAssistedServo(void)
{
    int16_t relativeYaw;
    int16_t yawError;
    int32_t correctionCentiDeg;
    float correctedSteerDeg;
    float frontSteerDeg;

    if (!s_gyroAssistEnabled || !s_gyroInitialized)
    {
        return;
    }

    relativeYaw = APP_GyroGetRelativeYawCentiDeg();
    yawError = APP_GyroAngleDiffCentiDeg(s_gyroTargetYawCentiDeg, relativeYaw);
    s_gyroYawErrorCentiDeg = yawError;

    correctionCentiDeg = ((int32_t)yawError * APP_GYRO_ASSIST_GAIN_NUM) / APP_GYRO_ASSIST_GAIN_DEN;
    correctedSteerDeg = s_gyroCommandSteerDeg + ((float)correctionCentiDeg / 100.0f);
    correctedSteerDeg = APP_ClampFloat(correctedSteerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);

    frontSteerDeg = APP_CalcAckermannFrontSteerDeg(correctedSteerDeg);
    APP_SetFrontSteeringAngleDeg(frontSteerDeg);
}

static float APP_CalcAckermannQ(float steerDeg)
{
    float steerRad;
    float tanSteer;

    steerDeg = APP_ClampFloat(steerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);
    if (APP_AbsFloat(steerDeg) < APP_CAR_SMALL_STEER_DEG)
    {
        return 0.0f;
    }

    steerRad = steerDeg * APP_DEG_TO_RAD;
    tanSteer = APP_TanApprox(steerRad);

    return (APP_CAR_REAR_TRACK_M / (2.0f * APP_CAR_WHEEL_BASE_M)) * tanSteer;
}

static float APP_CalcAckermannFrontSteerDeg(float steerDeg)
{
    float steerRad;
    float tanSteer;
    float absTanSteer;
    float turnRadius;
    float innerDenominator;
    float outerDenominator;
    float innerFrontDeg;
    float outerFrontDeg;
    float frontSteerDeg;

    steerDeg = APP_ClampFloat(steerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);
    if (APP_AbsFloat(steerDeg) < APP_CAR_SMALL_STEER_DEG)
    {
        return 0.0f;
    }

    steerRad = steerDeg * APP_DEG_TO_RAD;
    tanSteer = APP_TanApprox(steerRad);
    absTanSteer = APP_AbsFloat(tanSteer);
    if (absTanSteer < 0.001f)
    {
        return 0.0f;
    }

    turnRadius = APP_CAR_WHEEL_BASE_M / absTanSteer;
    innerDenominator = turnRadius - (APP_CAR_FRONT_KINGPIN_TRACK_M * 0.5f);
    outerDenominator = turnRadius + (APP_CAR_FRONT_KINGPIN_TRACK_M * 0.5f);
    if (innerDenominator < 0.001f)
    {
        innerDenominator = 0.001f;
    }

    innerFrontDeg = APP_AtanApprox(APP_CAR_WHEEL_BASE_M / innerDenominator) / APP_DEG_TO_RAD;
    outerFrontDeg = APP_AtanApprox(APP_CAR_WHEEL_BASE_M / outerDenominator) / APP_DEG_TO_RAD;
    frontSteerDeg = (innerFrontDeg + outerFrontDeg) * 0.5f;
    if (steerDeg < 0.0f)
    {
        frontSteerDeg = -frontSteerDeg;
    }

    return APP_ClampFloat(frontSteerDeg, -APP_CAR_MAX_STEER_DEG, APP_CAR_MAX_STEER_DEG);
}

static int32_t APP_MpsToEncoderCps(float wheelSpeedMps)
{
    float cps;

    cps = (APP_CAR_GEAR_RATIO * APP_ENCODER_COUNTS_PER_MOTOR_REV * wheelSpeedMps) /
          (APP_PI_F * APP_CAR_WHEEL_DIAMETER_M);

    if (cps >= 0.0f)
    {
        return (int32_t)(cps + 0.5f);
    }

    return (int32_t)(cps - 0.5f);
}

static float APP_TanApprox(float radians)
{
    float x2 = radians * radians;

    return radians + ((radians * x2) / 3.0f) + ((2.0f * radians * x2 * x2) / 15.0f);
}

static float APP_AtanApprox(float value)
{
    float sign = 1.0f;
    float x;
    float result;

    if (value < 0.0f)
    {
        sign  = -1.0f;
        value = -value;
    }

    if (value > 1.0f)
    {
        x      = 1.0f / value;
        result = (APP_PI_F * 0.5f) - (x / (1.0f + (0.28f * x * x)));
    }
    else
    {
        result = value / (1.0f + (0.28f * value * value));
    }

    return sign * result;
}

static float APP_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float APP_ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    else if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static uint16_t APP_ReadEncoderCounter(uint8_t index)
{
    if (index == 0U)
    {
        return QTMR_GetCurrentTimerCount(APP_ENCODER1_BASE, APP_ENCODER_COUNTER);
    }

    return QTMR_GetCurrentTimerCount(APP_ENCODER2_BASE, APP_ENCODER_COUNTER);
}

static int32_t APP_AbsInt32(int32_t value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}

static void APP_PrintSpeedStatus(void)
{
    PRINTF("SPD:%d,%d,%u,%u\n", (int)s_actualSpeedCps[0], (int)s_actualSpeedCps[1], s_pwmDuty[0], s_pwmDuty[1]);
}

static bool APP_IsAnySpeedControlEnabled(void)
{
    uint8_t index;

    for (index = 0U; index < APP_PWM_CHANNEL_COUNT; index++)
    {
        if (s_speedControlEnabled[index])
        {
            return true;
        }
    }

    return false;
}

static void APP_GyroI2cDelay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < APP_GYRO_I2C_DELAY_COUNT; count++)
    {
        __NOP();
    }
}

static void APP_GyroReleaseScl(void)
{
    APP_GYRO_SCL_GPIO->GDIR &= ~(1UL << APP_GYRO_SCL_PIN);
}

static void APP_GyroReleaseSda(void)
{
    APP_GYRO_SDA_GPIO->GDIR &= ~(1UL << APP_GYRO_SDA_PIN);
}

static void APP_GyroDriveSclLow(void)
{
    GPIO_PinWrite(APP_GYRO_SCL_GPIO, APP_GYRO_SCL_PIN, 0U);
    APP_GYRO_SCL_GPIO->GDIR |= (1UL << APP_GYRO_SCL_PIN);
}

static void APP_GyroDriveSdaLow(void)
{
    GPIO_PinWrite(APP_GYRO_SDA_GPIO, APP_GYRO_SDA_PIN, 0U);
    APP_GYRO_SDA_GPIO->GDIR |= (1UL << APP_GYRO_SDA_PIN);
}

static bool APP_GyroReadSda(void)
{
    return (GPIO_PinReadPadStatus(APP_GYRO_SDA_GPIO, APP_GYRO_SDA_PIN) != 0U);
}

static void APP_GyroI2cStart(void)
{
    APP_GyroReleaseSda();
    APP_GyroReleaseScl();
    APP_GyroI2cDelay();
    APP_GyroDriveSdaLow();
    APP_GyroI2cDelay();
    APP_GyroDriveSclLow();
    APP_GyroI2cDelay();
}

static void APP_GyroI2cStop(void)
{
    APP_GyroDriveSdaLow();
    APP_GyroI2cDelay();
    APP_GyroReleaseScl();
    APP_GyroI2cDelay();
    APP_GyroReleaseSda();
    APP_GyroI2cDelay();
}

static bool APP_GyroI2cWriteByte(uint8_t value)
{
    uint8_t bit;
    bool ack;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((value & 0x80U) != 0U)
        {
            APP_GyroReleaseSda();
        }
        else
        {
            APP_GyroDriveSdaLow();
        }

        APP_GyroI2cDelay();
        APP_GyroReleaseScl();
        APP_GyroI2cDelay();
        APP_GyroDriveSclLow();
        value <<= 1U;
    }

    APP_GyroReleaseSda();
    APP_GyroI2cDelay();
    APP_GyroReleaseScl();
    APP_GyroI2cDelay();
    ack = !APP_GyroReadSda();
    APP_GyroDriveSclLow();
    APP_GyroI2cDelay();

    return ack;
}

static uint8_t APP_GyroI2cReadByte(bool ack)
{
    uint8_t bit;
    uint8_t value = 0U;

    APP_GyroReleaseSda();
    for (bit = 0U; bit < 8U; bit++)
    {
        value <<= 1U;
        APP_GyroReleaseScl();
        APP_GyroI2cDelay();
        if (APP_GyroReadSda())
        {
            value |= 1U;
        }
        APP_GyroDriveSclLow();
        APP_GyroI2cDelay();
    }

    if (ack)
    {
        APP_GyroDriveSdaLow();
    }
    else
    {
        APP_GyroReleaseSda();
    }
    APP_GyroI2cDelay();
    APP_GyroReleaseScl();
    APP_GyroI2cDelay();
    APP_GyroDriveSclLow();
    APP_GyroReleaseSda();
    APP_GyroI2cDelay();

    return value;
}

static bool APP_GyroWriteRegister(uint8_t reg, uint8_t value)
{
    bool ok;

    APP_GyroI2cStart();
    ok = APP_GyroI2cWriteByte((uint8_t)(APP_GYRO_I2C_ADDRESS_7BIT << 1U)) && APP_GyroI2cWriteByte(reg) &&
         APP_GyroI2cWriteByte(value);
    APP_GyroI2cStop();

    return ok;
}

static bool APP_GyroReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t index;

    if ((buffer == NULL) || (length == 0U))
    {
        return false;
    }

    APP_GyroI2cStart();
    if (!APP_GyroI2cWriteByte((uint8_t)(APP_GYRO_I2C_ADDRESS_7BIT << 1U)) || !APP_GyroI2cWriteByte(reg))
    {
        APP_GyroI2cStop();
        return false;
    }

    APP_GyroI2cStart();
    if (!APP_GyroI2cWriteByte((uint8_t)((APP_GYRO_I2C_ADDRESS_7BIT << 1U) | 1U)))
    {
        APP_GyroI2cStop();
        return false;
    }

    for (index = 0U; index < length; index++)
    {
        buffer[index] = APP_GyroI2cReadByte(index < (length - 1U));
    }
    APP_GyroI2cStop();

    return true;
}

static bool APP_GyroReadAngles(void)
{
    uint8_t buffer[6];
    uint8_t axis;
    int32_t gyro;

    if (!s_gyroInitialized)
    {
        if (!APP_GyroWriteRegister(APP_GYRO_REG_PWR_MGMT0, 0x0FU))
        {
            return false;
        }
        s_gyroInitialized = true;
        return false;
    }

    if (!APP_GyroReadRegisters(APP_GYRO_REG_GYRO_DATA_X1, buffer, sizeof(buffer)))
    {
        return false;
    }

    if (s_gyroCalCount < APP_GYRO_CALIBRATION_SAMPLES)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            s_gyroOffsetSum[axis] += (int32_t)APP_GyroDecodeAngle(&buffer[axis * 2U]);
        }
        s_gyroCalCount++;
        if (s_gyroCalCount >= APP_GYRO_CALIBRATION_SAMPLES)
        {
            for (axis = 0U; axis < 3U; axis++)
            {
                s_gyroOffsetRaw[axis] = (int16_t)(s_gyroOffsetSum[axis] / (int32_t)APP_GYRO_CALIBRATION_SAMPLES);
                s_gyroAngleCentiDeg[axis] = 0;
            }
            s_gyroYawZeroCentiDeg = 0;
        }
        return true;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        gyro = (int32_t)APP_GyroDecodeAngle(&buffer[axis * 2U]) - (int32_t)s_gyroOffsetRaw[axis];
        s_gyroAngleCentiDeg[axis] = APP_GyroNormalizeCentiDeg((int32_t)s_gyroAngleCentiDeg[axis] +
                                                              ((gyro * (int32_t)APP_GYRO_READ_PERIOD_MS) / 164L));
    }

    return true;
}

static int16_t APP_GyroDecodeAngle(const uint8_t *buffer)
{
    return (int16_t)((((uint16_t)buffer[0]) << 8U) | (uint16_t)buffer[1]);
}

static int16_t APP_GyroNormalizeCentiDeg(int32_t angleCentiDeg)
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

static int16_t APP_GyroAngleDiffCentiDeg(int16_t targetCentiDeg, int16_t currentCentiDeg)
{
    return APP_GyroNormalizeCentiDeg((int32_t)targetCentiDeg - (int32_t)currentCentiDeg);
}

static int16_t APP_GyroGetRelativeYawCentiDeg(void)
{
    return APP_GyroNormalizeCentiDeg((int32_t)s_gyroYawCentiDeg - (int32_t)s_gyroYawZeroCentiDeg);
}

static void APP_PrintDeg(int16_t value)
{
    int32_t temp = value;

    if (temp < 0)
    {
        PUTCHAR('-');
        temp = -temp;
    }
    PRINTF("%u", (uint32_t)(temp / 100L));
}

static void APP_PrintGyroStatus(void)
{
    PRINTF("GYRO:");
    APP_PrintDeg(s_gyroXAngleCentiDeg);
    PUTCHAR(',');
    APP_PrintDeg(s_gyroYAngleCentiDeg);
    PUTCHAR(',');
    APP_PrintDeg(APP_GyroGetRelativeYawCentiDeg());
    PUTCHAR('\n');
}

static void APP_ProcessUartFrame(const char *frame, uint8_t length)
{
    uint8_t pos = 0U;
    uint16_t channel;
    uint16_t value;
    float speedMps;
    float steerDeg;
    uint8_t commandType = 0U;

    if (length < 5U)
    {
        return;
    }

    if ((frame[0] == 'G') && (frame[1] == 'Y') && (frame[2] == 'R') && (frame[3] == 'O') && (frame[4] == ','))
    {
        pos = 5U;
        if (!APP_ParseUintField(frame, length, &pos, &value) || (pos != length) || (value > 2U))
        {
            return;
        }

        if (value == 2U)
        {
            for (channel = 0U; channel < 3U; channel++)
            {
                s_gyroOffsetSum[channel] = 0;
                s_gyroOffsetRaw[channel] = 0;
                s_gyroAngleCentiDeg[channel] = 0;
            }
            s_gyroCalCount = 0;
            s_gyroYawZeroCentiDeg = s_gyroYawCentiDeg;
            s_gyroTargetYawCentiDeg = 0;
            s_gyroYawErrorCentiDeg = 0;
        }
        else
        {
            s_gyroPrintEnabled = (value != 0U);
        }
        return;
    }

    if ((frame[0] == 'G') && (frame[1] == 'C') && (frame[2] == 'A') && (frame[3] == 'R') && (frame[4] == ','))
    {
        pos = 5U;
        if (!APP_ParseFloatField(frame, length, &pos, &speedMps) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }

        pos++;
        if (!APP_ParseFloatField(frame, length, &pos, &steerDeg) || (pos != length))
        {
            return;
        }

        APP_ApplyGyroAssistedControl(speedMps, steerDeg);
        return;
    }

    if ((frame[0] == 'C') && (frame[1] == 'A') && (frame[2] == 'R') && (frame[3] == ','))
    {
        pos = 4U;
        if (!APP_ParseFloatField(frame, length, &pos, &speedMps) || (pos >= length) || (frame[pos] != ','))
        {
            return;
        }

        pos++;
        if (!APP_ParseFloatField(frame, length, &pos, &steerDeg) || (pos != length))
        {
            return;
        }

        APP_ApplyAckermannControl(speedMps, steerDeg);
        return;
    }

    if ((frame[0] == 'S') && (frame[1] == 'E') && (frame[2] == 'R') && (frame[3] == ','))
    {
        pos = 4U;
        if (!APP_ParseFloatField(frame, length, &pos, &steerDeg) || (pos != length))
        {
            return;
        }

        APP_SetServoAngleDeg(steerDeg);
        s_gyroAssistEnabled = false;
        return;
    }

    if ((frame[0] == 'P') && (frame[1] == 'W') && (frame[2] == 'M') && (frame[3] == ','))
    {
        commandType = 1U;
    }
    else if ((frame[0] == 'S') && (frame[1] == 'P') && (frame[2] == 'D') && (frame[3] == ','))
    {
        commandType = 2U;
    }
    else if ((frame[0] == 'D') && (frame[1] == 'I') && (frame[2] == 'R') && (frame[3] == ','))
    {
        commandType = 3U;
    }

    if (commandType == 0U)
    {
        return;
    }

    if (length < 7U)
    {
        return;
    }

    pos = 4U;
    if (!APP_ParseUintField(frame, length, &pos, &channel) || (pos >= length) || (frame[pos] != ','))
    {
        return;
    }

    pos++;
    if (!APP_ParseUintField(frame, length, &pos, &value) || (pos != length))
    {
        return;
    }

    if ((channel < 1U) || (channel > APP_PWM_CHANNEL_COUNT))
    {
        return;
    }

    if (commandType == 1U)
    {
        if (value > 100U)
        {
            return;
        }

        APP_UpdatePwmDuty((uint8_t)(channel - 1U), (uint8_t)value);
    }
    else if (commandType == 2U)
    {
        APP_SetSignedSpeedTarget((uint8_t)(channel - 1U), (int32_t)value);
    }
    else
    {
        if (value == 0U)
        {
            APP_SetMotorDirection((uint8_t)(channel - 1U), 0);
        }
        else if (value == 1U)
        {
            APP_SetMotorDirection((uint8_t)(channel - 1U), 1);
        }
        else if (value == 2U)
        {
            APP_SetMotorDirection((uint8_t)(channel - 1U), -1);
        }
        else
        {
            return;
        }
    }
}

static void APP_HandleUartByte(uint8_t data)
{
    if (data == APP_UART_FRAME_START)
    {
        s_uartFrameActive   = true;
        s_uartRxFrameLength = 0U;
        s_uartLastRxTick    = xTaskGetTickCount();
        return;
    }

    if (!s_uartFrameActive)
    {
        return;
    }

    if ((data == APP_UART_FRAME_END) || (data == '\r') || (data == '\n'))
    {
        APP_ProcessUartFrame(s_uartRxFrame, s_uartRxFrameLength);
        APP_ResetUartFrame();
        return;
    }

    if (s_uartRxFrameLength >= (APP_UART_RX_FRAME_MAX_LEN - 1U))
    {
        APP_ResetUartFrame();
        return;
    }

    s_uartRxFrame[s_uartRxFrameLength] = (char)data;
    s_uartRxFrameLength++;
    s_uartRxFrame[s_uartRxFrameLength] = '\0';
    s_uartLastRxTick = xTaskGetTickCount();
}

static void APP_ResetUartFrame(void)
{
    s_uartFrameActive   = false;
    s_uartRxFrameLength = 0U;
    s_uartRxFrame[0]    = '\0';
}

static bool APP_ParseUintField(const char *frame, uint8_t length, uint8_t *pos, uint16_t *value)
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

static bool APP_ParseFloatField(const char *frame, uint8_t length, uint8_t *pos, float *value)
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

        if ((ch == ',') || (ch == ';'))
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

        if ((ch == ',') || (ch == ';'))
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

static bool APP_UartHasRxData(LPUART_Type *base)
{
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    return (((base->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT) != 0U);
#else
    return ((LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag) != 0U);
#endif
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();
    USER_LED_INIT(LOGIC_LED_OFF);
    APP_InitMotorDirectionPins();
    APP_InitPwm();
    APP_InitEncoders();
    APP_InitGyroI2cPins();

    if ((xTaskCreate(led_task, "LED_task", configMINIMAL_STACK_SIZE + 64U, NULL, led_task_PRIORITY, NULL) != pdPASS) ||
        (xTaskCreate(uart_rx_task, "UART_RX_task", configMINIMAL_STACK_SIZE + 128U, NULL, uart_rx_task_PRIORITY,
                     NULL) != pdPASS) ||
        (xTaskCreate(pid_task, "PID_task", configMINIMAL_STACK_SIZE + 128U, NULL, pid_task_PRIORITY,
                     NULL) != pdPASS))
    {
        while (1)
        {
        }
    }

    vTaskStartScheduler();
    for (;;)
    {
    }
}

/*!
 * @brief Task responsible for 500 ms LED blinking.
 */
static void led_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        USER_LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

static void uart_rx_task(void *pvParameters)
{
    LPUART_Type *uartBase = (LPUART_Type *)BOARD_DEBUG_UART_BASEADDR;
    uint8_t data;

    (void)pvParameters;

    for (;;)
    {
        if ((LPUART_GetStatusFlags(uartBase) & kLPUART_RxOverrunFlag) != 0U)
        {
            (void)LPUART_ClearStatusFlags(uartBase, kLPUART_RxOverrunFlag);
        }

        if (s_uartFrameActive &&
            ((xTaskGetTickCount() - s_uartLastRxTick) > pdMS_TO_TICKS(APP_UART_FRAME_TIMEOUT_MS)))
        {
            APP_ResetUartFrame();
        }

        while (APP_UartHasRxData(uartBase))
        {
            data = LPUART_ReadByte(uartBase);
            APP_HandleUartByte(data);
        }

        taskYIELD();
    }
}

static void pid_task(void *pvParameters)
{
    TickType_t lastWakeTick;
    uint16_t lastEncoderCount[APP_ENCODER_COUNT];
    uint16_t currentEncoderCount;
    int32_t deltaCount;
    int32_t actualSpeed;
    uint32_t statusElapsedMs = 0U;
    uint32_t gyroElapsedMs = 0U;
    uint32_t gyroPrintElapsedMs = 0U;
    int32_t currentTarget;
    int32_t deltaTarget;
    uint8_t index;
    uint8_t duty;

    (void)pvParameters;

    lastWakeTick       = xTaskGetTickCount();
    lastEncoderCount[0] = APP_ReadEncoderCounter(0U);
    lastEncoderCount[1] = APP_ReadEncoderCounter(1U);

    for (;;)
    {
        vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(APP_PID_PERIOD_MS));

        for (index = 0U; index < APP_ENCODER_COUNT; index++)
        {
            currentEncoderCount  = APP_ReadEncoderCounter(index);
            deltaCount           = (int16_t)(currentEncoderCount - lastEncoderCount[index]);
            lastEncoderCount[index] = currentEncoderCount;
            actualSpeed = (int32_t)(((int64_t)APP_AbsInt32(deltaCount) * 1000L) / (int32_t)APP_PID_PERIOD_MS);
            actualSpeed = (int32_t)((((int64_t)s_actualSpeedCps[index] *
                                      (APP_SPEED_FILTER_DEN - APP_SPEED_FILTER_NEW_WEIGHT)) +
                                     ((int64_t)actualSpeed * APP_SPEED_FILTER_NEW_WEIGHT) +
                                     (APP_SPEED_FILTER_DEN / 2L)) /
                                    APP_SPEED_FILTER_DEN);
            s_actualSpeedCps[index] = actualSpeed;

            if (s_speedControlEnabled[index])
            {
                currentTarget = s_activeSignedTargetSpeedCps[index];
                deltaTarget = s_signedTargetSpeedCps[index] - currentTarget;
                if (deltaTarget > APP_SPEED_TARGET_RAMP_CPS_PER_PERIOD)
                {
                    deltaTarget = APP_SPEED_TARGET_RAMP_CPS_PER_PERIOD;
                }
                else if (deltaTarget < -APP_SPEED_TARGET_RAMP_CPS_PER_PERIOD)
                {
                    deltaTarget = -APP_SPEED_TARGET_RAMP_CPS_PER_PERIOD;
                }
                currentTarget += deltaTarget;
                s_activeSignedTargetSpeedCps[index] = currentTarget;
                s_targetSpeedCps[index] = (uint16_t)APP_AbsInt32(currentTarget);
                APP_SetMotorDirection(index, currentTarget);

                duty = APP_RunPid(index, actualSpeed);
                APP_ApplyPwmDuty(index, duty);
            }
        }

        gyroElapsedMs += APP_PID_PERIOD_MS;
        if ((s_gyroAssistEnabled || s_gyroPrintEnabled) && (gyroElapsedMs >= APP_GYRO_READ_PERIOD_MS))
        {
            gyroElapsedMs = 0U;
            if (APP_GyroReadAngles())
            {
                APP_UpdateGyroAssistedServo();
            }
        }

        gyroPrintElapsedMs += APP_PID_PERIOD_MS;
        if (gyroPrintElapsedMs >= APP_GYRO_PRINT_PERIOD_MS)
        {
            gyroPrintElapsedMs = 0U;
            if (s_gyroPrintEnabled)
            {
                APP_PrintGyroStatus();
            }
        }

        statusElapsedMs += APP_PID_PERIOD_MS;
        if (statusElapsedMs >= APP_STATUS_PERIOD_MS)
        {
            statusElapsedMs = 0U;
            if (APP_IsAnySpeedControlEnabled())
            {
                APP_PrintSpeedStatus();
            }
        }
    }
}
