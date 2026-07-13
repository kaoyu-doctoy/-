#include "bsp_motor.h"
#include "../app/app_config.h"

#include "fsl_gpio.h"
#include "fsl_pwm.h"
#include "board.h"
#include "app.h"

#define MOTOR_PWM_CHANNEL (kPWM_PwmA)
#define MOTOR_PWM_ALIGN_MODE (kPWM_SignedCenterAligned)
#define MOTOR_PHASE_FORWARD_LEVEL (1U)
#define MOTOR_PHASE_REVERSE_LEVEL (0U)

typedef struct
{
    pwm_submodule_t pwmModule;
    pwm_module_control_t pwmControl;
    GPIO_Type *phaseGpio;
    uint32_t phasePin;
} motor_hw_t;

static const motor_hw_t s_motorHw[MOTOR_COUNT] = {
    {kPWM_Module_0, kPWM_Control_Module_0, GPIO1, 0U},
    {kPWM_Module_1, kPWM_Control_Module_1, GPIO1, 1U},
    {kPWM_Module_2, kPWM_Control_Module_2, GPIO1, 2U},
    {kPWM_Module_3, kPWM_Control_Module_3, GPIO1, 3U},
};

static volatile uint16_t s_motorDuty[MOTOR_COUNT];

/* 检查电机编号是否在有效范围内。 */
static bool Motor_IsValid(motor_id_t motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT);
}

/* 配置单个 FLEXPWM 子模块输出指定初始占空比。 */
static status_t Motor_SetupPwm(pwm_submodule_t module, uint8_t duty)
{
    pwm_signal_param_t signal;

    signal.pwmChannel = MOTOR_PWM_CHANNEL;
    signal.level = kPWM_HighTrue;
    signal.dutyCyclePercent = duty;
    signal.deadtimeValue = 0U;
    signal.faultState = kPWM_PwmFaultState0;
    signal.pwmchannelenable = true;

    return PWM_SetupPwm(BOARD_PWM_BASEADDR, module, &signal, 1U, MOTOR_PWM_ALIGN_MODE,
                        APP_PWM_FREQUENCY_HZ, PWM_SRC_CLK_FREQ);
}

/* 初始化四路电机 PWM、PH 方向脚和驱动使能脚。 */
void BSP_MotorInit(void)
{
    pwm_config_t pwmConfig;
    gpio_pin_config_t outputConfig = {kGPIO_DigitalOutput, 0U, kGPIO_NoIntmode};
    uint8_t index;

    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    pwmConfig.pairOperation = kPWM_Independent;
    pwmConfig.prescale = kPWM_Prescale_Divide_8;
    pwmConfig.enableDebugMode = true;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        GPIO_PinInit(s_motorHw[index].phaseGpio, s_motorHw[index].phasePin, &outputConfig);
        s_motorDuty[index] = 0U;

        if (PWM_Init(BOARD_PWM_BASEADDR, s_motorHw[index].pwmModule, &pwmConfig) == kStatus_Fail)
        {
            while (1)
            {
            }
        }

        if (Motor_SetupPwm(s_motorHw[index].pwmModule, 0U) != kStatus_Success)
        {
            while (1)
            {
            }
        }

        PWM_SetupFaultDisableMap(BOARD_PWM_BASEADDR, s_motorHw[index].pwmModule, MOTOR_PWM_CHANNEL,
                                 kPWM_faultchannel_0, 0U);
    }

    outputConfig.outputLogic = 1U;
    GPIO_PinInit(GPIO1, 24U, &outputConfig);

    PWM_SetPwmLdok(BOARD_PWM_BASEADDR,
                   kPWM_Control_Module_0 | kPWM_Control_Module_1 | kPWM_Control_Module_2 | kPWM_Control_Module_3,
                   true);
    PWM_StartTimer(BOARD_PWM_BASEADDR,
                   kPWM_Control_Module_0 | kPWM_Control_Module_1 | kPWM_Control_Module_2 | kPWM_Control_Module_3);
}

/* 控制 DRV8701E 的睡眠/使能脚，关闭时电机不输出。 */
void BSP_MotorEnable(bool enable)
{
    GPIO_PinWrite(GPIO1, 24U, enable ? 1U : 0U);
}

/* 设置指定电机 PWM 占空比，并限制在 0 到 100。 */
void BSP_MotorSetDuty(motor_id_t motor, uint16_t duty)
{
    uint32_t highAccuracyDuty;

    if (!Motor_IsValid(motor))
    {
        return;
    }

    if (duty > (uint16_t)APP_PWM_DUTY_MAX)
    {
        duty = (uint16_t)APP_PWM_DUTY_MAX;
    }

    if (s_motorDuty[motor] == duty)
    {
        return;
    }

    highAccuracyDuty = (((uint32_t)duty * 65535UL) + ((uint32_t)APP_PWM_DUTY_MAX / 2UL)) /
                       (uint32_t)APP_PWM_DUTY_MAX;
    PWM_UpdatePwmDutycycleHighAccuracy(BOARD_PWM_BASEADDR, s_motorHw[motor].pwmModule, MOTOR_PWM_CHANNEL,
                                       MOTOR_PWM_ALIGN_MODE, (uint16_t)highAccuracyDuty);
    PWM_SetPwmLdok(BOARD_PWM_BASEADDR, s_motorHw[motor].pwmControl, true);
    s_motorDuty[motor] = duty;
}

/* 设置指定电机 PH 方向脚电平。 */
void BSP_MotorSetDirection(motor_id_t motor, int8_t direction)
{
    uint8_t phaseLevel = MOTOR_PHASE_FORWARD_LEVEL;

    if (!Motor_IsValid(motor))
    {
        return;
    }

    if (direction < 0)
    {
        phaseLevel = MOTOR_PHASE_REVERSE_LEVEL;
    }

    GPIO_PinWrite(s_motorHw[motor].phaseGpio, s_motorHw[motor].phasePin, phaseLevel);
}

/* 设置带符号占空比：符号决定方向，绝对值决定 PWM。 */
void BSP_MotorSetSignedDuty(motor_id_t motor, int16_t signedDuty)
{
    uint16_t duty;

    if (!Motor_IsValid(motor))
    {
        return;
    }

    if (signedDuty > (int16_t)APP_PWM_DUTY_MAX)
    {
        signedDuty = (int16_t)APP_PWM_DUTY_MAX;
    }
    else if (signedDuty < -(int16_t)APP_PWM_DUTY_MAX)
    {
        signedDuty = -(int16_t)APP_PWM_DUTY_MAX;
    }

    BSP_MotorSetDirection(motor, (signedDuty < 0) ? -1 : 1);
    duty = (uint16_t)((signedDuty < 0) ? -signedDuty : signedDuty);
    BSP_MotorSetDuty(motor, duty);
}

/* 停止单个电机。 */
void BSP_MotorStop(motor_id_t motor)
{
    BSP_MotorSetDuty(motor, 0U);
}

/* 停止全部电机。 */
void BSP_MotorStopAll(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)MOTOR_COUNT; index++)
    {
        BSP_MotorStop((motor_id_t)index);
    }
}
