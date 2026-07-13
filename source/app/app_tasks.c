#include "app_tasks.h"
#include "app_config.h"
#include "../bsp/bsp_uart.h"
#include "../chassis/chassis.h"
#include "../path/path_planner.h"
#include "../protocol/protocol.h"
#include "../sensor/imu.h"
#include "../sensor/vision.h"

#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

#define APP_LED_TASK_PRIORITY     (configMAX_PRIORITIES - 5)
#define APP_UART_TASK_PRIORITY    (configMAX_PRIORITIES - 3)
#define APP_CONTROL_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define APP_IMU_TASK_PRIORITY     (configMAX_PRIORITIES - 4)
#define APP_VISION_TASK_PRIORITY  (configMAX_PRIORITIES - 4)
#define APP_PATH_TASK_PRIORITY    (configMAX_PRIORITIES - 4)

/* LED 心跳任务，用于观察系统调度是否仍在运行。 */
static void LedTask(void *param);
/* 调试串口任务，接收并解析上位机/串口助手命令。 */
static void UartTask(void *param);
/* 底盘控制任务，按固定周期执行四轮速度闭环。 */
static void ControlTask(void *param);
/* IMU 任务，周期读取陀螺仪并按需打印状态。 */
static void ImuTask(void *param);
/* 视觉任务，周期轮询两个视觉串口的接收缓冲。 */
static void VisionTask(void *param);
/* 路径任务，周期执行路径规划状态机。 */
static void PathTask(void *param);

/* 创建本工程运行所需的全部 FreeRTOS 任务。 */
void AppTasks_Create(void)
{
    if ((xTaskCreate(LedTask, "LED_task", configMINIMAL_STACK_SIZE + 64U, 0, APP_LED_TASK_PRIORITY, 0) != pdPASS) ||
        (xTaskCreate(UartTask, "UART_task", configMINIMAL_STACK_SIZE + 128U, 0, APP_UART_TASK_PRIORITY, 0) != pdPASS) ||
        (xTaskCreate(ControlTask, "CTRL_task", configMINIMAL_STACK_SIZE + 128U, 0,
                     APP_CONTROL_TASK_PRIORITY, 0) != pdPASS) ||
        (xTaskCreate(ImuTask, "IMU_task", configMINIMAL_STACK_SIZE + 128U, 0, APP_IMU_TASK_PRIORITY, 0) != pdPASS) ||
        (xTaskCreate(VisionTask, "VISION_task", configMINIMAL_STACK_SIZE + 128U, 0,
                     APP_VISION_TASK_PRIORITY, 0) != pdPASS) ||
        (xTaskCreate(PathTask, "PATH_task", configMINIMAL_STACK_SIZE + 128U, 0, APP_PATH_TASK_PRIORITY, 0) != pdPASS))
    {
        while (1)
        {
        }
    }
}

/* 周期翻转用户 LED，作为系统仍在运行的简单指示。 */
static void LedTask(void *param)
{
    (void)param;

    for (;;)
    {
        USER_LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

/* 轮询调试串口字节流，并交给协议层按帧处理。 */
static void UartTask(void *param)
{
    uint8_t data;

    (void)param;

    for (;;)
    {
        Protocol_CheckTimeout();
        while (BSP_DebugUartReadByte(&data))
        {
            Protocol_InputByte(data);
        }
        Protocol_PeriodicSend();
        taskYIELD();
    }
}

/* 固定周期调用底盘控制更新，保证 PID 周期稳定。 */
static void ControlTask(void *param)
{
    TickType_t lastWakeTick;

    (void)param;

    lastWakeTick = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
        Chassis_UpdateControl();
    }
}

/* 固定周期更新 IMU 数据，并根据开关决定是否输出角度。 */
static void ImuTask(void *param)
{
    TickType_t lastWakeTick;
    uint32_t printElapsedMs = 0U;

    (void)param;

    lastWakeTick = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(APP_IMU_PERIOD_MS));
        (void)IMU_Update();

        printElapsedMs += APP_IMU_PERIOD_MS;
        if (printElapsedMs >= APP_IMU_PRINT_PERIOD_MS)
        {
            printElapsedMs = 0U;
            if (IMU_IsPrintEnabled())
            {
                IMU_PrintStatus();
            }
        }
    }
}

/* 周期读取两个视觉串口，后续视觉协议解析会基于这里的缓冲数据。 */
static void VisionTask(void *param)
{
    (void)param;

    for (;;)
    {
        Vision_Poll();
        vTaskDelay(pdMS_TO_TICKS(APP_VISION_UART_POLL_PERIOD_MS));
    }
}

/* 周期运行路径规划逻辑，未来在这里把视觉/IMU结果转换成底盘目标。 */
static void PathTask(void *param)
{
    TickType_t lastWakeTick;

    (void)param;

    lastWakeTick = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(APP_PATH_PERIOD_MS));
        PathPlanner_Update();
    }
}
