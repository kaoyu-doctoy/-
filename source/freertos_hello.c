/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "app.h"

#include "app/app_tasks.h"
#include "bsp/bsp_encoder.h"
#include "bsp/bsp_motor.h"
#include "bsp/bsp_uart.h"
#include "chassis/chassis.h"
#include "path/planner_benchmark.h"
#include "path/path_planner.h"
#include "protocol/protocol.h"
#include "sensor/imu.h"
#include "sensor/vision.h"

/* 程序入口：完成板级、外设和业务模块初始化，然后启动 FreeRTOS 调度器。 */
int main(void)
{
    BOARD_InitHardware();
    USER_LED_INIT(LOGIC_LED_OFF);

#if APP_PLANNER_BENCHMARK_ENABLE
    PlannerBenchmark_RunAll();
    USER_LED_ON();
    for (;;)
    {
        __NOP();
    }
#endif

    BSP_MotorInit();
    BSP_EncoderInit();
    BSP_UartInit();

    Chassis_Init();
    Protocol_Init();
    Vision_Init();
    PathPlanner_Init();
    (void)IMU_Init();

    AppTasks_Create();
    vTaskStartScheduler();

    for (;;)
    {
    }
}
