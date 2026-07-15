#ifndef PLANNER_BENCHMARK_H
#define PLANNER_BENCHMARK_H

#include <stdint.h>

#include "../app/app_config.h"

#if APP_PLANNER_BENCHMARK_ENABLE

#define PLANNER_BENCHMARK_CASE_COUNT (9U)
#define PLANNER_BENCHMARK_MAGIC      (0x50424E43UL)
#define PLANNER_BENCHMARK_VERSION    (1U)

typedef enum
{
    PLANNER_BENCHMARK_IDLE = 0,
    PLANNER_BENCHMARK_RUNNING,
    PLANNER_BENCHMARK_COMPLETE,
    PLANNER_BENCHMARK_FAILED
} planner_benchmark_state_t;

typedef struct
{
    uint32_t caseIndex;
    char caseName[24];
    uint32_t setupOk;
    uint32_t setupCycles;
    uint32_t beginCycles;
    uint64_t stepCycles;
    uint64_t totalCycles;
    uint32_t totalUs;
    uint32_t stepCalls;
    uint32_t maxStepCycles;
    uint32_t maxStepUs;
    int32_t plannerStatus;
    int32_t success;
    int32_t errorCode;
    int32_t score;
    int32_t planSteps;
    uint32_t expectedStepCalls;
    uint32_t expectedPlanSteps;
    uint32_t passed;
} planner_benchmark_result_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t state;
    uint32_t systemCoreClockHz;
    uint32_t caseCount;
    uint32_t currentCase;
    uint32_t passedCaseCount;
    uint32_t allPassed;
    planner_benchmark_result_t results[PLANNER_BENCHMARK_CASE_COUNT];
} planner_benchmark_report_t;

extern volatile planner_benchmark_report_t g_plannerBenchmarkReport;

void PlannerBenchmark_RunAll(void);

#endif

#endif
