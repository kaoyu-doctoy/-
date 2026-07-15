#include "planner_benchmark.h"

#if APP_PLANNER_BENCHMARK_ENABLE

#include "planner.h"

#include "fsl_common.h"

#include <stdbool.h>
#include <limits.h>
#include <string.h>

#define PLANNER_BENCHMARK_STEP_CALL_LIMIT (200000U)
#define PLANNER_BENCHMARK_STORAGE \
    __attribute__((used, section("PathPlannerRam"), aligned(32)))

typedef struct
{
    const char *name;
    const char *rows;
    uint8_t destinationLabels[3];
    uint32_t expectedStepCalls;
    uint32_t expectedPlanSteps;
} planner_benchmark_case_t;

/*
 * 地图来自：
 * 第二十一届全国大学生智能汽车竞赛智能视觉组调试环境搭建软件/
 * 上位机/比赛版本/SmartCar_VR_Race1.2/map_file
 *
 * 每张地图的三个箱子按地图扫描顺序赋值 0、1、2。destinationLabels
 * 是三个目标点按扫描顺序对应的标签。这里选择的是主机穷举六种配对后，
 * 可解方案中 planner_job_step(1) 调用次数最多的一组，用于压力测试。
 */
static const planner_benchmark_case_t s_cases[PLANNER_BENCHMARK_CASE_COUNT] = {
    {
        "level1/-map1",
        "################"
        "#----#---------#"
        "#----#---------#"
        "#--------.-----#"
        "#---$-----.----#"
        "#------$-------#"
        "#-----##-------#"
        "#-----#--------#"
        "#--------.-----#"
        "#----$---------#"
        "#--------------#"
        "################",
        {1U, 2U, 0U}, 60U, 12U
    },
    {
        "level1/-map2",
        "################"
        "#-#------------#"
        "#-.------#####-#"
        "##$###---#---#-#"
        "#----#---#.#-#-#"
        "#----#####.#-#-#"
        "#-------$--$-#-#"
        "#-----------##-#"
        "#--------------#"
        "#-----####-----#"
        "#--------------#"
        "################",
        {2U, 0U, 1U}, 232U, 21U
    },
    {
        "level1/-map3",
        "################"
        "#-#---#--#----.#"
        "#---#--#-####--#"
        "##-###-#-#-----#"
        "#-.#-#-#-#--#--#"
        "#-##-#---------#"
        "#--#-------##--#"
        "#--------------#"
        "#--$-#-----$---#"
        "#-$--########--#"
        "#----#.--------#"
        "################",
        {2U, 1U, 0U}, 305U, 28U
    },
    {
        "level2/map1",
        "################"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "#------$-...---#"
        "#------$-------#"
        "#------$-------#"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "################",
        {0U, 1U, 2U}, 23U, 3U
    },
    {
        "level2/map2",
        "################"
        "#-#------------#"
        "#--------#####-#"
        "##$###---#---#-#"
        "#----#---#-#-#-#"
        "#----#####-#-#-#"
        "#-------$--$-#-#"
        "#---...-----##-#"
        "#--------------#"
        "#-----####-----#"
        "#--------------#"
        "################",
        {2U, 1U, 0U}, 56U, 12U
    },
    {
        "level2/map3",
        "################"
        "#-#---#--#-----#"
        "#---#--#-####--#"
        "##-###-#-#-----#"
        "#--#-#-#-#--#--#"
        "#-##-#---...---#"
        "#--#-------##--#"
        "#-----------#--#"
        "#--$-#-----$---#"
        "#-$--########--#"
        "#----#---------#"
        "################",
        {1U, 2U, 0U}, 89U, 20U
    },
    {
        "level3/map1",
        "################"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "#------$-...---#"
        "#------$-------#"
        "#------$-------#"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "#--------------#"
        "################",
        {0U, 1U, 2U}, 23U, 3U
    },
    {
        "level3/map2",
        "################"
        "#-#------------#"
        "#-.------#####-#"
        "##$###---#---#-#"
        "#----#---#.#-#-#"
        "#----#####.#-#-#"
        "#-------$--$-#-#"
        "#-----------##-#"
        "#--------------#"
        "#-----####-----#"
        "#--------------#"
        "################",
        {2U, 0U, 1U}, 232U, 21U
    },
    {
        "level3/map3",
        "################"
        "#-#---#--#----.#"
        "#---#--#-####--#"
        "##-###-#-#-----#"
        "#-.#-#-#-#--#--#"
        "#-##-#---------#"
        "#--#-------##--#"
        "#-*-------*-#--#"
        "#--$-#-----$*--#"
        "#-$--########--#"
        "#----#.--------#"
        "################",
        {2U, 1U, 0U}, 8813U, 18U
    }
};

PLANNER_BENCHMARK_STORAGE
volatile planner_benchmark_report_t g_plannerBenchmarkReport;

PLANNER_BENCHMARK_STORAGE static PlannerState s_benchmarkState;
PLANNER_BENCHMARK_STORAGE static PlannerPlan s_benchmarkPlan;

static uint32_t PlannerBenchmark_CyclesNow(void)
{
    return DWT->CYCCNT;
}

static uint32_t PlannerBenchmark_CyclesToUs(uint64_t cycles)
{
    uint32_t cyclesPerUs = SystemCoreClock / 1000000U;
    uint64_t value;
    if (cyclesPerUs == 0U)
    {
        return UINT32_MAX;
    }
    value = cycles / cyclesPerUs;
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static void PlannerBenchmark_CopyName(char target[24], const char *source)
{
    uint32_t index = 0U;
    while (index < 23U && source[index] != '\0')
    {
        target[index] = source[index];
        index++;
    }
    target[index] = '\0';
}

static bool PlannerBenchmark_PrepareState(const planner_benchmark_case_t *testCase)
{
    int objectIndex;
    uint8_t boxLabel = 0U;
    uint8_t destinationIndex = 0U;

    planner_state_clear(&s_benchmarkState);
    if (!planner_state_set_map_rows(
            &s_benchmarkState,
            PLANNER_MAX_WIDTH,
            PLANNER_MAX_HEIGHT,
            testCase->rows) ||
        !planner_state_set_pose(
            &s_benchmarkState,
            1,
            1,
            PLANNER_DIR_RIGHT,
            0))
    {
        return false;
    }

    for (objectIndex = 0; objectIndex < s_benchmarkState.object_count; objectIndex++)
    {
        PlannerObject *object = &s_benchmarkState.objects[objectIndex];
        int result;
        if (object->kind == PLANNER_OBJECT_BOX)
        {
            if (boxLabel >= 3U)
            {
                return false;
            }
            result = planner_state_apply_recognition(
                &s_benchmarkState,
                object->id,
                boxLabel);
            boxLabel++;
        }
        else if (object->kind == PLANNER_OBJECT_DESTINATION)
        {
            if (destinationIndex >= 3U)
            {
                return false;
            }
            result = planner_state_apply_recognition(
                &s_benchmarkState,
                object->id,
                10 + testCase->destinationLabels[destinationIndex]);
            destinationIndex++;
        }
        else
        {
            return false;
        }
        if (result != PLANNER_RECOGNITION_ACCEPTED)
        {
            return false;
        }
    }

    return boxLabel == 3U && destinationIndex == 3U;
}

static void PlannerBenchmark_RunCase(
    uint32_t caseIndex,
    const planner_benchmark_case_t *testCase)
{
    volatile planner_benchmark_result_t *result =
        &g_plannerBenchmarkReport.results[caseIndex];
    uint32_t startCycles;
    uint32_t elapsedCycles;
    int status;

    memset((void *)result, 0, sizeof(*result));
    result->caseIndex = caseIndex;
    result->expectedStepCalls = testCase->expectedStepCalls;
    result->expectedPlanSteps = testCase->expectedPlanSteps;
    PlannerBenchmark_CopyName((char *)result->caseName, testCase->name);

    startCycles = PlannerBenchmark_CyclesNow();
    result->setupOk = PlannerBenchmark_PrepareState(testCase) ? 1U : 0U;
    result->setupCycles = PlannerBenchmark_CyclesNow() - startCycles;
    if (!result->setupOk)
    {
        result->plannerStatus = PLANNER_JOB_FAILED;
        return;
    }

    memset(&s_benchmarkPlan, 0, sizeof(s_benchmarkPlan));
    startCycles = PlannerBenchmark_CyclesNow();
    status = planner_job_begin(&s_benchmarkState);
    result->beginCycles = PlannerBenchmark_CyclesNow() - startCycles;

    while (status == PLANNER_JOB_RUNNING &&
           result->stepCalls < PLANNER_BENCHMARK_STEP_CALL_LIMIT)
    {
        startCycles = PlannerBenchmark_CyclesNow();
        status = planner_job_step(1U, &s_benchmarkPlan);
        elapsedCycles = PlannerBenchmark_CyclesNow() - startCycles;
        result->stepCycles += elapsedCycles;
        result->stepCalls++;
        if (elapsedCycles > result->maxStepCycles)
        {
            result->maxStepCycles = elapsedCycles;
        }
    }

    result->plannerStatus = status;
    result->success = s_benchmarkPlan.success;
    result->errorCode = s_benchmarkPlan.error_code;
    result->score = s_benchmarkPlan.score;
    result->planSteps = s_benchmarkPlan.step_count;
    result->totalCycles = (uint64_t)result->beginCycles + result->stepCycles;
    result->totalUs = PlannerBenchmark_CyclesToUs(result->totalCycles);
    result->maxStepUs = PlannerBenchmark_CyclesToUs(result->maxStepCycles);
    result->passed =
        status == PLANNER_JOB_DONE &&
        s_benchmarkPlan.success == 1 &&
        result->stepCalls == result->expectedStepCalls &&
        (uint32_t)s_benchmarkPlan.step_count == result->expectedPlanSteps;
}

void PlannerBenchmark_RunAll(void)
{
    uint32_t caseIndex;

    memset((void *)&g_plannerBenchmarkReport, 0, sizeof(g_plannerBenchmarkReport));
    g_plannerBenchmarkReport.magic = PLANNER_BENCHMARK_MAGIC;
    g_plannerBenchmarkReport.version = PLANNER_BENCHMARK_VERSION;
    g_plannerBenchmarkReport.state = PLANNER_BENCHMARK_RUNNING;
    g_plannerBenchmarkReport.systemCoreClockHz = SystemCoreClock;
    g_plannerBenchmarkReport.caseCount = PLANNER_BENCHMARK_CASE_COUNT;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    for (caseIndex = 0U; caseIndex < PLANNER_BENCHMARK_CASE_COUNT; caseIndex++)
    {
        g_plannerBenchmarkReport.currentCase = caseIndex;
        PlannerBenchmark_RunCase(caseIndex, &s_cases[caseIndex]);
        if (g_plannerBenchmarkReport.results[caseIndex].passed)
        {
            g_plannerBenchmarkReport.passedCaseCount++;
        }
    }

    g_plannerBenchmarkReport.currentCase = PLANNER_BENCHMARK_CASE_COUNT;
    g_plannerBenchmarkReport.allPassed =
        g_plannerBenchmarkReport.passedCaseCount == PLANNER_BENCHMARK_CASE_COUNT;
    g_plannerBenchmarkReport.state = g_plannerBenchmarkReport.allPassed ?
        PLANNER_BENCHMARK_COMPLETE : PLANNER_BENCHMARK_FAILED;
    __DSB();
    __ISB();
}

#endif
