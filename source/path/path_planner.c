#include "path_planner.h"

#include "../bsp/bsp_encoder.h"
#include "../chassis/chassis.h"
#include "../sensor/imu.h"
#include "../sensor/vision.h"

#include "FreeRTOS.h"
#include "fsl_common.h"
#include "semphr.h"
#include "task.h"

#include <math.h>
#include <string.h>

typedef enum
{
    PLANNER_JOB_MODE_NONE = 0,
    PLANNER_JOB_MODE_CURRENT,
    PLANNER_JOB_MODE_PREDICTED
} planner_job_mode_t;

static path_state_t s_pathState;
static mission_state_t s_missionState;
static SemaphoreHandle_t s_stateMutex;

/* PlannerState 是 car 中地图、物体标签、炸弹和离散位姿的唯一真值。 */
static PlannerState s_plannerState;
static PlannerState s_predictedState;
static PlannerState s_stepExpectedState;

static PlannerPlan s_activePlan;
static PlannerPlan s_queuedPlan;
static PlannerPlan s_jobPlan;
static bool s_activePlanValid;
static bool s_queuedPlanValid;
static bool s_predictionValid;
static bool s_predictionComplete;
static bool s_manualPlan;
static uint16_t s_activeStepIndex;
static planner_job_mode_t s_jobMode;
static int s_plannerJobStatus;

static uint16_t s_cellSizeMm;
static path_dir_t s_heading;

static bool s_motionActive;
static float s_motionVx;
static float s_motionVy;
static int16_t s_motionTargetYawCentiDeg;
static int32_t s_motionTargetCounts;
static int32_t s_motionTravelledCounts;

static path_dir_t s_rotationTargetHeading;
static int16_t s_rotationTargetYawCentiDeg;
static uint8_t s_rotationStableCount;
static TickType_t s_missionStateTick;

static recognition_point_t s_recognitionPoints[APP_MISSION_MAX_RECOGNITION_POINTS];
static uint16_t s_recognitionPointCount;
static uint16_t s_recognizedCount;
static uint16_t s_currentRecognitionPoint;
static uint16_t s_missionRequestSequence;
static uint16_t s_missionRetryCount;
static bool s_missionMapAccepted;
static uint8_t s_currentLevel;
static uint8_t s_completedLevels;
static uint8_t s_initialBoxCount;
static bool s_startAreaKnown;
static bool s_inStartArea;
static bool s_entryPoseValid;
static uint8_t s_entryX;
static uint8_t s_entryY;
static path_dir_t s_entryHeading;
static bool s_returningToStart;
static bool s_levelSolved;
static bool s_levelTimerRunning;
static TickType_t s_levelStartTick;
static TickType_t s_nextLevelStartTick;
static uint32_t s_levelElapsedMs;

static bool s_pendingPoseValid;
static uint8_t s_pendingPoseX;
static uint8_t s_pendingPoseY;
static path_dir_t s_pendingPoseHeading;
static uint32_t s_poseRevision;
static uint32_t s_motionStartPoseRevision;

static const int8_t s_plannerDx[4] = {0, 1, 0, -1};
static const int8_t s_plannerDy[4] = {-1, 0, 1, 0};

static void Mission_TryBeginPlanningLocked(void);
static void Planner_InstallPlanLocked(const PlannerPlan *plan, bool manualPlan);
static void Mission_FinishActivePlanLocked(void);
static void Mission_ConfirmMotionLocked(void);
static void Mission_BeginReturnLocked(bool solved);
static void Mission_StartLeavingLocked(void);
static void Mission_StartEnteringLocked(void);
static void PathPlanner_ClearRuntimeLocked(void);
static void Mission_ClearRecognitionLocked(void);
static void Mission_SetFinishedLocked(void);
static void PathPlanner_StartRotationLocked(path_dir_t targetHeading);
static uint32_t Mission_ElapsedMsSince(TickType_t startTick);

static bool PathPlanner_Lock(void)
{
    return (s_stateMutex != 0) && (xSemaphoreTake(s_stateMutex, portMAX_DELAY) == pdTRUE);
}

static void PathPlanner_Unlock(void)
{
    (void)xSemaphoreGive(s_stateMutex);
}

static bool PathPlanner_IsValidDir(path_dir_t dir)
{
    return ((uint32_t)dir <= (uint32_t)PATH_DIR_Y_NEG);
}

static int32_t PathPlanner_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t PathPlanner_MaxInt32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}

static float PathPlanner_ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

static int PathPlanner_PathDirToPlannerDir(path_dir_t dir)
{
    return (((int)dir + 1) & 3);
}

static path_dir_t PathPlanner_PlannerDirToPathDir(int dir)
{
    return (path_dir_t)((dir + 3) & 3);
}

static bool PathPlanner_MapReadyLocked(void)
{
    return (s_plannerState.width > 0) && (s_plannerState.height > 0);
}

static int16_t PathPlanner_NormalizeYaw(int32_t yawCentiDeg)
{
    while (yawCentiDeg > 18000L)
    {
        yawCentiDeg -= 36000L;
    }
    while (yawCentiDeg < -18000L)
    {
        yawCentiDeg += 36000L;
    }
    return (int16_t)yawCentiDeg;
}

static int16_t PathPlanner_HeadingToYaw(path_dir_t heading)
{
    int32_t yaw = APP_MAP_X_POS_YAW_CDEG +
                  (APP_MAP_YAW_DIRECTION_SIGN * (int32_t)heading * 9000L);
    return PathPlanner_NormalizeYaw(yaw);
}

static int16_t PathPlanner_GetYawError(int16_t targetCentiDeg, int16_t currentCentiDeg)
{
    return PathPlanner_NormalizeYaw((int32_t)targetCentiDeg - (int32_t)currentCentiDeg);
}

static int16_t PathPlanner_GetCurrentYaw(void)
{
    int16_t yaw;
    if (Vision_GetYawCentiDeg(&yaw))
    {
        return yaw;
    }
    return IMU_GetRelativeYawCentiDeg();
}

static uint16_t Mission_NextSequence(void)
{
    s_missionRequestSequence++;
    if (s_missionRequestSequence == 0U)
    {
        s_missionRequestSequence = 1U;
    }
    return s_missionRequestSequence;
}

static PlannerObject *PathPlanner_FindObjectByIdLocked(int objectId)
{
    int index;
    for (index = 0; index < s_plannerState.object_count; index++)
    {
        if (s_plannerState.objects[index].id == objectId)
        {
            return &s_plannerState.objects[index];
        }
    }
    return 0;
}

static void PathPlanner_ClearRuntimeLocked(void)
{
    Chassis_Stop();
    memset(&s_activePlan, 0, sizeof(s_activePlan));
    memset(&s_queuedPlan, 0, sizeof(s_queuedPlan));
    memset(&s_jobPlan, 0, sizeof(s_jobPlan));
    s_activePlanValid = false;
    s_queuedPlanValid = false;
    s_predictionValid = false;
    s_predictionComplete = false;
    s_manualPlan = false;
    s_activeStepIndex = 0U;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    s_plannerJobStatus = PLANNER_JOB_IDLE;
    s_motionActive = false;
    s_motionVx = 0.0f;
    s_motionVy = 0.0f;
    s_motionTargetCounts = 0;
    s_motionTravelledCounts = 0;
}

static void Mission_ClearRecognitionLocked(void)
{
    uint16_t index;
    memset(s_recognitionPoints, 0, sizeof(s_recognitionPoints));
    for (index = 0U; index < APP_MISSION_MAX_RECOGNITION_POINTS; index++)
    {
        s_recognitionPoints[index].resultCode = APP_RECOGNITION_CODE_UNKNOWN;
    }
    s_recognitionPointCount = 0U;
    s_recognizedCount = 0U;
    s_currentRecognitionPoint = 0U;
}

static bool Mission_BuildRecognitionRecordsLocked(void)
{
    int index;
    Mission_ClearRecognitionLocked();
    for (index = 0; index < s_plannerState.object_count; index++)
    {
        const PlannerObject *object = &s_plannerState.objects[index];
        recognition_point_t *point;
        if (object->id < 0 || object->id >= APP_MISSION_MAX_RECOGNITION_POINTS)
        {
            return false;
        }
        point = &s_recognitionPoints[object->id];
        point->objectX = (uint8_t)object->x;
        point->objectY = (uint8_t)object->y;
        point->observeX = (uint8_t)object->x;
        point->observeY = (uint8_t)object->y;
        point->observeDir = PATH_DIR_X_POS;
        if ((uint16_t)(object->id + 1) > s_recognitionPointCount)
        {
            s_recognitionPointCount = (uint16_t)(object->id + 1);
        }
    }
    return true;
}

static void Mission_UpdateRecognitionPositionsLocked(void)
{
    int index;
    for (index = 0; index < s_plannerState.object_count; index++)
    {
        const PlannerObject *object = &s_plannerState.objects[index];
        if (object->id >= 0 && object->id < s_recognitionPointCount)
        {
            s_recognitionPoints[object->id].objectX = (uint8_t)object->x;
            s_recognitionPoints[object->id].objectY = (uint8_t)object->y;
        }
    }
}

static bool PathPlanner_StatesEquivalentForPlanning(
    const PlannerState *left,
    const PlannerState *right)
{
    int index;
    if (left->width != right->width || left->height != right->height ||
        left->car_x != right->car_x || left->car_y != right->car_y ||
        left->car_dir != right->car_dir || left->pose_valid != right->pose_valid ||
        left->object_count != right->object_count ||
        left->forbidden_match_count != right->forbidden_match_count ||
        memcmp(left->wall_bits, right->wall_bits, PLANNER_BITSET_BYTES) != 0 ||
        memcmp(left->bomb_bits, right->bomb_bits, PLANNER_BITSET_BYTES) != 0)
    {
        return false;
    }
    for (index = 0; index < left->object_count; index++)
    {
        const PlannerObject *a = &left->objects[index];
        const PlannerObject *b = &right->objects[index];
        if (a->id != b->id || a->x != b->x || a->y != b->y ||
            a->label != b->label || a->kind != b->kind || a->known != b->known)
        {
            return false;
        }
    }
    for (index = 0; index < left->forbidden_match_count; index++)
    {
        if (left->forbidden_matches[index].box_id != right->forbidden_matches[index].box_id ||
            left->forbidden_matches[index].destination_id != right->forbidden_matches[index].destination_id)
        {
            return false;
        }
    }
    return true;
}

static bool Mission_ApplyPoseLocked(uint8_t x, uint8_t y, path_dir_t heading)
{
    if (!PathPlanner_IsValidDir(heading) ||
        !planner_state_set_pose(
            &s_plannerState,
            x,
            y,
            PathPlanner_PathDirToPlannerDir(heading),
            PathPlanner_HeadingToYaw(heading)))
    {
        return false;
    }
    s_heading = heading;
    return true;
}

static void Mission_ApplyPendingPoseLocked(void)
{
    if (!s_pendingPoseValid)
    {
        return;
    }
    (void)Mission_ApplyPoseLocked(s_pendingPoseX, s_pendingPoseY, s_pendingPoseHeading);
    s_pendingPoseValid = false;
}

static void Mission_SetErrorLocked(void)
{
    Chassis_Stop();
    s_motionActive = false;
    s_activePlanValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    s_plannerJobStatus = PLANNER_JOB_FAILED;
    s_pathState = PATH_STATE_ERROR;
    s_missionState = MISSION_STATE_ERROR;
}

static void Mission_SetFinishedLocked(void)
{
    Chassis_Stop();
    s_motionActive = false;
    s_activePlanValid = false;
    s_queuedPlanValid = false;
    s_predictionValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    s_plannerJobStatus = PLANNER_JOB_DONE;
    s_pathState = PATH_STATE_FINISHED;
    s_missionState = MISSION_STATE_FINISHED;
}

static void Mission_BeginReturnLocked(bool solved)
{
    PlannerPlan returnPlan;
    path_dir_t outwardHeading;

    Chassis_Stop();
    s_motionActive = false;
    s_activePlanValid = false;
    s_queuedPlanValid = false;
    s_predictionValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    if (!s_entryPoseValid || !s_plannerState.pose_valid)
    {
        Mission_SetErrorLocked();
        return;
    }

    s_levelSolved = solved;
    s_returningToStart = true;
    if (solved && s_levelTimerRunning)
    {
        s_levelElapsedMs = Mission_ElapsedMsSince(s_levelStartTick);
        s_levelTimerRunning = false;
        s_nextLevelStartTick = xTaskGetTickCount();
    }
    outwardHeading = (path_dir_t)(((uint32_t)s_entryHeading + 2U) & 3U);
    if (s_plannerState.car_x == (int)s_entryX &&
        s_plannerState.car_y == (int)s_entryY &&
        s_heading == outwardHeading)
    {
        Mission_StartEnteringLocked();
        return;
    }

    memset(&returnPlan, 0, sizeof(returnPlan));
    if (!planner_plan_to_pose(
            &s_plannerState,
            s_entryX,
            s_entryY,
            PathPlanner_PathDirToPlannerDir(outwardHeading),
            &returnPlan) ||
        !returnPlan.success)
    {
        Mission_SetErrorLocked();
        return;
    }
    if (returnPlan.step_count == 0)
    {
        PathPlanner_StartRotationLocked(outwardHeading);
        return;
    }
    Planner_InstallPlanLocked(&returnPlan, true);
}

static bool Planner_PlanContainsScan(const PlannerPlan *plan)
{
    int index;
    for (index = 0; index < plan->step_count; index++)
    {
        if (plan->steps[index].type == PLANNER_STEP_SCAN)
        {
            return true;
        }
    }
    return false;
}

static void Planner_StartPredictionLocked(void)
{
    int index;
    int status;

    s_predictionValid = false;
    s_predictionComplete = false;
    s_queuedPlanValid = false;
    if (s_manualPlan || Planner_PlanContainsScan(&s_activePlan))
    {
        return;
    }

    s_predictedState = s_plannerState;
    for (index = 0; index < s_activePlan.step_count; index++)
    {
        const PlannerStep *step = &s_activePlan.steps[index];
        if ((step->type != PLANNER_STEP_MOVE && step->type != PLANNER_STEP_PUSH) ||
            !planner_state_apply_executed_step(&s_predictedState, step))
        {
            return;
        }
        (void)planner_state_remove_completed_pairs(&s_predictedState);
    }

    s_predictionValid = true;
    if (planner_state_is_complete(&s_predictedState))
    {
        s_predictionComplete = true;
        return;
    }
    status = planner_job_begin(&s_predictedState);
    s_plannerJobStatus = status;
    if (status == PLANNER_JOB_RUNNING)
    {
        s_jobMode = PLANNER_JOB_MODE_PREDICTED;
    }
    else
    {
        s_jobMode = PLANNER_JOB_MODE_NONE;
    }
}

static void Planner_InstallPlanLocked(const PlannerPlan *plan, bool manualPlan)
{
    if (plan == 0 || !plan->success || plan->step_count <= 0 ||
        plan->step_count > PLANNER_MAX_PLAN_STEPS)
    {
        Mission_SetErrorLocked();
        return;
    }

    s_activePlan = *plan;
    s_activePlanValid = true;
    s_activeStepIndex = 0U;
    s_manualPlan = manualPlan;
    s_motionActive = false;
    s_queuedPlanValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    s_plannerJobStatus = PLANNER_JOB_DONE;
    s_pathState = PATH_STATE_RUNNING;
    s_missionState = MISSION_STATE_EXECUTING;
    Planner_StartPredictionLocked();
}

static void Mission_TryBeginPlanningLocked(void)
{
    int status;
    if (!s_missionMapAccepted || !PathPlanner_MapReadyLocked() || !s_plannerState.pose_valid)
    {
        s_missionState = s_missionMapAccepted ? MISSION_STATE_WAIT_INPUT : MISSION_STATE_WAIT_MAP;
        return;
    }

    Mission_ApplyPendingPoseLocked();
    (void)planner_state_remove_completed_pairs(&s_plannerState);
    if (planner_state_is_complete(&s_plannerState))
    {
        Mission_BeginReturnLocked(true);
        return;
    }

    Chassis_Stop();
    s_activePlanValid = false;
    s_queuedPlanValid = false;
    s_predictionValid = false;
    s_motionActive = false;
    status = planner_job_begin(&s_plannerState);
    s_plannerJobStatus = status;
    if (status != PLANNER_JOB_RUNNING)
    {
        Mission_BeginReturnLocked(false);
        return;
    }
    s_jobMode = PLANNER_JOB_MODE_CURRENT;
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_PLANNING;
}

static void Planner_UpdateJobLocked(void)
{
    planner_job_mode_t completedMode;
    int status;
    uint32_t startCycles;
    uint32_t sliceCycles;
    uint32_t sliceUs;
    if (s_jobMode == PLANNER_JOB_MODE_NONE)
    {
        return;
    }

    completedMode = s_jobMode;
    sliceUs = s_motionActive ? APP_PLANNER_MOVING_SLICE_US :
                              APP_PLANNER_STATIONARY_SLICE_US;
    sliceCycles = (SystemCoreClock / 1000000U) * sliceUs;
    startCycles = DWT->CYCCNT;
    do
    {
        status = planner_job_step(1U, &s_jobPlan);
    } while (status == PLANNER_JOB_RUNNING &&
             (uint32_t)(DWT->CYCCNT - startCycles) < sliceCycles);
    s_plannerJobStatus = status;
    if (status == PLANNER_JOB_RUNNING)
    {
        return;
    }
    s_jobMode = PLANNER_JOB_MODE_NONE;
    if (status != PLANNER_JOB_DONE || !s_jobPlan.success)
    {
        if (completedMode == PLANNER_JOB_MODE_CURRENT)
        {
            Mission_BeginReturnLocked(false);
        }
        return;
    }

    if (completedMode == PLANNER_JOB_MODE_CURRENT)
    {
        Planner_InstallPlanLocked(&s_jobPlan, false);
    }
    else
    {
        s_queuedPlan = s_jobPlan;
        s_queuedPlanValid = true;
    }
}

static void PathPlanner_StartRotationLocked(path_dir_t targetHeading)
{
    s_rotationTargetHeading = targetHeading;
    s_rotationTargetYawCentiDeg = PathPlanner_HeadingToYaw(targetHeading);
    s_rotationStableCount = 0U;
    s_missionStateTick = xTaskGetTickCount();
    s_missionState = MISSION_STATE_ROTATING;
}

static void PathPlanner_UpdateRotationLocked(void)
{
    int16_t currentYaw;
    int16_t error;
    int32_t absoluteError;
    float wz;

    if ((xTaskGetTickCount() - s_missionStateTick) > pdMS_TO_TICKS(APP_MISSION_ROTATE_TIMEOUT_MS))
    {
        Mission_SetErrorLocked();
        return;
    }
    currentYaw = PathPlanner_GetCurrentYaw();
    error = PathPlanner_GetYawError(s_rotationTargetYawCentiDeg, currentYaw);
    absoluteError = PathPlanner_AbsInt32((int32_t)error);
    if (absoluteError <= APP_MISSION_YAW_TOLERANCE_CDEG)
    {
        Chassis_Stop();
        s_rotationStableCount++;
        if (s_rotationStableCount >= APP_MISSION_YAW_STABLE_COUNT)
        {
            s_heading = s_rotationTargetHeading;
            s_plannerState.car_dir = PathPlanner_PathDirToPlannerDir(s_heading);
            s_plannerState.car_yaw_cdeg = s_rotationTargetYawCentiDeg;
            if (s_returningToStart && !s_activePlanValid)
            {
                Mission_StartEnteringLocked();
            }
            else
            {
                s_missionState = MISSION_STATE_EXECUTING;
            }
        }
        return;
    }

    s_rotationStableCount = 0U;
    wz = ((float)error / 100.0f) * APP_MISSION_ROTATE_KP_RADPS_PER_DEG;
    wz = PathPlanner_ClampFloat(wz, -APP_MISSION_ROTATE_MAX_RADPS, APP_MISSION_ROTATE_MAX_RADPS);
    if ((wz > 0.0f) && (wz < APP_MISSION_ROTATE_MIN_RADPS))
    {
        wz = APP_MISSION_ROTATE_MIN_RADPS;
    }
    else if ((wz < 0.0f) && (wz > -APP_MISSION_ROTATE_MIN_RADPS))
    {
        wz = -APP_MISSION_ROTATE_MIN_RADPS;
    }
    Chassis_SetVelocity(0.0f, 0.0f, wz);
}

static void PathPlanner_ApplyMotionVelocityLocked(void)
{
    int16_t currentYaw = PathPlanner_GetCurrentYaw();
    int16_t error = PathPlanner_GetYawError(s_motionTargetYawCentiDeg, currentYaw);
    float correction = ((float)error / 100.0f) * APP_PATH_YAW_HOLD_KP_RADPS_PER_DEG;
    correction = PathPlanner_ClampFloat(
        correction,
        -APP_PATH_YAW_HOLD_MAX_RADPS,
        APP_PATH_YAW_HOLD_MAX_RADPS);
    Chassis_SetVelocity(s_motionVx, s_motionVy, correction);
}

static bool PathPlanner_StartMoveLocked(const PlannerStep *step)
{
    int32_t dx = (int32_t)step->x - (int32_t)s_plannerState.car_x;
    int32_t dy = (int32_t)step->y - (int32_t)s_plannerState.car_y;
    int32_t maxCells;
    float distanceM;

    s_stepExpectedState = s_plannerState;
    if (!planner_state_apply_executed_step(&s_stepExpectedState, step))
    {
        return false;
    }
    if (dx == 0 && dy == 0)
    {
        s_plannerState = s_stepExpectedState;
        s_heading = PathPlanner_PlannerDirToPathDir(step->dir);
        s_activeStepIndex++;
        return true;
    }

    if ((dx != 0 && dy != 0) || (dx == 0 && dy == 0))
    {
        return false;
    }
    /*
     * Chassis_SetVelocity 使用车体坐标。执行器已经在每段运动前把车头
     * 转到 step->dir，因此地图上的四方向运动在车体系里恒为正向行驶。
     */
    s_motionVx = APP_PATH_DRIVE_SPEED_MPS;
    s_motionVy = 0.0f;
    maxCells = PathPlanner_MaxInt32(PathPlanner_AbsInt32(dx), PathPlanner_AbsInt32(dy));
    distanceM = ((float)s_cellSizeMm * (float)maxCells) / 1000.0f;
    s_motionTargetCounts = PathPlanner_AbsInt32(Chassis_DistanceMToEncoderCounts(distanceM));
    if (s_motionTargetCounts <= 0)
    {
        return false;
    }
    s_motionTargetYawCentiDeg = PathPlanner_HeadingToYaw(
        PathPlanner_PlannerDirToPathDir(step->dir));
    s_motionTravelledCounts = 0;
    s_motionStartPoseRevision = s_poseRevision;
    s_motionActive = true;
    BSP_EncoderClearAll();
    PathPlanner_ApplyMotionVelocityLocked();
    return true;
}

static void Mission_FinishActivePlanLocked(void)
{
    bool predictedMatches;
    Chassis_Stop();
    s_motionActive = false;
    s_activePlanValid = false;
    s_pathState = PATH_STATE_FINISHED;

    if (s_manualPlan)
    {
        s_manualPlan = false;
        if (s_returningToStart)
        {
            Mission_StartEnteringLocked();
            return;
        }
        s_missionState = MISSION_STATE_IDLE;
        return;
    }

    Mission_ApplyPendingPoseLocked();
    predictedMatches = s_predictionValid &&
                       PathPlanner_StatesEquivalentForPlanning(&s_plannerState, &s_predictedState);
    if (!predictedMatches)
    {
        s_jobMode = PLANNER_JOB_MODE_NONE;
        s_queuedPlanValid = false;
        s_predictionValid = false;
        Mission_TryBeginPlanningLocked();
        return;
    }
    if (s_predictionComplete)
    {
        Mission_BeginReturnLocked(true);
        return;
    }
    if (s_queuedPlanValid)
    {
        Planner_InstallPlanLocked(&s_queuedPlan, false);
        return;
    }
    if (s_jobMode == PLANNER_JOB_MODE_PREDICTED)
    {
        s_jobMode = PLANNER_JOB_MODE_CURRENT;
        s_missionState = MISSION_STATE_PLANNING;
        s_pathState = PATH_STATE_IDLE;
        return;
    }
    Mission_TryBeginPlanningLocked();
}

static void PathPlanner_UpdateMotionLocked(void)
{
    int32_t finishThreshold;
    if (!s_motionActive)
    {
        return;
    }
    s_motionTravelledCounts = Chassis_GetAverageAbsEncoderCounts();
    finishThreshold = s_motionTargetCounts - APP_PATH_FINISH_TOLERANCE_COUNTS;
    if (finishThreshold < 0)
    {
        finishThreshold = 0;
    }
    if (s_motionTravelledCounts < finishThreshold)
    {
        PathPlanner_ApplyMotionVelocityLocked();
        return;
    }

    Chassis_Stop();
    s_motionActive = false;
    s_missionState = MISSION_STATE_WAIT_POSE;
    s_missionStateTick = xTaskGetTickCount();
    Mission_ConfirmMotionLocked();
}

static uint8_t Mission_CountBoxesLocked(void)
{
    int index;
    uint8_t count = 0U;
    for (index = 0; index < s_plannerState.object_count; index++)
    {
        if (s_plannerState.objects[index].kind == PLANNER_OBJECT_BOX)
        {
            count++;
        }
    }
    return count;
}

static uint32_t Mission_ElapsedMsSince(TickType_t startTick)
{
    return (uint32_t)(xTaskGetTickCount() - startTick) * (uint32_t)portTICK_PERIOD_MS;
}

static void Mission_ApplyTransitVelocityLocked(void)
{
    int16_t currentYaw = PathPlanner_GetCurrentYaw();
    int16_t error = PathPlanner_GetYawError(s_motionTargetYawCentiDeg, currentYaw);
    float correction = ((float)error / 100.0f) * APP_PATH_YAW_HOLD_KP_RADPS_PER_DEG;
    correction = PathPlanner_ClampFloat(
        correction,
        -APP_PATH_YAW_HOLD_MAX_RADPS,
        APP_PATH_YAW_HOLD_MAX_RADPS);
    Chassis_SetVelocity(APP_MISSION_START_TRANSIT_SPEED_MPS, 0.0f, correction);
}

static void Mission_StartLeavingLocked(void)
{
    PathPlanner_ClearRuntimeLocked();
    planner_state_clear(&s_plannerState);
    Mission_ClearRecognitionLocked();
    s_pendingPoseValid = false;
    s_missionMapAccepted = false;
    s_initialBoxCount = 0U;
    s_motionTargetYawCentiDeg = PathPlanner_GetCurrentYaw();
    s_missionStateTick = xTaskGetTickCount();
    s_pathState = PATH_STATE_RUNNING;
    s_missionState = MISSION_STATE_LEAVING_START;
    Mission_ApplyTransitVelocityLocked();
}

static void Mission_StartEnteringLocked(void)
{
    s_motionTargetYawCentiDeg = PathPlanner_HeadingToYaw(
        (path_dir_t)(((uint32_t)s_entryHeading + 2U) & 3U));
    s_missionStateTick = xTaskGetTickCount();
    s_pathState = PATH_STATE_RUNNING;
    s_missionState = MISSION_STATE_ENTERING_START;
    Mission_ApplyTransitVelocityLocked();
}

static void Mission_AdvanceLevelLocked(void)
{
    s_completedLevels = s_currentLevel;
    if (s_currentLevel >= APP_MISSION_LEVEL_COUNT)
    {
        Mission_SetFinishedLocked();
        return;
    }
    s_currentLevel++;
    s_levelSolved = false;
    s_returningToStart = false;
    s_levelStartTick = s_nextLevelStartTick;
    s_levelElapsedMs = 0U;
    s_levelTimerRunning = true;
    Mission_StartLeavingLocked();
}

static void Mission_ConfirmMotionLocked(void)
{
    bool poseMatches;
    if (s_missionState != MISSION_STATE_WAIT_POSE || !s_pendingPoseValid ||
        s_poseRevision == s_motionStartPoseRevision)
    {
        return;
    }

    poseMatches =
        s_pendingPoseX == (uint8_t)s_stepExpectedState.car_x &&
        s_pendingPoseY == (uint8_t)s_stepExpectedState.car_y &&
        PathPlanner_PathDirToPlannerDir(s_pendingPoseHeading) == s_stepExpectedState.car_dir;
    if (!poseMatches)
    {
        /*
         * 编码器只证明轮子转过，不证明车和箱子到达预测格。此时不能提交
         * 预测世界状态；先采用实测车位，再重新获取包含箱子位置的地图。
         */
        (void)Mission_ApplyPoseLocked(
            s_pendingPoseX,
            s_pendingPoseY,
            s_pendingPoseHeading);
        s_pendingPoseValid = false;
        s_activePlanValid = false;
        s_queuedPlanValid = false;
        s_predictionValid = false;
        s_jobMode = PLANNER_JOB_MODE_NONE;
        s_missionMapAccepted = false;
        s_missionState = MISSION_STATE_WAIT_MAP;
        s_missionStateTick = xTaskGetTickCount();
        if (!Vision_RequestMap(Mission_NextSequence(), s_currentLevel))
        {
            Mission_SetErrorLocked();
        }
        return;
    }

    s_plannerState = s_stepExpectedState;
    s_heading = s_pendingPoseHeading;
    s_plannerState.car_dir = PathPlanner_PathDirToPlannerDir(s_heading);
    s_plannerState.car_yaw_cdeg = PathPlanner_HeadingToYaw(s_heading);
    s_pendingPoseValid = false;
    Mission_UpdateRecognitionPositionsLocked();
    (void)planner_state_remove_completed_pairs(&s_plannerState);
    s_activeStepIndex++;
    s_missionState = MISSION_STATE_EXECUTING;
    if (s_activeStepIndex >= (uint16_t)s_activePlan.step_count)
    {
        Mission_FinishActivePlanLocked();
    }
}

static bool Mission_SendRecognitionRequestLocked(void)
{
    uint16_t sequence = Mission_NextSequence();
    if (!Vision_RequestRecognition(sequence, s_currentRecognitionPoint))
    {
        Mission_SetErrorLocked();
        return false;
    }
    s_missionState = MISSION_STATE_WAIT_RECOGNITION;
    s_missionStateTick = xTaskGetTickCount();
    return true;
}

static bool PathPlanner_StartScanLocked(const PlannerStep *step)
{
    PlannerObject *object = PathPlanner_FindObjectByIdLocked(step->object_id);
    int dir = s_plannerState.car_dir;
    int expectedX;
    int expectedY;
    recognition_point_t *point;

    if (object == 0 || object->x != step->x || object->y != step->y ||
        dir < PLANNER_DIR_UP || dir > PLANNER_DIR_LEFT)
    {
        return false;
    }
    expectedX = s_plannerState.car_x + s_plannerDx[dir];
    expectedY = s_plannerState.car_y + s_plannerDy[dir];
    if (expectedX != object->x || expectedY != object->y ||
        step->object_id < 0 || step->object_id >= s_recognitionPointCount)
    {
        return false;
    }

    s_currentRecognitionPoint = (uint16_t)step->object_id;
    point = &s_recognitionPoints[s_currentRecognitionPoint];
    point->observeX = (uint8_t)s_plannerState.car_x;
    point->observeY = (uint8_t)s_plannerState.car_y;
    point->observeDir = PathPlanner_PlannerDirToPathDir(dir);
    s_activeStepIndex++;
    Chassis_Stop();
    s_missionState = MISSION_STATE_WAIT_STOP;
    s_missionStateTick = xTaskGetTickCount();
    return true;
}

static void PathPlanner_StartCurrentStepLocked(void)
{
    const PlannerStep *step;
    path_dir_t targetHeading;
    if (!s_activePlanValid || s_motionActive)
    {
        return;
    }
    if (s_activeStepIndex >= (uint16_t)s_activePlan.step_count)
    {
        Mission_FinishActivePlanLocked();
        return;
    }

    step = &s_activePlan.steps[s_activeStepIndex];
    if (step->type == PLANNER_STEP_SCAN)
    {
        targetHeading = PathPlanner_PlannerDirToPathDir(s_plannerState.car_dir);
        if (s_heading != targetHeading)
        {
            PathPlanner_StartRotationLocked(targetHeading);
            return;
        }
        if (!PathPlanner_StartScanLocked(step))
        {
            s_activePlanValid = false;
            Mission_TryBeginPlanningLocked();
        }
        return;
    }
    if ((step->type != PLANNER_STEP_MOVE && step->type != PLANNER_STEP_PUSH) ||
        step->dir < PLANNER_DIR_UP || step->dir > PLANNER_DIR_LEFT)
    {
        Mission_SetErrorLocked();
        return;
    }

    targetHeading = PathPlanner_PlannerDirToPathDir(step->dir);
    if (s_heading != targetHeading)
    {
        PathPlanner_StartRotationLocked(targetHeading);
        return;
    }
    if (!PathPlanner_StartMoveLocked(step))
    {
        s_activePlanValid = false;
        Mission_TryBeginPlanningLocked();
        return;
    }
    if (!s_motionActive && s_activeStepIndex >= (uint16_t)s_activePlan.step_count)
    {
        Mission_FinishActivePlanLocked();
    }
}

static void Mission_UpdateLocked(void)
{
    switch (s_missionState)
    {
        case MISSION_STATE_LEAVING_START:
        case MISSION_STATE_ENTERING_START:
            if ((xTaskGetTickCount() - s_missionStateTick) >=
                pdMS_TO_TICKS(APP_MISSION_START_TRANSIT_TIMEOUT_MS))
            {
                Mission_SetErrorLocked();
            }
            else
            {
                Mission_ApplyTransitVelocityLocked();
            }
            break;

        case MISSION_STATE_WAIT_MAP:
            if ((xTaskGetTickCount() - s_missionStateTick) >=
                pdMS_TO_TICKS(APP_MISSION_MAP_TIMEOUT_MS))
            {
                s_missionRetryCount++;
                if (!Vision_RequestMap(Mission_NextSequence(), s_currentLevel))
                {
                    Mission_SetErrorLocked();
                    break;
                }
                s_missionStateTick = xTaskGetTickCount();
            }
            break;

        case MISSION_STATE_EXECUTING:
            PathPlanner_StartCurrentStepLocked();
            break;

        case MISSION_STATE_ROTATING:
            PathPlanner_UpdateRotationLocked();
            break;

        case MISSION_STATE_WAIT_POSE:
            Mission_ConfirmMotionLocked();
            if (s_missionState == MISSION_STATE_WAIT_POSE &&
                (xTaskGetTickCount() - s_missionStateTick) >=
                    pdMS_TO_TICKS(APP_MISSION_POSE_TIMEOUT_MS))
            {
                Mission_SetErrorLocked();
            }
            break;

        case MISSION_STATE_WAIT_STOP:
            if ((xTaskGetTickCount() - s_missionStateTick) >=
                pdMS_TO_TICKS(APP_MISSION_STOP_SETTLE_MS))
            {
                (void)Mission_SendRecognitionRequestLocked();
            }
            break;

        case MISSION_STATE_WAIT_RECOGNITION:
            if ((xTaskGetTickCount() - s_missionStateTick) >=
                pdMS_TO_TICKS(APP_MISSION_RECOGNITION_TIMEOUT_MS))
            {
                s_missionRetryCount++;
                s_missionState = MISSION_STATE_RETRY_RECOGNITION;
            }
            break;

        case MISSION_STATE_RETRY_RECOGNITION:
            (void)Mission_SendRecognitionRequestLocked();
            break;

        case MISSION_STATE_WAIT_START_SETTLE:
            if ((xTaskGetTickCount() - s_missionStateTick) >=
                pdMS_TO_TICKS(APP_MISSION_START_SETTLE_MS))
            {
                if (s_levelTimerRunning)
                {
                    s_levelElapsedMs = Mission_ElapsedMsSince(s_levelStartTick);
                    s_levelTimerRunning = false;
                }
                s_nextLevelStartTick = xTaskGetTickCount();
                Mission_AdvanceLevelLocked();
            }
            break;

        default:
            break;
    }
}

void PathPlanner_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_stateMutex = xSemaphoreCreateMutex();
    planner_state_clear(&s_plannerState);
    memset(&s_predictedState, 0, sizeof(s_predictedState));
    memset(&s_stepExpectedState, 0, sizeof(s_stepExpectedState));
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_IDLE;
    s_cellSizeMm = APP_PATH_CELL_SIZE_MM;
    s_heading = PATH_DIR_X_POS;
    s_rotationTargetHeading = PATH_DIR_X_POS;
    s_rotationTargetYawCentiDeg = 0;
    s_rotationStableCount = 0U;
    s_missionStateTick = xTaskGetTickCount();
    s_missionRequestSequence = 0U;
    s_missionRetryCount = 0U;
    s_missionMapAccepted = false;
    s_pendingPoseValid = false;
    s_poseRevision = 0U;
    s_motionStartPoseRevision = 0U;
    s_currentLevel = 0U;
    s_completedLevels = 0U;
    s_initialBoxCount = 0U;
    s_startAreaKnown = false;
    s_inStartArea = false;
    s_entryPoseValid = false;
    s_returningToStart = false;
    s_levelSolved = false;
    s_levelTimerRunning = false;
    s_levelStartTick = 0U;
    s_nextLevelStartTick = 0U;
    s_levelElapsedMs = 0U;
    Mission_ClearRecognitionLocked();
    PathPlanner_ClearRuntimeLocked();
    if (s_stateMutex == 0)
    {
        s_pathState = PATH_STATE_ERROR;
        s_missionState = MISSION_STATE_ERROR;
    }
}

void PathPlanner_Start(void)
{
    if (!PathPlanner_Lock())
    {
        return;
    }
    if (s_activePlanValid)
    {
        s_pathState = PATH_STATE_RUNNING;
        s_missionState = MISSION_STATE_EXECUTING;
    }
    else
    {
        s_pathState = PATH_STATE_ERROR;
    }
    PathPlanner_Unlock();
}

void PathPlanner_Stop(void)
{
    if (!PathPlanner_Lock())
    {
        return;
    }
    PathPlanner_ClearRuntimeLocked();
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_STOPPED;
    PathPlanner_Unlock();
}

void PathPlanner_Update(void)
{
    if (!PathPlanner_Lock())
    {
        return;
    }
    PathPlanner_UpdateMotionLocked();
    Mission_UpdateLocked();
    Planner_UpdateJobLocked();
    PathPlanner_Unlock();
}

path_state_t PathPlanner_GetState(void)
{
    path_state_t result = PATH_STATE_ERROR;
    if (PathPlanner_Lock())
    {
        result = s_pathState;
        PathPlanner_Unlock();
    }
    return result;
}

void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm)
{
    if (cellSizeMm == 0U || !PathPlanner_Lock())
    {
        return;
    }
    s_cellSizeMm = cellSizeMm;
    PathPlanner_Unlock();
}

uint16_t PathPlanner_GetCellSizeMm(void)
{
    uint16_t result = 0U;
    if (PathPlanner_Lock())
    {
        result = s_cellSizeMm;
        PathPlanner_Unlock();
    }
    return result;
}

bool PathPlanner_IsMapCellValid(char cell)
{
    return (cell == PATH_MAP_CELL_WALL) || (cell == PATH_MAP_CELL_EMPTY) ||
           (cell == PATH_MAP_CELL_GOAL) || (cell == PATH_MAP_CELL_BOMB) ||
           (cell == PATH_MAP_CELL_BOX);
}

bool PathPlanner_IsMapCellPassableChar(char cell)
{
    return (cell == PATH_MAP_CELL_EMPTY) || (cell == PATH_MAP_CELL_GOAL);
}

static bool PathPlanner_SetMapLocked(uint8_t width, uint8_t height, const char *cells)
{
    bool result;
    if (s_activePlanValid || s_motionActive || s_jobMode != PLANNER_JOB_MODE_NONE)
    {
        return false;
    }
    result = planner_state_set_map_rows(&s_plannerState, width, height, cells) != 0;
    if (!result || !Mission_BuildRecognitionRecordsLocked())
    {
        return false;
    }
    s_heading = PathPlanner_PlannerDirToPathDir(s_plannerState.car_dir);
    s_pathState = PATH_STATE_IDLE;
    s_activePlanValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    return true;
}

bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells)
{
    bool result;
    if (cells == 0 || !PathPlanner_Lock())
    {
        return false;
    }
    result = PathPlanner_SetMapLocked(width, height, cells);
    PathPlanner_Unlock();
    return result;
}

bool PathPlanner_SetMapRows(uint8_t width, uint8_t height, const char *rows)
{
    bool result;
    if (rows == 0 || !PathPlanner_Lock())
    {
        return false;
    }
    result = PathPlanner_SetMapLocked(width, height, rows);
    PathPlanner_Unlock();
    return result;
}

bool PathPlanner_SetPose(uint8_t x, uint8_t y, path_dir_t heading)
{
    bool result = false;
    bool poseChanged;
    if (!PathPlanner_IsValidDir(heading) || !PathPlanner_Lock())
    {
        return false;
    }
    if (x >= PLANNER_MAX_WIDTH || y >= PLANNER_MAX_HEIGHT ||
        (PathPlanner_MapReadyLocked() &&
         (x >= (uint8_t)s_plannerState.width || y >= (uint8_t)s_plannerState.height)))
    {
        PathPlanner_Unlock();
        return false;
    }
    if (s_activePlanValid || s_motionActive || s_missionState == MISSION_STATE_ROTATING)
    {
        s_pendingPoseX = x;
        s_pendingPoseY = y;
        s_pendingPoseHeading = heading;
        s_pendingPoseValid = true;
        s_poseRevision++;
        result = true;
    }
    else
    {
        poseChanged = !s_plannerState.pose_valid ||
                      s_plannerState.car_x != (int)x ||
                      s_plannerState.car_y != (int)y ||
                      s_plannerState.car_dir != PathPlanner_PathDirToPlannerDir(heading);
        result = Mission_ApplyPoseLocked(x, y, heading);
        if (result)
        {
            s_poseRevision++;
            if (!s_entryPoseValid && !s_inStartArea &&
                (s_missionState == MISSION_STATE_WAIT_MAP ||
                 s_missionState == MISSION_STATE_WAIT_INPUT))
            {
                s_entryX = x;
                s_entryY = y;
                s_entryHeading = heading;
                s_entryPoseValid = true;
            }
        }
        if (result && poseChanged && s_missionState == MISSION_STATE_PLANNING)
        {
            Mission_TryBeginPlanningLocked();
        }
        else if (result && (s_missionState == MISSION_STATE_WAIT_INPUT ||
                            s_missionState == MISSION_STATE_WAIT_MAP))
        {
            Mission_TryBeginPlanningLocked();
        }
    }
    PathPlanner_Unlock();
    return result;
}

static bool PathPlanner_GotoLocked(uint8_t targetX, uint8_t targetY)
{
    PlannerPlan plan;
    bool missionIdle = s_missionState == MISSION_STATE_IDLE ||
                       s_missionState == MISSION_STATE_STOPPED ||
                       s_missionState == MISSION_STATE_FINISHED;
    if (!missionIdle || s_activePlanValid || s_motionActive ||
        s_jobMode != PLANNER_JOB_MODE_NONE ||
        !PathPlanner_MapReadyLocked() || !s_plannerState.pose_valid ||
        targetX >= (uint8_t)s_plannerState.width || targetY >= (uint8_t)s_plannerState.height ||
        !planner_plan_to_pose(
            &s_plannerState,
            targetX,
            targetY,
            s_plannerState.car_dir,
            &plan))
    {
        s_pathState = PATH_STATE_ERROR;
        return false;
    }
    if (plan.step_count == 0)
    {
        s_pathState = PATH_STATE_FINISHED;
        return true;
    }
    Planner_InstallPlanLocked(&plan, true);
    return s_pathState != PATH_STATE_ERROR;
}

bool PathPlanner_Goto(uint8_t targetX, uint8_t targetY)
{
    bool result;
    if (!PathPlanner_Lock())
    {
        return false;
    }
    result = PathPlanner_GotoLocked(targetX, targetY);
    PathPlanner_Unlock();
    return result;
}

bool PathPlanner_MoveCells(path_dir_t dir, uint8_t cells)
{
    int targetX;
    int targetY;
    bool result;
    if (!PathPlanner_IsValidDir(dir) || cells == 0U || !PathPlanner_Lock())
    {
        return false;
    }
    targetX = s_plannerState.car_x;
    targetY = s_plannerState.car_y;
    if (dir == PATH_DIR_X_POS)
    {
        targetX += cells;
    }
    else if (dir == PATH_DIR_X_NEG)
    {
        targetX -= cells;
    }
    else if (dir == PATH_DIR_Y_POS)
    {
        targetY += cells;
    }
    else
    {
        targetY -= cells;
    }
    result = (targetX >= 0 && targetY >= 0 && targetX < s_plannerState.width &&
              targetY < s_plannerState.height &&
              PathPlanner_GotoLocked((uint8_t)targetX, (uint8_t)targetY));
    PathPlanner_Unlock();
    return result;
}

void PathPlanner_GetStatus(path_status_t *status)
{
    if (status == 0 || !PathPlanner_Lock())
    {
        return;
    }
    status->state = s_pathState;
    status->mapReady = PathPlanner_MapReadyLocked();
    status->poseReady = s_plannerState.pose_valid != 0U;
    status->segmentActive = s_motionActive || s_missionState == MISSION_STATE_ROTATING;
    status->x = (uint8_t)s_plannerState.car_x;
    status->y = (uint8_t)s_plannerState.car_y;
    status->heading = s_heading;
    status->stepIndex = s_activeStepIndex;
    status->stepCount = s_activePlanValid ? (uint16_t)s_activePlan.step_count : 0U;
    status->cellSizeMm = s_cellSizeMm;
    status->travelledCounts = s_motionTravelledCounts;
    status->targetCounts = s_motionTargetCounts;
    PathPlanner_Unlock();
}

bool PathPlanner_MissionStart(void)
{
    if (!PathPlanner_Lock())
    {
        return false;
    }
    if (!s_startAreaKnown || !s_inStartArea)
    {
        PathPlanner_Unlock();
        return false;
    }
    s_missionRetryCount = 0U;
    s_missionRequestSequence = 0U;
    s_currentLevel = 1U;
    s_completedLevels = 0U;
    s_entryPoseValid = false;
    s_returningToStart = false;
    s_levelSolved = false;
    s_levelTimerRunning = false;
    s_levelElapsedMs = 0U;
    s_levelStartTick = 0U;
    s_nextLevelStartTick = 0U;
    Mission_StartLeavingLocked();
    PathPlanner_Unlock();
    return true;
}

void PathPlanner_MissionStop(void)
{
    PathPlanner_Stop();
}

void PathPlanner_MissionReset(void)
{
    if (!PathPlanner_Lock())
    {
        return;
    }
    PathPlanner_ClearRuntimeLocked();
    planner_state_clear(&s_plannerState);
    Mission_ClearRecognitionLocked();
    s_pendingPoseValid = false;
    s_missionMapAccepted = false;
    s_missionRequestSequence = 0U;
    s_missionRetryCount = 0U;
    s_currentLevel = 0U;
    s_completedLevels = 0U;
    s_initialBoxCount = 0U;
    s_entryPoseValid = false;
    s_returningToStart = false;
    s_levelSolved = false;
    s_levelTimerRunning = false;
    s_levelElapsedMs = 0U;
    s_levelStartTick = 0U;
    s_nextLevelStartTick = 0U;
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_IDLE;
    PathPlanner_Unlock();
}

void PathPlanner_MissionOnMapReceived(uint16_t sequence, uint8_t level, bool labeledMode)
{
    int index;
    if (!PathPlanner_Lock())
    {
        return;
    }
    if ((s_missionState == MISSION_STATE_WAIT_MAP ||
         s_missionState == MISSION_STATE_WAIT_INPUT) &&
        sequence == s_missionRequestSequence &&
        level == s_currentLevel &&
        PathPlanner_MapReadyLocked())
    {
        if (!labeledMode)
        {
            for (index = 0; index < s_plannerState.object_count; index++)
            {
                PlannerObject *object = &s_plannerState.objects[index];
                if (object->kind == PLANNER_OBJECT_BOX ||
                    object->kind == PLANNER_OBJECT_DESTINATION)
                {
                    object->known = 1;
                    object->label = 0;
                }
            }
        }
        s_initialBoxCount = Mission_CountBoxesLocked();
        s_missionMapAccepted = true;
        s_missionRetryCount = 0U;
        Mission_TryBeginPlanningLocked();
    }
    PathPlanner_Unlock();
}

bool PathPlanner_SetStartArea(bool inStartArea)
{
    if (!PathPlanner_Lock())
    {
        return false;
    }
    s_startAreaKnown = true;
    s_inStartArea = inStartArea;

    if (s_missionState == MISSION_STATE_LEAVING_START && !inStartArea)
    {
        Chassis_Stop();
        if (!s_levelTimerRunning)
        {
            s_levelStartTick = xTaskGetTickCount();
            s_levelTimerRunning = true;
        }
        if (s_plannerState.pose_valid && !s_entryPoseValid)
        {
            s_entryX = (uint8_t)s_plannerState.car_x;
            s_entryY = (uint8_t)s_plannerState.car_y;
            s_entryHeading = s_heading;
            s_entryPoseValid = true;
        }
        s_missionState = MISSION_STATE_WAIT_MAP;
        s_missionStateTick = xTaskGetTickCount();
        if (!Vision_RequestMap(Mission_NextSequence(), s_currentLevel))
        {
            Mission_SetErrorLocked();
        }
    }
    else if (s_missionState == MISSION_STATE_ENTERING_START && inStartArea)
    {
        Chassis_Stop();
        if (s_levelSolved)
        {
            Mission_AdvanceLevelLocked();
        }
        else
        {
            s_missionState = MISSION_STATE_WAIT_START_SETTLE;
            s_missionStateTick = xTaskGetTickCount();
        }
    }
    PathPlanner_Unlock();
    return true;
}

bool PathPlanner_MissionOnRecognitionResult(uint16_t sequence, uint8_t code)
{
    int result;
    recognition_point_t *point;
    if (!PathPlanner_Lock())
    {
        return false;
    }
    if (s_missionState != MISSION_STATE_WAIT_RECOGNITION || code > 20U ||
        (sequence != 0U && sequence != s_missionRequestSequence) ||
        s_currentRecognitionPoint >= s_recognitionPointCount)
    {
        PathPlanner_Unlock();
        return false;
    }

    result = planner_state_apply_recognition(
        &s_plannerState,
        s_currentRecognitionPoint,
        code);
    if (result == PLANNER_RECOGNITION_RETRY || result == PLANNER_RECOGNITION_INVALID)
    {
        s_missionRetryCount++;
        s_missionState = MISSION_STATE_RETRY_RECOGNITION;
        PathPlanner_Unlock();
        return true;
    }

    point = &s_recognitionPoints[s_currentRecognitionPoint];
    point->resultCode = code;
    if (!point->recognized)
    {
        point->recognized = true;
        s_recognizedCount++;
    }
    s_missionRetryCount = 0U;
    s_activePlanValid = false;
    s_predictionValid = false;
    s_jobMode = PLANNER_JOB_MODE_NONE;
    Mission_ApplyPendingPoseLocked();
    Mission_TryBeginPlanningLocked();
    PathPlanner_Unlock();
    return true;
}

void PathPlanner_GetMissionStatus(mission_status_t *status)
{
    if (status == 0 || !PathPlanner_Lock())
    {
        return;
    }
    status->state = s_missionState;
    status->currentPoint = s_currentRecognitionPoint;
    status->pointCount = s_recognitionPointCount;
    status->recognizedCount = s_recognizedCount;
    status->requestSequence = s_missionRequestSequence;
    status->retryCount = s_missionRetryCount;
    status->mapReady = PathPlanner_MapReadyLocked();
    status->poseReady = s_plannerState.pose_valid != 0U;
    status->activePlanStep = s_activeStepIndex;
    status->activePlanSteps = s_activePlanValid ? (uint16_t)s_activePlan.step_count : 0U;
    status->plannerJobStatus = (uint8_t)s_plannerJobStatus;
    status->backgroundPlanning = s_jobMode == PLANNER_JOB_MODE_PREDICTED;
    status->currentLevel = s_currentLevel;
    status->completedLevels = s_completedLevels;
    status->initialBoxes = s_initialBoxCount;
    status->remainingBoxes = Mission_CountBoxesLocked();
    status->startAreaKnown = s_startAreaKnown;
    status->inStartArea = s_inStartArea;
    status->levelSolved = s_levelSolved;
    status->levelElapsedMs = s_levelTimerRunning ?
        Mission_ElapsedMsSince(s_levelStartTick) : s_levelElapsedMs;
    PathPlanner_Unlock();
}

bool PathPlanner_GetRecognitionPoint(uint16_t index, recognition_point_t *point)
{
    bool result = false;
    if (point == 0 || !PathPlanner_Lock())
    {
        return false;
    }
    if (index < s_recognitionPointCount)
    {
        *point = s_recognitionPoints[index];
        result = true;
    }
    PathPlanner_Unlock();
    return result;
}
