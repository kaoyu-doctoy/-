#include "path_planner.h"
#include "../bsp/bsp_encoder.h"
#include "../chassis/chassis.h"
#include "../sensor/imu.h"
#include "../sensor/vision.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * 本文件负责“地图格子路线”到“底盘速度命令”的转换。
 *
 * 当前已经实现的能力：
 * 1. 接收并保存地图字符。
 * 2. 根据 @POSE 设置的小车位置作为起点。
 * 3. 用 BFS 在普通可通行格子中规划到目标点。
 * 4. 把连续同方向的格子合并成路径段。
 * 5. 周期性执行路径段，按编码器计数判断当前段是否完成。
 *
 * 当前还没有实现真正的推箱子状态搜索。
 * 因此 '$' 箱子在普通寻路中暂时当作不可通行格，后续队友写推箱子时
 * 可以复用地图字符宏，但需要另写“人和箱子状态一起搜索”的逻辑。
 */

/*
 * 一个路径段。
 * 例如连续向 X+ 走 3 格，会被合并为 {PATH_DIR_X_POS, 3}，
 * 这样执行时只需要启动一次底盘速度并等待对应编码器计数完成。
 */
typedef struct
{
    path_dir_t dir; /* 当前段移动方向。 */
    uint8_t cells;  /* 当前段连续移动的格子数。 */
} path_step_t;

/* 路径规划整体状态。 */
static path_state_t s_pathState;

/*
 * 路径规划专用大容量RAM。
 * 该数组由scatter文件放入0x20200000开始的700KB区域，
 * 不占用FreeRTOS的10KB动态内存。
 */
__attribute__((used, section("PathPlannerRam"), aligned(32)))
static uint8_t s_pathPlannerWorkspace[APP_PATH_PLANNER_RAM_SIZE];

/* 当前地图缓存，数组下标为 s_map[y][x]。 */
static char s_map[APP_MAP_MAX_HEIGHT][APP_MAP_MAX_WIDTH];
static uint8_t s_mapWidth;
static uint8_t s_mapHeight;
static bool s_mapReady;

/* 当前认为的小车地图位姿，由 @POSE 或视觉端 pose 数据设置。 */
static bool s_poseReady;
static uint8_t s_poseX;
static uint8_t s_poseY;
static path_dir_t s_heading;

/* 单个地图格子的实际边长，单位 mm，用于把“格数”换算成行驶距离。 */
static uint16_t s_cellSizeMm;

/* 已经规划好的路径段队列。 */
static path_step_t s_steps[APP_PATH_MAX_STEPS];
static uint16_t s_stepCount;
static uint16_t s_stepIndex;

/*
 * 当前正在执行的路径段状态。
 * 每启动一段会清零编码器，然后根据平均绝对编码器计数判断是否走够。
 */
static bool s_segmentActive;
static path_dir_t s_segmentDir;
static int32_t s_segmentTargetCounts;
static int32_t s_segmentTravelledCounts;
static int16_t s_segmentTargetYawCentiDeg;

/*
 * BFS 临时缓存。
 * s_bfsQueue 保存待搜索格子索引，s_bfsParent 记录每个格子从哪里来，
 * s_reverseDirs 用于从目标点反向回溯后再转成正向路径。
 */
static uint16_t s_bfsQueue[APP_PATH_MAX_STEPS];
static int16_t s_bfsParent[APP_PATH_MAX_STEPS];
static uint8_t s_bfsVisited[APP_PATH_MAX_STEPS];
static path_dir_t s_reverseDirs[APP_PATH_MAX_STEPS];
static uint16_t s_bfsDistance[APP_PATH_MAX_STEPS];

/* 巡检识别任务状态：地图字符保存类型，编号单独保存在 s_objectCode。 */
static volatile mission_state_t s_missionState;
static recognition_point_t s_recognitionPoints[APP_MISSION_MAX_RECOGNITION_POINTS];
static uint8_t s_objectCode[APP_MAP_MAX_HEIGHT][APP_MAP_MAX_WIDTH];
static uint16_t s_recognitionPointCount;
static volatile uint16_t s_recognizedCount;
static volatile uint16_t s_currentRecognitionPoint;
static volatile uint16_t s_missionRequestSequence;
static volatile uint16_t s_missionRetryCount;
static TickType_t s_missionStateTick;
static path_dir_t s_missionDefaultHeading;
static path_dir_t s_rotationTargetHeading;
static int16_t s_rotationTargetYawCentiDeg;
static uint8_t s_rotationStableCount;

/* 判断路径方向枚举是否有效。 */
static bool PathPlanner_IsValidDir(path_dir_t dir)
{
    return ((uint32_t)dir <= (uint32_t)PATH_DIR_Y_NEG);
}

/* 返回 int32_t 绝对值，避免到处重复写符号判断。 */
static int32_t PathPlanner_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 把浮点值限制到给定范围内，用于限制 yaw 修正角速度。 */
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

/* 清空当前路径段队列和正在执行的段。 */
static void PathPlanner_ClearSteps(void)
{
    s_stepCount = 0U;
    s_stepIndex = 0U;
    s_segmentActive = false;
    s_segmentTargetCounts = 0;
    s_segmentTravelledCounts = 0;
}

/* 判断地图坐标是否在当前已加载地图范围内。 */
static bool PathPlanner_IsInside(uint8_t x, uint8_t y)
{
    return (s_mapReady && (x < s_mapWidth) && (y < s_mapHeight));
}

/* 判断视觉端收到的地图字符是否合法。 */
bool PathPlanner_IsMapCellValid(char cell)
{
    return ((cell == PATH_MAP_CELL_WALL) || (cell == PATH_MAP_CELL_EMPTY) ||
            (cell == PATH_MAP_CELL_GOAL) || (cell == PATH_MAP_CELL_BOMB) ||
            (cell == PATH_MAP_CELL_BOX));
}

/*
 * 判断字符在普通车辆寻路中能否通过。
 * 注意：这是“车辆中心能不能直接走过去”的判断，不是推箱子判断。
 */
bool PathPlanner_IsMapCellPassableChar(char cell)
{
    return ((cell == PATH_MAP_CELL_EMPTY) || (cell == PATH_MAP_CELL_GOAL));
}

/* 判断某个地图坐标是否允许普通寻路通过。 */
static bool PathPlanner_IsPassable(uint8_t x, uint8_t y)
{
    char cell;

    if (!PathPlanner_IsInside(x, y))
    {
        return false;
    }

    cell = s_map[y][x];
    return PathPlanner_IsMapCellPassableChar(cell);
}

/* 把二维地图坐标换算为一维数组索引，供 BFS 队列和 parent 数组使用。 */
static uint16_t PathPlanner_ToIndex(uint8_t x, uint8_t y)
{
    return (uint16_t)(((uint16_t)y * (uint16_t)s_mapWidth) + (uint16_t)x);
}

/* 根据相邻两个格子的坐标推导从前一个格子到后一个格子的移动方向。 */
static path_dir_t PathPlanner_GetDirBetween(uint8_t fromX, uint8_t fromY, uint8_t toX, uint8_t toY)
{
    if (toX > fromX)
    {
        return PATH_DIR_X_POS;
    }
    if (toX < fromX)
    {
        return PATH_DIR_X_NEG;
    }
    if (toY > fromY)
    {
        return PATH_DIR_Y_POS;
    }
    return PATH_DIR_Y_NEG;
}

/*
 * 把一步移动加入路径队列。
 * 如果新方向和上一段方向相同，就合并成一段更长的移动，减少启停次数。
 */
static bool PathPlanner_AddStep(path_dir_t dir)
{
    path_step_t *lastStep;

    if (s_stepCount > 0U)
    {
        lastStep = &s_steps[s_stepCount - 1U];
        if ((lastStep->dir == dir) && (lastStep->cells < 255U))
        {
            lastStep->cells++;
            return true;
        }
    }

    if (s_stepCount >= APP_PATH_MAX_STEPS)
    {
        return false;
    }

    s_steps[s_stepCount].dir = dir;
    s_steps[s_stepCount].cells = 1U;
    s_stepCount++;
    return true;
}

/* 把地图方向转换成底盘 vx/vy 速度目标，单位 m/s。 */
static void PathPlanner_DirToVelocity(path_dir_t dir, float *vx, float *vy)
{
    *vx = 0.0f;
    *vy = 0.0f;

    if (dir == PATH_DIR_X_POS)
    {
        *vx = APP_PATH_DRIVE_SPEED_MPS;
    }
    else if (dir == PATH_DIR_X_NEG)
    {
        *vx = -APP_PATH_DRIVE_SPEED_MPS;
    }
    else if (dir == PATH_DIR_Y_POS)
    {
        *vy = APP_PATH_DRIVE_SPEED_MPS;
    }
    else
    {
        *vy = -APP_PATH_DRIVE_SPEED_MPS;
    }
}

/* 计算最短角度误差，单位 0.01 度，结果范围约为 -18000 到 18000。 */
static int16_t PathPlanner_GetYawError(int16_t targetCentiDeg, int16_t currentCentiDeg)
{
    int32_t error;

    error = (int32_t)targetCentiDeg - (int32_t)currentCentiDeg;
    while (error > 18000L)
    {
        error -= 36000L;
    }
    while (error < -18000L)
    {
        error += 36000L;
    }

    return (int16_t)error;
}

/*
 * 获取当前车体 yaw。
 * 优先使用视觉端角度；视觉数据超时或无效时，退回 IMU 相对 yaw。
 */
static int16_t PathPlanner_GetCurrentYaw(void)
{
    int16_t yaw;

    if (Vision_GetYawCentiDeg(&yaw))
    {
        return yaw;
    }

    return IMU_GetRelativeYawCentiDeg();
}

/*
 * 根据当前 yaw 偏差生成 wz 修正量。
 * 作用是让小车在执行一个直线段时尽量保持启动该段时的车身角度。
 */
static float PathPlanner_GetYawHoldWz(void)
{
    int16_t currentYaw;
    int16_t error;
    float correction;

    currentYaw = PathPlanner_GetCurrentYaw();
    error = PathPlanner_GetYawError(s_segmentTargetYawCentiDeg, currentYaw);
    correction = ((float)error / 100.0f) * APP_PATH_YAW_HOLD_KP_RADPS_PER_DEG;
    return PathPlanner_ClampFloat(correction, -APP_PATH_YAW_HOLD_MAX_RADPS, APP_PATH_YAW_HOLD_MAX_RADPS);
}

/* 按当前路径段方向和 yaw 修正量刷新底盘速度目标。 */
static void PathPlanner_UpdateSegmentVelocity(void)
{
    float vx;
    float vy;
    float wz;

    PathPlanner_DirToVelocity(s_segmentDir, &vx, &vy);
    wz = PathPlanner_GetYawHoldWz();
    Chassis_SetVelocity(vx, vy, wz);
}

/* 当前路径段完成后，更新模块内部记录的小车地图坐标。 */
static void PathPlanner_ApplyFinishedStep(const path_step_t *step)
{
    if (step == 0)
    {
        return;
    }

    if (step->dir == PATH_DIR_X_POS)
    {
        s_poseX = (uint8_t)(s_poseX + step->cells);
    }
    else if (step->dir == PATH_DIR_X_NEG)
    {
        s_poseX = (s_poseX >= step->cells) ? (uint8_t)(s_poseX - step->cells) : 0U;
    }
    else if (step->dir == PATH_DIR_Y_POS)
    {
        s_poseY = (uint8_t)(s_poseY + step->cells);
    }
    else
    {
        s_poseY = (s_poseY >= step->cells) ? (uint8_t)(s_poseY - step->cells) : 0U;
    }
}

/*
 * 启动路径队列中的下一段。
 * 这里会把“格子数”换算为实际距离，再换算为编码器计数目标。
 */
static bool PathPlanner_StartNextSegment(void)
{
    float distanceM;
    int32_t targetCounts;

    if (s_stepIndex >= s_stepCount)
    {
        Chassis_Stop();
        s_segmentActive = false;
        s_pathState = PATH_STATE_FINISHED;
        return false;
    }

    distanceM = ((float)s_cellSizeMm * (float)s_steps[s_stepIndex].cells) / 1000.0f;
    targetCounts = PathPlanner_AbsInt32(Chassis_DistanceMToEncoderCounts(distanceM));
    if (targetCounts <= 0)
    {
        Chassis_Stop();
        s_segmentActive = false;
        s_pathState = PATH_STATE_ERROR;
        return false;
    }

    s_segmentDir = s_steps[s_stepIndex].dir;
    s_segmentTargetCounts = targetCounts;
    s_segmentTravelledCounts = 0;
    s_segmentTargetYawCentiDeg = PathPlanner_GetCurrentYaw();
    s_segmentActive = true;

    BSP_EncoderClearAll();
    PathPlanner_UpdateSegmentVelocity();
    return true;
}

/* 把任意角度归一化到 -180.00～180.00 度。 */
static int16_t Mission_NormalizeYaw(int32_t yawCentiDeg)
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

/* 将地图四方向转换成视觉/IMU使用的绝对yaw目标。 */
static int16_t Mission_HeadingToYaw(path_dir_t heading)
{
    int32_t yaw;

    yaw = APP_MAP_X_POS_YAW_CDEG +
          (APP_MAP_YAW_DIRECTION_SIGN * (int32_t)heading * 9000L);
    return Mission_NormalizeYaw(yaw);
}

static uint8_t Mission_GetQuarterTurns(path_dir_t from, path_dir_t to)
{
    uint8_t difference;

    difference = ((uint8_t)from > (uint8_t)to) ?
                     (uint8_t)((uint8_t)from - (uint8_t)to) :
                     (uint8_t)((uint8_t)to - (uint8_t)from);
    return (difference > 2U) ? (uint8_t)(4U - difference) : difference;
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

/* 扫描地图中的箱子和目标点，并清空上一次识别得到的编号。 */
static bool Mission_BuildRecognitionPoints(void)
{
    uint8_t x;
    uint8_t y;
    recognition_point_t *point;

    s_recognitionPointCount = 0U;
    s_recognizedCount = 0U;
    s_currentRecognitionPoint = 0U;

    for (y = 0U; y < APP_MAP_MAX_HEIGHT; y++)
    {
        for (x = 0U; x < APP_MAP_MAX_WIDTH; x++)
        {
            s_objectCode[y][x] = APP_RECOGNITION_CODE_UNKNOWN;
        }
    }

    for (y = 0U; y < s_mapHeight; y++)
    {
        for (x = 0U; x < s_mapWidth; x++)
        {
            if ((s_map[y][x] != PATH_MAP_CELL_BOX) && (s_map[y][x] != PATH_MAP_CELL_GOAL))
            {
                continue;
            }
            if (s_recognitionPointCount >= APP_MISSION_MAX_RECOGNITION_POINTS)
            {
                return false;
            }

            point = &s_recognitionPoints[s_recognitionPointCount];
            point->objectX = x;
            point->objectY = y;
            point->observeX = x;
            point->observeY = y;
            point->observeDir = PATH_DIR_X_POS;
            point->resultCode = APP_RECOGNITION_CODE_UNKNOWN;
            point->recognized = false;
            s_recognitionPointCount++;
        }
    }

    return true;
}

/* 从小车当前位置进行一次BFS，得到所有可通行格子的最短距离。 */
static bool Mission_FillDistances(void)
{
    uint16_t cellCount;
    uint16_t head = 0U;
    uint16_t tail = 0U;
    uint16_t index;
    uint16_t currentIndex;
    uint16_t nextIndex;
    uint8_t currentX;
    uint8_t currentY;
    int16_t nextX;
    int16_t nextY;
    int8_t dx[4] = {1, 0, -1, 0};
    int8_t dy[4] = {0, 1, 0, -1};
    uint8_t direction;

    if (!s_poseReady || !PathPlanner_IsPassable(s_poseX, s_poseY))
    {
        return false;
    }

    cellCount = (uint16_t)((uint16_t)s_mapWidth * (uint16_t)s_mapHeight);
    for (index = 0U; index < cellCount; index++)
    {
        s_bfsVisited[index] = 0U;
        s_bfsDistance[index] = 0xFFFFU;
    }

    currentIndex = PathPlanner_ToIndex(s_poseX, s_poseY);
    s_bfsQueue[tail++] = currentIndex;
    s_bfsVisited[currentIndex] = 1U;
    s_bfsDistance[currentIndex] = 0U;

    while (head < tail)
    {
        currentIndex = s_bfsQueue[head++];
        currentX = (uint8_t)(currentIndex % s_mapWidth);
        currentY = (uint8_t)(currentIndex / s_mapWidth);

        for (direction = 0U; direction < 4U; direction++)
        {
            nextX = (int16_t)currentX + dx[direction];
            nextY = (int16_t)currentY + dy[direction];
            if ((nextX < 0) || (nextY < 0) ||
                (nextX >= (int16_t)s_mapWidth) || (nextY >= (int16_t)s_mapHeight) ||
                !PathPlanner_IsPassable((uint8_t)nextX, (uint8_t)nextY))
            {
                continue;
            }

            nextIndex = PathPlanner_ToIndex((uint8_t)nextX, (uint8_t)nextY);
            if (s_bfsVisited[nextIndex] != 0U)
            {
                continue;
            }

            s_bfsVisited[nextIndex] = 1U;
            s_bfsDistance[nextIndex] = (uint16_t)(s_bfsDistance[currentIndex] + 1U);
            s_bfsQueue[tail++] = nextIndex;
        }
    }

    return true;
}

/* 从全部未识别物体的相邻格中选择行驶距离和转向代价最小的观察位姿。 */
static bool Mission_SelectNextObservation(void)
{
    uint16_t pointIndex;
    uint16_t cellIndex;
    uint16_t distance;
    uint8_t direction;
    uint8_t turnCount;
    uint32_t score;
    uint32_t bestScore = 0xFFFFFFFFUL;
    uint16_t bestPoint = 0U;
    uint8_t bestX = 0U;
    uint8_t bestY = 0U;
    path_dir_t bestDirection = PATH_DIR_X_POS;
    int16_t observeX;
    int16_t observeY;
    int8_t dx[4] = {1, 0, -1, 0};
    int8_t dy[4] = {0, 1, 0, -1};
    bool found = false;
    recognition_point_t *point;

    if (!Mission_FillDistances())
    {
        return false;
    }

    for (pointIndex = 0U; pointIndex < s_recognitionPointCount; pointIndex++)
    {
        point = &s_recognitionPoints[pointIndex];
        if (point->recognized)
        {
            continue;
        }

        for (direction = 0U; direction < 4U; direction++)
        {
            /* direction 是从观察格朝向物体的方向，因此观察格位于反方向。 */
            observeX = (int16_t)point->objectX - dx[direction];
            observeY = (int16_t)point->objectY - dy[direction];
            if ((observeX < 0) || (observeY < 0) ||
                (observeX >= (int16_t)s_mapWidth) || (observeY >= (int16_t)s_mapHeight) ||
                !PathPlanner_IsPassable((uint8_t)observeX, (uint8_t)observeY))
            {
                continue;
            }

            cellIndex = PathPlanner_ToIndex((uint8_t)observeX, (uint8_t)observeY);
            distance = s_bfsDistance[cellIndex];
            if (distance == 0xFFFFU)
            {
                continue;
            }

            turnCount = Mission_GetQuarterTurns(s_missionDefaultHeading, (path_dir_t)direction);
            score = ((uint32_t)distance * APP_MISSION_PATH_COST_PER_CELL) +
                    ((uint32_t)turnCount * APP_MISSION_TURN_COST_PER_QUARTER);
            if (score < bestScore)
            {
                bestScore = score;
                bestPoint = pointIndex;
                bestX = (uint8_t)observeX;
                bestY = (uint8_t)observeY;
                bestDirection = (path_dir_t)direction;
                found = true;
            }
        }
    }

    if (!found)
    {
        return false;
    }

    s_currentRecognitionPoint = bestPoint;
    s_recognitionPoints[bestPoint].observeX = bestX;
    s_recognitionPoints[bestPoint].observeY = bestY;
    s_recognitionPoints[bestPoint].observeDir = bestDirection;
    return true;
}

static void Mission_StartRotation(path_dir_t targetHeading)
{
    s_rotationTargetHeading = targetHeading;
    s_rotationTargetYawCentiDeg = Mission_HeadingToYaw(targetHeading);
    s_rotationStableCount = 0U;
    s_missionStateTick = xTaskGetTickCount();
}

/* 运行原地转向角度闭环；连续稳定后返回true，超时会进入任务错误状态。 */
static bool Mission_UpdateRotation(void)
{
    int16_t currentYaw;
    int16_t error;
    int32_t absoluteError;
    float wz;

    if ((xTaskGetTickCount() - s_missionStateTick) > pdMS_TO_TICKS(APP_MISSION_ROTATE_TIMEOUT_MS))
    {
        Chassis_Stop();
        s_missionState = MISSION_STATE_ERROR;
        return false;
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
            return true;
        }
        return false;
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
    return false;
}

static bool Mission_SendRecognitionRequest(void)
{
    uint16_t sequence;

    sequence = Mission_NextSequence();
    if (!Vision_RequestRecognition(sequence, s_currentRecognitionPoint))
    {
        s_missionState = MISSION_STATE_ERROR;
        return false;
    }

    s_missionState = MISSION_STATE_WAIT_RECOGNITION;
    s_missionStateTick = xTaskGetTickCount();
    return true;
}

/* 识别任务状态机，由 PathPlanner_Update() 每20ms推进一次。 */
static void Mission_Update(void)
{
    recognition_point_t *point;

    switch (s_missionState)
    {
        case MISSION_STATE_WAIT_MAP:
            if ((xTaskGetTickCount() - s_missionStateTick) >= pdMS_TO_TICKS(APP_MISSION_MAP_TIMEOUT_MS))
            {
                s_missionRetryCount++;
                if (!Vision_RequestMap(Mission_NextSequence()))
                {
                    s_missionState = MISSION_STATE_ERROR;
                }
                s_missionStateTick = xTaskGetTickCount();
            }
            break;

        case MISSION_STATE_SELECT_POINT:
            if (s_recognizedCount >= s_recognitionPointCount)
            {
                Chassis_Stop();
                s_missionState = MISSION_STATE_READY_FOR_PUSH;
                break;
            }
            if (!Mission_SelectNextObservation())
            {
                s_missionState = MISSION_STATE_ERROR;
                break;
            }
            point = &s_recognitionPoints[s_currentRecognitionPoint];
            if (!PathPlanner_Goto(point->observeX, point->observeY))
            {
                s_missionState = MISSION_STATE_ERROR;
                break;
            }
            s_missionState = MISSION_STATE_MOVING_TO_OBSERVE;
            break;

        case MISSION_STATE_MOVING_TO_OBSERVE:
            if (s_pathState == PATH_STATE_ERROR)
            {
                s_missionState = MISSION_STATE_ERROR;
            }
            else if (s_pathState == PATH_STATE_FINISHED)
            {
                Chassis_Stop();
                s_missionState = MISSION_STATE_WAIT_STOP;
                s_missionStateTick = xTaskGetTickCount();
            }
            break;

        case MISSION_STATE_WAIT_STOP:
            if ((xTaskGetTickCount() - s_missionStateTick) >= pdMS_TO_TICKS(APP_MISSION_STOP_SETTLE_MS))
            {
                point = &s_recognitionPoints[s_currentRecognitionPoint];
                if (s_heading == point->observeDir)
                {
                    (void)Mission_SendRecognitionRequest();
                }
                else
                {
                    Mission_StartRotation(point->observeDir);
                    s_missionState = MISSION_STATE_ROTATE_TO_OBJECT;
                }
            }
            break;

        case MISSION_STATE_ROTATE_TO_OBJECT:
            if (Mission_UpdateRotation())
            {
                (void)Mission_SendRecognitionRequest();
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
            (void)Mission_SendRecognitionRequest();
            break;

        case MISSION_STATE_ROTATE_TO_DEFAULT:
            if (Mission_UpdateRotation())
            {
                s_missionState = MISSION_STATE_SELECT_POINT;
            }
            break;

        default:
            break;
    }
}
/* 初始化路径规划模块状态。 */
void PathPlanner_Init(void)
{
    /*
     * 对工作区首尾执行一次真实访问，防止链接器将整个专用RAM段删除。
     * 不需要清空全部700KB，否则会增加启动时间。
     */
    ((volatile uint8_t *)s_pathPlannerWorkspace)[0] = 0U;
    ((volatile uint8_t *)s_pathPlannerWorkspace)
        [APP_PATH_PLANNER_RAM_SIZE - 1U] = 0U;

    s_pathState = PATH_STATE_IDLE;
    s_mapWidth = 0U;
    s_mapHeight = 0U;
    s_mapReady = false;
    s_poseReady = false;
    s_poseX = 0U;
    s_poseY = 0U;
    s_heading = PATH_DIR_X_POS;
    s_cellSizeMm = APP_PATH_CELL_SIZE_MM;
    PathPlanner_ClearSteps();
    s_missionState = MISSION_STATE_IDLE;
    s_recognitionPointCount = 0U;
    s_recognizedCount = 0U;
    s_currentRecognitionPoint = 0U;
    s_missionRequestSequence = 0U;
    s_missionRetryCount = 0U;
    s_missionStateTick = xTaskGetTickCount();
    s_missionDefaultHeading = PATH_DIR_X_POS;
    s_rotationTargetHeading = PATH_DIR_X_POS;
    s_rotationTargetYawCentiDeg = 0;
    s_rotationStableCount = 0U;
}

/* 启动当前已经生成好的路径段队列。 */
void PathPlanner_Start(void)
{
    if ((s_stepCount == 0U) || !s_poseReady)
    {
        s_pathState = PATH_STATE_ERROR;
        return;
    }

    s_stepIndex = 0U;
    s_segmentActive = false;
    s_pathState = PATH_STATE_RUNNING;
}

/* 停止路径规划，并立刻停止底盘。 */
void PathPlanner_Stop(void)
{
    Chassis_Stop();
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    if ((s_missionState != MISSION_STATE_IDLE) &&
        (s_missionState != MISSION_STATE_READY_FOR_PUSH) &&
        (s_missionState != MISSION_STATE_ERROR))
    {
        s_missionState = MISSION_STATE_STOPPED;
    }
}

/*
 * 路径规划周期任务的核心函数。
 * 运行逻辑：
 * 1. 如果还没启动当前段，就启动下一段。
 * 2. 如果正在执行，就读取编码器平均计数。
 * 3. 达到目标计数后停止当前段并更新地图坐标。
 * 4. 没走够时继续刷新底盘速度和 yaw 修正。
 */
static void PathPlanner_UpdateMotion(void)
{
    int32_t finishThreshold;

    if (s_pathState != PATH_STATE_RUNNING)
    {
        return;
    }

    if (!s_segmentActive)
    {
        (void)PathPlanner_StartNextSegment();
        return;
    }

    s_segmentTravelledCounts = Chassis_GetAverageAbsEncoderCounts();
    finishThreshold = s_segmentTargetCounts - APP_PATH_FINISH_TOLERANCE_COUNTS;
    if (finishThreshold < 0)
    {
        finishThreshold = 0;
    }

    if (s_segmentTravelledCounts >= finishThreshold)
    {
        Chassis_Stop();
        PathPlanner_ApplyFinishedStep(&s_steps[s_stepIndex]);
        s_stepIndex++;
        s_segmentActive = false;

        if (s_stepIndex >= s_stepCount)
        {
            s_pathState = PATH_STATE_FINISHED;
        }
        return;
    }

    PathPlanner_UpdateSegmentVelocity();
}

/* 先推进底盘路径，再推进依赖路径结果的巡检识别状态机。 */
void PathPlanner_Update(void)
{
    PathPlanner_UpdateMotion();
    Mission_Update();
}
/* 获取当前路径规划状态。 */
path_state_t PathPlanner_GetState(void)
{
    return s_pathState;
}

/* 设置单个地图格子的实际边长，单位 mm。 */
void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm)
{
    if (cellSizeMm == 0U)
    {
        return;
    }

    s_cellSizeMm = cellSizeMm;
}

/* 获取当前地图格子边长，单位 mm。 */
uint16_t PathPlanner_GetCellSizeMm(void)
{
    return s_cellSizeMm;
}

/* 从连续字符数组加载地图，cells 长度至少为 width * height。 */
bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells)
{
    uint8_t x;
    uint8_t y;
    char ch;

    if ((cells == 0) || (width == 0U) || (height == 0U) ||
        (width > APP_MAP_MAX_WIDTH) || (height > APP_MAP_MAX_HEIGHT))
    {
        return false;
    }

    for (y = 0U; y < height; y++)
    {
        for (x = 0U; x < width; x++)
        {
            ch = cells[((uint16_t)y * (uint16_t)width) + (uint16_t)x];
            if (!PathPlanner_IsMapCellValid(ch))
            {
                return false;
            }
            s_map[y][x] = ch;
        }
    }

    s_mapWidth = width;
    s_mapHeight = height;
    s_mapReady = true;
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    return true;
}

/*
 * 从以 / 或 | 分隔的多行文本加载地图。
 * 例如 width=5、height=3 时，可以传入 "#####/#---#/#####"。
 */
bool PathPlanner_SetMapRows(uint8_t width, uint8_t height, const char *rows)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    char ch;

    if ((rows == 0) || (width == 0U) || (height == 0U) ||
        (width > APP_MAP_MAX_WIDTH) || (height > APP_MAP_MAX_HEIGHT))
    {
        return false;
    }

    while ((ch = *rows) != '\0')
    {
        rows++;
        if ((ch == '/') || (ch == '|'))
        {
            if (x != width)
            {
                return false;
            }
            x = 0U;
            y++;
            if (y >= height)
            {
                return false;
            }
            continue;
        }

        if (x >= width)
        {
            return false;
        }
        if (!PathPlanner_IsMapCellValid(ch))
        {
            return false;
        }
        s_map[y][x] = ch;
        x++;
    }

    if ((y != (uint8_t)(height - 1U)) || (x != width))
    {
        return false;
    }

    s_mapWidth = width;
    s_mapHeight = height;
    s_mapReady = true;
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    return true;
}

/*
 * 设置小车当前地图坐标和朝向。
 * 如果地图已经加载，则起点必须在可通行格子上。
 */
bool PathPlanner_SetPose(uint8_t x, uint8_t y, path_dir_t heading)
{
    if (!PathPlanner_IsValidDir(heading))
    {
        return false;
    }

    if (s_mapReady && !PathPlanner_IsPassable(x, y))
    {
        return false;
    }

    s_poseX = x;
    s_poseY = y;
    s_heading = heading;
    s_poseReady = true;
    return true;
}

/*
 * 直接生成一个“朝某方向走若干格”的路径。
 * 这个接口绕开地图和 BFS，适合单独测试距离执行效果。
 */
bool PathPlanner_MoveCells(path_dir_t dir, uint8_t cells)
{
    if (!PathPlanner_IsValidDir(dir) || (cells == 0U))
    {
        return false;
    }

    PathPlanner_ClearSteps();
    s_steps[0].dir = dir;
    s_steps[0].cells = cells;
    s_stepCount = 1U;
    s_poseReady = true;
    PathPlanner_Start();
    return true;
}

/*
 * 用 BFS 从当前位姿规划到目标格。
 * 当前 BFS 只处理普通车辆移动，不处理推箱子的箱子状态。
 */
bool PathPlanner_Goto(uint8_t targetX, uint8_t targetY)
{
    uint16_t cellCount;
    uint16_t head = 0U;
    uint16_t tail = 0U;
    uint16_t startIndex;
    uint16_t targetIndex;
    uint16_t currentIndex;
    uint16_t nextIndex;
    uint16_t reverseCount = 0U;
    int16_t parentIndex;
    int8_t dx[4] = {1, 0, -1, 0};
    int8_t dy[4] = {0, 1, 0, -1};
    uint8_t index;
    uint8_t currentX;
    uint8_t currentY;
    uint8_t nextX;
    uint8_t nextY;

    if (!s_mapReady || !s_poseReady || !PathPlanner_IsPassable(targetX, targetY) ||
        !PathPlanner_IsPassable(s_poseX, s_poseY))
    {
        s_pathState = PATH_STATE_ERROR;
        return false;
    }

    startIndex = PathPlanner_ToIndex(s_poseX, s_poseY);
    targetIndex = PathPlanner_ToIndex(targetX, targetY);
    if (startIndex == targetIndex)
    {
        Chassis_Stop();
        PathPlanner_ClearSteps();
        s_pathState = PATH_STATE_FINISHED;
        return true;
    }

    cellCount = (uint16_t)((uint16_t)s_mapWidth * (uint16_t)s_mapHeight);
    for (currentIndex = 0U; currentIndex < cellCount; currentIndex++)
    {
        s_bfsVisited[currentIndex] = 0U;
        s_bfsParent[currentIndex] = -1;
    }

    s_bfsQueue[tail] = startIndex;
    tail++;
    s_bfsVisited[startIndex] = 1U;

    while (head < tail)
    {
        currentIndex = s_bfsQueue[head];
        head++;
        if (currentIndex == targetIndex)
        {
            break;
        }

        currentX = (uint8_t)(currentIndex % s_mapWidth);
        currentY = (uint8_t)(currentIndex / s_mapWidth);
        for (index = 0U; index < 4U; index++)
        {
            if (((dx[index] < 0) && (currentX == 0U)) || ((dy[index] < 0) && (currentY == 0U)))
            {
                continue;
            }

            nextX = (uint8_t)(currentX + dx[index]);
            nextY = (uint8_t)(currentY + dy[index]);
            if (!PathPlanner_IsPassable(nextX, nextY))
            {
                continue;
            }

            nextIndex = PathPlanner_ToIndex(nextX, nextY);
            if (s_bfsVisited[nextIndex] != 0U)
            {
                continue;
            }

            s_bfsVisited[nextIndex] = 1U;
            s_bfsParent[nextIndex] = (int16_t)currentIndex;
            s_bfsQueue[tail] = nextIndex;
            tail++;
        }
    }

    if (s_bfsVisited[targetIndex] == 0U)
    {
        s_pathState = PATH_STATE_ERROR;
        return false;
    }

    /*
     * 从目标点沿 parent 反向回溯到起点。
     * 回溯得到的是反向顺序，所以先存到 s_reverseDirs，后面再倒序加入步骤队列。
     */
    currentIndex = targetIndex;
    while (currentIndex != startIndex)
    {
        parentIndex = s_bfsParent[currentIndex];
        if ((parentIndex < 0) || (reverseCount >= APP_PATH_MAX_STEPS))
        {
            s_pathState = PATH_STATE_ERROR;
            return false;
        }

        currentX = (uint8_t)(((uint16_t)parentIndex) % s_mapWidth);
        currentY = (uint8_t)(((uint16_t)parentIndex) / s_mapWidth);
        nextX = (uint8_t)(currentIndex % s_mapWidth);
        nextY = (uint8_t)(currentIndex / s_mapWidth);
        s_reverseDirs[reverseCount] = PathPlanner_GetDirBetween(currentX, currentY, nextX, nextY);
        reverseCount++;
        currentIndex = (uint16_t)parentIndex;
    }

    PathPlanner_ClearSteps();
    while (reverseCount > 0U)
    {
        reverseCount--;
        if (!PathPlanner_AddStep(s_reverseDirs[reverseCount]))
        {
            s_pathState = PATH_STATE_ERROR;
            return false;
        }
    }

    PathPlanner_Start();
    return true;
}

/* 复制当前路径规划状态，供串口 @PATH 返回使用。 */
void PathPlanner_GetStatus(path_status_t *status)
{
    if (status == 0)
    {
        return;
    }

    status->state = s_pathState;
    status->mapReady = s_mapReady;
    status->poseReady = s_poseReady;
    status->segmentActive = s_segmentActive;
    status->x = s_poseX;
    status->y = s_poseY;
    status->heading = s_heading;
    status->stepIndex = s_stepIndex;
    status->stepCount = s_stepCount;
    status->cellSizeMm = s_cellSizeMm;
    status->travelledCounts = s_segmentTravelledCounts;
    status->targetCounts = s_segmentTargetCounts;
}


/* 启动完整巡检任务：先向视觉串口一请求一次地图。 */
bool PathPlanner_MissionStart(void)
{
    if (!s_poseReady)
    {
        s_missionState = MISSION_STATE_ERROR;
        return false;
    }

    Chassis_Stop();
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    s_mapReady = false;
    s_mapWidth = 0U;
    s_mapHeight = 0U;
    s_recognitionPointCount = 0U;
    s_recognizedCount = 0U;
    s_currentRecognitionPoint = 0U;
    s_missionRetryCount = 0U;
    s_missionRequestSequence = 0U;
    s_missionDefaultHeading = s_heading;
    s_missionState = MISSION_STATE_WAIT_MAP;
    s_missionStateTick = xTaskGetTickCount();

    if (!Vision_RequestMap(Mission_NextSequence()))
    {
        s_missionState = MISSION_STATE_ERROR;
        return false;
    }
    return true;
}

void PathPlanner_MissionStop(void)
{
    Chassis_Stop();
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_STOPPED;
}

void PathPlanner_MissionReset(void)
{
    Chassis_Stop();
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    s_missionState = MISSION_STATE_IDLE;
    s_mapReady = false;
    s_mapWidth = 0U;
    s_mapHeight = 0U;
    s_poseReady = false;
    s_recognitionPointCount = 0U;
    s_recognizedCount = 0U;
    s_currentRecognitionPoint = 0U;
    s_missionRequestSequence = 0U;
    s_missionRetryCount = 0U;
}

void PathPlanner_MissionOnMapReceived(uint16_t sequence)
{
    if (s_missionState != MISSION_STATE_WAIT_MAP)
    {
        return;
    }
    if ((sequence != 0U) && (sequence != s_missionRequestSequence))
    {
        return;
    }
    if (!s_mapReady || !s_poseReady || !PathPlanner_IsPassable(s_poseX, s_poseY) ||
        !Mission_BuildRecognitionPoints())
    {
        s_missionState = MISSION_STATE_ERROR;
        return;
    }

    s_missionRetryCount = 0U;
    s_missionState = (s_recognitionPointCount == 0U) ?
                         MISSION_STATE_READY_FOR_PUSH : MISSION_STATE_SELECT_POINT;
}

bool PathPlanner_MissionOnRecognitionResult(uint16_t sequence, uint8_t code)
{
    recognition_point_t *point;

    if ((s_missionState != MISSION_STATE_WAIT_RECOGNITION) || (code > 20U) ||
        ((sequence != 0U) && (sequence != s_missionRequestSequence)) ||
        (s_currentRecognitionPoint >= s_recognitionPointCount))
    {
        return false;
    }

    point = &s_recognitionPoints[s_currentRecognitionPoint];
    if ((code == 20U) ||
        ((s_map[point->objectY][point->objectX] == PATH_MAP_CELL_BOX) && (code > 9U)) ||
        ((s_map[point->objectY][point->objectX] == PATH_MAP_CELL_GOAL) && (code < 10U)))
    {
        s_missionRetryCount++;
        s_missionState = MISSION_STATE_RETRY_RECOGNITION;
        return true;
    }

    point->resultCode = code;
    if (!point->recognized)
    {
        point->recognized = true;
        s_recognizedCount++;
    }
    s_objectCode[point->objectY][point->objectX] = code;
    s_missionRetryCount = 0U;

    if (s_heading == s_missionDefaultHeading)
    {
        s_missionState = MISSION_STATE_SELECT_POINT;
    }
    else
    {
        Mission_StartRotation(s_missionDefaultHeading);
        s_missionState = MISSION_STATE_ROTATE_TO_DEFAULT;
    }
    return true;
}

void PathPlanner_GetMissionStatus(mission_status_t *status)
{
    if (status == 0)
    {
        return;
    }

    status->state = s_missionState;
    status->currentPoint = s_currentRecognitionPoint;
    status->pointCount = s_recognitionPointCount;
    status->recognizedCount = s_recognizedCount;
    status->requestSequence = s_missionRequestSequence;
    status->retryCount = s_missionRetryCount;
    status->mapReady = s_mapReady;
    status->poseReady = s_poseReady;
}

bool PathPlanner_GetRecognitionPoint(uint16_t index, recognition_point_t *point)
{
    if ((point == 0) || (index >= s_recognitionPointCount))
    {
        return false;
    }

    *point = s_recognitionPoints[index];
    return true;
}
uint8_t *PathPlanner_GetWorkspace(void)
{
    return s_pathPlannerWorkspace;
}

uint32_t PathPlanner_GetWorkspaceSize(void)
{
    return (uint32_t)sizeof(s_pathPlannerWorkspace);
}


