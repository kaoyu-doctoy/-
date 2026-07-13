#include "path_planner.h"
#include "../bsp/bsp_encoder.h"
#include "../chassis/chassis.h"
#include "../sensor/imu.h"
#include "../sensor/vision.h"

typedef struct
{
    path_dir_t dir;
    uint8_t cells;
} path_step_t;

static path_state_t s_pathState;
static char s_map[APP_MAP_MAX_HEIGHT][APP_MAP_MAX_WIDTH];
static uint8_t s_mapWidth;
static uint8_t s_mapHeight;
static bool s_mapReady;
static bool s_poseReady;
static uint8_t s_poseX;
static uint8_t s_poseY;
static path_dir_t s_heading;
static uint16_t s_cellSizeMm;

static path_step_t s_steps[APP_PATH_MAX_STEPS];
static uint16_t s_stepCount;
static uint16_t s_stepIndex;

static bool s_segmentActive;
static path_dir_t s_segmentDir;
static int32_t s_segmentTargetCounts;
static int32_t s_segmentTravelledCounts;
static int16_t s_segmentTargetYawCentiDeg;

static uint16_t s_bfsQueue[APP_PATH_MAX_STEPS];
static int16_t s_bfsParent[APP_PATH_MAX_STEPS];
static uint8_t s_bfsVisited[APP_PATH_MAX_STEPS];
static path_dir_t s_reverseDirs[APP_PATH_MAX_STEPS];

/* 判断路径方向编号是否有效。 */
static bool PathPlanner_IsValidDir(path_dir_t dir)
{
    return ((uint32_t)dir <= (uint32_t)PATH_DIR_Y_NEG);
}

/* 返回 int32_t 绝对值。 */
static int32_t PathPlanner_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 将浮点值限制在给定范围内。 */
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

/* 清空当前路线队列和正在执行的段。 */
static void PathPlanner_ClearSteps(void)
{
    s_stepCount = 0U;
    s_stepIndex = 0U;
    s_segmentActive = false;
    s_segmentTargetCounts = 0;
    s_segmentTravelledCounts = 0;
}

/* 判断地图坐标是否在当前地图范围内。 */
static bool PathPlanner_IsInside(uint8_t x, uint8_t y)
{
    return (s_mapReady && (x < s_mapWidth) && (y < s_mapHeight));
}

/* 判断格子是否允许车体中心通过。 */
static bool PathPlanner_IsPassable(uint8_t x, uint8_t y)
{
    char cell;

    if (!PathPlanner_IsInside(x, y))
    {
        return false;
    }

    cell = s_map[y][x];
    if ((cell == '#') || (cell == 'W') || (cell == 'w') || (cell == '1') ||
        (cell == 'B') || (cell == 'b') || (cell == 'X') || (cell == 'x'))
    {
        return false;
    }

    return true;
}

/* 将地图坐标换算为一维索引。 */
static uint16_t PathPlanner_ToIndex(uint8_t x, uint8_t y)
{
    return (uint16_t)(((uint16_t)y * (uint16_t)s_mapWidth) + (uint16_t)x);
}

/* 根据两个相邻格子的坐标推导运动方向。 */
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

/* 把一个方向加入路线队列，相邻同向步骤会合并成多格移动。 */
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

/* 将方向转换为底盘 vx/vy 速度目标。 */
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

/* 计算最短角度误差，单位 0.01 度。 */
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

/* 优先使用视觉角度，超时后退回 IMU 相对偏航角。 */
static int16_t PathPlanner_GetCurrentYaw(void)
{
    int16_t yaw;

    if (Vision_GetYawCentiDeg(&yaw))
    {
        return yaw;
    }

    return IMU_GetRelativeYawCentiDeg();
}

/* 根据角度误差生成保持车身平行的角速度补偿。 */
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

/* 按当前段方向和角度补偿刷新底盘速度目标。 */
static void PathPlanner_UpdateSegmentVelocity(void)
{
    float vx;
    float vy;
    float wz;

    PathPlanner_DirToVelocity(s_segmentDir, &vx, &vy);
    wz = PathPlanner_GetYawHoldWz();
    Chassis_SetVelocity(vx, vy, wz);
}

/* 根据已经完成的段更新地图位置。 */
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

/* 启动路线队列中的下一段格子移动。 */
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

/* 初始化路径规划模块状态。 */
void PathPlanner_Init(void)
{
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
}

/* 启动当前已经生成的路径队列。 */
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

/* 停止路径规划并立即停止底盘。 */
void PathPlanner_Stop(void)
{
    Chassis_Stop();
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
}

/* 周期更新路径状态机，执行当前格子移动。 */
void PathPlanner_Update(void)
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

/* 获取当前路径规划状态。 */
path_state_t PathPlanner_GetState(void)
{
    return s_pathState;
}

/* 设置地图格子尺寸，单位 mm。 */
void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm)
{
    if (cellSizeMm == 0U)
    {
        return;
    }

    s_cellSizeMm = cellSizeMm;
}

/* 获取当前地图格子尺寸，单位 mm。 */
uint16_t PathPlanner_GetCellSizeMm(void)
{
    return s_cellSizeMm;
}

/* 从扁平格子数组加载地图，长度至少为 width * height。 */
bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells)
{
    uint8_t x;
    uint8_t y;

    if ((cells == 0) || (width == 0U) || (height == 0U) ||
        (width > APP_MAP_MAX_WIDTH) || (height > APP_MAP_MAX_HEIGHT))
    {
        return false;
    }

    for (y = 0U; y < height; y++)
    {
        for (x = 0U; x < width; x++)
        {
            s_map[y][x] = cells[((uint16_t)y * (uint16_t)width) + (uint16_t)x];
        }
    }

    s_mapWidth = width;
    s_mapHeight = height;
    s_mapReady = true;
    PathPlanner_ClearSteps();
    s_pathState = PATH_STATE_IDLE;
    return true;
}

/* 从以 / 或 | 分隔的多行文本加载地图。 */
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

/* 设置小车在地图中的当前位置和朝向。 */
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

/* 直接执行一个方向上的格子移动，用于单独调试编码器闭环。 */
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

/* 根据当前地图和当前位置，用 BFS 规划到指定目标格。 */
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

/* 复制路径规划当前状态。 */
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
