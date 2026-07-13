#include "path_planner.h"
#include "../bsp/bsp_encoder.h"
#include "../chassis/chassis.h"
#include "../sensor/imu.h"
#include "../sensor/vision.h"

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
}

/*
 * 路径规划周期任务的核心函数。
 * 运行逻辑：
 * 1. 如果还没启动当前段，就启动下一段。
 * 2. 如果正在执行，就读取编码器平均计数。
 * 3. 达到目标计数后停止当前段并更新地图坐标。
 * 4. 没走够时继续刷新底盘速度和 yaw 修正。
 */
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
