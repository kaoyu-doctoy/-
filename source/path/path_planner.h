#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "../app/app_config.h"
#include "planner.h"

typedef enum
{
    PATH_STATE_IDLE = 0,
    PATH_STATE_RUNNING,
    PATH_STATE_FINISHED,
    PATH_STATE_ERROR
} path_state_t;

/* 地图坐标方向：右、下、左、上。视觉端 pose 的 direction 继续使用该顺序。 */
typedef enum
{
    PATH_DIR_X_POS = 0,
    PATH_DIR_Y_POS = 1,
    PATH_DIR_X_NEG = 2,
    PATH_DIR_Y_NEG = 3
} path_dir_t;

#define PATH_MAP_CELL_WALL  ('#')
#define PATH_MAP_CELL_EMPTY ('-')
#define PATH_MAP_CELL_GOAL  ('.')
#define PATH_MAP_CELL_BOMB  ('*')
#define PATH_MAP_CELL_BOX   ('$')

typedef struct
{
    path_state_t state;
    bool mapReady;
    bool poseReady;
    bool segmentActive;
    uint8_t x;
    uint8_t y;
    path_dir_t heading;
    uint16_t stepIndex;
    uint16_t stepCount;
    uint16_t cellSizeMm;
    int32_t travelledCounts;
    int32_t targetCounts;
} path_status_t;

typedef enum
{
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_LEAVING_START,
    MISSION_STATE_WAIT_MAP,
    MISSION_STATE_WAIT_INPUT,
    MISSION_STATE_PLANNING,
    MISSION_STATE_EXECUTING,
    MISSION_STATE_ROTATING,
    MISSION_STATE_WAIT_POSE,
    MISSION_STATE_WAIT_STOP,
    MISSION_STATE_WAIT_RECOGNITION,
    MISSION_STATE_RETRY_RECOGNITION,
    MISSION_STATE_ENTERING_START,
    MISSION_STATE_WAIT_START_SETTLE,
    MISSION_STATE_FINISHED,
    MISSION_STATE_STOPPED,
    MISSION_STATE_ERROR
} mission_state_t;

typedef struct
{
    uint8_t objectX;
    uint8_t objectY;
    uint8_t observeX;
    uint8_t observeY;
    path_dir_t observeDir;
    uint8_t resultCode;
    bool recognized;
} recognition_point_t;

typedef struct
{
    mission_state_t state;
    uint16_t currentPoint;
    uint16_t pointCount;
    uint16_t recognizedCount;
    uint16_t requestSequence;
    uint16_t retryCount;
    bool mapReady;
    bool poseReady;
    uint16_t activePlanStep;
    uint16_t activePlanSteps;
    uint8_t plannerJobStatus;
    bool backgroundPlanning;
    uint8_t currentLevel;
    uint8_t completedLevels;
    uint8_t initialBoxes;
    uint8_t remainingBoxes;
    bool startAreaKnown;
    bool inStartArea;
    bool levelSolved;
    uint32_t levelElapsedMs;
} mission_status_t;

void PathPlanner_Init(void);
void PathPlanner_Start(void);
void PathPlanner_Stop(void);
void PathPlanner_Update(void);
path_state_t PathPlanner_GetState(void);

void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm);
uint16_t PathPlanner_GetCellSizeMm(void);
bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells);
bool PathPlanner_SetMapRows(uint8_t width, uint8_t height, const char *rows);
bool PathPlanner_IsMapCellValid(char cell);
bool PathPlanner_IsMapCellPassableChar(char cell);
bool PathPlanner_SetPose(uint8_t x, uint8_t y, path_dir_t heading);

/* 调试命令也使用同一个 planner 的连续位姿寻路，不再保留旧 BFS。 */
bool PathPlanner_MoveCells(path_dir_t dir, uint8_t cells);
bool PathPlanner_Goto(uint8_t targetX, uint8_t targetY);
void PathPlanner_GetStatus(path_status_t *status);

bool PathPlanner_MissionStart(void);
void PathPlanner_MissionStop(void);
void PathPlanner_MissionReset(void);
void PathPlanner_MissionOnMapReceived(uint16_t sequence, uint8_t level, bool labeledMode);
bool PathPlanner_SetStartArea(bool inStartArea);
bool PathPlanner_MissionOnRecognitionResult(uint16_t sequence, uint8_t code);
void PathPlanner_GetMissionStatus(mission_status_t *status);
bool PathPlanner_GetRecognitionPoint(uint16_t index, recognition_point_t *point);

#endif
