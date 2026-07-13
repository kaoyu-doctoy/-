#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "../app/app_config.h"

typedef enum
{
    /* 路径规划空闲，未执行自动路线。 */
    PATH_STATE_IDLE = 0,
    /* 路径规划正在执行格子路线。 */
    PATH_STATE_RUNNING,
    /* 路线已经执行完成。 */
    PATH_STATE_FINISHED,
    /* 地图、起点或目标点无效，路线无法执行。 */
    PATH_STATE_ERROR
} path_state_t;

typedef enum
{
    /* 地图 X 正方向，对应底盘 vx 正方向。 */
    PATH_DIR_X_POS = 0,
    /* 地图 Y 正方向，对应底盘 vy 正方向。 */
    PATH_DIR_Y_POS = 1,
    /* 地图 X 负方向，对应底盘 vx 负方向。 */
    PATH_DIR_X_NEG = 2,
    /* 地图 Y 负方向，对应底盘 vy 负方向。 */
    PATH_DIR_Y_NEG = 3
} path_dir_t;

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

/* 初始化路径规划模块状态。 */
void PathPlanner_Init(void);

/* 启动当前已经生成的路径队列。 */
void PathPlanner_Start(void);

/* 停止路径规划并立即停止底盘。 */
void PathPlanner_Stop(void);

/* 周期更新路径状态机，执行当前格子移动。 */
void PathPlanner_Update(void);

/* 获取当前路径规划状态。 */
path_state_t PathPlanner_GetState(void);

/* 设置地图格子尺寸，单位 mm。 */
void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm);

/* 获取当前地图格子尺寸，单位 mm。 */
uint16_t PathPlanner_GetCellSizeMm(void);

/* 从扁平格子数组加载地图，长度至少为 width * height。 */
bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells);

/* 从以 / 或 | 分隔的多行文本加载地图，例如 #####/#S..#/#..T#/#####。 */
bool PathPlanner_SetMapRows(uint8_t width, uint8_t height, const char *rows);

/* 设置小车在地图中的当前位置和朝向。 */
bool PathPlanner_SetPose(uint8_t x, uint8_t y, path_dir_t heading);

/* 直接执行一个方向上的格子移动，用于单独调试编码器闭环。 */
bool PathPlanner_MoveCells(path_dir_t dir, uint8_t cells);

/* 根据当前地图和当前位置，用 BFS 规划到指定目标格。 */
bool PathPlanner_Goto(uint8_t targetX, uint8_t targetY);

/* 复制路径规划当前状态。 */
void PathPlanner_GetStatus(path_status_t *status);

#endif
