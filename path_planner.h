#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "../app/app_config.h"

/*
 * 路径规划模块对外状态。
 * 串口 @PATH/@GO/@STEP 返回的 state 字段使用这些枚举值。
 */
typedef enum
{
    /* 空闲：当前没有正在执行的自动路径。 */
    PATH_STATE_IDLE = 0,
    /* 运行中：PathPlanner_Update() 正在按路径段驱动车辆。 */
    PATH_STATE_RUNNING,
    /* 完成：当前路径已经全部执行结束。 */
    PATH_STATE_FINISHED,
    /* 错误：地图、起点、目标点或路径缓存无效，无法继续执行。 */
    PATH_STATE_ERROR
} path_state_t;

/*
 * 地图坐标方向。
 * 这里的 X/Y 是地图格子坐标方向，最终会被转换成底盘 vx/vy。
 */
typedef enum
{
    /* X 正方向，对应底盘 vx 正方向。 */
    PATH_DIR_X_POS = 0,
    /* Y 正方向，对应底盘 vy 正方向。 */
    PATH_DIR_Y_POS = 1,
    /* X 负方向，对应底盘 vx 负方向。 */
    PATH_DIR_X_NEG = 2,
    /* Y 负方向，对应底盘 vy 负方向。 */
    PATH_DIR_Y_NEG = 3
} path_dir_t;

/* 视觉端和路径规划共同使用的地图字符定义。 */
#define PATH_MAP_CELL_WALL   ('#')  /* 墙壁：不可通行。 */
#define PATH_MAP_CELL_EMPTY  ('-')  /* 空地：普通寻路可通行。 */
#define PATH_MAP_CELL_GOAL   ('.')  /* 目的地：普通寻路可通行。 */
#define PATH_MAP_CELL_BOMB   ('*')  /* 炸弹：普通寻路不可通行。 */
#define PATH_MAP_CELL_BOX    ('$')  /* 箱子：普通寻路不可通行，推箱子算法需要单独处理。 */

/*
 * 路径规划状态快照。
 * Protocol_PrintPathStatus() 会把这些字段按顺序打包成 path:... 返回。
 */
typedef struct
{
    path_state_t state;       /* 当前状态：空闲、运行、完成或错误。 */
    bool mapReady;            /* 是否已经成功加载地图。 */
    bool poseReady;           /* 是否已经设置小车当前位置和朝向。 */
    bool segmentActive;       /* 是否正在执行某一段格子移动。 */
    uint8_t x;                /* 当前认为的小车地图 X 坐标，单位为格。 */
    uint8_t y;                /* 当前认为的小车地图 Y 坐标，单位为格。 */
    path_dir_t heading;       /* 当前记录的小车朝向。 */
    uint16_t stepIndex;       /* 当前执行到第几个路径段。 */
    uint16_t stepCount;       /* 当前路径一共有多少段。 */
    uint16_t cellSizeMm;      /* 单个地图格子的边长，单位 mm。 */
    int32_t travelledCounts;  /* 当前段已经走过的编码器计数。 */
    int32_t targetCounts;     /* 当前段目标编码器计数。 */
} path_status_t;

/* 初始化路径规划模块状态。main 中启动任务前调用一次。 */
void PathPlanner_Init(void);

/* 启动当前已经生成好的路径队列。通常由 PathPlanner_Goto() 内部调用。 */
void PathPlanner_Start(void);

/* 停止路径执行、清空路径段，并立即让底盘停止。 */
void PathPlanner_Stop(void);

/*
 * 路径规划周期更新函数。
 * 在 PathTask 中每 APP_PATH_PERIOD_MS 调用一次，用于执行当前路径段。
 */
void PathPlanner_Update(void);

/* 获取当前路径规划状态枚举。 */
path_state_t PathPlanner_GetState(void);

/* 设置地图格子尺寸，单位 mm；影响“走几格”到编码器计数的换算。 */
void PathPlanner_SetCellSizeMm(uint16_t cellSizeMm);

/* 获取当前地图格子尺寸，单位 mm。 */
uint16_t PathPlanner_GetCellSizeMm(void);

/* 从一段连续的 width * height 字符数组加载地图。 */
bool PathPlanner_SetMap(uint8_t width, uint8_t height, const char *cells);

/*
 * 从多行文本加载地图，行之间用 / 或 | 分隔。
 * 例：#####/#---#/#-.-#/#####
 */
bool PathPlanner_SetMapRows(uint8_t width, uint8_t height, const char *rows);

/* 判断地图字符是否合法，目前只允许 #、-、.、*、$。 */
bool PathPlanner_IsMapCellValid(char cell);

/* 判断某个字符在普通车辆寻路中是否可通行，目前 - 和 . 可通行。 */
bool PathPlanner_IsMapCellPassableChar(char cell);

/* 设置小车当前地图坐标和朝向；这是 @GO 寻路的起点。 */
bool PathPlanner_SetPose(uint8_t x, uint8_t y, path_dir_t heading);

/*
 * 直接按指定方向移动若干格，不经过地图 BFS。
 * 主要用于测试“按格子距离行走”和编码器距离控制。
 */
bool PathPlanner_MoveCells(path_dir_t dir, uint8_t cells);

/*
 * 根据当前地图和当前位姿，用 BFS 规划到目标格。
 * 规划成功后会自动启动路径执行。
 */
bool PathPlanner_Goto(uint8_t targetX, uint8_t targetY);

/* 复制当前路径规划状态，供串口状态返回使用。 */
void PathPlanner_GetStatus(path_status_t *status);

#endif
