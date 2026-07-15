#ifndef SOKOBAN_PLANNER_H
#define SOKOBAN_PLANNER_H

#include <stdint.h>

#ifdef _WIN32
#define PLANNER_EXPORT __declspec(dllexport)
#else
#define PLANNER_EXPORT
#endif

#define PLANNER_MAX_WIDTH 16
#define PLANNER_MAX_HEIGHT 12
#define PLANNER_MAX_CELLS 192
#define PLANNER_BITSET_BYTES 24
#define PLANNER_OBJECT_CAPACITY PLANNER_MAX_CELLS
#define PLANNER_MAX_PLAN_STEPS 256

#define PLANNER_DIR_UP 0
#define PLANNER_DIR_RIGHT 1
#define PLANNER_DIR_DOWN 2
#define PLANNER_DIR_LEFT 3

#define PLANNER_STEP_MOVE 1
#define PLANNER_STEP_PUSH 2
#define PLANNER_STEP_SCAN 3

#define PLANNER_JOB_IDLE 0
#define PLANNER_JOB_RUNNING 1
#define PLANNER_JOB_DONE 2
#define PLANNER_JOB_FAILED 3

#define PLANNER_OBJECT_BOX 1
#define PLANNER_OBJECT_DESTINATION 2

#define PLANNER_RECOGNITION_INVALID 0
#define PLANNER_RECOGNITION_ACCEPTED 1
#define PLANNER_RECOGNITION_RETRY 2

/* car 视觉串口定义的“识别失败，请重新识别”结果。 */
#define PLANNER_RECOGNITION_CODE_RETRY 20
#define PLANNER_MAX_LABEL 99

/*
 * 统一运行时物体记录。
 *
 * id 是地图加载时按 y、x 扫描产生的稳定索引，也就是 car 识别串口
 * 当前使用的 pointIndex。label 在状态中统一保存为箱子/目标的匹配值；
 * car 的原始目标码 10～19 由 planner_state_apply_recognition() 归一化。
 */
typedef struct {
    int16_t id;
    int16_t x;
    int16_t y;
    int16_t label;
    uint8_t kind;
    uint8_t known;
} PlannerObject;

typedef struct {
    int16_t box_id;
    int16_t destination_id;
} PlannerForbiddenMatch;

/*
 * planner 的直接输入状态。
 * 所有存储由调用方提供，适合 RT1064 的静态内存；对象数量是运行时
 * object_count，不按“最多几个箱子”写死，192 只是 16x12 地图的物理
 * 单元上限。
 */
typedef struct {
    int width;
    int height;
    uint8_t wall_bits[PLANNER_BITSET_BYTES];
    uint8_t bomb_bits[PLANNER_BITSET_BYTES];
    uint8_t seen_bits[PLANNER_BITSET_BYTES];
    int car_x;
    int car_y;
    int car_dir;
    int16_t car_yaw_cdeg;
    uint8_t pose_valid;
    int object_count;
    PlannerObject objects[PLANNER_OBJECT_CAPACITY];
    int forbidden_match_count;
    PlannerForbiddenMatch forbidden_matches[PLANNER_OBJECT_CAPACITY];
} PlannerState;

typedef struct {
    int id;
    int x;
    int y;
    int class_id;
    int known;
} PlannerMovableBox;

typedef struct {
    int id;
    int x;
    int y;
    int number;
    int known;
} PlannerDestinationBox;

typedef struct {
    int width;
    int height;
    uint8_t wall_bits[PLANNER_BITSET_BYTES];
    uint8_t bomb_bits[PLANNER_BITSET_BYTES];
    int car_x;
    int car_y;
    int car_dir;
    int movable_count;
    const PlannerMovableBox *movable_boxes;
    int destination_count;
    const PlannerDestinationBox *destination_boxes;
    /* Row is the movable-box index, column is the destination index. */
    const uint8_t *forbidden_match_bits;
} PlannerInput;

typedef struct {
    int type;
    int x;
    int y;
    int dir;
    int object_id;
} PlannerStep;

typedef struct {
    int success;
    int score;
    int step_count;
    int error_code;
    char message[128];
    PlannerStep steps[PLANNER_MAX_PLAN_STEPS];
} PlannerPlan;

/* 直接从地图、位姿和识别结果状态生成计划。 */
PLANNER_EXPORT int planner_plan(const PlannerState *state, PlannerPlan *plan);

/* Cooperative API: each step expands at most expansion_budget push-search nodes. */
PLANNER_EXPORT int planner_job_begin(const PlannerState *state);
PLANNER_EXPORT int planner_job_step(uint32_t expansion_budget, PlannerPlan *plan);

/* 与 car 当前双串口语义一致的状态更新入口。 */
PLANNER_EXPORT void planner_state_clear(PlannerState *state);
PLANNER_EXPORT int planner_state_set_map_rows(PlannerState *state, int width, int height, const char *rows);
PLANNER_EXPORT int planner_state_set_pose(
    PlannerState *state,
    int x,
    int y,
    int dir,
    int yaw_cdeg
);
PLANNER_EXPORT int planner_state_set_yaw(PlannerState *state, int yaw_cdeg);
PLANNER_EXPORT int planner_state_apply_recognition(PlannerState *state, int object_id, int code);
PLANNER_EXPORT int planner_state_update_object_position(PlannerState *state, int object_id, int x, int y);

/*
 * 在真实执行器确认一个 MOVE/PUSH 步骤完成后更新同一份状态。
 * 函数会同步小车位姿、箱子/炸弹位置以及炸弹破墙结果；返回 0 表示
 * 当前状态与步骤不一致，执行层必须停止采用这份计划并重新规划。
 */
PLANNER_EXPORT int planner_state_apply_executed_step(PlannerState *state, const PlannerStep *step);

/* 删除已经位于同编号目标点上的箱子/目标点对，返回删除的配对数量。 */
PLANNER_EXPORT int planner_state_remove_completed_pairs(PlannerState *state);

/* 当状态中已经没有箱子和目标点时返回 1。 */
PLANNER_EXPORT int planner_state_is_complete(const PlannerState *state);

/* 使用与主规划器相同的连续直线路径模型生成一个调试用目标位姿计划。 */
PLANNER_EXPORT int planner_plan_to_pose(
    const PlannerState *state,
    int target_x,
    int target_y,
    int target_dir,
    PlannerPlan *plan
);

#endif
