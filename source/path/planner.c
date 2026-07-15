#define PLANNER_RT1064 1
#include "planner.h"

#include <limits.h>
#include <string.h>

#define MOVE_COST 10
#define TURN_COST 15
#define PUSH_COST MOVE_COST
#define BOMB_PUSH_COST MOVE_COST
#define SCAN_COST 5

#define PUSH_NODE_CAPACITY 25088
#define PUSH_NODE_TABLE_CAPACITY 28657
#define PUSH_ENV_CAPACITY 1024
#define PUSH_ENV_TABLE_CAPACITY 2053
#define PUSH_INVALID_INDEX 65535u
#define TRANSITION_NONE 0
#define TRANSITION_BOX_PUSH 1
#define TRANSITION_BOMB_PUSH 2

typedef struct {
    int success;
    int cost;
    int step_count;
    PlannerStep steps[PLANNER_MAX_PLAN_STEPS];
} MovePlan;

typedef struct {
    uint16_t g;
    uint16_t disruption;
    uint16_t parent;
    uint16_t hash_next;
    uint16_t env_id;
    uint16_t heap_index;
    uint8_t car_cell;
    uint8_t box_cell;
    uint8_t transition_cell;
    uint8_t car_dir : 2;
    uint8_t transition_kind : 2;
    uint8_t closed : 1;
    uint8_t reserved : 3;
} PushNode;

typedef struct {
    uint8_t walls[PLANNER_BITSET_BYTES];
    uint8_t bombs[PLANNER_BITSET_BYTES];
    uint16_t hash_next;
} PushEnv;

typedef struct {
    PushNode nodes[PUSH_NODE_CAPACITY];
    uint16_t heap[PUSH_NODE_CAPACITY];
    uint16_t node_table[PUSH_NODE_TABLE_CAPACITY];
    PushEnv envs[PUSH_ENV_CAPACITY];
    uint16_t env_table[PUSH_ENV_TABLE_CAPACITY];
    uint16_t push_dist[PLANNER_MAX_CELLS];
    uint16_t best_goal;
    int box_index;
    int target_x;
    int target_y;
    uint8_t active;
    uint16_t node_count;
    uint16_t heap_count;
    uint16_t env_count;
} PushSearch;

typedef struct {
    int parent[PLANNER_MAX_CELLS];
    int g_cost[PLANNER_MAX_CELLS];
    int visited[PLANNER_MAX_CELLS];
    int free_cells[PLANNER_MAX_CELLS];
    int path_cells[PLANNER_MAX_CELLS];
} FreeMotionScratch;

typedef struct {
    int match_destination[PLANNER_OBJECT_CAPACITY];
    int match_box[PLANNER_OBJECT_CAPACITY];
    int previous_destination[PLANNER_OBJECT_CAPACITY];
    int queue[PLANNER_OBJECT_CAPACITY];
    uint8_t seen_destination[PLANNER_OBJECT_CAPACITY];
    uint8_t seen_box[PLANNER_OBJECT_CAPACITY];
    uint8_t unresolved_destination_used[PLANNER_OBJECT_CAPACITY];
} MatchingScratch;

typedef struct {
    PushSearch push_search;
    FreeMotionScratch free_motion;
    MatchingScratch matching;
    MovePlan move_plans[3];
    int cell_queue[PLANNER_MAX_CELLS];
    int push_chain[PLANNER_MAX_PLAN_STEPS];
} PlannerWorkspace;

#if defined(PLANNER_RT1064)
#define PLANNER_LARGE_STORAGE __attribute__((used, section("PathPlannerRam"), aligned(32)))
#else
#define PLANNER_LARGE_STORAGE
#endif

PLANNER_LARGE_STORAGE static PlannerWorkspace g_workspace;

typedef struct {
    int success;
    int cost;
    int step_count;
    PlannerStep steps[PLANNER_MAX_PLAN_STEPS];
    int end_car_x;
    int end_car_y;
    int end_car_dir;
} PushPlan;

typedef struct {
    int quick_score;
    int box_index;
    int destination_index;
} TaskCandidate;

typedef struct {
    const PlannerInput *input;
    uint8_t walls[PLANNER_MAX_CELLS];
    uint8_t bombs[PLANNER_MAX_CELLS];
    uint8_t dead_squares[PLANNER_MAX_CELLS];
    uint8_t destinations[PLANNER_MAX_CELLS];
} PlannerContext;

typedef struct {
    int valid;
    int unresolved_count;
    int unresolved_box_indices[PLANNER_OBJECT_CAPACITY];
    int unresolved_destination_indices[PLANNER_OBJECT_CAPACITY];
    int fixed_destination_for_box[PLANNER_OBJECT_CAPACITY];
    uint8_t allowed_destinations[PLANNER_OBJECT_CAPACITY][PLANNER_BITSET_BYTES];
} MatchingInfo;

typedef struct {
    PlannerContext context;
    PlannerInput input;
    PlannerMovableBox movable_storage[PLANNER_OBJECT_CAPACITY];
    PlannerDestinationBox destination_storage[PLANNER_OBJECT_CAPACITY];
    uint8_t forbidden_storage[PLANNER_OBJECT_CAPACITY * PLANNER_BITSET_BYTES];
    TaskCandidate candidates[PLANNER_OBJECT_CAPACITY];
    int candidate_count;
    int candidate_index;
    int best_found;
    int best_score;
    int status;
    PlannerPlan best_plan;
    PlannerPlan result;
    PushPlan push_plan;
} PlannerJob;

/* Matching is rebuilt per observation and must not consume the RT task stack. */
PLANNER_LARGE_STORAGE static MatchingInfo g_matching_info;
PLANNER_LARGE_STORAGE static PlannerJob g_job;

static const int DIR_DX[4] = {0, 1, 0, -1};
static const int DIR_DY[4] = {-1, 0, 1, 0};

static void plan_clear(PlannerPlan *plan) {
    memset(plan, 0, sizeof(*plan));
}

static void plan_set_message(PlannerPlan *plan, const char *message) {
    size_t index = 0U;
    if (message == NULL) {
        plan->message[0] = '\0';
        return;
    }
    while (message[index] != '\0' && index + 1U < sizeof(plan->message)) {
        plan->message[index] = message[index];
        index += 1U;
    }
    plan->message[index] = '\0';
}

static int min_int(int a, int b) {
    return a < b ? a : b;
}

static int abs_int(int value) {
    return value < 0 ? -value : value;
}

static int ceil_sqrt_int64(long long value) {
    long long low = 0;
    long long high = 1;
    while (high * high < value) {
        high <<= 1;
    }
    while (low < high) {
        long long mid = low + (high - low) / 2;
        if (mid * mid < value) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return (int)low;
}

static int manhattan(int x1, int y1, int x2, int y2) {
    return abs_int(x1 - x2) + abs_int(y1 - y2);
}

static int scaled_distance_cost(int dx_cells, int dy_cells, int per_cell_cost) {
    long long squared = (long long)dx_cells * (long long)dx_cells +
                        (long long)dy_cells * (long long)dy_cells;
    if (squared == 0) {
        return 0;
    }
    squared *= (long long)per_cell_cost * (long long)per_cell_cost;
    return ceil_sqrt_int64(squared);
}

static int cell_index(const PlannerContext *ctx, int x, int y) {
    return y * ctx->input->width + x;
}

static int bitset_get(const uint8_t bits[PLANNER_BITSET_BYTES], int index) {
    return (bits[index >> 3] >> (index & 7)) & 1;
}

static void bitset_copy(uint8_t target[PLANNER_BITSET_BYTES], const uint8_t source[PLANNER_BITSET_BYTES]) {
    memcpy(target, source, PLANNER_BITSET_BYTES);
}

static void bitset_set(uint8_t bits[PLANNER_BITSET_BYTES], int index) {
    bits[index >> 3] |= (uint8_t)(1u << (index & 7));
}

static void bitset_clear(uint8_t bits[PLANNER_BITSET_BYTES], int index) {
    bits[index >> 3] &= (uint8_t)~(1u << (index & 7));
}

static int planner_state_is_inside(const PlannerState *state, int x, int y) {
    return state != NULL && x >= 0 && y >= 0 && x < state->width && y < state->height;
}

static int planner_state_cell_index(const PlannerState *state, int x, int y) {
    return y * state->width + x;
}

static void planner_state_normalize_yaw(int16_t *yaw_cdeg) {
    int32_t value = *yaw_cdeg;
    while (value > 18000) {
        value -= 36000;
    }
    while (value < -18000) {
        value += 36000;
    }
    *yaw_cdeg = (int16_t)value;
}

void planner_state_clear(PlannerState *state) {
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->car_dir = PLANNER_DIR_UP;
    state->car_yaw_cdeg = -9000;
    for (int index = 0; index < PLANNER_OBJECT_CAPACITY; ++index) {
        state->objects[index].id = -1;
        state->objects[index].label = -1;
    }
}

int planner_state_set_map_rows(PlannerState *state, int width, int height, const char *rows) {
    int x = 0;
    int y = 0;
    int index;
    int previous_car_x;
    int previous_car_y;
    int previous_car_dir;
    int16_t previous_yaw_cdeg;
    uint8_t previous_pose_valid;
    int previous_pose_occupied = 0;

    if (state == NULL || rows == NULL || width <= 0 || width > PLANNER_MAX_WIDTH ||
        height <= 0 || height > PLANNER_MAX_HEIGHT) {
        return 0;
    }

    previous_car_x = state->car_x;
    previous_car_y = state->car_y;
    previous_car_dir = state->car_dir;
    previous_yaw_cdeg = state->car_yaw_cdeg;
    previous_pose_valid = state->pose_valid;
    planner_state_clear(state);
    state->width = width;
    state->height = height;
    for (index = 0; rows[index] != '\0'; ++index) {
        char cell = rows[index];
        PlannerObject *object;
        int cell_index_value;

        if (cell == '/' || cell == '|' || cell == '\n' || cell == '\r') {
            continue;
        }
        if (x >= width || y >= height) {
            planner_state_clear(state);
            return 0;
        }
        cell_index_value = planner_state_cell_index(state, x, y);
        if (cell == '#') {
            bitset_set(state->wall_bits, cell_index_value);
        } else if (cell == '*') {
            bitset_set(state->bomb_bits, cell_index_value);
        } else if (cell == '$' || cell == '.') {
            if (state->object_count >= PLANNER_OBJECT_CAPACITY) {
                planner_state_clear(state);
                return 0;
            }
            object = &state->objects[state->object_count];
            object->id = (int16_t)state->object_count;
            object->x = (int16_t)x;
            object->y = (int16_t)y;
            object->label = -1;
            object->kind = cell == '$' ? PLANNER_OBJECT_BOX : PLANNER_OBJECT_DESTINATION;
            object->known = 0U;
            state->object_count += 1;
        } else if (cell != '-') {
            planner_state_clear(state);
            return 0;
        }

        x += 1;
        if (x == width) {
            x = 0;
            y += 1;
        }
    }

    if (y != height || x != 0) {
        planner_state_clear(state);
        return 0;
    }
    for (index = 0; index < state->object_count; ++index) {
        const PlannerObject *object = &state->objects[index];
        if (object->kind == PLANNER_OBJECT_BOX &&
            object->x == previous_car_x && object->y == previous_car_y) {
            previous_pose_occupied = 1;
            break;
        }
    }
    if (previous_pose_valid && !previous_pose_occupied &&
        planner_state_is_inside(state, previous_car_x, previous_car_y) &&
        !bitset_get(state->wall_bits, planner_state_cell_index(state, previous_car_x, previous_car_y)) &&
        !bitset_get(state->bomb_bits, planner_state_cell_index(state, previous_car_x, previous_car_y))) {
        state->car_x = previous_car_x;
        state->car_y = previous_car_y;
        state->car_dir = previous_car_dir;
        state->car_yaw_cdeg = previous_yaw_cdeg;
        state->pose_valid = 1U;
    }
    return 1;
}

int planner_state_set_pose(PlannerState *state, int x, int y, int dir, int yaw_cdeg) {
    int cell;
    int object_index;
    if (state == NULL || dir < PLANNER_DIR_UP || dir > PLANNER_DIR_LEFT ||
        x < 0 || y < 0 || x >= PLANNER_MAX_WIDTH || y >= PLANNER_MAX_HEIGHT) {
        return 0;
    }
    if (state->width > 0 && state->height > 0) {
        if (!planner_state_is_inside(state, x, y)) {
            return 0;
        }
        cell = planner_state_cell_index(state, x, y);
        if (bitset_get(state->wall_bits, cell) || bitset_get(state->bomb_bits, cell)) {
            return 0;
        }
        for (object_index = 0; object_index < state->object_count; ++object_index) {
            const PlannerObject *object = &state->objects[object_index];
            if (object->kind == PLANNER_OBJECT_BOX && object->x == x && object->y == y) {
                return 0;
            }
        }
    }
    state->car_x = x;
    state->car_y = y;
    state->car_dir = dir;
    state->car_yaw_cdeg = (int16_t)yaw_cdeg;
    planner_state_normalize_yaw(&state->car_yaw_cdeg);
    state->pose_valid = 1U;
    return 1;
}

int planner_state_set_yaw(PlannerState *state, int yaw_cdeg) {
    if (state == NULL) {
        return 0;
    }
    state->car_yaw_cdeg = (int16_t)yaw_cdeg;
    planner_state_normalize_yaw(&state->car_yaw_cdeg);
    return 1;
}

int planner_state_apply_recognition(PlannerState *state, int object_id, int code) {
    int index;
    if (state == NULL || code < 0 || code > PLANNER_RECOGNITION_CODE_RETRY) {
        return PLANNER_RECOGNITION_INVALID;
    }
    for (index = 0; index < state->object_count; ++index) {
        PlannerObject *object = &state->objects[index];
        if (object->id != object_id) {
            continue;
        }
        if (code == PLANNER_RECOGNITION_CODE_RETRY) {
            return PLANNER_RECOGNITION_RETRY;
        }
        if (object->kind == PLANNER_OBJECT_BOX && code <= 9) {
            object->label = (int16_t)code;
        } else if (object->kind == PLANNER_OBJECT_DESTINATION && code >= 10 && code <= 19) {
            object->label = (int16_t)(code - 10);
        } else {
            return PLANNER_RECOGNITION_INVALID;
        }
        object->known = 1U;
        if (planner_state_is_inside(state, object->x, object->y)) {
            bitset_set(state->seen_bits, planner_state_cell_index(state, object->x, object->y));
        }
        return PLANNER_RECOGNITION_ACCEPTED;
    }
    return PLANNER_RECOGNITION_INVALID;
}

int planner_state_update_object_position(PlannerState *state, int object_id, int x, int y) {
    int index;
    int cell;
    if (state == NULL || !planner_state_is_inside(state, x, y)) {
        return 0;
    }
    cell = planner_state_cell_index(state, x, y);
    if (bitset_get(state->wall_bits, cell) || bitset_get(state->bomb_bits, cell)) {
        return 0;
    }
    for (index = 0; index < state->object_count; ++index) {
        PlannerObject *object = &state->objects[index];
        if (object->id != object_id || object->kind != PLANNER_OBJECT_BOX) {
            continue;
        }
        object->x = (int16_t)x;
        object->y = (int16_t)y;
        return 1;
    }
    return 0;
}

static PlannerObject *planner_state_find_box_at(PlannerState *state, int x, int y) {
    int index;
    for (index = 0; index < state->object_count; ++index) {
        PlannerObject *object = &state->objects[index];
        if (object->kind == PLANNER_OBJECT_BOX && object->x == x && object->y == y) {
            return object;
        }
    }
    return NULL;
}

static int planner_state_is_border(const PlannerState *state, int x, int y) {
    return x == 0 || y == 0 || x == state->width - 1 || y == state->height - 1;
}

static void planner_state_build_blocked(const PlannerState *state, uint8_t blocked[PLANNER_BITSET_BYTES]) {
    int index;
    bitset_copy(blocked, state->wall_bits);
    for (index = 0; index < PLANNER_BITSET_BYTES; ++index) {
        blocked[index] |= state->bomb_bits[index];
    }
    for (index = 0; index < state->object_count; ++index) {
        const PlannerObject *object = &state->objects[index];
        if (object->kind == PLANNER_OBJECT_BOX && planner_state_is_inside(state, object->x, object->y)) {
            bitset_set(blocked, planner_state_cell_index(state, object->x, object->y));
        }
    }
}

static int planner_state_segment_is_clear(
    const PlannerState *state,
    int start_x,
    int start_y,
    int target_x,
    int target_y,
    const uint8_t blocked[PLANNER_BITSET_BYTES]
) {
    int dx = abs_int(target_x - start_x);
    int dy = abs_int(target_y - start_y);
    int step_x = target_x > start_x ? 1 : (target_x < start_x ? -1 : 0);
    int step_y = target_y > start_y ? 1 : (target_y < start_y ? -1 : 0);
    int current_x = start_x;
    int current_y = start_y;
    int t_delta_x = step_x == 0 ? INT_MAX : 2 * dy;
    int t_delta_y = step_y == 0 ? INT_MAX : 2 * dx;
    int t_max_x = step_x == 0 ? INT_MAX : dy;
    int t_max_y = step_y == 0 ? INT_MAX : dx;

    while (current_x != target_x || current_y != target_y) {
        if (t_max_x < t_max_y) {
            current_x += step_x;
            t_max_x += t_delta_x;
        } else if (t_max_y < t_max_x) {
            current_y += step_y;
            t_max_y += t_delta_y;
        } else {
            int side_x = current_x + step_x;
            int side_y = current_y + step_y;
            if ((step_x != 0 && (!planner_state_is_inside(state, side_x, current_y) ||
                 bitset_get(blocked, planner_state_cell_index(state, side_x, current_y)))) ||
                (step_y != 0 && (!planner_state_is_inside(state, current_x, side_y) ||
                 bitset_get(blocked, planner_state_cell_index(state, current_x, side_y))))) {
                return 0;
            }
            current_x = side_x;
            current_y = side_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
        }
        if (!planner_state_is_inside(state, current_x, current_y) ||
            bitset_get(blocked, planner_state_cell_index(state, current_x, current_y))) {
            return 0;
        }
    }
    return 1;
}

static void planner_state_explode_walls(PlannerState *state, int center_x, int center_y) {
    int x;
    int y;
    for (y = center_y - 1; y <= center_y + 1; ++y) {
        for (x = center_x - 1; x <= center_x + 1; ++x) {
            if (!planner_state_is_inside(state, x, y) || planner_state_is_border(state, x, y)) {
                continue;
            }
            bitset_clear(state->wall_bits, planner_state_cell_index(state, x, y));
        }
    }
}

static int planner_direction_yaw_cdeg(int dir) {
    static const int yaw_by_dir[4] = {-9000, 0, 9000, 18000};
    return yaw_by_dir[dir];
}

int planner_state_apply_executed_step(PlannerState *state, const PlannerStep *step) {
    int dx;
    int dy;
    int target_cell;
    uint8_t blocked[PLANNER_BITSET_BYTES];
    PlannerObject *box;

    if (state == NULL || step == NULL || !state->pose_valid ||
        (step->type != PLANNER_STEP_MOVE && step->type != PLANNER_STEP_PUSH) ||
        step->dir < PLANNER_DIR_UP || step->dir > PLANNER_DIR_LEFT ||
        !planner_state_is_inside(state, step->x, step->y)) {
        return 0;
    }

    dx = step->x - state->car_x;
    dy = step->y - state->car_y;
    target_cell = planner_state_cell_index(state, step->x, step->y);
    box = planner_state_find_box_at(state, step->x, step->y);

    if (abs_int(dx) + abs_int(dy) == 1 && box != NULL) {
        int box_target_x = step->x + dx;
        int box_target_y = step->y + dy;
        int box_target_cell;
        if (step->type != PLANNER_STEP_PUSH ||
            !planner_state_is_inside(state, box_target_x, box_target_y) ||
            planner_state_find_box_at(state, box_target_x, box_target_y) != NULL) {
            return 0;
        }
        box_target_cell = planner_state_cell_index(state, box_target_x, box_target_y);
        if (bitset_get(state->wall_bits, box_target_cell) || bitset_get(state->bomb_bits, box_target_cell)) {
            return 0;
        }
        box->x = (int16_t)box_target_x;
        box->y = (int16_t)box_target_y;
        bitset_set(state->seen_bits, box_target_cell);
    } else if (abs_int(dx) + abs_int(dy) == 1 && bitset_get(state->bomb_bits, target_cell)) {
        int bomb_target_x = step->x + dx;
        int bomb_target_y = step->y + dy;
        int bomb_target_cell;
        if (step->type != PLANNER_STEP_PUSH ||
            !planner_state_is_inside(state, bomb_target_x, bomb_target_y) ||
            planner_state_find_box_at(state, bomb_target_x, bomb_target_y) != NULL) {
            return 0;
        }
        bomb_target_cell = planner_state_cell_index(state, bomb_target_x, bomb_target_y);
        if (bitset_get(state->bomb_bits, bomb_target_cell)) {
            return 0;
        }
        bitset_clear(state->bomb_bits, target_cell);
        if (bitset_get(state->wall_bits, bomb_target_cell)) {
            if (planner_state_is_border(state, bomb_target_x, bomb_target_y)) {
                bitset_set(state->bomb_bits, target_cell);
                return 0;
            }
            planner_state_explode_walls(state, bomb_target_x, bomb_target_y);
        } else {
            bitset_set(state->bomb_bits, bomb_target_cell);
            bitset_set(state->seen_bits, bomb_target_cell);
        }
    } else {
        if (step->type != PLANNER_STEP_MOVE) {
            return 0;
        }
        planner_state_build_blocked(state, blocked);
        if (!planner_state_segment_is_clear(
                state,
                state->car_x,
                state->car_y,
                step->x,
                step->y,
                blocked)) {
            return 0;
        }
    }

    state->car_x = step->x;
    state->car_y = step->y;
    state->car_dir = step->dir;
    state->car_yaw_cdeg = (int16_t)planner_direction_yaw_cdeg(step->dir);
    bitset_set(state->seen_bits, target_cell);
    return 1;
}

static void planner_state_reset_object_slot(PlannerObject *object) {
    memset(object, 0, sizeof(*object));
    object->id = -1;
    object->label = -1;
}

int planner_state_remove_completed_pairs(PlannerState *state) {
    int removed = 0;
    int found = 1;
    if (state == NULL) {
        return 0;
    }
    while (found) {
        int box_index;
        found = 0;
        for (box_index = 0; box_index < state->object_count && !found; ++box_index) {
            const PlannerObject *box = &state->objects[box_index];
            int destination_index;
            if (box->kind != PLANNER_OBJECT_BOX || !box->known) {
                continue;
            }
            for (destination_index = 0; destination_index < state->object_count; ++destination_index) {
                const PlannerObject *destination = &state->objects[destination_index];
                if (destination->kind == PLANNER_OBJECT_DESTINATION && destination->known &&
                    destination->label == box->label && destination->x == box->x && destination->y == box->y) {
                    int box_id = box->id;
                    int destination_id = destination->id;
                    int read_index;
                    int write_index = 0;
                    int forbidden_write = 0;
                    for (read_index = 0; read_index < state->object_count; ++read_index) {
                        int id = state->objects[read_index].id;
                        if (id == box_id || id == destination_id) {
                            continue;
                        }
                        if (write_index != read_index) {
                            state->objects[write_index] = state->objects[read_index];
                        }
                        write_index += 1;
                    }
                    while (write_index < state->object_count) {
                        planner_state_reset_object_slot(&state->objects[write_index++]);
                    }
                    state->object_count -= 2;
                    for (read_index = 0; read_index < state->forbidden_match_count; ++read_index) {
                        PlannerForbiddenMatch pair = state->forbidden_matches[read_index];
                        if (pair.box_id == box_id || pair.destination_id == destination_id) {
                            continue;
                        }
                        state->forbidden_matches[forbidden_write++] = pair;
                    }
                    state->forbidden_match_count = forbidden_write;
                    removed += 1;
                    found = 1;
                    break;
                }
            }
        }
    }
    return removed;
}

int planner_state_is_complete(const PlannerState *state) {
    int index;
    int box_count = 0;
    int destination_count = 0;
    if (state == NULL) {
        return 0;
    }
    for (index = 0; index < state->object_count; ++index) {
        if (state->objects[index].kind == PLANNER_OBJECT_BOX) {
            box_count += 1;
        } else if (state->objects[index].kind == PLANNER_OBJECT_DESTINATION) {
            destination_count += 1;
        }
    }
    return box_count == 0 && destination_count == 0;
}

static void int_array_fill(int *values, int count, int fill_value) {
    int index;
    for (index = 0; index < count; ++index) {
        values[index] = fill_value;
    }
}

static void uint16_array_fill(uint16_t *values, int count, uint16_t fill_value) {
    int index;
    for (index = 0; index < count; ++index) {
        values[index] = fill_value;
    }
}

static int is_inside(const PlannerContext *ctx, int x, int y) {
    return x >= 0 && y >= 0 && x < ctx->input->width && y < ctx->input->height;
}

static int is_border_pos(const PlannerContext *ctx, int x, int y) {
    return x == 0 || y == 0 || x == ctx->input->width - 1 || y == ctx->input->height - 1;
}

static int is_static_blocked(const PlannerContext *ctx, int x, int y) {
    if (!is_inside(ctx, x, y)) {
        return 1;
    }
    return ctx->walls[cell_index(ctx, x, y)] || ctx->bombs[cell_index(ctx, x, y)];
}

static int turn_distance(int from_dir, int to_dir) {
    int clockwise = (to_dir - from_dir + 4) % 4;
    int counter_clockwise = (from_dir - to_dir + 4) % 4;
    return min_int(clockwise, counter_clockwise);
}

static int append_step(PlannerStep steps[PLANNER_MAX_PLAN_STEPS], int *step_count, int type, int x, int y, int dir) {
    if (*step_count >= PLANNER_MAX_PLAN_STEPS) {
        return 0;
    }
    steps[*step_count].type = type;
    steps[*step_count].x = x;
    steps[*step_count].y = y;
    steps[*step_count].dir = dir;
    steps[*step_count].object_id = -1;
    *step_count += 1;
    return 1;
}

static int append_steps(
    PlannerStep target[PLANNER_MAX_PLAN_STEPS],
    int *target_count,
    const PlannerStep source[PLANNER_MAX_PLAN_STEPS],
    int source_count
) {
    int index;
    for (index = 0; index < source_count; ++index) {
        if (!append_step(target, target_count, source[index].type, source[index].x, source[index].y, source[index].dir)) {
            return 0;
        }
        target[*target_count - 1].object_id = source[index].object_id;
    }
    return 1;
}

static int planner_input_from_state(
    PlannerInput *input,
    PlannerMovableBox movable_storage[PLANNER_OBJECT_CAPACITY],
    PlannerDestinationBox destination_storage[PLANNER_OBJECT_CAPACITY],
    uint8_t forbidden_storage[PLANNER_OBJECT_CAPACITY * PLANNER_BITSET_BYTES],
    const PlannerState *state,
    PlannerPlan *plan
) {
    int object_index;
    int movable_count = 0;
    int destination_count = 0;

    if (input == NULL || state == NULL || plan == NULL) {
        return 0;
    }
    if (state->width <= 0 || state->width > PLANNER_MAX_WIDTH ||
        state->height <= 0 || state->height > PLANNER_MAX_HEIGHT) {
        plan->error_code = 1;
        plan_set_message(plan, "地图尺寸非法");
        return 0;
    }
    if (state->object_count < 0 || state->object_count > PLANNER_OBJECT_CAPACITY ||
        state->object_count > state->width * state->height ||
        state->forbidden_match_count < 0 ||
        state->forbidden_match_count > PLANNER_OBJECT_CAPACITY) {
        plan->error_code = 2;
        plan_set_message(plan, "地图对象数量非法");
        return 0;
    }
    if (state->pose_valid == 0U || !planner_state_is_inside(state, state->car_x, state->car_y) ||
        state->car_dir < PLANNER_DIR_UP || state->car_dir > PLANNER_DIR_LEFT) {
        plan->error_code = 1;
        plan_set_message(plan, "小车位姿非法");
        return 0;
    }
    if (bitset_get(state->wall_bits, planner_state_cell_index(state, state->car_x, state->car_y)) ||
        bitset_get(state->bomb_bits, planner_state_cell_index(state, state->car_x, state->car_y))) {
        plan->error_code = 1;
        plan_set_message(plan, "小车位于不可通行格");
        return 0;
    }

    memset(input, 0, sizeof(*input));
    memset(movable_storage, 0, sizeof(PlannerMovableBox) * PLANNER_OBJECT_CAPACITY);
    memset(destination_storage, 0, sizeof(PlannerDestinationBox) * PLANNER_OBJECT_CAPACITY);
    memset(forbidden_storage, 0, PLANNER_OBJECT_CAPACITY * PLANNER_BITSET_BYTES);
    input->width = state->width;
    input->height = state->height;
    memcpy(input->wall_bits, state->wall_bits, PLANNER_BITSET_BYTES);
    memcpy(input->bomb_bits, state->bomb_bits, PLANNER_BITSET_BYTES);
    input->car_x = state->car_x;
    input->car_y = state->car_y;
    input->car_dir = state->car_dir;
    input->movable_boxes = movable_storage;
    input->destination_boxes = destination_storage;
    input->forbidden_match_bits = forbidden_storage;

    for (object_index = 0; object_index < state->object_count; ++object_index) {
        const PlannerObject *object = &state->objects[object_index];
        int label = object->label;
        int known = object->known != 0U && label >= 0;

        if (!planner_state_is_inside(state, object->x, object->y)) {
            plan->error_code = 2;
            plan_set_message(plan, "地图对象坐标非法");
            return 0;
        }
        if (!known) {
            label = -1;
        } else if (object->kind == PLANNER_OBJECT_BOX) {
            if (label > PLANNER_MAX_LABEL) {
                plan->error_code = 2;
                plan_set_message(plan, "箱子识别码非法");
                return 0;
            }
        } else if (object->kind == PLANNER_OBJECT_DESTINATION) {
            if (label > PLANNER_MAX_LABEL) {
                plan->error_code = 2;
                plan_set_message(plan, "目标点识别码非法");
                return 0;
            }
        } else {
            plan->error_code = 2;
            plan_set_message(plan, "地图对象类型非法");
            return 0;
        }

        if (object->kind == PLANNER_OBJECT_BOX) {
            PlannerMovableBox *box = &movable_storage[movable_count++];
            box->id = object->id;
            box->x = object->x;
            box->y = object->y;
            box->class_id = label;
            box->known = known;
        } else {
            PlannerDestinationBox *destination = &destination_storage[destination_count++];
            destination->id = object->id;
            destination->x = object->x;
            destination->y = object->y;
            destination->number = label;
            destination->known = known;
        }
    }

    input->movable_count = movable_count;
    input->destination_count = destination_count;
    for (object_index = 0; object_index < state->forbidden_match_count; ++object_index) {
        int box_index;
        int destination_index;
        int box_id = state->forbidden_matches[object_index].box_id;
        int destination_id = state->forbidden_matches[object_index].destination_id;
        for (box_index = 0; box_index < movable_count; ++box_index) {
            if (movable_storage[box_index].id != box_id) {
                continue;
            }
            for (destination_index = 0; destination_index < destination_count; ++destination_index) {
                if (destination_storage[destination_index].id == destination_id) {
                    bitset_set(
                        forbidden_storage + box_index * PLANNER_BITSET_BYTES,
                        destination_index);
                    break;
                }
            }
            break;
        }
    }
    return 1;
}

static int context_init(PlannerContext *ctx, const PlannerInput *input, PlannerPlan *plan) {
    int x;
    int y;

    if (input == NULL || plan == NULL || input->movable_boxes == NULL ||
        input->destination_boxes == NULL) {
        return 0;
    }
    if (input->width <= 0 || input->width > PLANNER_MAX_WIDTH || input->height <= 0 || input->height > PLANNER_MAX_HEIGHT) {
        plan->error_code = 1;
        plan_set_message(plan, "地图尺寸非法");
        return 0;
    }
    if (input->movable_count < 0 || input->movable_count > PLANNER_OBJECT_CAPACITY ||
        input->destination_count < 0 || input->destination_count > PLANNER_OBJECT_CAPACITY ||
        input->movable_count > input->width * input->height ||
        input->destination_count > input->width * input->height) {
        plan->error_code = 2;
        plan_set_message(plan, "地图对象数量非法");
        return 0;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->input = input;
    for (y = 0; y < input->height; ++y) {
        for (x = 0; x < input->width; ++x) {
            int index = cell_index(ctx, x, y);
            ctx->walls[index] = (uint8_t)bitset_get(input->wall_bits, index);
            ctx->bombs[index] = (uint8_t)bitset_get(input->bomb_bits, index);
        }
    }
    for (x = 0; x < input->destination_count; ++x) {
        if (is_inside(ctx, input->destination_boxes[x].x, input->destination_boxes[x].y)) {
            ctx->destinations[cell_index(ctx, input->destination_boxes[x].x, input->destination_boxes[x].y)] = 1;
        }
    }
    return 1;
}

static int is_forbidden_match(const PlannerInput *input, int box_index, int destination_index) {
    if (input->forbidden_match_bits == NULL || box_index < 0 || destination_index < 0 ||
        box_index >= PLANNER_OBJECT_CAPACITY || destination_index >= PLANNER_OBJECT_CAPACITY) {
        return 0;
    }
    return bitset_get(
        input->forbidden_match_bits + box_index * PLANNER_BITSET_BYTES,
        destination_index);
}

static int matching_augment_iterative(const MatchingInfo *info, int start_box) {
    MatchingScratch *scratch = &g_workspace.matching;
    int head = 0;
    int tail = 0;
    int destination_count = info->unresolved_count;

    memset(scratch->seen_destination, 0, (size_t)destination_count);
    memset(scratch->seen_box, 0, (size_t)destination_count);
    int_array_fill(scratch->previous_destination, destination_count, -1);
    scratch->queue[tail++] = start_box;
    scratch->seen_box[start_box] = 1U;

    while (head < tail) {
        int box_local_index = scratch->queue[head++];
        int destination_local_index;
        for (destination_local_index = 0;
             destination_local_index < destination_count;
             ++destination_local_index) {
            int matched_box;
            if (scratch->seen_destination[destination_local_index] ||
                !bitset_get(info->allowed_destinations[box_local_index], destination_local_index)) {
                continue;
            }
            scratch->seen_destination[destination_local_index] = 1U;
            scratch->previous_destination[destination_local_index] = box_local_index;
            matched_box = scratch->match_destination[destination_local_index];
            if (matched_box < 0) {
                int current_destination = destination_local_index;
                while (current_destination >= 0) {
                    int current_box = scratch->previous_destination[current_destination];
                    int previous_match = scratch->match_box[current_box];
                    scratch->match_destination[current_destination] = current_box;
                    scratch->match_box[current_box] = current_destination;
                    current_destination = previous_match;
                }
                return 1;
            }
            if (!scratch->seen_box[matched_box]) {
                scratch->seen_box[matched_box] = 1U;
                scratch->queue[tail++] = matched_box;
            }
        }
    }
    return 0;
}

static int matching_exists(const MatchingInfo *info) {
    MatchingScratch *scratch = &g_workspace.matching;
    int box_local_index;
    int matched_count = 0;

    int_array_fill(scratch->match_destination, info->unresolved_count, -1);
    int_array_fill(scratch->match_box, info->unresolved_count, -1);
    for (box_local_index = 0; box_local_index < info->unresolved_count; ++box_local_index) {
        if (matching_augment_iterative(info, box_local_index)) {
            matched_count += 1;
        }
    }
    return matched_count == info->unresolved_count;
}

/* Exact edge support is kept for small belief graphs; larger graphs use the
 * polynomial matching feasibility result without allocating 2^N memory. */
static uint32_t matching_edge_support(
    const MatchingInfo *info,
    int forced_box_local_index,
    int forced_destination_local_index
) {
    enum { SMALL_MATCHING_LIMIT = 8 };
    uint32_t dp[1u << SMALL_MATCHING_LIMIT];
    unsigned int state_count;
    unsigned int full_mask;
    int box_local_index;
    unsigned int mask;

    if (info->unresolved_count > SMALL_MATCHING_LIMIT ||
        !bitset_get(info->allowed_destinations[forced_box_local_index], forced_destination_local_index)) {
        return 1;
    }
    state_count = 1u << info->unresolved_count;
    full_mask = state_count - 1u;
    memset(dp, 0, sizeof(dp));
    dp[0] = 1;
    for (box_local_index = 0; box_local_index < info->unresolved_count; ++box_local_index) {
        for (mask = full_mask; ; --mask) {
            uint32_t ways = dp[mask];
            uint16_t available;
            if (ways != 0) {
                int destination_local_index;
                if (box_local_index == forced_box_local_index) {
                    available = (mask & (1u << forced_destination_local_index)) == 0
                        ? (uint16_t)(1u << forced_destination_local_index)
                        : 0;
                } else {
                    available = 0;
                    for (destination_local_index = 0;
                         destination_local_index < info->unresolved_count;
                         ++destination_local_index) {
                        if (bitset_get(info->allowed_destinations[box_local_index], destination_local_index)) {
                            available |= (uint16_t)(1u << destination_local_index);
                        }
                    }
                    available = (uint16_t)(available & (uint16_t)~mask);
                }
                while (available != 0) {
                    uint16_t bit = (uint16_t)(available & (uint16_t)(~available + 1u));
                    unsigned int next_mask = mask | (unsigned int)bit;
                    if (UINT_MAX - dp[next_mask] < ways) {
                        dp[next_mask] = UINT_MAX;
                    } else {
                        dp[next_mask] += ways;
                    }
                    available = (uint16_t)(available & (uint16_t)(available - 1u));
                }
            }
            if (mask == 0) {
                break;
            }
        }
    }
    return dp[full_mask];
}

static void matching_info_init(MatchingInfo *info) {
    int box_index;
    memset(info, 0, sizeof(*info));
    info->valid = 0;
    for (box_index = 0; box_index < PLANNER_OBJECT_CAPACITY; ++box_index) {
        info->fixed_destination_for_box[box_index] = -1;
    }
}

static int build_matching_info(const PlannerInput *input, MatchingInfo *info) {
    uint8_t *unresolved_destination_used = g_workspace.matching.unresolved_destination_used;
    int box_index;
    int destination_index;
    int unresolved_box_local_index;

    matching_info_init(info);
    memset(unresolved_destination_used, 0, PLANNER_OBJECT_CAPACITY);

    for (box_index = 0; box_index < input->movable_count; ++box_index) {
        const PlannerMovableBox *box = &input->movable_boxes[box_index];
        if (box->known && box->class_id >= 0) {
            int fixed_destination_index = -1;
            for (destination_index = 0; destination_index < input->destination_count; ++destination_index) {
                const PlannerDestinationBox *destination = &input->destination_boxes[destination_index];
                if (destination->known && destination->number == box->class_id) {
                    fixed_destination_index = destination_index;
                    break;
                }
            }
            if (fixed_destination_index >= 0) {
                if (is_forbidden_match(input, box_index, fixed_destination_index)) {
                    return 0;
                }
                info->fixed_destination_for_box[box_index] = fixed_destination_index;
                unresolved_destination_used[fixed_destination_index] = 1;
                continue;
            }
        }
        if (info->unresolved_count >= PLANNER_OBJECT_CAPACITY) {
            return 0;
        }
        info->unresolved_box_indices[info->unresolved_count++] = box_index;
    }

    {
        int unresolved_destination_count = 0;
        for (destination_index = 0; destination_index < input->destination_count; ++destination_index) {
            if (!unresolved_destination_used[destination_index]) {
                info->unresolved_destination_indices[unresolved_destination_count++] = destination_index;
            }
        }
        if (unresolved_destination_count != info->unresolved_count) {
            return 0;
        }
    }

    for (unresolved_box_local_index = 0; unresolved_box_local_index < info->unresolved_count; ++unresolved_box_local_index) {
        int global_box_index = info->unresolved_box_indices[unresolved_box_local_index];
        const PlannerMovableBox *box = &input->movable_boxes[global_box_index];
        int unresolved_destination_local_index;
        int allowed_count = 0;
        for (unresolved_destination_local_index = 0;
             unresolved_destination_local_index < info->unresolved_count;
             ++unresolved_destination_local_index) {
            int global_destination_index = info->unresolved_destination_indices[unresolved_destination_local_index];
            const PlannerDestinationBox *destination = &input->destination_boxes[global_destination_index];
            if (is_forbidden_match(input, global_box_index, global_destination_index)) {
                continue;
            }
            if (box->known && destination->known && box->class_id != destination->number) {
                continue;
            }
            bitset_set(info->allowed_destinations[unresolved_box_local_index], unresolved_destination_local_index);
            allowed_count += 1;
        }
        if (allowed_count == 0) {
            return 0;
        }
    }

    if (!matching_exists(info)) {
        return 0;
    }
    info->valid = 1;
    return 1;
}

static int certain_destination_for_box(const MatchingInfo *info, int global_box_index) {
    int unresolved_box_local_index;
    if (!info->valid) {
        return -1;
    }
    if (info->fixed_destination_for_box[global_box_index] >= 0) {
        return info->fixed_destination_for_box[global_box_index];
    }
    for (unresolved_box_local_index = 0; unresolved_box_local_index < info->unresolved_count; ++unresolved_box_local_index) {
        if (info->unresolved_box_indices[unresolved_box_local_index] == global_box_index) {
            int unresolved_destination_local_index;
            int only_destination = -1;
            for (unresolved_destination_local_index = 0;
                 unresolved_destination_local_index < info->unresolved_count;
                 ++unresolved_destination_local_index) {
                if (bitset_get(info->allowed_destinations[unresolved_box_local_index], unresolved_destination_local_index)) {
                    if (only_destination >= 0) {
                        return -1;
                    }
                    only_destination = info->unresolved_destination_indices[unresolved_destination_local_index];
                }
            }
            return only_destination;
        }
    }
    return -1;
}

static uint64_t scan_partition_score_for_box(const PlannerInput *input, const MatchingInfo *info, int global_box_index) {
    uint64_t groups[PLANNER_OBJECT_CAPACITY];
    int signatures[PLANNER_OBJECT_CAPACITY];
    int group_count = 0;
    int unresolved_box_local_index;
    int destination_local_index;

    memset(groups, 0, sizeof(groups));
    int_array_fill(signatures, PLANNER_OBJECT_CAPACITY, INT_MIN);
    for (unresolved_box_local_index = 0; unresolved_box_local_index < info->unresolved_count; ++unresolved_box_local_index) {
        if (info->unresolved_box_indices[unresolved_box_local_index] != global_box_index) {
            continue;
        }
        for (destination_local_index = 0; destination_local_index < info->unresolved_count; ++destination_local_index) {
            int global_destination_index;
            const PlannerDestinationBox *destination;
            int signature;
            int group_index;

            if (!bitset_get(info->allowed_destinations[unresolved_box_local_index], destination_local_index)) {
                continue;
            }
            global_destination_index = info->unresolved_destination_indices[destination_local_index];
            destination = &input->destination_boxes[global_destination_index];
            signature = destination->known ? destination->number : (1000 + destination->id);
            for (group_index = 0; group_index < group_count; ++group_index) {
                if (signatures[group_index] == signature) {
                    break;
                }
            }
            if (group_index == group_count) {
                signatures[group_count] = signature;
                group_count += 1;
            }
            groups[group_index] += matching_edge_support(
                info,
                unresolved_box_local_index,
                destination_local_index);
        }
        break;
    }

    {
        uint64_t score = 0;
        int index;
        for (index = 0; index < group_count; ++index) {
            score += groups[index] * groups[index];
        }
        return score;
    }
}

static uint64_t scan_partition_score_for_destination(const PlannerInput *input, const MatchingInfo *info, int global_destination_index) {
    uint64_t groups[PLANNER_OBJECT_CAPACITY];
    int signatures[PLANNER_OBJECT_CAPACITY];
    int group_count = 0;
    int unresolved_box_local_index;
    int destination_local_index = -1;

    memset(groups, 0, sizeof(groups));
    int_array_fill(signatures, PLANNER_OBJECT_CAPACITY, INT_MIN);
    for (destination_local_index = 0; destination_local_index < info->unresolved_count; ++destination_local_index) {
        if (info->unresolved_destination_indices[destination_local_index] == global_destination_index) {
            break;
        }
    }
    if (destination_local_index >= info->unresolved_count) {
        return UINT_MAX;
    }

    for (unresolved_box_local_index = 0; unresolved_box_local_index < info->unresolved_count; ++unresolved_box_local_index) {
        int global_box_index;
        const PlannerMovableBox *box;
        int signature;
        int group_index;

        if (!bitset_get(info->allowed_destinations[unresolved_box_local_index], destination_local_index)) {
            continue;
        }
        global_box_index = info->unresolved_box_indices[unresolved_box_local_index];
        box = &input->movable_boxes[global_box_index];
        signature = box->known ? box->class_id : (1000 + box->id);
        for (group_index = 0; group_index < group_count; ++group_index) {
            if (signatures[group_index] == signature) {
                break;
            }
        }
        if (group_index == group_count) {
            signatures[group_count] = signature;
            group_count += 1;
        }
        groups[group_index] += matching_edge_support(
            info,
            unresolved_box_local_index,
            destination_local_index);
    }

    {
        uint64_t score = 0;
        int index;
        for (index = 0; index < group_count; ++index) {
            score += groups[index] * groups[index];
        }
        return score;
    }
}

static int is_other_box_at(
    const PlannerInput *input,
    int x,
    int y,
    int active_box_index,
    int active_box_x,
    int active_box_y
) {
    int index;
    for (index = 0; index < input->movable_count; ++index) {
        const PlannerMovableBox *box = &input->movable_boxes[index];
        int box_x = box->x;
        int box_y = box->y;
        if (index == active_box_index) {
            box_x = active_box_x;
            box_y = active_box_y;
        }
        if (box_x == x && box_y == y) {
            return 1;
        }
    }
    return 0;
}

static int dynamic_bitset_cell_blocked(
    const PlannerContext *ctx,
    const uint8_t walls[PLANNER_BITSET_BYTES],
    const uint8_t bombs[PLANNER_BITSET_BYTES],
    int x,
    int y
) {
    int index;
    if (!is_inside(ctx, x, y)) {
        return 1;
    }
    index = cell_index(ctx, x, y);
    return bitset_get(walls, index) || bitset_get(bombs, index);
}

static int dynamic_bitset_is_corner_deadlock(
    const PlannerContext *ctx,
    const uint8_t walls[PLANNER_BITSET_BYTES],
    const uint8_t bombs[PLANNER_BITSET_BYTES],
    int x,
    int y,
    int target_x,
    int target_y
) {
    int up;
    int down;
    int left;
    int right;
    if (x == target_x && y == target_y) {
        return 0;
    }
    up = dynamic_bitset_cell_blocked(ctx, walls, bombs, x, y - 1);
    down = dynamic_bitset_cell_blocked(ctx, walls, bombs, x, y + 1);
    left = dynamic_bitset_cell_blocked(ctx, walls, bombs, x - 1, y);
    right = dynamic_bitset_cell_blocked(ctx, walls, bombs, x + 1, y);
    return (up && left) || (up && right) || (down && left) || (down && right);
}

static void explode_internal_wall_bits(
    const PlannerContext *ctx,
    uint8_t walls[PLANNER_BITSET_BYTES],
    int center_x,
    int center_y
) {
    int x;
    int y;
    for (y = center_y - 1; y <= center_y + 1; ++y) {
        for (x = center_x - 1; x <= center_x + 1; ++x) {
            if (!is_inside(ctx, x, y) || is_border_pos(ctx, x, y)) {
                continue;
            }
            bitset_clear(walls, cell_index(ctx, x, y));
        }
    }
}

static void compute_dead_squares(PlannerContext *ctx) {
    int *queue = g_workspace.cell_queue;
    int head = 0;
    int tail = 0;
    int index;
    int dir;
    const PlannerInput *input = ctx->input;

    memset(ctx->dead_squares, 1, sizeof(ctx->dead_squares));
    for (index = 0; index < input->destination_count; ++index) {
        int x = input->destination_boxes[index].x;
        int y = input->destination_boxes[index].y;
        int cell = cell_index(ctx, x, y);
        if (ctx->dead_squares[cell]) {
            ctx->dead_squares[cell] = 0;
            queue[tail++] = cell;
        }
    }

    while (head < tail) {
        int current = queue[head++];
        int box_x = current % input->width;
        int box_y = current / input->width;

        for (dir = 0; dir < 4; ++dir) {
            int prev_box_x = box_x - DIR_DX[dir];
            int prev_box_y = box_y - DIR_DY[dir];
            int car_stand_x = prev_box_x - DIR_DX[dir];
            int car_stand_y = prev_box_y - DIR_DY[dir];
            int previous_cell;
            if (!is_inside(ctx, prev_box_x, prev_box_y) || !is_inside(ctx, car_stand_x, car_stand_y)) {
                continue;
            }
            if (is_static_blocked(ctx, prev_box_x, prev_box_y) || is_static_blocked(ctx, car_stand_x, car_stand_y)) {
                continue;
            }
            previous_cell = cell_index(ctx, prev_box_x, prev_box_y);
            if (ctx->dead_squares[previous_cell]) {
                ctx->dead_squares[previous_cell] = 0;
                queue[tail++] = previous_cell;
            }
        }
    }

    for (index = 0; index < input->width * input->height; ++index) {
        if (ctx->walls[index] || ctx->bombs[index] || ctx->destinations[index]) {
            ctx->dead_squares[index] = 0;
        }
    }
}

static int cell_x(const PlannerContext *ctx, int cell);
static int cell_y(const PlannerContext *ctx, int cell);

static void build_motion_block_bits(
    const PlannerContext *ctx,
    const uint8_t walls[PLANNER_BITSET_BYTES],
    const uint8_t bombs[PLANNER_BITSET_BYTES],
    int active_box_index,
    int active_box_cell,
    uint8_t blocked[PLANNER_BITSET_BYTES]
) {
    const PlannerInput *input = ctx->input;
    int index;

    bitset_copy(blocked, walls);
    for (index = 0; index < PLANNER_BITSET_BYTES; ++index) {
        blocked[index] |= bombs[index];
    }

    for (index = 0; index < input->movable_count; ++index) {
        int box_cell;
        if (index == active_box_index) {
            if (active_box_cell < 0) {
                continue;
            }
            box_cell = active_box_cell;
        } else {
            box_cell = cell_index(ctx, input->movable_boxes[index].x, input->movable_boxes[index].y);
        }
        bitset_set(blocked, box_cell);
    }
}

static int segment_is_clear_for_motion(
    const PlannerContext *ctx,
    int start_cell,
    int target_cell,
    const uint8_t blocked[PLANNER_BITSET_BYTES]
) {
    int start_x = cell_x(ctx, start_cell);
    int start_y = cell_y(ctx, start_cell);
    int target_x = cell_x(ctx, target_cell);
    int target_y = cell_y(ctx, target_cell);
    int current_x = start_x;
    int current_y = start_y;
    int dx = abs_int(target_x - start_x);
    int dy = abs_int(target_y - start_y);
    int step_x = target_x > start_x ? 1 : (target_x < start_x ? -1 : 0);
    int step_y = target_y > start_y ? 1 : (target_y < start_y ? -1 : 0);
    int t_delta_x = step_x == 0 ? INT_MAX : 2 * dy;
    int t_delta_y = step_y == 0 ? INT_MAX : 2 * dx;
    int t_max_x = step_x == 0 ? INT_MAX : dy;
    int t_max_y = step_y == 0 ? INT_MAX : dx;

    if (start_cell == target_cell) {
        return 1;
    }

    while (current_x != target_x || current_y != target_y) {
        if (t_max_x < t_max_y) {
            int sample_cell;
            current_x += step_x;
            if (!is_inside(ctx, current_x, current_y)) {
                return 0;
            }
            sample_cell = cell_index(ctx, current_x, current_y);
            if (sample_cell != start_cell && bitset_get(blocked, sample_cell)) {
                return 0;
            }
            t_max_x += t_delta_x;
            continue;
        }
        if (t_max_y < t_max_x) {
            int sample_cell;
            current_y += step_y;
            if (!is_inside(ctx, current_x, current_y)) {
                return 0;
            }
            sample_cell = cell_index(ctx, current_x, current_y);
            if (sample_cell != start_cell && bitset_get(blocked, sample_cell)) {
                return 0;
            }
            t_max_y += t_delta_y;
            continue;
        }

        if (step_x != 0) {
            int side_x = current_x + step_x;
            int side_y = current_y;
            int sample_cell;
            if (!is_inside(ctx, side_x, side_y)) {
                return 0;
            }
            sample_cell = cell_index(ctx, side_x, side_y);
            if (sample_cell != start_cell && bitset_get(blocked, sample_cell)) {
                return 0;
            }
        }
        if (step_y != 0) {
            int side_x = current_x;
            int side_y = current_y + step_y;
            int sample_cell;
            if (!is_inside(ctx, side_x, side_y)) {
                return 0;
            }
            sample_cell = cell_index(ctx, side_x, side_y);
            if (sample_cell != start_cell && bitset_get(blocked, sample_cell)) {
                return 0;
            }
        }

        current_x += step_x;
        current_y += step_y;
        if (!is_inside(ctx, current_x, current_y)) {
            return 0;
        }
        if (bitset_get(blocked, cell_index(ctx, current_x, current_y))) {
            return 0;
        }
        t_max_x += t_delta_x;
        t_max_y += t_delta_y;
    }
    return 1;
}

static int free_motion_plan_bits(
    const PlannerContext *ctx,
    int start_cell,
    int goal_cell,
    int start_dir,
    int goal_dir,
    int require_goal_orientation,
    const uint8_t blocked[PLANNER_BITSET_BYTES],
    MovePlan *plan
) {
    int *parent = g_workspace.free_motion.parent;
    int *g_cost = g_workspace.free_motion.g_cost;
    int *visited = g_workspace.free_motion.visited;
    int *free_cells = g_workspace.free_motion.free_cells;
    int *path_cells = g_workspace.free_motion.path_cells;
    int free_count = 0;
    int path_count = 0;
    int cell_count = ctx->input->width * ctx->input->height;
    int orientation_cost = 0;
    int index;

    memset(plan, 0, sizeof(*plan));
    if (require_goal_orientation && start_dir >= 0 && goal_dir >= 0) {
        orientation_cost = turn_distance(start_dir, goal_dir) * TURN_COST;
    }
    if (start_cell == goal_cell) {
        if (orientation_cost > 0) {
            if (!append_step(
                    plan->steps,
                    &plan->step_count,
                    PLANNER_STEP_MOVE,
                    cell_x(ctx, goal_cell),
                    cell_y(ctx, goal_cell),
                    goal_dir)) {
                return 0;
            }
            plan->success = 1;
            plan->cost = orientation_cost;
            return 1;
        }
        plan->success = 1;
        plan->cost = orientation_cost;
        return 1;
    }
    if (bitset_get(blocked, goal_cell)) {
        return 0;
    }

    int_array_fill(parent, PLANNER_MAX_CELLS, -2);
    int_array_fill(g_cost, PLANNER_MAX_CELLS, INT_MAX / 4);
    int_array_fill(visited, PLANNER_MAX_CELLS, 0);
    for (index = 0; index < cell_count; ++index) {
        if (!bitset_get(blocked, index) || index == start_cell || index == goal_cell) {
            free_cells[free_count++] = index;
        }
    }

    parent[start_cell] = -1;
    g_cost[start_cell] = 0;
    while (1) {
        int current_cell = -1;
        int best_cost = INT_MAX / 4;
        for (index = 0; index < free_count; ++index) {
            int candidate = free_cells[index];
            if (!visited[candidate] && g_cost[candidate] < best_cost) {
                best_cost = g_cost[candidate];
                current_cell = candidate;
            }
        }
        if (current_cell < 0) {
            break;
        }
        if (current_cell == goal_cell) {
            break;
        }
        visited[current_cell] = 1;
        for (index = 0; index < free_count; ++index) {
            int next_cell = free_cells[index];
            int tentative_cost;
            int dx_cells;
            int dy_cells;
            if (next_cell == current_cell || visited[next_cell]) {
                continue;
            }
            if (!segment_is_clear_for_motion(ctx, current_cell, next_cell, blocked)) {
                continue;
            }
            dx_cells = abs_int(cell_x(ctx, next_cell) - cell_x(ctx, current_cell));
            dy_cells = abs_int(cell_y(ctx, next_cell) - cell_y(ctx, current_cell));
            tentative_cost = g_cost[current_cell] + scaled_distance_cost(dx_cells, dy_cells, MOVE_COST);
            if (tentative_cost >= g_cost[next_cell]) {
                continue;
            }
            g_cost[next_cell] = tentative_cost;
            parent[next_cell] = current_cell;
        }
    }

    if (parent[goal_cell] == -2) {
        return 0;
    }

    {
        int cursor = goal_cell;
        while (cursor >= 0 && path_count < PLANNER_MAX_CELLS) {
            path_cells[path_count++] = cursor;
            cursor = parent[cursor];
        }
    }
    for (index = 0; index < path_count / 2; ++index) {
        int temp = path_cells[index];
        path_cells[index] = path_cells[path_count - 1 - index];
        path_cells[path_count - 1 - index] = temp;
    }

    for (index = 1; index < path_count; ++index) {
        int cell = path_cells[index];
        if (!append_step(
                plan->steps,
                &plan->step_count,
                PLANNER_STEP_MOVE,
                cell_x(ctx, cell),
                cell_y(ctx, cell),
                goal_dir)) {
            return 0;
        }
    }

    plan->success = 1;
    plan->cost = g_cost[goal_cell] + orientation_cost;
    return 1;
}

static int cell_x(const PlannerContext *ctx, int cell) {
    return cell % ctx->input->width;
}

static int cell_y(const PlannerContext *ctx, int cell) {
    return cell / ctx->input->width;
}

static unsigned int hash_mix(unsigned int value) {
    value ^= value >> 16;
    value *= 0x7feb352dU;
    value ^= value >> 15;
    value *= 0x846ca68bU;
    value ^= value >> 16;
    return value;
}

static unsigned int push_env_hash(
    const uint8_t walls[PLANNER_BITSET_BYTES],
    const uint8_t bombs[PLANNER_BITSET_BYTES]
) {
    unsigned int hash = 2166136261u;
    int index;
    for (index = 0; index < PLANNER_BITSET_BYTES; ++index) {
        hash = (hash ^ walls[index]) * 16777619u;
    }
    for (index = 0; index < PLANNER_BITSET_BYTES; ++index) {
        hash = (hash ^ bombs[index]) * 16777619u;
    }
    return hash_mix(hash);
}

static unsigned int push_node_hash(uint16_t car_cell, uint16_t box_cell, uint16_t env_id, int car_dir) {
    unsigned int packed = (unsigned int)car_cell |
                          ((unsigned int)box_cell << 8) |
                          ((unsigned int)env_id << 16) |
                          ((unsigned int)(car_dir & 3) << 26);
    return hash_mix(packed * 2654435761u);
}

static int relaxed_push_heuristic(
    const PlannerContext *ctx,
    int car_cell,
    int box_cell,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
);

static void push_search_reset(PushSearch *search) {
    search->node_count = 0;
    search->heap_count = 0;
    search->env_count = 0;
    search->best_goal = PUSH_INVALID_INDEX;
    search->active = 0U;
    uint16_array_fill(search->node_table, PUSH_NODE_TABLE_CAPACITY, PUSH_INVALID_INDEX);
    uint16_array_fill(search->env_table, PUSH_ENV_TABLE_CAPACITY, PUSH_INVALID_INDEX);
}

static int push_env_find_or_add(
    PushSearch *search,
    const uint8_t walls[PLANNER_BITSET_BYTES],
    const uint8_t bombs[PLANNER_BITSET_BYTES]
) {
    unsigned int bucket = push_env_hash(walls, bombs) % PUSH_ENV_TABLE_CAPACITY;
    uint16_t cursor = search->env_table[bucket];
    while (cursor != PUSH_INVALID_INDEX) {
        if (memcmp(search->envs[cursor].walls, walls, PLANNER_BITSET_BYTES) == 0 &&
            memcmp(search->envs[cursor].bombs, bombs, PLANNER_BITSET_BYTES) == 0) {
            return cursor;
        }
        cursor = search->envs[cursor].hash_next;
    }

    if (search->env_count >= PUSH_ENV_CAPACITY) {
        return -1;
    }

    cursor = search->env_count++;
    memcpy(search->envs[cursor].walls, walls, PLANNER_BITSET_BYTES);
    memcpy(search->envs[cursor].bombs, bombs, PLANNER_BITSET_BYTES);
    search->envs[cursor].hash_next = search->env_table[bucket];
    search->env_table[bucket] = cursor;
    return cursor;
}

static int push_node_find(
    const PushSearch *search,
    uint16_t car_cell,
    uint16_t box_cell,
    uint16_t env_id,
    int car_dir
) {
    unsigned int bucket = push_node_hash(car_cell, box_cell, env_id, car_dir) % PUSH_NODE_TABLE_CAPACITY;
    uint16_t cursor = search->node_table[bucket];
    while (cursor != PUSH_INVALID_INDEX) {
        const PushNode *node = &search->nodes[cursor];
        if (node->car_cell == car_cell && node->box_cell == box_cell && node->env_id == env_id &&
            node->car_dir == car_dir) {
            return cursor;
        }
        cursor = node->hash_next;
    }
    return -1;
}

static int push_node_add(
    PushSearch *search,
    uint16_t car_cell,
    uint16_t box_cell,
    uint16_t env_id,
    int g,
    int disruption,
    int parent,
    int car_dir,
    int transition_cell,
    int transition_kind
) {
    PushNode *node;
    unsigned int bucket;
    uint16_t index;

    if (search->node_count >= PUSH_NODE_CAPACITY) {
        return -1;
    }

    index = search->node_count++;
    node = &search->nodes[index];
    node->g = (uint16_t)g;
    node->disruption = (uint16_t)disruption;
    node->parent = parent >= 0 ? (uint16_t)parent : PUSH_INVALID_INDEX;
    node->hash_next = PUSH_INVALID_INDEX;
    node->car_cell = car_cell;
    node->box_cell = box_cell;
    node->env_id = env_id;
    node->heap_index = PUSH_INVALID_INDEX;
    node->transition_cell = (uint8_t)transition_cell;
    node->car_dir = (uint8_t)car_dir;
    node->transition_kind = (uint8_t)transition_kind;
    node->closed = 0;
    node->reserved = 0;
    bucket = push_node_hash(car_cell, box_cell, env_id, car_dir) % PUSH_NODE_TABLE_CAPACITY;
    node->hash_next = search->node_table[bucket];
    search->node_table[bucket] = index;
    return index;
}

static int push_node_priority(
    const PushSearch *search,
    const PlannerContext *ctx,
    int node_index,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    const PushNode *node = &search->nodes[node_index];
    return node->g + relaxed_push_heuristic(ctx, node->car_cell, node->box_cell, push_dist);
}

static int push_heap_less(
    const PushSearch *search,
    const PlannerContext *ctx,
    int left_index,
    int right_index,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    int left_priority = push_node_priority(search, ctx, left_index, push_dist);
    int right_priority = push_node_priority(search, ctx, right_index, push_dist);
    if (left_priority != right_priority) {
        return left_priority < right_priority;
    }
    if (search->nodes[left_index].disruption != search->nodes[right_index].disruption) {
        return search->nodes[left_index].disruption < search->nodes[right_index].disruption;
    }
    return search->nodes[left_index].g < search->nodes[right_index].g;
}

static void push_heap_swap(PushSearch *search, int left_pos, int right_pos) {
    uint16_t left_index = search->heap[left_pos];
    uint16_t right_index = search->heap[right_pos];
    search->heap[left_pos] = right_index;
    search->heap[right_pos] = left_index;
    search->nodes[left_index].heap_index = right_pos;
    search->nodes[right_index].heap_index = left_pos;
}

static void push_heap_sift_up(
    PushSearch *search,
    const PlannerContext *ctx,
    int heap_pos,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    while (heap_pos > 0) {
        int parent_pos = (heap_pos - 1) / 2;
        if (!push_heap_less(search, ctx, search->heap[heap_pos], search->heap[parent_pos], push_dist)) {
            break;
        }
        push_heap_swap(search, heap_pos, parent_pos);
        heap_pos = parent_pos;
    }
}

static void push_heap_sift_down(
    PushSearch *search,
    const PlannerContext *ctx,
    int heap_pos,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    for (;;) {
        int left_child = heap_pos * 2 + 1;
        int right_child = left_child + 1;
        int best_pos = heap_pos;

        if (left_child < search->heap_count &&
            push_heap_less(search, ctx, search->heap[left_child], search->heap[best_pos], push_dist)) {
            best_pos = left_child;
        }
        if (right_child < search->heap_count &&
            push_heap_less(search, ctx, search->heap[right_child], search->heap[best_pos], push_dist)) {
            best_pos = right_child;
        }
        if (best_pos == heap_pos) {
            break;
        }
        push_heap_swap(search, heap_pos, best_pos);
        heap_pos = best_pos;
    }
}

static void push_heap_push(
    PushSearch *search,
    const PlannerContext *ctx,
    int node_index,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    int heap_pos = search->heap_count++;
    search->heap[heap_pos] = node_index;
    search->nodes[node_index].heap_index = heap_pos;
    push_heap_sift_up(search, ctx, heap_pos, push_dist);
}

static int push_heap_pop_min(
    PushSearch *search,
    const PlannerContext *ctx,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    uint16_t result = search->heap[0];
    search->nodes[result].heap_index = PUSH_INVALID_INDEX;
    search->heap[0] = search->heap[--search->heap_count];
    if (search->heap_count > 0) {
        search->nodes[search->heap[0]].heap_index = 0;
        push_heap_sift_down(search, ctx, 0, push_dist);
    }
    return result;
}

static int is_relaxed_push_blocked(
    const PlannerContext *ctx,
    int box_index,
    int x,
    int y
) {
    int index;
    if (!is_inside(ctx, x, y)) {
        return 1;
    }
    if (is_border_pos(ctx, x, y) && ctx->walls[cell_index(ctx, x, y)]) {
        return 1;
    }
    for (index = 0; index < ctx->input->movable_count; ++index) {
        if (index == box_index) {
            continue;
        }
        if (ctx->input->movable_boxes[index].x == x && ctx->input->movable_boxes[index].y == y) {
            return 1;
        }
    }
    return 0;
}

static void compute_relaxed_push_distances(
    const PlannerContext *ctx,
    int box_index,
    int target_x,
    int target_y,
    uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    int *queue = g_workspace.cell_queue;
    int head = 0;
    int tail = 0;
    int index;
    int dir;
    int target_cell = cell_index(ctx, target_x, target_y);

    for (index = 0; index < PLANNER_MAX_CELLS; ++index) {
        push_dist[index] = PUSH_INVALID_INDEX;
    }
    if (is_relaxed_push_blocked(ctx, box_index, target_x, target_y)) {
        return;
    }

    push_dist[target_cell] = 0;
    queue[tail++] = target_cell;
    while (head < tail) {
        int current_cell = queue[head++];
        int current_x = cell_x(ctx, current_cell);
        int current_y = cell_y(ctx, current_cell);
        uint16_t current_dist = push_dist[current_cell];

        for (dir = 0; dir < 4; ++dir) {
            int prev_box_x = current_x - DIR_DX[dir];
            int prev_box_y = current_y - DIR_DY[dir];
            int prev_car_x = prev_box_x - DIR_DX[dir];
            int prev_car_y = prev_box_y - DIR_DY[dir];
            int prev_cell;
            if (is_relaxed_push_blocked(ctx, box_index, prev_box_x, prev_box_y) ||
                is_relaxed_push_blocked(ctx, box_index, prev_car_x, prev_car_y)) {
                continue;
            }
            prev_cell = cell_index(ctx, prev_box_x, prev_box_y);
            if (push_dist[prev_cell] != PUSH_INVALID_INDEX) {
                continue;
            }
            push_dist[prev_cell] = (uint16_t)(current_dist + 1);
            queue[tail++] = prev_cell;
        }
    }
}

static int relaxed_push_heuristic(
    const PlannerContext *ctx,
    int car_cell,
    int box_cell,
    const uint16_t push_dist[PLANNER_MAX_CELLS]
) {
    uint16_t box_steps = push_dist[box_cell];
    int car_x = cell_x(ctx, car_cell);
    int car_y = cell_y(ctx, car_cell);
    int box_x = cell_x(ctx, box_cell);
    int box_y = cell_y(ctx, box_cell);
    int extra_move = 0;

    if (box_steps == PUSH_INVALID_INDEX) {
        return INT_MAX / 8;
    }
    if (box_steps > 0 && manhattan(car_x, car_y, box_x, box_y) != 1) {
        extra_move = MOVE_COST;
    }
    return box_steps * PUSH_COST + extra_move;
}

static int push_astar_step(
    const PlannerContext *ctx,
    int box_index,
    int target_x,
    int target_y,
    uint32_t expansion_budget,
    PushPlan *plan
) {
    PushSearch *search = &g_workspace.push_search;
    const uint16_t *push_dist = search->push_dist;
    const PlannerInput *input = ctx->input;
    uint32_t expanded = 0;

    if (!search->active) {
        int start_car_cell;
        int start_box_cell;
        int initial_env_id;
        int start_node;

        memset(plan, 0, sizeof(*plan));
        push_search_reset(search);
        compute_relaxed_push_distances(ctx, box_index, target_x, target_y, search->push_dist);

        initial_env_id = push_env_find_or_add(search, input->wall_bits, input->bomb_bits);
        start_car_cell = cell_index(ctx, input->car_x, input->car_y);
        start_box_cell = cell_index(ctx, input->movable_boxes[box_index].x, input->movable_boxes[box_index].y);
        if (push_dist[start_box_cell] == PUSH_INVALID_INDEX || initial_env_id < 0) {
            return 0;
        }
        start_node = push_node_add(
            search,
            (uint16_t)start_car_cell,
            (uint16_t)start_box_cell,
            (uint16_t)initial_env_id,
            0,
            0,
            -1,
            input->car_dir,
            0,
            TRANSITION_NONE);
        if (start_node < 0) {
            return 0;
        }
        push_heap_push(search, ctx, start_node, push_dist);
        search->best_goal = PUSH_INVALID_INDEX;
        search->box_index = box_index;
        search->target_x = target_x;
        search->target_y = target_y;
        search->active = 1U;
    }

    while (search->heap_count > 0 && expanded < expansion_budget) {
        expanded++;
        int current_index = push_heap_pop_min(search, ctx, push_dist);
        int current_car_cell = search->nodes[current_index].car_cell;
        int current_car_dir = search->nodes[current_index].car_dir;
        int current_box_cell = search->nodes[current_index].box_cell;
        int current_env_id = search->nodes[current_index].env_id;
        int current_g = search->nodes[current_index].g;
        int current_box_x = cell_x(ctx, current_box_cell);
        int current_box_y = cell_y(ctx, current_box_cell);
        uint8_t current_walls[PLANNER_BITSET_BYTES];
        uint8_t current_bombs[PLANNER_BITSET_BYTES];
        uint8_t blocked[PLANNER_BITSET_BYTES];
        int dir;
        int bomb_cell;

        bitset_copy(current_walls, search->envs[current_env_id].walls);
        bitset_copy(current_bombs, search->envs[current_env_id].bombs);
        build_motion_block_bits(ctx, current_walls, current_bombs, box_index, current_box_cell, blocked);

        if (current_box_x == target_x && current_box_y == target_y) {
            search->best_goal = (uint16_t)current_index;
            break;
        }
        search->nodes[current_index].closed = 1;

        for (dir = 0; dir < 4; ++dir) {
            int stand_x = current_box_x - DIR_DX[dir];
            int stand_y = current_box_y - DIR_DY[dir];
            int next_box_x = current_box_x + DIR_DX[dir];
            int next_box_y = current_box_y + DIR_DY[dir];
            int stand_cell;
            int next_box_cell;
            int tentative_g;
            int node_slot;
            MovePlan *move_plan = &g_workspace.move_plans[0];

            if (!is_inside(ctx, stand_x, stand_y) || !is_inside(ctx, next_box_x, next_box_y)) {
                continue;
            }
            stand_cell = cell_index(ctx, stand_x, stand_y);
            next_box_cell = cell_index(ctx, next_box_x, next_box_y);
            if (bitset_get(blocked, stand_cell) || bitset_get(blocked, next_box_cell)) {
                continue;
            }
            if (dynamic_bitset_is_corner_deadlock(ctx, current_walls, current_bombs, next_box_x, next_box_y, target_x, target_y)) {
                continue;
            }
            if (!free_motion_plan_bits(ctx, current_car_cell, stand_cell, current_car_dir, dir, 1, blocked, move_plan)) {
                continue;
            }

            tentative_g = current_g + move_plan->cost + PUSH_COST;
            node_slot = push_node_find(
                search,
                (uint16_t)current_box_cell,
                (uint16_t)next_box_cell,
                (uint16_t)current_env_id,
                dir);
            if (node_slot >= 0 && search->nodes[node_slot].closed) {
                continue;
            }
            if (node_slot < 0) {
                node_slot = push_node_add(
                    search,
                    (uint16_t)current_box_cell,
                    (uint16_t)next_box_cell,
                    (uint16_t)current_env_id,
                    tentative_g,
                    search->nodes[current_index].disruption,
                    current_index,
                    dir,
                    current_box_cell,
                    TRANSITION_BOX_PUSH);
                if (node_slot < 0) {
                    continue;
                }
                push_heap_push(search, ctx, node_slot, push_dist);
                continue;
            }
            if (tentative_g < search->nodes[node_slot].g ||
                (tentative_g == search->nodes[node_slot].g &&
                 search->nodes[current_index].disruption < search->nodes[node_slot].disruption)) {
                search->nodes[node_slot].g = (uint16_t)tentative_g;
                search->nodes[node_slot].disruption = search->nodes[current_index].disruption;
                search->nodes[node_slot].parent = (uint16_t)current_index;
                search->nodes[node_slot].car_dir = (uint8_t)dir;
                search->nodes[node_slot].transition_cell = (uint8_t)current_box_cell;
                search->nodes[node_slot].transition_kind = TRANSITION_BOX_PUSH;
                if (search->nodes[node_slot].heap_index != PUSH_INVALID_INDEX) {
                    push_heap_sift_up(search, ctx, search->nodes[node_slot].heap_index, push_dist);
                } else {
                    push_heap_push(search, ctx, node_slot, push_dist);
                }
            }
        }

        for (bomb_cell = 0; bomb_cell < ctx->input->width * ctx->input->height; ++bomb_cell) {
            int bomb_x;
            int bomb_y;
            if (!bitset_get(current_bombs, bomb_cell)) {
                continue;
            }
            bomb_x = cell_x(ctx, bomb_cell);
            bomb_y = cell_y(ctx, bomb_cell);

            for (dir = 0; dir < 4; ++dir) {
                int stand_x = bomb_x - DIR_DX[dir];
                int stand_y = bomb_y - DIR_DY[dir];
                int next_bomb_x = bomb_x + DIR_DX[dir];
                int next_bomb_y = bomb_y + DIR_DY[dir];
                int stand_cell;
                int next_bomb_cell;
                int next_env_id;
                int next_box_cell;
                int tentative_g;
                uint8_t next_walls[PLANNER_BITSET_BYTES];
                uint8_t next_bombs[PLANNER_BITSET_BYTES];
                int node_slot;
                MovePlan *move_plan = &g_workspace.move_plans[0];

                if (!is_inside(ctx, stand_x, stand_y) || !is_inside(ctx, next_bomb_x, next_bomb_y)) {
                    continue;
                }
                stand_cell = cell_index(ctx, stand_x, stand_y);
                next_bomb_cell = cell_index(ctx, next_bomb_x, next_bomb_y);
                if (bitset_get(blocked, stand_cell)) {
                    continue;
                }
                if ((next_bomb_x == current_box_x && next_bomb_y == current_box_y) ||
                    is_other_box_at(input, next_bomb_x, next_bomb_y, box_index, current_box_x, current_box_y) ||
                    bitset_get(current_bombs, next_bomb_cell)) {
                    continue;
                }
                if (!free_motion_plan_bits(ctx, current_car_cell, stand_cell, current_car_dir, dir, 1, blocked, move_plan)) {
                    continue;
                }
                bitset_copy(next_walls, current_walls);
                bitset_copy(next_bombs, current_bombs);
                bitset_clear(next_bombs, bomb_cell);
                if (bitset_get(current_walls, next_bomb_cell)) {
                    if (is_border_pos(ctx, next_bomb_x, next_bomb_y)) {
                        continue;
                    }
                    explode_internal_wall_bits(ctx, next_walls, next_bomb_x, next_bomb_y);
                } else {
                    bitset_set(next_bombs, next_bomb_cell);
                }
                next_env_id = push_env_find_or_add(search, next_walls, next_bombs);
                if (next_env_id < 0) {
                    continue;
                }
                next_box_cell = current_box_cell;
                tentative_g = current_g + move_plan->cost + BOMB_PUSH_COST;

                node_slot = push_node_find(
                    search,
                    (uint16_t)bomb_cell,
                    (uint16_t)next_box_cell,
                    (uint16_t)next_env_id,
                    dir);
                if (node_slot >= 0 && search->nodes[node_slot].closed) {
                    continue;
                }
                if (node_slot < 0) {
                    node_slot = push_node_add(
                        search,
                        (uint16_t)bomb_cell,
                        (uint16_t)next_box_cell,
                        (uint16_t)next_env_id,
                        tentative_g,
                        search->nodes[current_index].disruption + 1,
                        current_index,
                        dir,
                        bomb_cell,
                        TRANSITION_BOMB_PUSH);
                    if (node_slot < 0) {
                        continue;
                    }
                    push_heap_push(search, ctx, node_slot, push_dist);
                    continue;
                }
                if (tentative_g < search->nodes[node_slot].g ||
                    (tentative_g == search->nodes[node_slot].g &&
                     search->nodes[current_index].disruption + 1 < search->nodes[node_slot].disruption)) {
                    search->nodes[node_slot].g = (uint16_t)tentative_g;
                    search->nodes[node_slot].disruption = (uint16_t)(search->nodes[current_index].disruption + 1);
                    search->nodes[node_slot].parent = (uint16_t)current_index;
                    search->nodes[node_slot].car_dir = (uint8_t)dir;
                    search->nodes[node_slot].transition_cell = (uint8_t)bomb_cell;
                    search->nodes[node_slot].transition_kind = TRANSITION_BOMB_PUSH;
                    if (search->nodes[node_slot].heap_index != PUSH_INVALID_INDEX) {
                        push_heap_sift_up(search, ctx, search->nodes[node_slot].heap_index, push_dist);
                    } else {
                        push_heap_push(search, ctx, node_slot, push_dist);
                    }
                }
            }
        }
    }

    if (search->best_goal == PUSH_INVALID_INDEX) {
        if (search->heap_count > 0) {
            return 2;
        }
        search->active = 0U;
        return 0;
    }

    {
        int *chain = g_workspace.push_chain;
        int chain_count = 0;
        int cursor = search->best_goal;
        int index;

        while (cursor >= 0 && search->nodes[cursor].parent != PUSH_INVALID_INDEX) {
            if (chain_count >= PLANNER_MAX_PLAN_STEPS) {
                return 0;
            }
            chain[chain_count++] = cursor;
            cursor = search->nodes[cursor].parent;
        }

        for (index = chain_count - 1; index >= 0; --index) {
            const PushNode *current = &search->nodes[chain[index]];
            const PushNode *parent_node = &search->nodes[current->parent];
            uint8_t parent_walls[PLANNER_BITSET_BYTES];
            uint8_t parent_bombs[PLANNER_BITSET_BYTES];
            uint8_t parent_blocked[PLANNER_BITSET_BYTES];
            MovePlan *move_plan = &g_workspace.move_plans[0];
            int action_cell = current->transition_cell;
            int action_x = cell_x(ctx, action_cell);
            int action_y = cell_y(ctx, action_cell);
            int stand_x = action_x - DIR_DX[current->car_dir];
            int stand_y = action_y - DIR_DY[current->car_dir];
            int stand_cell = cell_index(ctx, stand_x, stand_y);

            if (current->transition_kind == TRANSITION_NONE) {
                return 0;
            }

            bitset_copy(parent_walls, search->envs[parent_node->env_id].walls);
            bitset_copy(parent_bombs, search->envs[parent_node->env_id].bombs);
            build_motion_block_bits(ctx, parent_walls, parent_bombs, box_index, parent_node->box_cell, parent_blocked);
            if (!free_motion_plan_bits(
                    ctx,
                    parent_node->car_cell,
                    stand_cell,
                    parent_node->car_dir,
                    current->car_dir,
                    1,
                    parent_blocked,
                    move_plan)) {
                return 0;
            }
            if (!append_steps(plan->steps, &plan->step_count, move_plan->steps, move_plan->step_count)) {
                return 0;
            }
            if (!append_step(plan->steps, &plan->step_count, PLANNER_STEP_PUSH, action_x, action_y, current->car_dir)) {
                return 0;
            }
        }
    }

    plan->success = 1;
    plan->cost = search->nodes[search->best_goal].g;
    plan->end_car_x = cell_x(ctx, search->nodes[search->best_goal].car_cell);
    plan->end_car_y = cell_y(ctx, search->nodes[search->best_goal].car_cell);
    plan->end_car_dir = search->nodes[search->best_goal].car_dir;
    search->active = 0U;
    return 1;
}

static int estimate_next_task_cost(
    const PlannerContext *ctx,
    int current_box_index,
    int current_destination_index,
    int car_x,
    int car_y,
    int car_dir
) {
    int best = 0;
    int index;
    for (index = 0; index < ctx->input->movable_count; ++index) {
        int destination_index;
        const PlannerMovableBox *box = &ctx->input->movable_boxes[index];
        if (!box->known || index == current_box_index) {
            continue;
        }
        for (destination_index = 0; destination_index < ctx->input->destination_count; ++destination_index) {
            const PlannerDestinationBox *destination = &ctx->input->destination_boxes[destination_index];
            int estimate;
            if (!destination->known || destination_index == current_destination_index) {
                continue;
            }
            if (destination->number != box->class_id) {
                continue;
            }
            estimate = scaled_distance_cost(abs_int(car_x - box->x), abs_int(car_y - box->y), MOVE_COST) +
                       manhattan(box->x, box->y, destination->x, destination->y) * PUSH_COST +
                       turn_distance(car_dir, PLANNER_DIR_RIGHT) * TURN_COST;
            if (best == 0 || estimate < best) {
                best = estimate;
            }
        }
    }
    return best;
}

static int build_task_candidates(
    const PlannerContext *ctx,
    const MatchingInfo *matching_info,
    TaskCandidate candidates[PLANNER_OBJECT_CAPACITY]
) {
    int candidate_count = 0;
    int box_index;

    memset(candidates, 0, sizeof(TaskCandidate) * PLANNER_OBJECT_CAPACITY);
    for (box_index = 0; box_index < ctx->input->movable_count; ++box_index) {
        const PlannerMovableBox *box = &ctx->input->movable_boxes[box_index];
        int destination_index = certain_destination_for_box(matching_info, box_index);
        const PlannerDestinationBox *destination;
        if (destination_index < 0) {
            continue;
        }
        destination = &ctx->input->destination_boxes[destination_index];
        if (box->x == destination->x && box->y == destination->y) {
            continue;
        }
        candidates[candidate_count].box_index = box_index;
        candidates[candidate_count].destination_index = destination_index;
        candidates[candidate_count].quick_score =
            scaled_distance_cost(
                abs_int(ctx->input->car_x - box->x),
                abs_int(ctx->input->car_y - box->y),
                MOVE_COST) +
            manhattan(box->x, box->y, destination->x, destination->y) * PUSH_COST;
        candidate_count += 1;
    }

    /* 固定上限 192，插入排序不依赖 libc，也保持相同分数下的稳定顺序。 */
    for (box_index = 1; box_index < candidate_count; ++box_index) {
        TaskCandidate current = candidates[box_index];
        int insert_index = box_index;
        while (insert_index > 0 &&
               candidates[insert_index - 1].quick_score > current.quick_score) {
            candidates[insert_index] = candidates[insert_index - 1];
            insert_index -= 1;
        }
        candidates[insert_index] = current;
    }
    return candidate_count;
}

static int build_scan_plan_for_target(
    const PlannerContext *ctx,
    int target_x,
    int target_y,
    MovePlan *best_move,
    int *best_score
) {
    uint8_t blocked[PLANNER_BITSET_BYTES];
    int start_cell = cell_index(ctx, ctx->input->car_x, ctx->input->car_y);
    int dir;
    int found = 0;

    build_motion_block_bits(ctx, ctx->input->wall_bits, ctx->input->bomb_bits, -1, -1, blocked);
    for (dir = 0; dir < 4; ++dir) {
        int stand_x = target_x - DIR_DX[dir];
        int stand_y = target_y - DIR_DY[dir];
        int stand_cell;
        MovePlan *move_plan = &g_workspace.move_plans[0];
        int score;
        if (!is_inside(ctx, stand_x, stand_y)) {
            continue;
        }
        stand_cell = cell_index(ctx, stand_x, stand_y);
        if (bitset_get(blocked, stand_cell)) {
            continue;
        }
        if (!free_motion_plan_bits(ctx, start_cell, stand_cell, ctx->input->car_dir, dir, 1, blocked, move_plan)) {
            continue;
        }
        score = move_plan->cost + SCAN_COST;
        if (!found || score < *best_score) {
            found = 1;
            *best_score = score;
            *best_move = *move_plan;
        }
    }
    return found;
}

static int select_scan_plan(const PlannerContext *ctx, const MatchingInfo *matching_info, PlannerPlan *plan) {
    int box_index;
    int destination_index;
    uint64_t best_partition_score = ULLONG_MAX;
    int best_score = INT_MAX / 4;
    MovePlan *best_move = &g_workspace.move_plans[1];
    MovePlan *candidate_move = &g_workspace.move_plans[2];
    int found = 0;
    int target_x = -1;
    int target_y = -1;
    int target_object_id = -1;

    for (destination_index = 0; destination_index < ctx->input->destination_count; ++destination_index) {
        const PlannerDestinationBox *destination = &ctx->input->destination_boxes[destination_index];
        int score;
        uint64_t partition_score;
        if (destination->known) {
            continue;
        }
        if (is_other_box_at(ctx->input, destination->x, destination->y, -1, 0, 0)) {
            continue;
        }
        if (!build_scan_plan_for_target(ctx, destination->x, destination->y, candidate_move, &score)) {
            continue;
        }
        partition_score = scan_partition_score_for_destination(ctx->input, matching_info, destination_index);
        if (!found || partition_score < best_partition_score || (partition_score == best_partition_score && score < best_score)) {
            best_partition_score = partition_score;
            best_score = score;
            *best_move = *candidate_move;
            target_x = destination->x;
            target_y = destination->y;
            target_object_id = destination->id;
            found = 1;
        }
    }

    for (box_index = 0; box_index < ctx->input->movable_count; ++box_index) {
        const PlannerMovableBox *box = &ctx->input->movable_boxes[box_index];
        int score;
        uint64_t partition_score;
        if (box->known) {
            continue;
        }
        if (!build_scan_plan_for_target(ctx, box->x, box->y, candidate_move, &score)) {
            continue;
        }
        partition_score = scan_partition_score_for_box(ctx->input, matching_info, box_index);
        if (!found || partition_score < best_partition_score || (partition_score == best_partition_score && score < best_score)) {
            best_partition_score = partition_score;
            best_score = score;
            *best_move = *candidate_move;
            target_x = box->x;
            target_y = box->y;
            target_object_id = box->id;
            found = 1;
        }
    }

    if (!found) {
        return 0;
    }
    plan->success = 1;
    plan->score = best_score;
    if (!append_steps(plan->steps, &plan->step_count, best_move->steps, best_move->step_count)) {
        plan->success = 0;
        return 0;
    }
    if (!append_step(plan->steps, &plan->step_count, PLANNER_STEP_SCAN, target_x, target_y, 0)) {
        plan->success = 0;
        return 0;
    }
    plan->steps[plan->step_count - 1].object_id = target_object_id;
    plan_set_message(plan, "已生成扫描计划");
    return 1;
}

int planner_plan_to_pose(
    const PlannerState *state,
    int target_x,
    int target_y,
    int target_dir,
    PlannerPlan *plan
) {
    uint8_t blocked[PLANNER_BITSET_BYTES];
    MovePlan *move_plan = &g_workspace.move_plans[1];
    int start_cell;
    int target_cell;

    if (plan == NULL) {
        return 0;
    }
    plan_clear(plan);
    memset(&g_job, 0, sizeof(g_job));
    push_search_reset(&g_workspace.push_search);
    if (target_dir < PLANNER_DIR_UP || target_dir > PLANNER_DIR_LEFT ||
        !planner_input_from_state(
            &g_job.input,
            g_job.movable_storage,
            g_job.destination_storage,
            g_job.forbidden_storage,
            state,
            plan) ||
        !context_init(&g_job.context, &g_job.input, plan) ||
        !is_inside(&g_job.context, target_x, target_y)) {
        return 0;
    }
    start_cell = cell_index(&g_job.context, g_job.input.car_x, g_job.input.car_y);
    target_cell = cell_index(&g_job.context, target_x, target_y);
    build_motion_block_bits(
        &g_job.context,
        g_job.input.wall_bits,
        g_job.input.bomb_bits,
        -1,
        -1,
        blocked);
    if (!free_motion_plan_bits(
            &g_job.context,
            start_cell,
            target_cell,
            g_job.input.car_dir,
            target_dir,
            1,
            blocked,
            move_plan)) {
        plan->error_code = 3;
        plan_set_message(plan, "目标位姿不可达");
        return 0;
    }
    plan->success = 1;
    plan->score = move_plan->cost;
    if (!append_steps(plan->steps, &plan->step_count, move_plan->steps, move_plan->step_count)) {
        plan_clear(plan);
        plan->error_code = 5;
        plan_set_message(plan, "目标位姿计划过长");
        return 0;
    }
    plan_set_message(plan, "已生成目标位姿计划");
    return 1;
}

int planner_job_begin(const PlannerState *state) {
    memset(&g_job, 0, sizeof(g_job));
    push_search_reset(&g_workspace.push_search);
    g_job.status = PLANNER_JOB_FAILED;
    plan_clear(&g_job.result);

    if (!planner_input_from_state(
            &g_job.input,
            g_job.movable_storage,
            g_job.destination_storage,
            g_job.forbidden_storage,
            state,
            &g_job.result)) {
        return g_job.status;
    }
    if (!context_init(&g_job.context, &g_job.input, &g_job.result)) {
        return g_job.status;
    }

    compute_dead_squares(&g_job.context);
    if (!build_matching_info(&g_job.input, &g_matching_info)) {
        g_job.result.error_code = 4;
        plan_set_message(&g_job.result, "ObservedState 不存在可行匹配");
        return g_job.status;
    }

    g_job.candidate_count = build_task_candidates(
        &g_job.context,
        &g_matching_info,
        g_job.candidates);
    g_job.candidate_index = 0;
    g_job.best_found = 0;
    g_job.best_score = INT_MAX / 4;
    g_job.status = PLANNER_JOB_RUNNING;
    return g_job.status;
}

int planner_job_step(uint32_t expansion_budget, PlannerPlan *plan) {
    if (plan == NULL) {
        return PLANNER_JOB_FAILED;
    }

    plan_clear(plan);
    if (g_job.status == PLANNER_JOB_DONE || g_job.status == PLANNER_JOB_FAILED) {
        memcpy(plan, &g_job.result, sizeof(*plan));
        return g_job.status;
    }
    if (g_job.status != PLANNER_JOB_RUNNING) {
        return PLANNER_JOB_IDLE;
    }
    if (expansion_budget == 0U) {
        expansion_budget = 1U;
    }

    if (g_job.candidate_index < g_job.candidate_count) {
        TaskCandidate *candidate = &g_job.candidates[g_job.candidate_index];
        const PlannerDestinationBox *destination =
            &g_job.context.input->destination_boxes[candidate->destination_index];
        int push_status = push_astar_step(
            &g_job.context,
            candidate->box_index,
            destination->x,
            destination->y,
            expansion_budget,
            &g_job.push_plan);

        if (push_status == 2) {
            return PLANNER_JOB_RUNNING;
        }
        if (push_status == 1) {
            int next_estimate = estimate_next_task_cost(
                &g_job.context,
                candidate->box_index,
                candidate->destination_index,
                g_job.push_plan.end_car_x,
                g_job.push_plan.end_car_y,
                g_job.push_plan.end_car_dir);
            int total_score = g_job.push_plan.cost + next_estimate / 2;
            if (!g_job.best_found || total_score < g_job.best_score) {
                g_job.best_found = 1;
                g_job.best_score = total_score;
                g_job.best_plan.score = g_job.push_plan.cost;
                g_job.best_plan.step_count = g_job.push_plan.step_count;
                memcpy(
                    g_job.best_plan.steps,
                    g_job.push_plan.steps,
                    sizeof(PlannerStep) * g_job.push_plan.step_count);
            }
        }
        g_job.candidate_index += 1;
        return PLANNER_JOB_RUNNING;
    }

    if (g_job.best_found) {
        g_job.result = g_job.best_plan;
        g_job.result.success = 1;
        plan_set_message(&g_job.result, "已生成推箱计划");
        g_job.status = PLANNER_JOB_DONE;
    } else if (select_scan_plan(&g_job.context, &g_matching_info, &g_job.result)) {
        g_job.status = PLANNER_JOB_DONE;
    } else {
        plan_clear(&g_job.result);
        g_job.result.error_code = 3;
        plan_set_message(&g_job.result, "当前没有可执行计划");
        g_job.status = PLANNER_JOB_FAILED;
    }

    memcpy(plan, &g_job.result, sizeof(*plan));
    return g_job.status;
}

int planner_plan(const PlannerState *state, PlannerPlan *plan) {
    int status;

    if (plan == NULL) {
        return 0;
    }
    status = planner_job_begin(state);
    if (status == PLANNER_JOB_FAILED) {
        memcpy(plan, &g_job.result, sizeof(*plan));
        return 0;
    }
    do {
        status = planner_job_step(UINT32_MAX, plan);
    } while (status == PLANNER_JOB_RUNNING);
    return status == PLANNER_JOB_DONE;
}
