#ifndef NOETICMAZE_DEBUG_REPORT_H
#define NOETICMAZE_DEBUG_REPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "bt_protocol.h"

typedef enum {
    DBG_STAGE_SLAM_IMU_NOT_READY = 1,
    DBG_STAGE_SLAM_ICP_INITIALIZED = 2,
    DBG_STAGE_SLAM_MAP_SHARED = 3,
    DBG_STAGE_SLAM_REPLAN_REQUESTED = 4,
    DBG_STAGE_SLAM_STACK_WATERMARK = 5,
} DebugSlamStage_t;

typedef enum {
    DBG_STAGE_PLANNER_REQUEST_RECEIVED = 1,
    DBG_STAGE_PLANNER_ASTAR_SUCCESS = 2,
    DBG_STAGE_PLANNER_ASTAR_FAILED = 3,
    DBG_STAGE_PLANNER_ABORTED = 4,
    DBG_STAGE_PLANNER_PATH_PUBLISHED = 5,
    DBG_STAGE_PLANNER_STACK_WATERMARK = 6,
} DebugPlannerStage_t;

typedef enum {
    DBG_STAGE_MOTION_PATH_LOADED = 1,
    DBG_STAGE_MOTION_PROJECTION_TOO_FAR = 2,
    DBG_STAGE_MOTION_PATH_RESET = 3,
    DBG_STAGE_MOTION_TRIMMED_PUBLISHED = 4,
    DBG_STAGE_MOTION_STACK_WATERMARK = 5,
} DebugMotionStage_t;

typedef enum {
    DBG_STAGE_UART_HEARTBEAT = 1,
    DBG_STAGE_UART_DMA_STUCK = 2,
    DBG_STAGE_UART_DROP_COUNT = 3,
} DebugUartStage_t;

typedef struct {
    uint32_t tick;
    uint8_t module;
    uint8_t stage;
    int16_t code;
    float a;
    float b;
    float c;
    float d;
} DebugEvent_t;

void Debug_Post(uint8_t module,
                uint8_t stage,
                int16_t code,
                float a,
                float b,
                float c,
                float d);
bool Debug_TryPop(DebugEvent_t *out_event);
uint32_t Debug_GetDropCount(void);

#endif //NOETICMAZE_DEBUG_REPORT_H
