//
// Created by lmtgy on 2026/4/15.
//

#ifndef NOETICMAZE_PLANNER_CORE_H
#define NOETICMAZE_PLANNER_CORE_H

#ifndef CSTAR_PLANNER_CORE_H
#define CSTAR_PLANNER_CORE_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>

// 引入配置层，获取 MAX_MAP_BYTES 等宏
#include "planner_config.h"

// ==========================================
// 线程间通信：控制流队列结构体 (极简，用于极速入队)
// ==========================================
typedef enum {
    PLANNER_REQ_NEW_GOAL = 1,
    PLANNER_REQ_REPLAN = 2
} PlannerCmdType;

typedef enum {
    PLANNER_REPLAN_REASON_NONE = 0,
    PLANNER_REPLAN_REASON_GOAL_CHANGED = 1,
    PLANNER_REPLAN_REASON_NO_PATH = 2,
    PLANNER_REPLAN_REASON_PATH_BLOCKED = 3,
    PLANNER_REPLAN_REASON_START_DRIFT = 4
} PlannerReplanReason;

typedef struct {
    uint8_t cmd_type;               // [`PlannerCmdType`](App/Planner/Inc/planner_core.h)
    bool is_return;                 // true: 返程模式 (未知区域视为障碍) | false: 探索模式
    uint8_t reason;                 // [`PlannerReplanReason`](App/Planner/Inc/planner_core.h)
    uint32_t map_version;           // 本次请求对应的 planner 地图版本
    float target_x;                 // 物理目标 X 坐标 (米)
    float target_y;                 // 物理目标 Y 坐标 (米)
    float start_x;                  // 当前小车 X 坐标
    float start_y;                  // 当前小车 Y 坐标
} PlannerReqMsg;

typedef struct {
    void* path_ptr;     // 指向当前可用输出缓冲片 (Point2D数组) 的指针
    int path_len;       // 生成的路径总节点数
    float exec_time_ms; // 算法执行耗时
} PlannerRespMsg;

// ==========================================
// 线程间通信：广播流全局状态 (多消费者安全读取)
// ==========================================
typedef struct {
    Point2D* path_ptr;      // 指向最新有效乒乓缓冲片的指针
    int path_len;           // 最新路径的长度
    float exec_time_ms;     // 算法耗时
    uint32_t sequence;      // 更新序列号 (偶数=稳定快照, 奇数=写入中)
} GlobalPathState;

typedef struct {
    Point2D* path_ptr;
    int path_len;
    float exec_time_ms;
    uint32_t sequence;
} GlobalPathSnapshot;

// 暴露全局公告板
extern volatile GlobalPathState g_current_safe_path;

bool Get_Global_Path_Snapshot(GlobalPathSnapshot* out_snapshot);

// 暴露事件标志组句柄 (用于唤醒所有沉睡的读取线程)
extern osEventFlagsId_t PathEventHandle;

// 定义路径更新事件的 Bit 掩码 (第0位)
#define EVENT_PATH_UPDATED  0x00000001U

// ==========================================
// 暴露给 SLAM 线程的数据流与异常流接口
// ==========================================
// 1. SLAM 写地图专用指针 (SLAM 线程只能向这个指针写入降采样后的地图数据)
extern uint8_t* volatile s_write_map_ptr;

// 2. 全局打断标志 (SLAM 发现原路径被堵死时，立刻将其置为 true)
extern volatile bool g_abort_astar;

// ==========================================
// 任务入口
// ==========================================
void StartPlannerTask(void *argument);

#endif //CSTAR_PLANNER_CORE_H
#endif //NOETICMAZE_PLANNER_CORE_H
