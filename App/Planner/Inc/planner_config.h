//
// Created by lmtgy on 2026/4/15.
// 1. 公共配置层

#ifndef STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H
#define STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H

// ==========================================
// 算法配置层
// ==========================================
#define SERIAL_PORT "COM19"
#define BAUD_RATE 1500000

#define MAP_RES 0.1f
#define ASTAR_GOAL_TOL_GRIDS 1.5f
#define PHYSICAL_RADIUS 0.10f
#define INFLATE_L1_RAD 0.15f
#define INFLATE_L1_PENALTY 100.0f
#define INFLATE_L2_RAD 0.22f
#define INFLATE_L2_PENALTY 15.0f
#define INFLATE_L3_RAD 0.28f
#define INFLATE_L3_PENALTY 3.0f
#define SAFE_SMOOTH_RAD 0.15f




// 新增：历史路径亲和力配置
#define PATH_AFFINITY_DISCOUNT 0.6f
#define PATH_AFFINITY_RADIUS_GRIDS 3

// ==========================================
// 【嵌入式预备修改】：静态内存池宏定义与分配
// ==========================================
#define MAX_GRID_SIZE 50                           // 你的地图是5m/0.1=50
#define MAX_CELLS (MAX_GRID_SIZE * MAX_GRID_SIZE)  // 2500
#define MAX_MAP_BYTES ((MAX_CELLS + 1) / 2)        // 1250 字节
#define MAX_PQ_SIZE 2048                           // 优先队列最大容量
#define MAX_PATH_LEN 400                           // 路径最大节点数

// 【核心优化修改 1】：新增用于临时局部计算的短数组长度宏
#define MAX_TEMP_LEN 100                           // 局部插值只需很短的数组


// ==========================================
// 极致压缩：地图与代价定点数宏定义
// ==========================================
#define COST_SCALE 10                  // 浮点数放大倍数
#define STEP_STRAIGHT 10               // 直行代价 (1.0 * 10)
#define STEP_DIAG 14                   // 斜行代价 (1.414 * 10)
#define MAX_COST_INF 65535             // uint16_t 的无穷大

#define INFLATE_L1_PENALTY_INT 1000    // 100.0 * 10
#define INFLATE_L2_PENALTY_INT 150     // 15.0 * 10
#define INFLATE_L3_PENALTY_INT 30      // 3.0 * 10
#define AFFINITY_DISCOUNT_INT 6        // 0.6 * 10

// 3. 路径计算静态内存 (避免局部爆栈，全部放进 BSS 段)
typedef struct { float x; float y; } Point2D;


// ==========================================
// 极致优化：分时共享内存池 (Union)
// ==========================================
typedef union {
    // 阶段 1：A* 寻路时使用 (例如 2048 * 4 = 8192 Bytes)
    uint32_t pq_data[MAX_PQ_SIZE];

    // 阶段 2：路径平滑时使用 (例如 400*8 + 100*8 + 100*8 = 4800 Bytes)
    struct {
        Point2D path_buffer_B[MAX_PATH_LEN];
        Point2D temp_bezier[MAX_TEMP_LEN];
        Point2D temp_line[MAX_TEMP_LEN];
    } smooth;
} PlannerMemoryPool;

// 声明外部全局变量（告知所有引用此头文件的 .c 文件）
extern PlannerMemoryPool g_mem_pool;

#endif //STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H