//
// Created by lmtgy on 2026/4/15.
//

#include "planner_core.h"

//
// Created by lmtgy on 2026/4/15.
// 规划器主控模块实现
//

#include "planner_core.h"
#include <string.h>

// 引入算法头文件
#include "Cstar_map_core.h"
#include "path_smoother.h"
#include "planner_algo.h"
#include "planner_config.h"
#include "cmsis_os.h"

// 仅保留读取 DWT 计数值的宏，取消初始化相关的宏
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004

// 引入 CubeMX 生成的队列句柄
extern osMessageQueueId_t ReqQueueHandle;
extern osMessageQueueId_t RespQueueHandle;
// ==========================================
// 【修复核心】：在这里真正实例化全局公告板，分配内存！
// ==========================================
volatile GlobalPathState g_current_safe_path = {NULL, 0, 0.0f, 0};
// ==========================================
// 算法全局内存池 (BSS段)
// ==========================================
// 1. 实例化核心共享内存池 (A* 与 平滑器 分时复用)
// __attribute__((section(".ccmram"))) // 若有 CCMRAM 可解开此注释
PlannerMemoryPool g_mem_pool;

// 2. 内部扭转与状态记忆缓冲
static Point2D s_prev_path[MAX_PATH_LEN];
static int last_raw_path_len = 0;
static Point2D s_path_buffer_A[MAX_PATH_LEN]; // 内部扭转管道

// 3. 【A* -> 运动控制】的乒乓双缓冲输出
static Point2D s_path_buffer_C[2][MAX_PATH_LEN];
static uint8_t out_ping_pong_idx = 0;

// 4. 【SLAM -> A*】的地图乒乓双缓冲实体与指针
static uint8_t s_map_ping[MAX_MAP_BYTES];
static uint8_t s_map_pong[MAX_MAP_BYTES];
uint8_t* volatile s_write_map_ptr = s_map_ping; // 暴露给 SLAM 写入
uint8_t* volatile s_read_map_ptr  = s_map_pong; // A* 内部读取用

// 5. 紧急打断标志
volatile bool g_abort_astar = false;

// ==========================================
// FreeRTOS 任务主循环
// ==========================================
void StartPlannerTask(void *argument) {
    PlannerReqMsg req;

    for(;;) {
        // 1. 阻塞等待规划请求 (正常规划 或 SLAM发起的紧急重算)
        osMessageQueueGet(ReqQueueHandle, &req, NULL, osWaitForever);

        // 2. 【队列清洗】：抽干积压的请求，永远只处理最新的一条
        while (osMessageQueueGetCount(ReqQueueHandle) > 0) {
            osMessageQueueGet(ReqQueueHandle, &req, NULL, 0);
        }

        // 3. 清除打断标志，准备开始新的轮回
        g_abort_astar = false;

        // 4. 【地图交接】：挂起中断 1微秒，瞬间交换读写指针
        taskENTER_CRITICAL();
        uint8_t* temp = s_write_map_ptr;
        s_write_map_ptr = s_read_map_ptr;
        s_read_map_ptr = temp;
        taskEXIT_CRITICAL();

        // 5. 【底图同步】：防止 SLAM 在下一帧丢失静态环境
        memcpy((void*)s_write_map_ptr, (void*)s_read_map_ptr, MAX_MAP_BYTES);

        // 绑定最新地图给核心算法
        g_map.grid = (uint8_t*)s_read_map_ptr;
        g_map.grid_size = MAX_GRID_SIZE;
        g_map.res = PLANNER_MAP_RES;

        if (g_map.grid == NULL || g_map.res <= 0.0f) {
            continue;
        }

        // 获取当前周期的输出缓冲区指针
        Point2D* current_out_buf = s_path_buffer_C[out_ping_pong_idx];
        int final_path_len = 0;

        // 打下开始时间戳
        uint32_t start_cycles = DWT_CYCCNT;

        if (req.cmd_type == PLANNER_REQ_NEW_GOAL || req.cmd_type == PLANNER_REQ_REPLAN) {
            Point2D start = {req.start_x, req.start_y};
            Point2D goal  = {req.target_x, req.target_y};

            // 【阶段 1】：A* 寻路。注意内部会高频检测 g_abort_astar
            int raw_path_len = astar_plan(&g_map, start, goal,
                                          last_raw_path_len ? s_prev_path : NULL,
                                          last_raw_path_len, req.is_return, s_path_buffer_A);

            if (raw_path_len > 0) {
                // 备份当前成功生成的初步路径，供下次做亲和力参考
                memcpy(s_prev_path, s_path_buffer_A, raw_path_len * sizeof(Point2D));
                last_raw_path_len = raw_path_len;

                // 【阶段 2】：A* 结束！g_mem_pool 被移交给平滑器
                int smoothed_len = double_smooth_path(&g_map, s_path_buffer_A, raw_path_len, SAFE_SMOOTH_RAD, req.is_return, s_path_buffer_A);

                // 生成最终的安全倒角路径，并直接写入当前乒乓输出缓冲片
                final_path_len = generate_safe_corner_path(&g_map, s_path_buffer_A, smoothed_len, 0.5f, 0.05f, req.is_return, current_out_buf);
            } else {
                // 规划失败或被 SLAM 强行打断 (Early Abort)
                last_raw_path_len = 0;
            }
        }

        // 打下结束时间戳，计算耗时
        uint32_t end_cycles = DWT_CYCCNT;
        float elapsed_ms = (float)(end_cycles - start_cycles) / 180000.0f; // 180MHz 主频

        // 如果中途没有被 SLAM 打断，且生成了有效路径，则发送结果
        if (!g_abort_astar && final_path_len > 0) {
            // 【修改点 1】：不再塞入队列，而是更新全局公告板
            // sequence 快照协议：写前置奇数，写后置偶数，读者仅接受前后相等且为偶数的快照
            taskENTER_CRITICAL();
            g_current_safe_path.sequence++;
            g_current_safe_path.path_ptr = current_out_buf;
            g_current_safe_path.path_len = final_path_len;
            g_current_safe_path.exec_time_ms = elapsed_ms;
            g_current_safe_path.sequence++;
            taskEXIT_CRITICAL();

            // 【修改点 2】：广播事件，唤醒所有正在等待该事件的线程
            osEventFlagsSet(PathEventHandle, EVENT_PATH_UPDATED);

            // 翻转输出缓冲片，下次循环使用另一片，保护正被其他线程读取的数据
            out_ping_pong_idx ^= 1;
        }
    }
}

bool Get_Global_Path_Snapshot(GlobalPathSnapshot* out_snapshot) {
    uint32_t seq_begin;
    uint32_t seq_end;

    if (out_snapshot == NULL) {
        return false;
    }

    do {
        seq_begin = g_current_safe_path.sequence;
        if ((seq_begin & 1U) != 0U) {
            continue;
        }

        out_snapshot->path_ptr = g_current_safe_path.path_ptr;
        out_snapshot->path_len = g_current_safe_path.path_len;
        out_snapshot->exec_time_ms = g_current_safe_path.exec_time_ms;
        out_snapshot->sequence = seq_begin;

        seq_end = g_current_safe_path.sequence;
    } while (seq_begin != seq_end || (seq_end & 1U) != 0U);

    return (out_snapshot->path_ptr != NULL && out_snapshot->path_len > 0);
}
