//
// Created by lmtgy on 2026/4/10.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\algorithmBrain\Inc\map_core_write.h

#ifndef NOETICMAZE_MAP_CORE_WRITE_H
#define NOETICMAZE_MAP_CORE_WRITE_H

#include "map_core.h" // 包含基础定义和只读接口
#include "FreeRTOS.h"
#include "task.h"     // <==== 【新增这一行】提供 taskDISABLE_INTERRUPTS 的定义
// =========================================================================
// 🛡️ 编译期断言保护 (Compile-time Protection)
// 如果包含此文件的 .c 没有声明自己是 SLAM 线程，直接引发编译报错！
// =========================================================================
#ifndef I_AM_SLAM_THREAD
#error "FATAL ARCHITECTURE ERROR: You are trying to include Map Write API outside the SLAM thread! This violates the SWMR architecture."
#endif

// 获取 SLAM 任务句柄 (根据你 CubeMX 生成的实际名字修改，通常在 main.c 或 freertos.c 中)
extern osThreadId_t AlgorithmBrainHandle;

extern uint32_t diff_cnt;
extern uint8_t diff_payload[MAX_MAP_DIFF * 3];

/**
 * @brief  写入栅格状态 (2-bit 压缩版) - 仅限 SLAM 线程
 */
static inline void SetMapState(int x, int y, uint8_t state) {
    // =========================================================================
    // 🛡️ 运行期断言保护 (Runtime RTOS Protection)
    // 即使有人恶意定义了宏绕过编译检查，在运行时只要不是 SLAM 线程调用，立刻死机报错！
    // =========================================================================
    // 注意：如果在中断里调用 osThreadGetId 会返回 NULL，也会触发断言，非常安全
    configASSERT(osThreadGetId() == AlgorithmBrainHandle);

    if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return;

    uint32_t flat_idx = y * MAP_SIZE + x;
    uint32_t byte_idx = flat_idx >> 2;
    uint8_t bit_shift = (flat_idx & 0x03) << 1;

    global_map[byte_idx] = (global_map[byte_idx] & ~(0x03 << bit_shift)) | ((state & 0x03) << bit_shift);
}

/**
 * @brief 尝试更新地图并记录增量 - 仅限 SLAM 线程
 */
static inline int UpdateAndRecordMap(int x, int y, uint8_t new_state) {
    // 同样继承 SetMapState 的断言保护
    if (x >= MAP_SIZE || y >= MAP_SIZE) return 0;

    uint8_t old_state = GetMapState(x, y);

#if REJECT_DYNAMIC_RELOADED_COSTMAP
    if (old_state == MAP_OCCUPIED && new_state == MAP_FREE) {
        return 0;
    }
#endif

    if (old_state == new_state) return 0;

    if (diff_cnt < MAX_MAP_DIFF) {
        SetMapState(x, y, new_state); // 这里面有线程断言

        diff_payload[diff_cnt * 3 + 0] = x;
        diff_payload[diff_cnt * 3 + 1] = y;
        diff_payload[diff_cnt * 3 + 2] = new_state;
        diff_cnt++;
        return 1;
    }
    return 0;
}

// 射线追踪也移到这里，因为它会写入地图
void trace_ray_bresenham_diff(int x0, int y0, int x1, int y1);

#endif //NOETICMAZE_MAP_CORE_WRITE_H