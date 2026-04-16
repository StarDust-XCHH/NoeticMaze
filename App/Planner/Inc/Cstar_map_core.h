//
// Created by lmtgy on 2026/4/15.
// 2. 数据结构与地图层

#ifndef STM32_PLANNER_PC_TEST_MAP_CORE_H
#define STM32_PLANNER_PC_TEST_MAP_CORE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t* grid;
    float res;
    int grid_size;
} ServerMap;

extern ServerMap g_map;

// 优化 1：无分支 (Branchless) 读取网格状态
static inline uint8_t get_grid_val(ServerMap* map, int x, int y) {
    int idx = x * map->grid_size + y;
    uint8_t shift = (idx & 1) << 2; // 如果是奇数移4位，偶数移0位
    return (map->grid[idx >> 1] >> shift) & 0x03;
}

// 优化 2：无分支掩码写入，精准保留亲和力等其他位
static inline void set_grid_val(ServerMap* map, int x, int y, uint8_t val) {
    int idx = x * map->grid_size + y;
    int byte_idx = idx >> 1;
    uint8_t shift = (idx & 1) << 2;
    uint8_t mask = ~(0x03 << shift); // 偶数为0xFC，奇数为0xCF

    // 一次性清除旧状态，并写入新状态，不影响周边Bit
    map->grid[byte_idx] = (map->grid[byte_idx] & mask) | ((val & 0x03) << shift);
}

// 优化 3：无分支获取亲和力
static inline bool has_affinity(ServerMap* map, int idx) {
    uint8_t shift = ((idx & 1) << 2) + 2; // 偶数取Bit2，奇数取Bit6
    return (map->grid[idx >> 1] >> shift) & 0x01;
}

static inline void set_affinity(ServerMap* map, int idx) {
    uint8_t shift = ((idx & 1) << 2) + 2;
    map->grid[idx >> 1] |= (1 << shift);
}

// 优化 4：利用 32-bit (Word) 宽度进行内存批量擦除，提升4倍速度
static inline void clear_all_affinity(ServerMap* map) {
    int bytes = (map->grid_size * map->grid_size + 1) / 2;
    uint32_t* grid32 = (uint32_t*)map->grid;
    int words = bytes >> 2; // 转换为 32-bit 的长度

    for(int i = 0; i < words; i++) {
        grid32[i] &= 0x33333333; // Cortex-M 处理 32bit 和 8bit 周期相同
    }

    // 处理末尾不对齐的剩余字节
    for(int i = words << 2; i < bytes; i++) {
        map->grid[i] &= 0x33;
    }
}

/**
 * @brief 将 2cm SLAM 全局图重建为 10cm planner 基础图并写入 planner 写缓冲。
 * @param planner_map_out 输出 planner 基础图视图；可为 NULL。
 */
void process_map_update(ServerMap* planner_map_out);

/**
 * @brief 基于 planner 基础图构建“路径阻断判定专用”的硬膨胀视图。
 * @param src_map 基础 planner 图。
 * @param dst_map 目标 inflated 图；可与基础图不同。
 */
void build_blocked_view_from_base(const ServerMap* src_map, ServerMap* dst_map);

#endif //STM32_PLANNER_PC_TEST_MAP_CORE_H
