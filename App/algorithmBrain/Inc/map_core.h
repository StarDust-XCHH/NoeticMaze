//
// Created by lmtgy on 2026/4/9.
//

#ifndef NOETICMAZE_MAP_CORE_H
#define NOETICMAZE_MAP_CORE_H
#include <stdint.h>
extern uint32_t diff_cnt;
int world_to_grid(float world_val, int *grid_idx);
// 定义 2-bit 的三种明确状态
#define MAP_UNKNOWN  0x00  // 二进制 00：雷达还没扫到的地方
#define MAP_FREE     0x01  // 二进制 01：安全可通行区域
#define MAP_OCCUPIED 0x02  // 二进制 10：障碍物/墙壁
#define MAX_MAP_DIFF 1000  // 每帧最多发送 1000 个变化格子 (总计 3KB)
#define MAP_OFFSET 0.0f    // 根据实际情况调整偏移
#define MAP_RES 0.02f
#define MAP_SIZE 250        // 5m / 0.02m

void trace_ray_bresenham_diff(int x0, int y0, int x1, int y1);
extern uint8_t diff_payload[MAX_MAP_DIFF * 3];
extern uint8_t global_map[15625];


/**
 * @brief  读取栅格状态 (2-bit 压缩版)
 */
static inline uint8_t GetMapState(int x, int y) {
    if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return MAP_UNKNOWN;

    uint32_t flat_idx = y * MAP_SIZE + x;
    uint32_t byte_idx = flat_idx >> 2;
    uint8_t bit_shift = (flat_idx & 0x03) << 1;

    // 将对应的 2 bit 移回最低位，并用 0x03 掩码提取
    return (global_map[byte_idx] >> bit_shift) & 0x03;
}


/**
 * @brief  写入栅格状态 (2-bit 压缩版)
 * @param  x      地图 X 索引 (0~249)
 * @param  y      地图 Y 索引 (0~249)
 * @param  state  MAP_UNKNOWN / MAP_FREE / MAP_OCCUPIED
 */
static inline void SetMapState(int x, int y, uint8_t state) {
    // 1. 安全边界检查
    if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return;

    // 2. 将 2D 坐标展平为 1D 线性索引 (0 ~ 62499)
    uint32_t flat_idx = y * MAP_SIZE + x;

    // 3. 计算这个栅格落在哪一个 Byte 里 (相当于 flat_idx / 4)
    uint32_t byte_idx = flat_idx >> 2;

    // 4. 计算这个栅格在这个 Byte 中的偏移位数 (0, 2, 4, 6)
    // flat_idx & 0x03 相当于 flat_idx % 4
    uint8_t bit_shift = (flat_idx & 0x03) << 1;

    // 5. 核心位运算：先清零目标位置的 2 个 bit，然后将新状态或 (OR) 进去
    // ~(0x03 << bit_shift) 会生成一个类似 11110011 的遮罩来掏空旧数据
    global_map[byte_idx] = (global_map[byte_idx] & ~(0x03 << bit_shift)) | ((state & 0x03) << bit_shift);
}



/**
 * @brief 尝试更新地图并记录增量
 * @return 1: 发生变化并记录成功; 0: 无变化或缓冲区满
 */
static inline int UpdateAndRecordMap(int x, int y, uint8_t new_state) {
    if (x >= MAP_SIZE || y >= MAP_SIZE) return 0;

    uint8_t old_state = GetMapState(x, y);

#if REJECT_DYNAMIC_RELOADED_COSTMAP
    // --- 修改开始：永久保留障碍物逻辑 ---
    // 如果之前已经是障碍物，且现在想把它标为空闲，直接拦截！
    if (old_state == MAP_OCCUPIED && new_state == MAP_FREE) {
        return 0;
    }
    // --- 修改结束 ---
#endif
    // 1. 如果状态没有发生改变，直接过滤掉！(这就是节省 90% 带宽的秘诀)
    if (old_state == new_state) return 0;

    // 2. 如果状态变了，检查增量缓冲区是否还有空位
    if (diff_cnt < MAX_MAP_DIFF) {
        // 只有成功放进缓冲区准备发给上位机了，才真正更新 STM32 的本地地图
        SetMapState(x, y, new_state);

        diff_payload[diff_cnt * 3 + 0] = x;
        diff_payload[diff_cnt * 3 + 1] = y;
        diff_payload[diff_cnt * 3 + 2] = new_state;
        diff_cnt++;
        return 1;
    }

    // 3. 如果缓冲区满了，就不更新本地地图。
    // 这样下一帧雷达扫过来时，发现还是旧状态，会再次触发更新机制。天然防丢包！
    return 0;
}


#endif //NOETICMAZE_MAP_CORE_H