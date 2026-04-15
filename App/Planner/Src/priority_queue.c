//
// Created by lmtgy on 2026/4/15.
// 2. 数据结构与地图层

#include "priority_queue.h"
#include "planner_config.h"

// 1. 【删除这行】：原来独占的静态内存，现在不需要了
// static uint32_t s_pq_data[MAX_PQ_SIZE];

// 2. 修改初始化函数
void pq_init(PriorityQueue* pq) {
    // 将指针指向共享内存池的阶段 1 区域
    pq->data = g_mem_pool.pq_data;
    pq->size = 0;
    pq->capacity = MAX_PQ_SIZE;
}

// ---------------- 最小堆实现 (极限优化版) ----------------

void pq_push(PriorityQueue* pq, uint16_t idx, uint16_t f) {
    if (pq->size >= pq->capacity) return;

    // 【核心优化 1】：打包数据。f 放高 16 位，idx 放低 16 位
    uint32_t element = ((uint32_t)f << 16) | idx;

    int i = pq->size++;
    while (i > 0) {
        // 【核心优化 2】：位移代替除法 (单周期指令)
        int p = (i - 1) >> 1;

        uint32_t parent_val = pq->data[p];
        // 这里的 <= 直接比较了 32 位数据。
        // 因为 f 在高 16 位，所以优先比较 f；
        // 如果 f 相同，会自动比较低 16 位的 idx 作为 Tie-breaker。
        if (parent_val <= element) break;

        pq->data[i] = parent_val;
        i = p;
    }
    pq->data[i] = element;
}

bool pq_pop(PriorityQueue* pq, uint16_t* idx, uint16_t* f) {
    if (pq->size == 0) return false;

    // 【核心优化 3】：解包数据
    uint32_t top = pq->data[0];
    *idx = (uint16_t)(top & 0xFFFF);
    *f   = (uint16_t)(top >> 16);

    pq->size--;
    if (pq->size == 0) return true;

    uint32_t temp = pq->data[pq->size];
    int i = 0;
    int size = pq->size;

    // 移除多余的容量计算，精简循环内部逻辑
    while (1) {
        // 【核心优化 4】：位移代替乘法
        int left = (i << 1) + 1;
        if (left >= size) break; // 没有左孩子，说明是叶子节点

        int right = left + 1;
        int min_child = left;
        uint32_t min_val = pq->data[left];

        // 寻找左右孩子中较小的一个（同样利用 32 位全比较）
        if (right < size) {
            uint32_t right_val = pq->data[right];
            if (right_val < min_val) {
                min_child = right;
                min_val = right_val;
            }
        }

        if (temp <= min_val) break;

        pq->data[i] = min_val;
        i = min_child;
    }
    pq->data[i] = temp;
    return true;
}