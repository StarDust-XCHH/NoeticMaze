//
// Created by lmtgy on 2026/4/15.
// 2. 数据结构与地图层

#ifndef STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H
#define STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H
#include <stdbool.h>
#include <stdint.h>

#include "planner_config.h"

typedef struct {
    uint32_t* data;
    int size;
    int capacity;
    bool overflowed;
} PriorityQueue;

void pq_init(PriorityQueue* pq);
bool pq_push(PriorityQueue* pq, uint16_t idx, uint16_t f);
bool pq_pop(PriorityQueue* pq, uint16_t* idx, uint16_t* f);

#endif //STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H
