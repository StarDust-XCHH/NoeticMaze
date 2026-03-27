//
// Created by lmtgy on 2026/3/27.
//

#ifndef NOETICMAZE_LIDAR_H
#define NOETICMAZE_LIDAR_H

#include "main.h"
#include "arm_math.h"
#include "cmsis_os2.h"

// 宏定义参数设置
#define LIDAR_FIFO_SIZE        2048  // 适当减小原始 FIFO，因为 RTOS 响应更快，节省 RAM
#define LIDAR_FRAME_LEN        84
#define LIDAR_MAP_SIZE         360
#define LIDAR_POINTS_PER_FRAME 40
#define LIDAR_ANGLE_STEP       0.705f

#define MAP_POOL_SIZE          3     // 缓冲池大小，对应你的 LidarQueue Size

// 运动学跳帧逻辑
#define KINEMATIC_FILTER_JUMPING_FORWARD 10
#define KINEMATIC_FILTER_JUMPING_ROTATE 1


// 雷达完整一圈点云结构体
typedef struct {
    uint16_t distance[LIDAR_MAP_SIZE];
    uint32_t timestamp;              // 记录帧产生的时间戳
    uint32_t sweep_count;            // 圈数
} LidarMap_t;

// 雷达控制句柄
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_dma_buf[128];        // DMA 直接接收缓冲区

    // 原始字节 FIFO
    uint8_t  fifo[LIDAR_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;

    // 解析状态
    float32_t last_start_angle;
    uint32_t  sweep_count;

    // 当前正在写入的雷达地图指针
    LidarMap_t *current_map;
} Lidar_HandleTypeDef;

// 外部引用声明
extern Lidar_HandleTypeDef hlidar1;
extern osMessageQueueId_t LidarQueueHandle;      // CubeMX 生成的可用数据队列
extern osMessageQueueId_t LidarFreeQueueHandle;  // 需要你手动或在代码中创建的空闲内存池队列
extern osThreadId_t LidarRouteTaskHandle;        // Task 3 的句柄

// 接口函数
void Lidar_Init(Lidar_HandleTypeDef *hlidar, UART_HandleTypeDef *huart);
void Lidar_Start(Lidar_HandleTypeDef *hlidar);
void Lidar_ParseDMA_ISR(Lidar_HandleTypeDef *hlidar, uint16_t size);

// Task 3 声明
void StartLidarRouteTask(void *argument);

#endif //NOETICMAZE_LIDAR_H