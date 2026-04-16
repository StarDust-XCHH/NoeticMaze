//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\lidar\Inc\lidar.h

#ifndef NOETICMAZE_LIDAR_H
#define NOETICMAZE_LIDAR_H

#include "main.h"
#include "arm_math.h"
#include "cmsis_os2.h"

// 宏定义参数设置
#define LIDAR_FIFO_SIZE        2048  // 适当减小原始 FIFO，因为 RTOS 响应更快，节省 RAM

// --- 新增：极速 FIFO 掩码与断言 ---
#if (LIDAR_FIFO_SIZE & (LIDAR_FIFO_SIZE - 1)) != 0
    #error "LIDAR_FIFO_SIZE 必须是 2 的 n 次方才能使用 MASK 优化！"
#endif
#define LIDAR_FIFO_MASK        (LIDAR_FIFO_SIZE - 1)
// ---------------------------------

#define LIDAR_FRAME_LEN        84
#define LIDAR_MAP_SIZE         360
#define LIDAR_POINTS_PER_FRAME 40
#define LIDAR_ANGLE_STEP       0.705f

#define MAP_POOL_SIZE          3     // 缓冲池大小，对应你的 LidarQueue Size

// 运动学跳帧逻辑
#define KINEMATIC_FILTER_JUMPING_FORWARD 10
#define KINEMATIC_FILTER_JUMPING_ROTATE 1
#define TURN_THRESHOLD  0.2f // 角速度转弯判定阈值 (根据实际单位调整)

#define LIDAR_MIN_RANGE    30   // 最小有效距离 30mm (避开近处结构件遮挡)
#define LIDAR_MAX_RANGE    10000  // 最大有效距离 8000mm (根据实际需求缩短，减少远端噪点)
// 雷达完整一圈点云结构体
typedef struct {
    uint16_t distance[LIDAR_MAP_SIZE];
    uint32_t timestamp;              // 记录帧产生的时间戳
    uint32_t sweep_count;            // 圈数

    float    scan_time;              // 【新增】：本圈真实物理耗时 (秒)
    uint32_t point_count;  // 【新增】：存储本圈实际收到的原始点数

} LidarMap_t;


// ==========================================
// 【新增】帧缓存结构体 (用于两帧插值)
// ==========================================
typedef struct {
    uint8_t  is_valid;           // 缓存有效标志位
    float    start_angle;        // 上一帧的起始角度
    uint16_t distances[LIDAR_POINTS_PER_FRAME]; // 上一帧的40个原始距离
} Lidar_Cache_t;

// 雷达控制句柄
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_dma_buf[1024];        // DMA 直接接收缓冲区

    // 原始字节 FIFO
    uint8_t  fifo[LIDAR_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;

    // [新增：配合 Circular DMA 使用，追踪上一次读取的位置]
    uint16_t last_dma_pos;

    // 解析状态

    // --- 删除原来的 last_start_angle，替换为缓存结构 ---
    Lidar_Cache_t cache;
    // ------------------------------------------------
    uint32_t  sweep_count;


    // 【新增】：用于精准计时的点数累加器
    uint32_t  points_since_last_sweep;

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