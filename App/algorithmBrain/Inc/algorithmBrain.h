//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\algorithmBrain\Inc\algorithmBrain.h

#ifndef NOETICMAZE_ALGORITHMBRAIN_H
#define NOETICMAZE_ALGORITHMBRAIN_H

#include "icp_core.h"

// --- 修改为 extern 声明 ---
extern Point ref_scan[SCAN_SIZE];
extern Point ref_normals[SCAN_SIZE];
extern uint8_t ref_mask[SCAN_SIZE];

// 1. 定义 ICP 帧数据传输结构体 (用于跨线程传递)
typedef struct {
    volatile uint8_t is_busy; // 【新增】：0代表空闲，1代表正在被ICP算法使用
    float odom_x;
    float odom_y;
    float odom_theta;
    float linear_v;
    float angular_w;
    uint8_t is_init;
    uint8_t update_map;
    uint32_t num_points;
    Point scan_data[SCAN_SIZE]; // 巨大的点云数据直接放在静态区
} IcpFrame_t;

/**
 * @brief 将雷达的顺时针极坐标数据转换为右手直角坐标系点云
 * @param distances 雷达原始距离数组 (假设单位为毫米)
 * @param cloud     输出的 2D 点云数组 (单位转换为米)
 */
void Transform_Lidar_To_Cartesian(uint16_t *distances, Point *cloud);


/**
 * @brief 尝试初始化 ICP 参考帧
 * @param frame 前端传入的雷达数据帧
 * @return uint8_t 1: 系统已就绪(已初始化或刚完成初始化)，0: 数据不达标，继续等待
 */
uint8_t Try_Initialize_ICP(IcpFrame_t *frame);

#endif //NOETICMAZE_ALGORITHMBRAIN_H