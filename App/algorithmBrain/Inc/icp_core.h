//
// Created by lmtgy on 2026/4/9.
//

#ifndef NOETICMAZE_ICP_CORE_H
#define NOETICMAZE_ICP_CORE_H


#include <stdint.h>

#define SCAN_SIZE 360
#define MAX_ITER 10
#define MAX_DIST 0.20f
#define MATCH_DIST_SQ 0.16f  // 0.4m^2
#define SEARCH_WINDOW 30


/* 新增：ICP 参考帧更新频率 (每隔几帧作为一次 KeyFrame) */
// 如果不进行运动检测，建议为5，运动检测建议直行10，转弯2（待定）


#pragma pack(push, 1)
typedef struct {
    uint8_t is_init;
    uint8_t update_map;
    float odom_x;
    float odom_y;
    float odom_theta;
    uint32_t num_points;
} ICPRequestHeader;

typedef struct {
    float icp_x;
    float icp_y;
    float icp_theta;
} ICPResponse;
#pragma pack(pop)

typedef struct { float x, y; } Point;
typedef struct { float x, y, theta; } Pose;

// 函数接口
void get_surface_normals(Point* scan, Point* normals, uint8_t* valid_mask);

Pose point_to_line_icp(Point* curr_local, uint8_t* curr_mask, Point* ref_global, Point* ref_normals, uint8_t* ref_mask, Pose init_pose) ;

// 点云畸变修正
void motion_deskew(Point* curr_local, uint8_t* curr_mask, float linear_v, float angular_w, float scan_time);
#endif //NOETICMAZE_ICP_CORE_H