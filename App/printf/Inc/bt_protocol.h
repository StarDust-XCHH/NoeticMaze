//
// Created by lmtgy on 2026/4/8.
//

#ifndef NOETICMAZE_BT_PROTOCOL_H
#define NOETICMAZE_BT_PROTOCOL_H
#include <stdint.h>

// 强制 1 字节对齐，防止内存填充
#pragma pack(push, 1)

// 1. 机器人状态包 (里程计 + IMU + 扫图计数)
typedef struct {
    uint16_t header;       // 帧头: 0x55AA
    uint8_t  type;         // 类型: 0x01 (状态包)
    uint32_t sweep_count;  // 时间同步戳：当前雷达的圈数
    float    x;            // X 坐标 (m)
    float    y;            // Y 坐标 (m)
    float    linear_vel;   // 线速度 (m/s)
    float    yaw;          // 航向角 (deg)
    float    yaw_rate;     // 角速度 (deg/s)
    uint8_t  checksum;     // 校验和
} RobotState_Packet_t;

// 2. 雷达数据包
typedef struct {
    uint16_t header;       // 帧头: 0x55AA
    uint8_t  type;         // 类型: 0x02 (雷达包)
    int16_t  distance[360]; // 360个点的距离 (使用 int16_t / q15_t，共 720 字节)
    uint8_t  checksum;     // 校验和
} LidarData_Packet_t;

#pragma pack(pop)
#endif //NOETICMAZE_BT_PROTOCOL_H