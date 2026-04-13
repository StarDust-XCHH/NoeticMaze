//
// Created by lmtgy on 2026/4/8.
//

#ifndef NOETICMAZE_BT_PROTOCOL_H
#define NOETICMAZE_BT_PROTOCOL_H
#include <stdint.h>

// ✅ 将宏定义移动到这里：这是协议级的数据载荷限制
#define MAX_MAP_DIFF 1000


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
    float    yaw_rate;     // 当前实际角速度 (deg/s)
    float    target_yaw_rate; // <--- 【新增】目标/预期角速度 (deg/s)
    uint8_t  checksum;     // 校验和
} RobotState_Packet_t;

// 2. 雷达数据包
typedef struct {
    uint16_t header;       // 帧头: 0x55AA
    uint8_t  type;         // 类型: 0x02 (雷达包)
    int16_t  distance[360]; // 360个点的距离 (使用 int16_t / q15_t，共 720 字节)
    // 【新增】：将雷达这一圈的真实物理耗时传给前端
    float    scan_time;      // 真实扫描耗时 (4 bytes)
    uint32_t point_count;  // 【新增】：占用 4 字节
    uint8_t  checksum;     // 校验和
} LidarData_Packet_t;


// 定义指令结构体（确保字节对齐）
// 定义指令结构体（确保字节对齐）
typedef struct {
    uint16_t header;     // 0x5A5A
    uint8_t  type;       // 0x03
    float    yaw_rate;   // 【修改】预期角速度 (rad/s)
    float    linear_vel; // 【修改】预期线速度 (m/s)
    uint8_t  checksum;
} ControlPacket_t;

// 回显包结构：原样返还指令，用于验证
typedef struct {
    uint16_t header;     // 0x5A5A
    uint8_t  type;       // 0x04 (ACK 类型)
    float    yaw_rate;   // 【修改】预期角速度 (rad/s)
    float    linear_vel; // 【修改】预期线速度 (m/s)
    uint8_t  checksum;
} AckPacket_t;

typedef struct {
    uint16_t header;       // 0x55AA
    uint8_t  type;         // 0x05
    uint16_t data_len;     // 【关键】后面有效载荷的总字节数 (Pose + Payload)

    // 以下为有效载荷 (Payload)
    float    icp_x;
    float    icp_y;
    float    icp_theta;
    uint16_t diff_count;   // 增量点的个数
    uint8_t  diff_payload[MAX_MAP_DIFF * 3];

    // 注意：checksum 将被动态放置在有效载荷的最后一个字节
    uint8_t  checksum;
} MapIcp_Packet_t;

#pragma pack(pop)
#endif //NOETICMAZE_BT_PROTOCOL_H