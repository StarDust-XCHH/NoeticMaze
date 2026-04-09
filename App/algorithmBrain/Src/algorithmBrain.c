//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\algorithmBrain\Src\algorithmBrain.c

#include "algorithmBrain.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lidar.h" // 包含 LidarMap_t 定义
#include "roboConifg.h"
#include "robot_state.h" // 引入刚才写的安全状态读取接口
#include <math.h>
#include "arm_math.h" // 引入 CMSIS-DSP 库加速三角函数
#include "map_core.h"
#ifndef PI
#define PI 3.14159265358979f
#endif


// 1. 引入雷达数据队列句柄
extern osMessageQueueId_t LidarQueueHandle;
extern osMessageQueueId_t LidarFreeQueueHandle;

// --- ICP 内部维护的参考帧 (Target Point Cloud) ---
static uint32_t ref_num_points = 0;
static uint8_t  icp_internal_init = 0;

// 在 Task 外部定义静态变量，用于跨帧记忆位姿状态
static Pose last_icp_pose = {0.0f, 0.0f, 0.0f};
static Pose last_raw_odom = {0.0f, 0.0f, 0.0f};

// --- 【关键修复：将大数组放入全局 BSS 段，防止任务爆栈】 ---
Point ref_scan[SCAN_SIZE];
Point ref_normals[SCAN_SIZE];
int ref_mask[SCAN_SIZE];

static Point curr_scan[SCAN_SIZE];
static int   curr_mask[SCAN_SIZE];
// ------------------------------------------------------------

void StartAlgorithmBrain(void *argument)
{
    osStatus_t status;
    LidarMap_t *received_map = NULL;
    static uint32_t icp_frame_counter = 0;

    for(;;)
    {
#if ENABLE_LIDAR_BT_TX
        osDelay(100);
#else


        // 1. 阻塞等待雷达一帧数据就绪
        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, osWaitForever);

        if (status == osOK && received_map != NULL)
        {
            // ==========================================
            // 步骤 A: 获取当前底盘状态 & 组装当前帧数据
            // ==========================================

            // 1. 获取对应时刻的安全全局状态
            RobotState_t current_robot_state;
            Get_Robot_State_Snapshot(&current_robot_state);

            // 【修正 1】：IMU 读出的是角度制，ICP 内部需要严格的标准弧度制
            Pose curr_raw_odom;
            curr_raw_odom.x = current_robot_state.x_encoder;
            curr_raw_odom.y = current_robot_state.y_encoder;
            curr_raw_odom.theta = current_robot_state.yaw * (PI / 180.0f); // Deg to Rad

            // 2. 将极坐标 Lidar 数据转为 Cartesian (XY) 点云，并生成 Mask
            // 【修正 2】：处理 360 长度数组，顺时针(CW)转逆时针(CCW)，并用 DSP 加速
            for(int i = 0; i < 360; i++) {
                uint16_t dist_mm = received_map->distance[i];

                if(dist_mm > MIN_VALID_DIST && dist_mm < MAX_VALID_DIST) {
                    // Lidar 索引 i 是顺时针(0~359度)
                    // 右手标准系要求逆时针为正。转换公式：标准角度 = 360 - i (或 -i)
                    float angle_rad = (360.0f - (float)i) * (PI / 180.0f);

                    // 保证角度在 0 ~ 2PI 之间（虽然 DSP 库的 sin/cos 能处理越界，但规范化更安全）
                    if (angle_rad >= 2.0f * PI) {
                        angle_rad -= 2.0f * PI;
                    }

                    // 调用 CMSIS-DSP 硬件加速三角函数计算 (内部为查表+泰勒插值，极快)
                    float cos_val = arm_cos_f32(angle_rad);
                    float sin_val = arm_sin_f32(angle_rad);

                    float dist_m = dist_mm / 1000.0f;

                    // 极坐标转直角坐标
                    curr_scan[i].x = dist_m * cos_val;
                    curr_scan[i].y = dist_m * sin_val;
                    curr_mask[i] = 1; // 标记为有效点
                } else {
                    // 无效测距点（太近、太远、或吸收不反射）
                    curr_scan[i].x = 0.0f;
                    curr_scan[i].y = 0.0f;
                    curr_mask[i] = 0; // 剔除无效点
                }
            }

            // ==========================================
            // 步骤 B: ICP 核心逻辑
            // ==========================================
            Pose result;

            if (!icp_internal_init)
            {
                // --- 初始化流程：建立参考帧 ---
                float ct = cosf(curr_raw_odom.theta), st = sinf(curr_raw_odom.theta);
                for(int k=0; k<SCAN_SIZE; k++) {
                    if (curr_mask[k]) {
                        ref_scan[k].x = ct * curr_scan[k].x - st * curr_scan[k].y + curr_raw_odom.x;
                        ref_scan[k].y = st * curr_scan[k].x + ct * curr_scan[k].y + curr_raw_odom.y;
                    } else {
                        ref_scan[k].x = 0.0f; ref_scan[k].y = 0.0f;
                    }
                    ref_mask[k] = curr_mask[k];
                }

                get_surface_normals(ref_scan, ref_normals, ref_mask);

                result = curr_raw_odom;
                last_icp_pose = result;
                last_raw_odom = curr_raw_odom;

                icp_internal_init = 1;
            }
            else
            {
                // --- 正常匹配流程 ---
                // 1. 基于里程计增量计算先验 Guess
                float dx_odom_global = curr_raw_odom.x - last_raw_odom.x;
                float dy_odom_global = curr_raw_odom.y - last_raw_odom.y;
                float dtheta = curr_raw_odom.theta - last_raw_odom.theta;

                // 角度规范化
                while (dtheta >  M_PI) dtheta -= 2.0f * M_PI;
                while (dtheta < -M_PI) dtheta += 2.0f * M_PI;

                float cos_odom_prev = cosf(last_raw_odom.theta);
                float sin_odom_prev = sinf(last_raw_odom.theta);
                float dx_local = dx_odom_global * cos_odom_prev + dy_odom_global * sin_odom_prev;
                float dy_local = -dx_odom_global * sin_odom_prev + dy_odom_global * cos_odom_prev;

                Pose guess;
                float cos_icp_prev = cosf(last_icp_pose.theta);
                float sin_icp_prev = sinf(last_icp_pose.theta);
                guess.x = last_icp_pose.x + (dx_local * cos_icp_prev - dy_local * sin_icp_prev);
                guess.y = last_icp_pose.y + (dx_local * sin_icp_prev + dy_local * cos_icp_prev);
                guess.theta = last_icp_pose.theta + dtheta;

                while (guess.theta >  M_PI) guess.theta -= 2.0f * M_PI;
                while (guess.theta < -M_PI) guess.theta += 2.0f * M_PI;

                // 2. 调用点到线 ICP 算法
                result = point_to_line_icp(curr_scan, curr_mask, ref_scan, ref_normals, ref_mask, guess);

                // 3. 更新历史记录供下帧推算
                last_icp_pose = result;
                last_raw_odom = curr_raw_odom;

                // 4. 定期更新参考帧
                icp_frame_counter++;
                if (icp_frame_counter >= ICP_REF_UPDATE_FRAMES) {
                    icp_frame_counter = 0;
                    float ct = cosf(result.theta), st = sinf(result.theta);
                    for(int k=0; k<SCAN_SIZE; k++) {
                        if (curr_mask[k]) {
                            ref_scan[k].x = ct * curr_scan[k].x - st * curr_scan[k].y + result.x;
                            ref_scan[k].y = st * curr_scan[k].x + ct * curr_scan[k].y + result.y;
                        } else {
                            ref_scan[k].x = 0.0f; ref_scan[k].y = 0.0f;
                        }
                        ref_mask[k] = curr_mask[k];
                    }
                    get_surface_normals(ref_scan, ref_normals, ref_mask);
                }
            }

            // ==========================================
            // 步骤 C: 地图更新/后处理 (Bresenham 射线追踪)
            // ==========================================
            diff_cnt = 0; // 【关键】每帧开始前清空增量缓冲区
            int rx_grid, ry_grid;

            // 将机器人的世界坐标转换为栅格坐标，作为射线的起点
            world_to_grid(result.x, &rx_grid);
            world_to_grid(result.y, &ry_grid);

            // 使用 DSP 库加速三角函数计算
            // 注意：虽然 result.theta 的范围应该是 [-PI, PI]，但 arm_cos/sin_f32 可以自动处理任意弧度
            float c_res = arm_cos_f32(result.theta);
            float s_res = arm_sin_f32(result.theta);

            // 【替换 1】：固定遍历 360 个雷达角度
            for (int k = 0; k < 360; k++) {

                // 【核心保护 ！！！】：跳过无效的噪点或超出量程的点
                // 如果不加这句，你就会朝着坐标 (0,0) 发射射线，建出一堆幽灵墙！
                if (!curr_mask[k]) {
                    continue;
                }

                // 【替换 2】：使用步骤 A 解析出来的 curr_scan
                float wx = c_res * curr_scan[k].x - s_res * curr_scan[k].y + result.x;
                float wy = s_res * curr_scan[k].x + c_res * curr_scan[k].y + result.y;

                int gx, gy;
                if (world_to_grid(wx, &gx) && world_to_grid(wy, &gy)) {

                    // 1. 终点：尝试更新为占用状态 (如果原本就是占用，引擎会自动过滤)
                    UpdateAndRecordMap(gx, gy, MAP_OCCUPIED);

                    // 2. 射线经过的点：尝试更新为空闲状态
                    // 取消降采样，全量覆盖！
                    trace_ray_bresenham_diff(rx_grid, ry_grid, gx, gy);
                }
            }


            // ==========================================
            // 步骤 D: 内存回收 (Memory Release)
            // ==========================================
            // 【至关重要】处理完后，必须将内存块归还给空闲队列
            osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            received_map = NULL;

        }

#endif

    } // for(;;)
}