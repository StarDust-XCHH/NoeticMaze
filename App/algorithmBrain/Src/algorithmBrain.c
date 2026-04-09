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
// 在文件头部静态变量区新增
static Pose last_ref_pose = {0.0f, 0.0f, 0.0f};

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


            // ==========================================
            // 【新增】：IMU 状态拦截
            // ==========================================
            if (current_robot_state.imu_ready == 0) {
                // 如果 IMU 未校准完毕，则丢弃当前帧雷达数据，归还内存块给空闲队列
                osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
                received_map = NULL;

                // 可选：加个极短的 delay 防止死等时过度占用 CPU（通常不需要，因为前面 osMessageQueueGet 是阻塞的）
                continue; // 跳过本帧的 ICP 和建图，直接进入下一次 for(;;) 循环等待新雷达数据
            }
            // ==========================================

            // 【修正 1】：IMU 读出的是角度制，ICP 内部需要严格的标准弧度制
            Pose curr_raw_odom;
            curr_raw_odom.x = current_robot_state.x_encoder;
            curr_raw_odom.y = current_robot_state.y_encoder;
            curr_raw_odom.theta = current_robot_state.yaw * (PI / 180.0f); // Deg to Rad


            // 你的 yaw 是 0~360 逆时针，直接转弧度
            float yaw_rad = current_robot_state.yaw * (PI / 180.0f);

            // 强制规范化到 [-PI, PI] 区间，防止 359度 和 0度 之间发生数值跳变崩盘！
            while (yaw_rad >  PI) yaw_rad -= 2.0f * PI;
            while (yaw_rad < -PI) yaw_rad += 2.0f * PI;
            curr_raw_odom.theta = yaw_rad;

            // 2. 将极坐标 Lidar 数据转为 Cartesian (XY) 点云，并生成 Mask
            // 【修正 2】：处理 360 长度数组，顺时针(CW)转逆时针(CCW)，并用 DSP 加速
            for(int i = 0; i < 360; i++) {
                uint16_t dist_mm = received_map->distance[i];

                if(dist_mm > MIN_VALID_DIST && dist_mm < MAX_VALID_DIST) {
                    // 1. 顺时针(CW)转逆时针(CCW)
                    float angle_deg = 360.0f - (float)i;

                    // 2. 【关键修复】：补偿雷达硬件安装角度！
                    // 如果雷达的 0度(排线/尾部) 朝向车尾，必须加上 180 度。
                    // (如果排线朝左，加 270度；朝右，加 90度。请根据实际情况微调)
                    angle_deg += 180.0f;

                    // 3. 将角度规范化到 0~360 度之间
                    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
                    while (angle_deg < 0.0f) angle_deg += 360.0f;

                    // 4. 转为弧度
                    float angle_rad = angle_deg * (PI / 180.0f);

                    float cos_val = arm_cos_f32(angle_rad);
                    float sin_val = arm_sin_f32(angle_rad);
                    float dist_m = dist_mm / 1000.0f;

                    curr_scan[i].x = dist_m * cos_val;
                    curr_scan[i].y = dist_m * sin_val;
                    curr_mask[i] = 1;
                } else {
                    curr_scan[i].x = 0.0f;
                    curr_scan[i].y = 0.0f;
                    curr_mask[i] = 0;
                }
            }


            // ==========================================
            // 【关键修复】：调用运动畸变补偿 (消除旋转漂移)
            // ==========================================
            // 注意！根据你之前 UI 截图的显示，你的角速度很可能是 角度/秒 (°/s)
            // 而 motion_deskew 函数里要求的是 弧度/秒 (rad/s)！
            // 这里必须进行严格的单位转换，否则点云会瞬间炸毁！

            float current_linear_v = current_robot_state.linear_vel_encoder; // 确保是 m/s

            // 假设你的状态机里角速度是度/秒，转成弧度/秒。
            // ⚠️ 同样要注意正负号：必须符合逆时针为正 (CCW+)！如果原数据是顺时针为正，这里要加负号！
            float current_angular_w = current_robot_state.yaw_rate * (PI / 180.0f);

            // 调用你写好的 FPU 优化版去畸变函数
            motion_deskew(curr_scan, curr_mask, current_linear_v, current_angular_w);

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
                // 【进阶机制：里程计信任死区】
                // 如果本帧相比上一帧，里程计的平移小于 2mm，且旋转小于 0.005 rad
                if ((dx_local*dx_local + dy_local*dy_local) <ZUPT_DEADZONE_DIST_SQ && fabsf(dtheta) < ZUPT_DEADZONE_ANGLE_RAD) {
                    // 小车基本静止或移动极慢，直接沿用先验 Guess，跳过 ICP 计算！
                    result = guess;
                } else {
                    // 只有车真正动起来了，才去跑高耗时的 ICP
                    result = point_to_line_icp(curr_scan, curr_mask, ref_scan, ref_normals, ref_mask, guess);
                }


                // 3. 更新历史记录供下帧推算
                last_icp_pose = result;
                last_raw_odom = curr_raw_odom;

                // 4. 定期更新参考帧
                // 4. 【进阶机制：空间关键帧更新】
                float dx_ref = result.x - last_ref_pose.x;
                float dy_ref = result.y - last_ref_pose.y;
                float dtheta_ref = result.theta - last_ref_pose.theta;

                // 角度规范化
                while (dtheta_ref >  M_PI) dtheta_ref -= 2.0f * M_PI;
                while (dtheta_ref < -M_PI) dtheta_ref += 2.0f * M_PI;

                float dist_sq = dx_ref * dx_ref + dy_ref * dy_ref;

                // 当平移大于 0.1米 (平方为 0.01) 或 旋转大于 0.1 rad (约5.7度) 时，才更新参考帧
                if (dist_sq > ICP_KF_UPDATE_DIST_SQ || fabsf(dtheta_ref) > ICP_KF_UPDATE_ANGLE_RAD) {

                    last_ref_pose = result; // 记录本次参考帧的位姿

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
            // 步骤 C 结束: 安全地将地图增量交接给发送线程
            // ==========================================
            // 1. 尝试获取互斥锁（等待 5ms，拿不到就放弃，保证算法线程实时性不被卡死）
            if (osMutexAcquire(MapDataMutexHandle, pdMS_TO_TICKS(5)) == osOK) {

                // 2. 组装固定包头和长度
                g_Shared_MapIcp_Data.header = 0x55AA;
                g_Shared_MapIcp_Data.type = 0x05;
                g_Shared_MapIcp_Data.diff_count = diff_cnt;

                // 3. 写入 ICP 预测位姿
                g_Shared_MapIcp_Data.icp_x = result.x;
                g_Shared_MapIcp_Data.icp_y = result.y;
                g_Shared_MapIcp_Data.icp_theta = result.theta;

                // 4. 将刚才累积的局部增量地图快速拷贝到全局共享区 (3KB 数据 memcpy 极快，约几十微秒)
                if (diff_cnt > 0) {
                    memcpy(g_Shared_MapIcp_Data.diff_payload, diff_payload, diff_cnt * 3);
                }

                // 5. 设置就绪标志位并释放锁
                g_MapIcp_Ready = 1;
                osMutexRelease(MapDataMutexHandle);
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