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
#include <string.h>
#include <stdlib.h>
#include "arm_math.h" // 引入 CMSIS-DSP 库加速三角函数
#include "map_core.h"
#include "planner_core.h"
#include "debug_report.h"
#include "Cstar_map_core.h"
#include "path_smoother.h"
#include "planner_config.h"

// 1. 声明身份：我是合法的 SLAM 线程！
#define I_AM_SLAM_THREAD 1
// 2. 然后再包含私有写入头文件
#include "map_core_write.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

// 1. 引入雷达数据队列句柄
extern osMessageQueueId_t LidarQueueHandle;
extern osMessageQueueId_t LidarFreeQueueHandle;

// 引入Astar请求队列
extern osMessageQueueId_t ReqQueueHandle;

// --- ICP 内部维护的参考帧 (Target Point Cloud) ---
static uint32_t ref_num_points = 0;
static uint8_t  icp_internal_init = 0;

// 在 Task 外部定义静态变量，用于跨帧记忆位姿状态
static Pose last_icp_pose = {0.0f, 0.0f, 0.0f};
static Pose last_raw_odom = {0.0f, 0.0f, 0.0f};
// 在文件头部静态变量区新增
static Pose last_ref_pose = {0.0f, 0.0f, 0.0f};

static float s_last_goal_x = 0.0f;
static float s_last_goal_y = 0.0f;
static bool s_goal_initialized = false;
static uint32_t s_planner_map_version = 0;
static uint32_t s_last_replan_tick = 0;
static uint32_t s_slam_stack_debug_tick = 0;
static uint32_t s_slam_map_debug_tick = 0;
static uint32_t s_slam_imu_debug_tick = 0;
static uint8_t s_blocked_streak = 0;
static uint8_t s_blocked_map_storage[MAX_MAP_BYTES];
static ServerMap s_blocked_map = {s_blocked_map_storage, PLANNER_MAP_RES, MAX_GRID_SIZE};


// <--- 新增：记录用户下发的最新目标点 (赋初值避免刚开机无目标)
static float g_user_goal_x = INITIAL_ODOM_X + 0.7f;
static float g_user_goal_y = INITIAL_ODOM_Y;

// <--- 新增：供串口接收线程调用的安全写入接口
void Set_New_Target_Goal(float x, float y) {
    taskENTER_CRITICAL(); // 挂起调度器，防止被算法线程抢占造成数据撕裂
    g_user_goal_x = x;
    g_user_goal_y = y;
    taskEXIT_CRITICAL();
}


// --- 【关键修复：将大数组放入全局 BSS 段，防止任务爆栈】 ---
Point ref_scan[SCAN_SIZE];
Point ref_normals[SCAN_SIZE];
uint8_t ref_mask[SCAN_SIZE];

static Point curr_scan[SCAN_SIZE];
static uint8_t curr_mask[SCAN_SIZE];
// ------------------------------------------------------------
// 辅助函数，计算是否路线被挡住
static inline float planner_map_extent_m(void) {
    return (float)MAX_GRID_SIZE * PLANNER_MAP_RES;
}

static inline void world_to_planner_cell(float x, float y, int* gx, int* gy) {
    float inv_res = 1.0f / PLANNER_MAP_RES;
    *gx = (int)(x * inv_res);
    *gy = (int)(y * inv_res);
}

static bool planner_cell_is_blocked(ServerMap* map_view, int gx, int gy, bool is_return) {
    if (gx < 0 || gx >= map_view->grid_size || gy < 0 || gy >= map_view->grid_size) {
        return true;
    }

    uint8_t val = get_grid_val(map_view, gx, gy);
    if (val == 1) {
        return true;
    }
    if (is_return && val == 0) {
        return true;
    }
    return false;
}

static bool path_is_blocked(ServerMap* blocked_view, bool is_return, const GlobalPathSnapshot* path_snapshot) {
    if (path_snapshot == NULL || path_snapshot->path_ptr == NULL || path_snapshot->path_len <= 0) {
        return true;
    }

    for (int i = 0; i < path_snapshot->path_len; ++i) {
        int gx, gy;
        world_to_planner_cell(path_snapshot->path_ptr[i].x, path_snapshot->path_ptr[i].y, &gx, &gy);
        if (planner_cell_is_blocked(blocked_view, gx, gy, is_return)) {
            return true;
        }
    }

    return false;
}
// 发送规划路线请求
static void send_planner_request(uint8_t cmd_type,
                                 uint8_t reason,
                                 uint32_t map_version,
                                 bool is_return,
                                 float start_x,
                                 float start_y,
                                 float goal_x,
                                 float goal_y) {
    PlannerReqMsg req;
    req.cmd_type = cmd_type;
    req.is_return = is_return;
    req.reason = reason;
    req.map_version = map_version;
    req.target_x = goal_x;
    req.target_y = goal_y;
    req.start_x = start_x;
    req.start_y = start_y;
    osMessageQueuePut(ReqQueueHandle, &req, 0, 0);
}

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
                uint32_t now_tick = osKernelGetTickCount();
                if ((now_tick - s_slam_imu_debug_tick) >= pdMS_TO_TICKS(1000)) {
                    s_slam_imu_debug_tick = now_tick;
                    Debug_Post(DBG_MOD_SLAM,
                               DBG_STAGE_SLAM_IMU_NOT_READY,
                               0,
                               (float)received_map->sweep_count,
                               0.0f,
                               0.0f,
                               0.0f);
                }
                // 如果 IMU 未校准完毕，则丢弃当前帧雷达数据，归还内存块给空闲队列
                osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
                received_map = NULL;

                // 可选：加个极短的 delay 防止死等时过度占用 CPU（通常不需要，因为前面 osMessageQueueGet 是阻塞的）
                continue; // 跳过本帧的 ICP 和建图，直接进入下一次 for(;;) 循环等待新雷达数据
            }
            // ==========================================

            // 【修正 1】：IMU 读出的是角度制，ICP 内部需要严格的标准弧度制
            Pose curr_raw_odom;
            curr_raw_odom.x = current_robot_state.x_encoder_m;
            curr_raw_odom.y = current_robot_state.y_encoder_m;
            curr_raw_odom.theta = Robot_DegToRad(current_robot_state.yaw_deg); // Deg to Rad

            // 你的 yaw 是 0~360 逆时针，直接转弧度
            float yaw_rad = Robot_DegToRad(current_robot_state.yaw_deg);

            // 强制规范化到 [-PI, PI] 区间，防止 359度 和 0度 之间发生数值跳变崩盘！
            yaw_rad = Normalize_Angle_Rad(yaw_rad);
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
                    float angle_rad = Robot_DegToRad(angle_deg);

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

            float current_linear_v = current_robot_state.linear_vel_encoder_m_s; // 确保是 m/s

            // 假设你的状态机里角速度是度/秒，转成弧度/秒。
            // ⚠️ 同样要注意正负号：必须符合逆时针为正 (CCW+)！如果原数据是顺时针为正，这里要加负号！
            float current_angular_w = Robot_DegToRad(current_robot_state.yaw_rate_deg_s);

            // 取出由 5KHz 硬件时钟积分得来的完美无抖动时间
            float real_scan_time = received_map->scan_time;

            // 传给运动畸变补偿函数
            motion_deskew(curr_scan, curr_mask, current_linear_v, current_angular_w, real_scan_time);

            // ==========================================
            // 步骤 B: ICP 核心逻辑
            // ==========================================
            Pose result;

            if (!icp_internal_init)
            {
                // --- 初始化流程：建立参考帧 ---
                float ct = arm_cos_f32(curr_raw_odom.theta), st = arm_sin_f32(curr_raw_odom.theta);
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
                // last_ref_pose = result; // 暂时注释，可以观察是否会影响
                Update_Robot_TF_MapOdom(0.0f, 0.0f, 0.0f);

                icp_internal_init = 1;
                Debug_Post(DBG_MOD_SLAM,
                           DBG_STAGE_SLAM_ICP_INITIALIZED,
                           0,
                           result.x,
                           result.y,
                           result.theta,
                           (float)received_map->sweep_count);
            }
            else
            {
                // --- 正常匹配流程 ---
                // 1. 基于里程计增量计算先验 Guess
                float dx_odom_global = curr_raw_odom.x - last_raw_odom.x;
                float dy_odom_global = curr_raw_odom.y - last_raw_odom.y;
                float dtheta = curr_raw_odom.theta - last_raw_odom.theta;

                // 角度规范化
                dtheta = Normalize_Angle_Rad(dtheta);

                float cos_odom_prev = arm_cos_f32(last_raw_odom.theta);
                float sin_odom_prev = arm_sin_f32(last_raw_odom.theta);
                float dx_local = dx_odom_global * cos_odom_prev + dy_odom_global * sin_odom_prev;
                float dy_local = -dx_odom_global * sin_odom_prev + dy_odom_global * cos_odom_prev;

                Pose guess;
                float cos_icp_prev = arm_cos_f32(last_icp_pose.theta);
                float sin_icp_prev = arm_sin_f32(last_icp_pose.theta);
                guess.x = last_icp_pose.x + (dx_local * cos_icp_prev - dy_local * sin_icp_prev);
                guess.y = last_icp_pose.y + (dx_local * sin_icp_prev + dy_local * cos_icp_prev);
                guess.theta = Normalize_Angle_Rad(last_icp_pose.theta + dtheta);

                // 2. 调用点到线 ICP 算法
                // 【进阶机制：里程计信任死区】
                // 如果本帧相比上一帧，里程计的平移小于 2mm，且旋转小于 0.005 rad
                if ((dx_local*dx_local + dy_local*dy_local) < ZUPT_DEADZONE_DIST_SQ && fabsf(dtheta) < ZUPT_DEADZONE_ANGLE_RAD) {
                    // 小车基本静止或移动极慢，直接沿用先验 Guess，跳过 ICP 计算！
                    result = guess;
                } else {
                    // 只有车真正动起来了，才去跑高耗时的 ICP
                    result = point_to_line_icp(curr_scan, curr_mask, ref_scan, ref_normals, ref_mask, guess);
                }


#if jumpOutIcp
                // 1. 直接将当前预测里程计原始位姿给到 result
                result = guess;
#endif


                // 维护 map<-odom TF 漂移补偿，供 100Hz 运动线程实时合成高频全局位姿
                {
                    float tf_theta = Normalize_Angle_Rad(result.theta - curr_raw_odom.theta);
                    float cos_tf = arm_cos_f32(tf_theta);
                    float sin_tf = arm_sin_f32(tf_theta);
                    float tf_x = result.x - (cos_tf * curr_raw_odom.x - sin_tf * curr_raw_odom.y);
                    float tf_y = result.y - (sin_tf * curr_raw_odom.x + cos_tf * curr_raw_odom.y);
                    Update_Robot_TF_MapOdom(tf_x, tf_y, tf_theta);
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
                dtheta_ref = Normalize_Angle_Rad(dtheta_ref);

                float dist_sq = dx_ref * dx_ref + dy_ref * dy_ref;

                // 当平移大于 0.1米 (平方为 0.01) 或 旋转大于 0.1 rad (约5.7度) 时，才更新参考帧
                if (dist_sq > ICP_KF_UPDATE_DIST_SQ || fabsf(dtheta_ref) > ICP_KF_UPDATE_ANGLE_RAD) {

                    last_ref_pose = result; // 记录本次参考帧的位姿

                    float ct = arm_cos_f32(result.theta), st = arm_sin_f32(result.theta);
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
                {
                    uint32_t now_tick = osKernelGetTickCount();
                    if ((now_tick - s_slam_map_debug_tick) >= pdMS_TO_TICKS(500)) {
                        s_slam_map_debug_tick = now_tick;
                        Debug_Post(DBG_MOD_SLAM,
                                   DBG_STAGE_SLAM_MAP_SHARED,
                                   0,
                                   (float)diff_cnt,
                                   result.x,
                                   result.y,
                                   result.theta);
                    }
                }
            }


#if IS_ICP2ASTAR

            // ==========================================
            // 步骤 D: 维护 planner 图并判定是否触发重规划
            // ==========================================
            ServerMap planner_base_map;
            process_map_update(&planner_base_map);
            build_blocked_view_from_base(&planner_base_map, &s_blocked_map);
            s_planner_map_version++;

            // ==========================================
            // 【修改】：使用线程安全的方式读取最新的目标点
            // ==========================================
            float goal_x;
            float goal_y;
            taskENTER_CRITICAL();
            goal_x = g_user_goal_x;
            goal_y = g_user_goal_y;
            taskEXIT_CRITICAL();


            // TODO: 在这里接入最终目标
            if (goal_x < 0.0f) goal_x = 0.0f;
            if (goal_y < 0.0f) goal_y = 0.0f;
            if (goal_x > planner_map_extent_m()) goal_x = planner_map_extent_m() - PLANNER_MAP_RES;
            if (goal_y > planner_map_extent_m()) goal_y = planner_map_extent_m() - PLANNER_MAP_RES;

            PlannerReplanReason replan_reason = PLANNER_REPLAN_REASON_NONE;
            GlobalPathSnapshot path_snapshot;
            bool has_path_snapshot = Get_Global_Path_Snapshot(&path_snapshot);
            int start_cell_x, start_cell_y, last_goal_cell_x, last_goal_cell_y, goal_cell_x, goal_cell_y;
            world_to_planner_cell(result.x, result.y, &start_cell_x, &start_cell_y);
            world_to_planner_cell(goal_x, goal_y, &goal_cell_x, &goal_cell_y);
            world_to_planner_cell(s_last_goal_x, s_last_goal_y, &last_goal_cell_x, &last_goal_cell_y);

            if (!s_goal_initialized || abs(goal_cell_x - last_goal_cell_x) > PLANNER_REPLAN_GOAL_SHIFT_CELLS ||
                abs(goal_cell_y - last_goal_cell_y) > PLANNER_REPLAN_GOAL_SHIFT_CELLS) {
                replan_reason = PLANNER_REPLAN_REASON_GOAL_CHANGED;
            } else if (!has_path_snapshot) {
                replan_reason = PLANNER_REPLAN_REASON_NO_PATH;
            } else {
                int path_start_cell_x, path_start_cell_y;
                world_to_planner_cell(path_snapshot.path_ptr[0].x, path_snapshot.path_ptr[0].y,
                                      &path_start_cell_x, &path_start_cell_y);

                if (abs(start_cell_x - path_start_cell_x) > PLANNER_REPLAN_DRIFT_CELLS ||
                    abs(start_cell_y - path_start_cell_y) > PLANNER_REPLAN_DRIFT_CELLS) {
                    replan_reason = PLANNER_REPLAN_REASON_START_DRIFT;
                } else if (path_is_blocked(&s_blocked_map, false, &path_snapshot)) {
                    if (s_blocked_streak < 255) {
                        s_blocked_streak++;
                    }
                    if (s_blocked_streak >= PLANNER_REPLAN_BLOCKED_HITS) {
                        replan_reason = PLANNER_REPLAN_REASON_PATH_BLOCKED;
                    }
                } else {
                    s_blocked_streak = 0;
                }
            }

            uint32_t now_tick = osKernelGetTickCount();
            bool allow_replan = (now_tick - s_last_replan_tick) >= pdMS_TO_TICKS(PLANNER_REPLAN_MIN_INTERVAL_MS);

            if (replan_reason != PLANNER_REPLAN_REASON_NONE && allow_replan) {
                if (replan_reason == PLANNER_REPLAN_REASON_PATH_BLOCKED) {
                    g_abort_astar = true;
                }

                send_planner_request(
                    s_goal_initialized ? PLANNER_REQ_REPLAN : PLANNER_REQ_NEW_GOAL,
                    (uint8_t)replan_reason,
                    s_planner_map_version,
                    false,
                    result.x,
                    result.y,
                    goal_x,
                    goal_y
                );

                s_last_goal_x = goal_x;
                s_last_goal_y = goal_y;
                s_goal_initialized = true;
                s_last_replan_tick = now_tick;
                Debug_Post(DBG_MOD_SLAM,
                           DBG_STAGE_SLAM_REPLAN_REQUESTED,
                           (int16_t)replan_reason,
                           result.x,
                           result.y,
                           goal_x,
                           goal_y);
                if (replan_reason != PLANNER_REPLAN_REASON_PATH_BLOCKED) {
                    s_blocked_streak = 0;
                }
            }
#endif

            {
                uint32_t stack_tick = osKernelGetTickCount();
                if ((stack_tick - s_slam_stack_debug_tick) >= pdMS_TO_TICKS(1000)) {
                    s_slam_stack_debug_tick = stack_tick;
                    Debug_Post(DBG_MOD_SLAM,
                               DBG_STAGE_SLAM_STACK_WATERMARK,
                               0,
                               (float)uxTaskGetStackHighWaterMark(NULL),
                               (float)diff_cnt,
                               (float)s_planner_map_version,
                               (float)icp_frame_counter);
                }
            }

            // 步骤 E: 归还内存块给 Lidar 空闲队列
            osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            received_map = NULL;
            icp_frame_counter++;
        }
#endif
    }
}
