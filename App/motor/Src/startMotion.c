//
// Created by lmtgy on 2026/3/27.
//

// 在 freertos.c 或对应的 task.c 中

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

// 引入你的各个驱动头文件
#include "pwm.h"
#include "encoder.h"
#include "motor_pid.h"
#include "robot_state.h"
#include "arm_math.h"
#include "planner_core.h"
#include "roboConifg.h"
#include <float.h>
#include <string.h>
#include <stdbool.h>

// 假设 IMU 校准标志位为全局变量（可以通过信号量或事件标志组优化，此处先保持简单）
extern uint8_t g_imu_is_calibrated;

static Point2D s_motion_path_buffer[MAX_PATH_LEN];
static float s_motion_path_arc_len[MAX_PATH_LEN];
static uint16_t s_motion_path_len = 0U;
static uint32_t s_motion_path_sequence = 0U;
static uint32_t s_trimmed_tf_version = 0U;
static int s_last_projection_index = 0;
static float s_progress_s_m = 0.0f;

typedef struct {
    Point2D point;
    float progress_s_m;
    float dist_sq_m;
    int segment_index;
    bool valid;
} PathProjectionResult;

static void Motion_Reset_Runtime_Path(void)
{
    s_motion_path_len = 0U;
    s_motion_path_sequence = 0U;
    s_trimmed_tf_version = 0U;
    s_last_projection_index = 0;
    s_progress_s_m = 0.0f;
    memset(s_motion_path_arc_len, 0, sizeof(s_motion_path_arc_len));
    Reset_Trimmed_Path_State();
}

static void Motion_Load_New_Path(const GlobalPathSnapshot* path_snapshot)
{
    if ((path_snapshot == NULL) || (path_snapshot->path_ptr == NULL) || (path_snapshot->path_len <= 0)) {
        Motion_Reset_Runtime_Path();
        return;
    }

    uint16_t safe_len = (uint16_t)path_snapshot->path_len;
    if (safe_len > MAX_PATH_LEN) {
        safe_len = MAX_PATH_LEN;
    }

    memcpy(s_motion_path_buffer, path_snapshot->path_ptr, ((size_t)safe_len) * sizeof(Point2D));

    s_motion_path_arc_len[0] = 0.0f;
    for (uint16_t i = 1U; i < safe_len; ++i) {
        float dx = s_motion_path_buffer[i].x - s_motion_path_buffer[i - 1U].x;
        float dy = s_motion_path_buffer[i].y - s_motion_path_buffer[i - 1U].y;
        float seg_len = 0.0f;
        arm_sqrt_f32(dx * dx + dy * dy, &seg_len); // 正确调用：传入指针接收结果
        s_motion_path_arc_len[i] = s_motion_path_arc_len[i - 1U] + seg_len;
    }

    s_motion_path_len = safe_len;
    s_motion_path_sequence = path_snapshot->sequence;
    s_trimmed_tf_version = 0U;
    s_last_projection_index = 0;
    s_progress_s_m = 0.0f;
}

static PathProjectionResult Motion_Project_On_Segment(const Point2D* p0,
                                                      const Point2D* p1,
                                                      float base_s,
                                                      float robot_x,
                                                      float robot_y,
                                                      int segment_index)
{
    PathProjectionResult result = {0};
    float vx = p1->x - p0->x;
    float vy = p1->y - p0->y;
    float seg_len_sq = vx * vx + vy * vy;

    if (seg_len_sq < (PATH_MIN_SEG_LEN_M * PATH_MIN_SEG_LEN_M)) {
        float dx0 = robot_x - p0->x;
        float dy0 = robot_y - p0->y;
        result.point = *p0;
        result.progress_s_m = base_s;
        result.dist_sq_m = dx0 * dx0 + dy0 * dy0;
        result.segment_index = segment_index;
        result.valid = true;
        return result;
    }

    float wx = robot_x - p0->x;
    float wy = robot_y - p0->y;
    float t = (wx * vx + wy * vy) / seg_len_sq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    result.point.x = p0->x + t * vx;
    result.point.y = p0->y + t * vy;

    float dx = robot_x - result.point.x;
    float dy = robot_y - result.point.y;
    float seg_len = 0.0f;
    arm_sqrt_f32(seg_len_sq, &seg_len); // 正确调用：传入指针接收结果

    result.progress_s_m = base_s + (t * seg_len);

    result.progress_s_m = base_s + (t * seg_len);
    result.dist_sq_m = dx * dx + dy * dy;
    result.segment_index = segment_index;
    result.valid = true;
    return result;
}

static PathProjectionResult Motion_Find_Local_Projection(float robot_x, float robot_y)
{
    PathProjectionResult best = {0};
    best.dist_sq_m = FLT_MAX;

    if (s_motion_path_len == 0U) {
        return best;
    }

    if (s_motion_path_len == 1U) {
        float dx = robot_x - s_motion_path_buffer[0].x;
        float dy = robot_y - s_motion_path_buffer[0].y;
        best.point = s_motion_path_buffer[0];
        best.progress_s_m = 0.0f;
        best.dist_sq_m = dx * dx + dy * dy;
        best.segment_index = 0;
        best.valid = true;
        return best;
    }

    int max_segment = (int)s_motion_path_len - 2;
    int start_idx = s_last_projection_index - PATH_SEARCH_BACK_WINDOW;
    int end_idx = s_last_projection_index + PATH_SEARCH_FORWARD_WINDOW;

    if (start_idx < 0) {
        start_idx = 0;
    }
    if (end_idx > max_segment) {
        end_idx = max_segment;
    }

    if (start_idx > end_idx) {
        start_idx = 0;
        end_idx = max_segment;
    }

    for (int i = start_idx; i <= end_idx; ++i) {
        PathProjectionResult candidate = Motion_Project_On_Segment(&s_motion_path_buffer[i],
                                                                   &s_motion_path_buffer[i + 1],
                                                                   s_motion_path_arc_len[i],
                                                                   robot_x,
                                                                   robot_y,
                                                                   i);
        if (candidate.valid && candidate.dist_sq_m < best.dist_sq_m) {
            best = candidate;
        }
    }

    if (!best.valid) {
        for (int i = 0; i <= max_segment; ++i) {
            PathProjectionResult candidate = Motion_Project_On_Segment(&s_motion_path_buffer[i],
                                                                       &s_motion_path_buffer[i + 1],
                                                                       s_motion_path_arc_len[i],
                                                                       robot_x,
                                                                       robot_y,
                                                                       i);
            if (candidate.valid && candidate.dist_sq_m < best.dist_sq_m) {
                best = candidate;
            }
        }
    }

    return best;
}

static void Motion_Publish_Trimmed_Path(const PathProjectionResult* projection, uint32_t tf_version)
{
    Point2D trimmed_points[MAX_PATH_LEN];
    uint16_t out_len = 0U;
    float trim_keep_s;
    int start_index;

    if ((projection == NULL) || (!projection->valid) || (s_motion_path_len == 0U)) {
        Reset_Trimmed_Path_State();
        return;
    }

    trim_keep_s = projection->progress_s_m - PATH_TRIM_BACK_MARGIN_M;
    if (trim_keep_s < 0.0f) {
        trim_keep_s = 0.0f;
    }

    start_index = projection->segment_index;
    while ((start_index > 0) && (s_motion_path_arc_len[start_index] > trim_keep_s)) {
        --start_index;
    }

    trimmed_points[out_len++] = projection->point;

    for (uint16_t i = (uint16_t)(projection->segment_index + 1); i < s_motion_path_len && out_len < MAX_PATH_LEN; ++i) {
        trimmed_points[out_len++] = s_motion_path_buffer[i];
    }

    if ((out_len == 1U) && (projection->segment_index < ((int)s_motion_path_len - 1))) {
        trimmed_points[out_len++] = s_motion_path_buffer[projection->segment_index + 1];
    }

    if (start_index < projection->segment_index) {
        for (int i = projection->segment_index - 1; i >= start_index && out_len < MAX_PATH_LEN; --i) {
            // 保留少量历史余量会导致这里的离散点位于投影点前方；
            // 当前实现只重建投影首点并拼接未来路径，以避免上位机显示回头折返。
            (void)i;
            break;
        }
    }

    Update_Trimmed_Path_State(trimmed_points,
                              out_len,
                              s_progress_s_m,
                              s_motion_path_sequence,
                              tf_version,
                              1U);
}

static void Motion_Update_Trimmed_Path_View(const RobotState_t* current_robot_state)
{
    GlobalPathSnapshot path_snapshot;
    PathProjectionResult projection;
    float max_projection_dist_sq = PATH_PROJECTION_MAX_DIST_M * PATH_PROJECTION_MAX_DIST_M;

    if (current_robot_state == NULL) {
        return;
    }

    if (Get_Global_Path_Snapshot(&path_snapshot)) {
        if (path_snapshot.sequence != s_motion_path_sequence) {
            Motion_Load_New_Path(&path_snapshot);
        }
    } else if (s_motion_path_len == 0U) {
        Reset_Trimmed_Path_State();
        return;
    }

    if (s_motion_path_len == 0U) {
        Reset_Trimmed_Path_State();
        return;
    }

    if ((current_robot_state->tf_map_odom_version == s_trimmed_tf_version) &&
        (path_snapshot.sequence == s_motion_path_sequence)) {
        return;
    }

    projection = Motion_Find_Local_Projection(current_robot_state->global_fast_x_m,
                                              current_robot_state->global_fast_y_m);
    if (!projection.valid) {
        return;
    }

    if (projection.dist_sq_m > max_projection_dist_sq) {
        return;
    }

    if (projection.progress_s_m > s_progress_s_m) {
        float step = projection.progress_s_m - s_progress_s_m;
        if (step > PATH_PROGRESS_MAX_STEP_M) {
            projection.progress_s_m = s_progress_s_m + PATH_PROGRESS_MAX_STEP_M;
        }
        s_progress_s_m = projection.progress_s_m;
    } else {
        projection.progress_s_m = s_progress_s_m;
    }

    s_last_projection_index = projection.segment_index;
    s_trimmed_tf_version = current_robot_state->tf_map_odom_version;
    Motion_Publish_Trimmed_Path(&projection, current_robot_state->tf_map_odom_version);
}

void StartMotionTask(void *argument)
{
    // 1. 硬件外设与算法的初始化 (务必在死循环前完成)
    PWM_Init();
    Encoder_Init();
    Motor_PID_Init();
    void Motor_YawRatePID_Init(void);

    Motion_Reset_Runtime_Path();

    // 2. 初始化绝对延时所需的时间戳变量
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 严格 10ms 周期

    // 获取当前时间
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        // ==========================================
        // 1. 数据采集层
        // ==========================================
        // 更新编码器与里程计 (内部依赖 dt = 0.01s)
        Encoder_Update(NULL);

        // icp TF坐标变换：将高频里程计转为高频全局位姿
        {
            RobotState_t current_robot_state;
            Get_Robot_State_Snapshot(&current_robot_state);

            float theta_odom_rad = Normalize_Angle_Rad(Robot_DegToRad(current_robot_state.yaw_deg));
            float tf_theta_rad = current_robot_state.tf_map_odom_theta_rad;
            float cos_tf = arm_cos_f32(tf_theta_rad);
            float sin_tf = arm_sin_f32(tf_theta_rad);

            float global_fast_x_m = current_robot_state.tf_map_odom_x_m +
                                    (cos_tf * current_robot_state.x_encoder_m - sin_tf * current_robot_state.y_encoder_m);
            float global_fast_y_m = current_robot_state.tf_map_odom_y_m +
                                    (sin_tf * current_robot_state.x_encoder_m + cos_tf * current_robot_state.y_encoder_m);
            float global_fast_theta_rad = Normalize_Angle_Rad(tf_theta_rad + theta_odom_rad);

            Update_Robot_Global_Fast_Pose(global_fast_x_m, global_fast_y_m, global_fast_theta_rad);
        }

        // ==========================================
        // 2. 运动学与安全层 (Priority 4 的核心)
        // ==========================================
        {
            RobotState_t current_robot_state;
            Get_Robot_State_Snapshot(&current_robot_state);

            // 在这里维护路径的 TF 执行视图与安全裁切结果：
            // 1) 使用 map 系高频 pose 投影到路径弧长上
            // 2) 通过局部窗口 + 单周期推进限幅，避免 U 型弯跳段
            // 3) 重建投影首点，并将裁切后的剩余路径发布给上位机
            Motion_Update_Trimmed_Path_View(&current_robot_state);

            // [TODO] 在上述安全裁切路径视图稳定后，于此处接入 Pure Pursuit 轨迹跟踪。
            // [TODO] 当前阶段严格只维护 TF 变换与裁切路径，不下发轨迹跟踪控制量。
            // float desired_v, desired_w;
            // Run_Pure_Pursuit(&desired_v, &desired_w);
        }

        // ==========================================
        // 3. 底层闭环控制层
        // ==========================================
        // 执行 PID 运算并输出 PWM (传入 IMU 校准状态以供拦截)
        Task_MotorPID_Update(&g_imu_is_calibrated);

        // ==========================================
        // 4. 严格绝对延时
        // ==========================================
        // 任务会被挂起，直到下一个 10ms 周期到来
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
