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

#define PATH_DISPLAY_MAX_POINTS 80U

// 假设 IMU 校准标志位为全局变量（可以通过信号量或事件标志组优化，此处先保持简单）
extern uint8_t g_imu_is_calibrated;

static Point2D s_motion_path_buffer[MAX_PATH_LEN];
static float s_motion_path_arc_len[MAX_PATH_LEN];
static Point2D s_trimmed_points[PATH_DISPLAY_MAX_POINTS];
static uint16_t s_motion_path_len = 0U;
static uint32_t s_motion_path_sequence = 0U;
static uint32_t s_trimmed_tf_version = 0U;
static int s_last_projection_index = 0;
static float s_progress_s_m = 0.0f;
static float s_track_last_linear_cmd_m_s = 0.0f;

typedef struct {
    Point2D point;
    float progress_s_m;
    float dist_sq_m;
    int segment_index;
    bool valid;
} PathProjectionResult;

typedef struct {
    Point2D lookahead_point;
    Point2D goal_point;
    float goal_dist_m;
    float heading_error_deg;
    float linear_cmd_m_s;
    float yaw_rate_cmd_deg_s;
    bool valid;
    bool goal_reached;
} TrackingCommand;

static Point2D Motion_MapPoint_To_BodyFrame(const RobotState_t* current_robot_state, const Point2D* map_point)
{
    Point2D body_point = {0};
    float dx;
    float dy;
    float cos_theta;
    float sin_theta;

    if ((current_robot_state == NULL) || (map_point == NULL)) {
        return body_point;
    }

    dx = map_point->x - current_robot_state->global_fast_x_m;
    dy = map_point->y - current_robot_state->global_fast_y_m;
    cos_theta = arm_cos_f32(current_robot_state->global_fast_theta_rad);
    sin_theta = arm_sin_f32(current_robot_state->global_fast_theta_rad);

    body_point.x = (cos_theta * dx) + (sin_theta * dy);
    body_point.y = (-sin_theta * dx) + (cos_theta * dy);
    return body_point;
}

static float Motion_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Motion_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Motion_Distance_Between(const Point2D* a, const Point2D* b)
{
    float distance = 0.0f;

    if ((a == NULL) || (b == NULL)) {
        return 0.0f;
    }

    arm_sqrt_f32(((a->x - b->x) * (a->x - b->x)) + ((a->y - b->y) * (a->y - b->y)), &distance);
    return distance;
}

static float Motion_Apply_Linear_Slew(float desired_linear_m_s)
{
#if TRACK_ENABLE_LINEAR_SLEW
    float max_step = TRACK_LINEAR_ACCEL_LIMIT_M_S2 * 0.01f;
    float delta = desired_linear_m_s - s_track_last_linear_cmd_m_s;

    if (delta > max_step) {
        desired_linear_m_s = s_track_last_linear_cmd_m_s + max_step;
    } else if (delta < -max_step) {
        desired_linear_m_s = s_track_last_linear_cmd_m_s - max_step;
    }
#endif

    s_track_last_linear_cmd_m_s = desired_linear_m_s;
    return desired_linear_m_s;
}

static float Motion_Compute_Turn_Scale(float abs_heading_error_deg)
{
    if (abs_heading_error_deg <= TRACK_TURN_SLOWDOWN_START_DEG) {
        return 1.0f;
    }

    if (abs_heading_error_deg >= TRACK_TURN_SLOWDOWN_STOP_DEG) {
        return 0.0f;
    }

    return (TRACK_TURN_SLOWDOWN_STOP_DEG - abs_heading_error_deg) /
           (TRACK_TURN_SLOWDOWN_STOP_DEG - TRACK_TURN_SLOWDOWN_START_DEG);
}

static float Motion_Compute_Goal_Scale(float goal_dist_m)
{
    if (goal_dist_m >= TRACK_GOAL_SLOWDOWN_DIST_M) {
        return 1.0f;
    }

    if (goal_dist_m <= TRACK_GOAL_STOP_DIST_M) {
        return 0.0f;
    }

    return (goal_dist_m - TRACK_GOAL_STOP_DIST_M) /
           (TRACK_GOAL_SLOWDOWN_DIST_M - TRACK_GOAL_STOP_DIST_M);
}

static Point2D Motion_Select_Lookahead_Point(float current_progress_s)
{
    Point2D selected_point = {0};
    float target_progress_s = current_progress_s + TRACK_LOOKAHEAD_DIST_M;

    if (s_motion_path_len == 0U) {
        return selected_point;
    }

    selected_point = s_motion_path_buffer[s_motion_path_len - 1U];
    for (uint16_t i = 0U; i < s_motion_path_len; ++i) {
        if (s_motion_path_arc_len[i] >= target_progress_s) {
            selected_point = s_motion_path_buffer[i];
            break;
        }
    }

    return selected_point;
}

static TrackingCommand Motion_Build_Tracking_Command(const RobotState_t* current_robot_state)
{
    TrackingCommand cmd = {0};
    Point2D robot_point;
    Point2D lookahead_body_point;
    float abs_heading_error_deg;
    float desired_linear_m_s;
    float direction = 1.0f; // 【修改点1】新增：行驶方向标志，默认正向

    if ((current_robot_state == NULL) || (s_motion_path_len == 0U)) {
        return cmd;
    }

    robot_point.x = current_robot_state->global_fast_x_m;
    robot_point.y = current_robot_state->global_fast_y_m;
    cmd.goal_point = s_motion_path_buffer[s_motion_path_len - 1U];
    cmd.lookahead_point = Motion_Select_Lookahead_Point(s_progress_s_m);
    cmd.goal_dist_m = Motion_Distance_Between(&robot_point, &cmd.goal_point);

    lookahead_body_point = Motion_MapPoint_To_BodyFrame(current_robot_state, &cmd.lookahead_point);

    // 【修改点2】判断前瞻点在车前还是车后，动态调整跟踪姿态
    if (lookahead_body_point.x < 0.0f) {
        direction = -1.0f; // 目标在后方，准备倒车
        // 倒车时，将车尾视为车头：计算目标点与车尾的夹角
        cmd.heading_error_deg = Robot_RadToDeg(atan2f(-lookahead_body_point.y, -lookahead_body_point.x));
    } else {
        direction = 1.0f;  // 目标在前方，正常正走
        cmd.heading_error_deg = Robot_RadToDeg(atan2f(lookahead_body_point.y, lookahead_body_point.x));
    }

    abs_heading_error_deg = Motion_Abs(cmd.heading_error_deg);

    // 计算角速度（无论是正走还是倒车，算出的有效偏差可以直接乘Kp）
    cmd.yaw_rate_cmd_deg_s = TRACK_HEADING_KP * cmd.heading_error_deg;
    cmd.yaw_rate_cmd_deg_s = Motion_Clamp(cmd.yaw_rate_cmd_deg_s,
                                          -TRACK_MAX_YAW_RATE_DEG_S,
                                          TRACK_MAX_YAW_RATE_DEG_S);

    cmd.goal_reached = (cmd.goal_dist_m <= TRACK_GOAL_STOP_DIST_M) &&
                       (abs_heading_error_deg <= TRACK_HEADING_STOP_DEG);

    if (cmd.goal_reached) {
        cmd.linear_cmd_m_s = 0.0f;
        cmd.yaw_rate_cmd_deg_s = 0.0f;
        cmd.valid = true;
        return cmd;
    }

    // 计算线速度标量（基于有效偏差和距离降速）
    desired_linear_m_s = TRACK_CRUISE_LINEAR_M_S;
    desired_linear_m_s *= Motion_Compute_Turn_Scale(abs_heading_error_deg);
    desired_linear_m_s *= Motion_Compute_Goal_Scale(cmd.goal_dist_m);

    if ((desired_linear_m_s > 0.0f) && (desired_linear_m_s < TRACK_MIN_LINEAR_M_S)) {
        desired_linear_m_s = TRACK_MIN_LINEAR_M_S;
    }

    // 【修改点3】赋予速度方向，并将限幅下界改为允许最大的倒车速度
    desired_linear_m_s *= direction;
    cmd.linear_cmd_m_s = Motion_Clamp(desired_linear_m_s,
                                      -TRACK_MAX_LINEAR_M_S,
                                      TRACK_MAX_LINEAR_M_S);

    // 现有的线性加速度约束平滑函数（Motion_Apply_Linear_Slew）天然兼容正负值过渡，无需修改
    cmd.linear_cmd_m_s = Motion_Apply_Linear_Slew(cmd.linear_cmd_m_s);
    cmd.valid = true;
    return cmd;
}

static void Motion_Reset_Runtime_Path(void)
{
    s_motion_path_len = 0U;
    s_motion_path_sequence = 0U;
    s_trimmed_tf_version = 0U;
    s_last_projection_index = 0;
    s_progress_s_m = 0.0f;
    s_track_last_linear_cmd_m_s = 0.0f;
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
    s_track_last_linear_cmd_m_s = 0.0f;
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
    uint16_t out_len = 0U;
    uint16_t next_index;
    uint16_t remaining_points;
    uint16_t available_slots;

    if ((projection == NULL) || (!projection->valid) || (s_motion_path_len == 0U)) {
        Reset_Trimmed_Path_State();
        return;
    }

    s_trimmed_points[out_len++] = projection->point;

    next_index = (projection->segment_index < 0) ? 0U : (uint16_t)(projection->segment_index + 1);
    if (next_index < s_motion_path_len) {
        remaining_points = (uint16_t)(s_motion_path_len - next_index);
        available_slots = (uint16_t)(PATH_DISPLAY_MAX_POINTS - out_len);

        if (remaining_points <= available_slots) {
            for (uint16_t i = next_index; i < s_motion_path_len; ++i) {
                s_trimmed_points[out_len++] = s_motion_path_buffer[i];
            }
        } else if (available_slots == 1U) {
            s_trimmed_points[out_len++] = s_motion_path_buffer[s_motion_path_len - 1U];
        } else if (available_slots > 1U) {
            for (uint16_t slot = 0U; slot < available_slots; ++slot) {
                uint32_t offset;
                uint16_t sample_index;

                if ((slot + 1U) == available_slots) {
                    sample_index = (uint16_t)(s_motion_path_len - 1U);
                } else {
                    offset = ((uint32_t)slot * (uint32_t)(remaining_points - 1U)) /
                             (uint32_t)(available_slots - 1U);
                    sample_index = (uint16_t)(next_index + offset);
                }

                s_trimmed_points[out_len++] = s_motion_path_buffer[sample_index];
            }
        }
    }

    Update_Trimmed_Path_State(s_trimmed_points,
                              out_len,
                              s_progress_s_m,
                              s_motion_path_sequence,
                              tf_version,
                              1U);
}

static void Motion_Update_Trimmed_Path_View(const RobotState_t* current_robot_state)
{
    GlobalPathSnapshot path_snapshot = {0};
    PathProjectionResult projection;
    float max_projection_dist_sq = PATH_PROJECTION_MAX_DIST_M * PATH_PROJECTION_MAX_DIST_M;
    bool has_path_snapshot;

    if (current_robot_state == NULL) {
        return;
    }

    has_path_snapshot = Get_Global_Path_Snapshot(&path_snapshot);
    if (has_path_snapshot) {
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
        has_path_snapshot &&
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
    Motor_YawRatePID_Init();

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
            TrackingCommand tracking_cmd;
            Get_Robot_State_Snapshot(&current_robot_state);

            // 在这里维护路径的 TF 执行视图与安全裁切结果：
            // 1) 使用 map 系高频 pose 投影到路径弧长上
            // 2) 通过局部窗口 + 单周期推进限幅，避免 U 型弯跳段
            // 3) 重建投影首点，并将裁切后的剩余路径发布给上位机
            Motion_Update_Trimmed_Path_View(&current_robot_state);

#if TRACK_ENABLE_AUTONOMOUS_FOLLOW
            tracking_cmd = Motion_Build_Tracking_Command(&current_robot_state);
            if (tracking_cmd.valid) {
                if (tracking_cmd.goal_reached) {
                    Motor_SetTargetVelocity(0.0f, 0.0f);
                    Motor_NormalStop();
                } else {
                    Motor_SetTargetVelocity(tracking_cmd.linear_cmd_m_s,
                                            tracking_cmd.yaw_rate_cmd_deg_s);
                }
            } else {
                s_track_last_linear_cmd_m_s = 0.0f;
                Motor_SetTargetVelocity(0.0f, 0.0f);
            }
#else
            Motor_SetTargetVelocity(0.0f, 0.0f);
#endif
        }

        // ==========================================
        // 3. 底层闭环控制层
        // ==========================================
        // 执行 PID 运算并输出 PWM
        Task_MotorPID_Update();

        // ==========================================
        // 4. 严格绝对延时
        // ==========================================
        // 任务会被挂起，直到下一个 10ms 周期到来
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
