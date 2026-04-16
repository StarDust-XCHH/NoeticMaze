//
// Created by lmtgy on 2026/4/8.
//

#include "robot_state.h"
#include "FreeRTOS.h"
#include "roboConifg.h"
#include "task.h"
#include <math.h>
#include <string.h>

// static 关键字将变量作用域限制在本文件内，彻底杜绝外部 extern 绕过保护
// 使用 C99 语法，只给 odom 和 fast pose 初值，结构体中未指定的其他成员会自动初始化为 0
static RobotState_t global_robot_state = {
    .x_encoder_m = INITIAL_ODOM_X,
    .y_encoder_m = INITIAL_ODOM_Y,
    .global_fast_x_m = INITIAL_ODOM_X,
    .global_fast_y_m = INITIAL_ODOM_Y
};

static TrimmedPathState g_trimmed_path_state = {0};

float Normalize_Angle_Rad(float angle_rad) {
    while (angle_rad > PI) {
        angle_rad -= 2.0f * PI;
    }
    while (angle_rad < -PI) {
        angle_rad += 2.0f * PI;
    }
    return angle_rad;
}

float Robot_DegToRad(float angle_deg) {
    return DEG_TO_RAD_F(angle_deg);
}

float Robot_RadToDeg(float angle_rad) {
    return RAD_TO_DEG_F(angle_rad);
}

// ==========================================
// 1. 快照读取 (最安全的读取方式)
// ==========================================
void Get_Robot_State_Snapshot(RobotState_t *out_state) {
    if (out_state == NULL) return;

    // 进入临界区：暂停任务调度和中断，保证拷贝过程中数据不被篡改
    taskENTER_CRITICAL();
    // 结构体直接赋值，编译器会优化为高效的内存块拷贝 (类似 memcpy)
    *out_state = global_robot_state;
    taskEXIT_CRITICAL();
}

bool Get_Trimmed_Path_Snapshot(TrimmedPathSnapshot *out_snapshot) {
    if (out_snapshot == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    out_snapshot->path_ptr = g_trimmed_path_state.path_points;
    out_snapshot->path_len = g_trimmed_path_state.path_len;
    out_snapshot->progress_s_m = g_trimmed_path_state.progress_s_m;
    out_snapshot->source_path_sequence = g_trimmed_path_state.source_path_sequence;
    out_snapshot->tf_version = g_trimmed_path_state.tf_version;
    out_snapshot->trimmed_sequence = g_trimmed_path_state.trimmed_sequence;
    out_snapshot->valid = (g_trimmed_path_state.valid != 0U);
    taskEXIT_CRITICAL();

    return out_snapshot->valid;
}

// ==========================================
// 2. 分类更新 (防止不同任务互相覆盖数据)
// ==========================================
void Update_Robot_IMU_State(float new_yaw_deg, float new_yaw_rate_deg_s, uint8_t ready) {
    taskENTER_CRITICAL();
    global_robot_state.yaw_deg = new_yaw_deg;
    global_robot_state.yaw_rate_deg_s = new_yaw_rate_deg_s;
    global_robot_state.imu_ready = ready; // 同步状态
    taskEXIT_CRITICAL();
}

void Update_Robot_Odom_State(float new_x_m, float new_y_m, float new_v_m_s) {
    taskENTER_CRITICAL();
    global_robot_state.x_encoder_m = new_x_m;
    global_robot_state.y_encoder_m = new_y_m;
    global_robot_state.linear_vel_encoder_m_s = new_v_m_s;
    taskEXIT_CRITICAL();
}

void Update_Robot_TF_MapOdom(float tf_x_m, float tf_y_m, float tf_theta_rad) {
    taskENTER_CRITICAL();
    global_robot_state.tf_map_odom_x_m = tf_x_m;
    global_robot_state.tf_map_odom_y_m = tf_y_m;
    global_robot_state.tf_map_odom_theta_rad = Normalize_Angle_Rad(tf_theta_rad);
    global_robot_state.tf_map_odom_version++;
    taskEXIT_CRITICAL();
}

void Update_Robot_Global_Fast_Pose(float x_m, float y_m, float theta_rad) {
    taskENTER_CRITICAL();
    global_robot_state.global_fast_x_m = x_m;
    global_robot_state.global_fast_y_m = y_m;
    global_robot_state.global_fast_theta_rad = Normalize_Angle_Rad(theta_rad);
    taskEXIT_CRITICAL();
}

void Update_Trimmed_Path_State(const Point2D* path_points,
                               uint16_t path_len,
                               float progress_s_m,
                               uint32_t source_path_sequence,
                               uint32_t tf_version,
                               uint8_t valid) {
    uint16_t safe_len = path_len;
    if (safe_len > MAX_PATH_LEN) {
        safe_len = MAX_PATH_LEN;
    }

    taskENTER_CRITICAL();

    if ((path_points != NULL) && (safe_len > 0U)) {
        memcpy(g_trimmed_path_state.path_points, path_points, ((size_t)safe_len) * sizeof(Point2D));
    }

    g_trimmed_path_state.path_len = safe_len;
    g_trimmed_path_state.progress_s_m = progress_s_m;
    g_trimmed_path_state.source_path_sequence = source_path_sequence;
    g_trimmed_path_state.tf_version = tf_version;
    g_trimmed_path_state.valid = valid;
    g_trimmed_path_state.trimmed_sequence++;

    taskEXIT_CRITICAL();
}

void Reset_Trimmed_Path_State(void) {
    taskENTER_CRITICAL();
    g_trimmed_path_state.path_len = 0U;
    g_trimmed_path_state.progress_s_m = 0.0f;
    g_trimmed_path_state.source_path_sequence = 0U;
    g_trimmed_path_state.tf_version = 0U;
    g_trimmed_path_state.valid = 0U;
    g_trimmed_path_state.trimmed_sequence++;
    taskEXIT_CRITICAL();
}

// ==========================================
// 3. 兼容旧的单个参数读取接口
// ==========================================
float Get_Global_Yaw(void) {
    float temp;
    taskENTER_CRITICAL();
    temp = global_robot_state.yaw_deg;
    taskEXIT_CRITICAL();
    return temp;
}

float Get_Global_Yaw_Rate(void) {
    float temp;
    taskENTER_CRITICAL();
    temp = global_robot_state.yaw_rate_deg_s;
    taskEXIT_CRITICAL();
    return temp;
}
