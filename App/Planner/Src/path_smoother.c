//
// Created by lmtgy on 2026/4/15.
//

#include "path_smoother.h"

#include <math.h>
#include "arm_math.h"
#include "planner_algo.h"
// 【核心优化修改 3】：贝塞尔和直线插值只发生在一个局部拐角，100 长度绰绰有余
// static Point2D s_temp_bezier[MAX_TEMP_LEN];
// static Point2D s_temp_line[MAX_TEMP_LEN];
// static Point2D s_path_buffer_B[MAX_PATH_LEN]; // 用于: fwd_path(平滑中转)

#define s_temp_bezier   (g_mem_pool.smooth.temp_bezier)
#define s_temp_line     (g_mem_pool.smooth.temp_line)
#define s_path_buffer_B (g_mem_pool.smooth.path_buffer_B)

// ---------------- 安全倒角(Bezier)切弯算法 ----------------
// 【嵌入式预备修改】：直接向数组末尾添加，去除 realloc
// ---------------- 安全倒角(Bezier)切弯算法 ----------------
// 【核心优化修改 5】：引入 max_capacity 参数，防止写爆被缩短的 temp 数组
void _push_point(Point2D* arr, int* len, int max_capacity, Point2D pt) {
    if (*len < max_capacity) arr[(*len)++] = pt;
}

void _interpolate_lines(Point2D* out_path, int* len, int max_capacity, Point2D p1, Point2D p2, float step_dist) {
    float dist = sqrtf(powf(p2.x - p1.x, 2) + powf(p2.y - p1.y, 2));
    int num_steps = (int)(dist / step_dist);
    if (num_steps < 2) num_steps = 2;
    for (int j = 0; j < num_steps; j++) {
        Point2D pt;
        float t = (float)j / num_steps;
        pt.x = p1.x + (p2.x - p1.x) * t; pt.y = p1.y + (p2.y - p1.y) * t;
        _push_point(out_path, len, max_capacity, pt);
    }
}

// 内部单向拉直函数
// 【嵌入式预备修改】：接收输出数组指针代替 malloc 返回
int _pull_string(ServerMap* map, Point2D* in_path, int path_len, float safe_radius, bool is_return, Point2D* out_path) {
    out_path[0] = in_path[0];
    int s_len = 1, curr_idx = 0;
    while (curr_idx < path_len - 1) {
        int furthest = curr_idx + 1;
        for (int i = path_len - 1; i > curr_idx; i--) {
            if (!bresenham_line_check(map, in_path[curr_idx], in_path[i], safe_radius, 0.05f, is_return)) {
                furthest = i; break;
            }
        }
        if(s_len < MAX_PATH_LEN) out_path[s_len++] = in_path[furthest];
        curr_idx = furthest;
    }
    return s_len;
}


// ---------------- 视线检测及路径平滑 ----------------
// 优化后的视线检测：消除 powf，引入 DDA 步进，减少 FPU 压力
bool bresenham_line_check(ServerMap* map, Point2D p1, Point2D p2, float radius, float ignore_start_dist, bool is_return) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dist_sq = dx * dx + dy * dy;

    if (dist_sq < 0.01f) return false; // 0.1f * 0.1f

    float dist;
    arm_sqrt_f32(dist_sq, &dist); // 【DSP 优化】：单指令硬件开方，约 14 周期

    // 乘法替代除法
    int steps = (int)(dist * (2.0f / map->res));
    if (steps <= 0) return false;

    // 预计算单步偏移量
    float step_x = dx / steps;
    float step_y = dy / steps;
    float ignore_sq = ignore_start_dist * ignore_start_dist;

    Point2D pt = p1;
    float curr_dx = 0.0f, curr_dy = 0.0f;

    for (int i = 1; i <= steps; i++) {
        pt.x += step_x;
        pt.y += step_y;

        // 增量计算距离平方，避免多次减法
        curr_dx += step_x;
        curr_dy += step_y;
        float d2 = curr_dx * curr_dx + curr_dy * curr_dy;

        if (d2 < ignore_sq) continue;
        if (check_collision(map, pt, radius, is_return)) return true;
    }
    return false;
}

// 【核心优化修改 4】：平滑算法中转改用 s_path_buffer_B
int double_smooth_path(ServerMap* map, Point2D* path, int path_len, float safe_radius, bool is_return, Point2D* out_path) {
    if (!path || path_len < 3) {
        if (path) {
            for(int i=0; i<path_len; i++) out_path[i] = path[i];
            return path_len;
        }
        return 0;
    }

    // 第一遍拉直：从 path 拉直到 Buffer_B
    int fwd_len = _pull_string(map, path, path_len, safe_radius, is_return, s_path_buffer_B);

    // 翻转 Buffer_B
    for(int i=0; i < fwd_len / 2; i++) {
        Point2D temp = s_path_buffer_B[i];
        s_path_buffer_B[i] = s_path_buffer_B[fwd_len - 1 - i];
        s_path_buffer_B[fwd_len - 1 - i] = temp;
    }

    // 第二遍拉直：从 Buffer_B 拉直到 out_path (在主循环里会指回 Buffer_A)
    int bwd_len = _pull_string(map, s_path_buffer_B, fwd_len, safe_radius, is_return, out_path);

    for(int i=0; i < bwd_len / 2; i++) {
        Point2D temp = out_path[i]; out_path[i] = out_path[bwd_len - 1 - i]; out_path[bwd_len - 1 - i] = temp;
    }
    return bwd_len;
}


// 辅助内联函数：使用 DSP 硬件开方
static inline float fast_sqrt(float x) {
    float result;
    arm_sqrt_f32(x, &result);
    return result;
}

// 【极致优化版】：安全倒角(Bezier)切弯算法
int generate_safe_corner_path(ServerMap* map, Point2D* path, int path_len, float max_corner_radius, float point_dist, bool is_return, Point2D* out_path) {
    int len = 0;

    // 边界条件处理
    if (!path || path_len < 3) {
        if (!path) return 0;
        for(int i = 0; i < path_len - 1; i++) {
            _interpolate_lines(out_path, &len, MAX_PATH_LEN, path[i], path[i+1], point_dist);
        }
        _push_point(out_path, &len, MAX_PATH_LEN, path[path_len-1]);
        return len;
    }

    _push_point(out_path, &len, MAX_PATH_LEN, path[0]);

    for (int i = 1; i < path_len - 1; i++) {
        Point2D P_prev = path[i-1];
        Point2D P_curr = path[i];
        Point2D P_next = path[i+1];

        // 优化：计算向量与平方距离，彻底消灭 powf
        float dx_prev = P_curr.x - P_prev.x;
        float dy_prev = P_curr.y - P_prev.y;
        float dx_next = P_next.x - P_curr.x;
        float dy_next = P_next.y - P_curr.y;

        float dist_prev_sq = dx_prev * dx_prev + dy_prev * dy_prev;
        float dist_next_sq = dx_next * dx_next + dy_next * dy_next;

        // 优化：DSP 硬件开方 (单指令约 14 周期)
        float dist_prev = fast_sqrt(dist_prev_sq);
        float dist_next = fast_sqrt(dist_next_sq);

        float r = max_corner_radius;
        if (dist_prev * 0.45f < r) r = dist_prev * 0.45f;
        if (dist_next * 0.45f < r) r = dist_next * 0.45f;

        // 优化：乘法代替除法 (FPU 乘法仅 1 周期，除法 14 周期)
        float inv_prev = 1.0f / dist_prev;
        float inv_next = 1.0f / dist_next;
        float vec1_x = dx_prev * inv_prev;
        float vec1_y = dy_prev * inv_prev;
        float vec2_x = dx_next * inv_next;
        float vec2_y = dy_next * inv_next;

        // 优化：直接计算点积求夹角余弦值
        float cos_theta = vec1_x * vec2_x + vec1_y * vec2_y;

        // 锐角或半径过小，放弃切弯
        if (r < 0.1f || cos_theta > 0.98f) {
            _push_point(out_path, &len, MAX_PATH_LEN, P_curr);
            continue;
        }

        float safe_r = r;
        bool curve_safe = false;
        int b_len = 0;

        while (safe_r >= 0.1f) {
            Point2D A = { P_curr.x - vec1_x * safe_r, P_curr.y - vec1_y * safe_r };
            Point2D B = { P_curr.x + vec2_x * safe_r, P_curr.y + vec2_y * safe_r };

            // 优化：使用平方和 DSP 开方计算近似弧长
            float dA_sq = (P_curr.x - A.x)*(P_curr.x - A.x) + (P_curr.y - A.y)*(P_curr.y - A.y);
            float dB_sq = (B.x - P_curr.x)*(B.x - P_curr.x) + (B.y - P_curr.y)*(B.y - P_curr.y);
            float dist_approx = fast_sqrt(dA_sq) + fast_sqrt(dB_sq);

            // 优化：预先计算步数的倒数
            int num_steps = (int)(dist_approx / point_dist);
            if (num_steps < 3) num_steps = 3;
            if (num_steps > MAX_TEMP_LEN) num_steps = MAX_TEMP_LEN;

            float inv_steps = 1.0f / (num_steps - 1);
            b_len = num_steps;

            bool collision = false;
            for (int j = 0; j < num_steps; j++) {
                // 优化：使用倒数乘法代替循环内除法
                float t = (float)j * inv_steps;
                float mt = 1.0f - t;

                // 优化：展平多项式乘法，避免 powf
                float mt2 = mt * mt;
                float t2 = t * t;
                float mt_t_2 = 2.0f * mt * t;

                s_temp_bezier[j].x = mt2 * A.x + mt_t_2 * P_curr.x + t2 * B.x;
                s_temp_bezier[j].y = mt2 * A.y + mt_t_2 * P_curr.y + t2 * B.y;

                if (check_collision(map, s_temp_bezier[j], PHYSICAL_RADIUS, is_return)) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                curve_safe = true;
                int temp_len = 0;
                _interpolate_lines(s_temp_line, &temp_len, MAX_TEMP_LEN, out_path[len-1], A, point_dist);

                for(int k = 1; k < temp_len; k++) _push_point(out_path, &len, MAX_PATH_LEN, s_temp_line[k]);
                for(int k = 0; k < b_len; k++) _push_point(out_path, &len, MAX_PATH_LEN, s_temp_bezier[k]);
                break;
            }
            safe_r -= 0.1f;
        }

        if (!curve_safe) {
            int temp_len = 0;
            _interpolate_lines(s_temp_line, &temp_len, MAX_TEMP_LEN, out_path[len-1], P_curr, point_dist);
            for(int k = 1; k < temp_len; k++) _push_point(out_path, &len, MAX_PATH_LEN, s_temp_line[k]);
        }
    }

    int temp_len = 0;
    _interpolate_lines(s_temp_line, &temp_len, MAX_TEMP_LEN, out_path[len-1], path[path_len-1], point_dist);
    for(int k = 1; k < temp_len; k++) _push_point(out_path, &len, MAX_PATH_LEN, s_temp_line[k]);
    _push_point(out_path, &len, MAX_PATH_LEN, path[path_len-1]);

    return len;
}

