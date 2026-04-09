//
// Created by lmtgy on 2026/4/9.
//

#include "icp_core.h"


#include <math.h>
#include <string.h>



/**
 * @brief 点云运动畸变补偿 (Motion Deskew) - STM32 FPU 终极优化版
 * @param curr_local 当前帧雷达原始点云
 * @param curr_mask  有效点掩码
 * @param linear_v   小车当前线速度 (m/s)，向前为正
 * @param angular_w  小车当前角速度 (rad/s)，逆时针为正 (CCW+)
 * @param scan_time  本圈雷达扫描的真实物理耗时 (s)
 */
void motion_deskew(Point* curr_local, int* curr_mask, float linear_v, float angular_w, float scan_time) {
    // 【性能优化】：将循环内的除法提取到外部，计算出每个点之间的时间间隔比例
    // 用乘法代替除法，可为 360 次循环省下数百个时钟周期
    float time_step_ratio = scan_time / (float)SCAN_SIZE;

    for (int i = 0; i < SCAN_SIZE; i++) {
        if (!curr_mask[i]) continue;

        // 计算距离扫描结束的负时间差 (-scan_time 到 0.0s)
        float dt_from_end = (float)i * time_step_ratio - scan_time;

        // 使用相对结束时刻的时间差计算历史位姿偏移
        float d_theta = angular_w * dt_from_end;
        float dx, dy;

        // 防止除以 0 的极小值保护
        if (fabsf(angular_w) > 1e-4f) {
            float v_w = linear_v / angular_w;
            dx = v_w * sinf(d_theta);
            dy = v_w * (1.0f - cosf(d_theta));
        } else {
            dx = linear_v * dt_from_end;
            dy = 0.0f;
        }

        // 将历史测量的点，推演到扫描结束时的当前坐标系下
        float cos_t = cosf(d_theta);
        float sin_t = sinf(d_theta);
        float px = curr_local[i].x;
        float py = curr_local[i].y;

        curr_local[i].x = dx + cos_t * px - sin_t * py;
        curr_local[i].y = dy + sin_t * px + cos_t * py;
    }
}


/**
 * 鲁棒 3x3 线性方程求解器 (Gaussian Elimination with Partial Pivoting & Truncation)
 * 模拟 Python numpy.linalg.lstsq(rcond=1e-3) 的稳定行为
 */
static int solve3x3_robust(double A[3][3], double B[3], float res[3]) {
    double mat[3][4];
    // 1. 复制数据到增广矩阵，并添加微小阻尼 (Ridge Regression)
    double lambda = 1e-4;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) mat[i][j] = A[i][j];
        mat[i][i] += lambda;
        mat[i][3] = B[i];
    }

    // 2. 高斯消元：带主元选择
    for (int i = 0; i < 3; i++) {
        int pivot = i;
        for (int j = i + 1; j < 3; j++) {
            if (fabs(mat[j][i]) > fabs(mat[pivot][i])) pivot = j;
        }

        // 交换行
        for (int k = i; k < 4; k++) {
            double temp = mat[i][k];
            mat[i][k] = mat[pivot][k];
            mat[pivot][k] = temp;
        }

        // 3. 奇异性检查 (核心：效仿 Python rcond)
        if (fabs(mat[i][i]) < 1e-7) {
            res[i] = 0.0f;
            for (int k = 0; k < 4; k++) mat[i][k] = 0;
            mat[i][i] = 1.0;
            continue;
        }

        for (int j = i + 1; j < 3; j++) {
            double factor = mat[j][i] / mat[i][i];
            for (int k = i; k < 4; k++) mat[j][k] -= factor * mat[i][k];
        }
    }

    // 4. 回代求解
    double temp_res[3] = {0};
    for (int i = 2; i >= 0; i--) {
        double sum = 0;
        for (int j = i + 1; j < 3; j++) sum += mat[i][j] * temp_res[j];
        temp_res[i] = (mat[i][3] - sum) / mat[i][i];
        res[i] = (float)temp_res[i];
    }
    return 0;
}

void get_surface_normals(Point* scan, Point* normals, int* valid_mask) {
    for (int i = 0; i < SCAN_SIZE; i++) {
        // 【新增修改 1：检查当前点本身是否为无效点】
        // 假设你的无效点被赋值为 (0,0)，我们用极小值 1e-6f 来避免浮点误差导致的误判
        if (fabsf(scan[i].x) < 1e-6f && fabsf(scan[i].y) < 1e-6f) {
            valid_mask[i] = 0; // 标记为无效
            continue;          // 直接跳过计算
        }

        int prev = (i - 1 + SCAN_SIZE) % SCAN_SIZE;
        int next = (i + 1) % SCAN_SIZE;

        // 【新增修改 2：检查相邻点是否为无效点】
        // 如果左右邻居没扫到数据（为0），计算切线向量会拉出一条跨越原点的错误连线
        if ((fabsf(scan[prev].x) < 1e-6f && fabsf(scan[prev].y) < 1e-6f) ||
            (fabsf(scan[next].x) < 1e-6f && fabsf(scan[next].y) < 1e-6f)) {
            valid_mask[i] = 0; // 相邻点无效，当前点的法向量也不可信，标记为无效
            continue;
            }

        // 【第一阶段优化】干掉 powf，直接相乘，不影响精度，极大提升性能
        float dx1 = scan[i].x - scan[prev].x;
        float dy1 = scan[i].y - scan[prev].y;
        float d1 = sqrtf(dx1*dx1 + dy1*dy1);

        float dx2 = scan[next].x - scan[i].x;
        float dy2 = scan[next].y - scan[i].y;
        float d2 = sqrtf(dx2*dx2 + dy2*dy2);

        if (d1 < MAX_DIST && d2 < MAX_DIST) {
            float tx = scan[next].x - scan[prev].x;
            float ty = scan[next].y - scan[prev].y;
            float mag = sqrtf(tx*tx + ty*ty);
            if (mag > 1e-6f) {
                normals[i].x = -ty / mag;
                normals[i].y = tx / mag;
                valid_mask[i] = 1;
                continue;
            }
        }
        valid_mask[i] = 0;
    }
}

Pose point_to_line_icp(Point* curr_local, int* curr_mask, Point* ref_global, Point* ref_normals, int* ref_mask, Pose init_pose) {    Pose est = init_pose;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        // 【第二阶段优化】内层循环使用 float 累加矩阵，牺牲微小精度换取 FPU 硬件加速
        float AT_A_f[3][3] = {0};
        float AT_B_f[3] = {0};
        int match_count = 0;

        float ct = cosf(est.theta), st = sinf(est.theta);

        for (int i = 0; i < SCAN_SIZE; i++) {

            // 【新增修改 4：核心拦截逻辑！跳过当前帧的无效点】
            // 如果这个角度的雷达没扫到数据，千万不能把它拿去匹配！
            if (!curr_mask[i]) {
                continue;
            }

            float p_src_x = ct * curr_local[i].x - st * curr_local[i].y + est.x;
            float p_src_y = st * curr_local[i].x + ct * curr_local[i].y + est.y;

            float min_d2 = 1e10f;
            int best_j = -1;

            int search_start = i - SEARCH_WINDOW;
            int search_end = i + SEARCH_WINDOW;

            for (int j = search_start; j <= search_end; j++) {
                // 【第一阶段优化】干掉内层循环的除法(%)运算，使用加减法进行越界处理
                int idx = j;
                if (idx < 0) idx += SCAN_SIZE;
                else if (idx >= SCAN_SIZE) idx -= SCAN_SIZE;

                if (!ref_mask[idx]) continue;

                float dx = p_src_x - ref_global[idx].x;
                float dy = p_src_y - ref_global[idx].y;
                float d2 = dx*dx + dy*dy;
                if (d2 < min_d2) { min_d2 = d2; best_j = idx; }
            }

            if (best_j != -1 && min_d2 < MATCH_DIST_SQ) {
                float nx = ref_normals[best_j].x;
                float ny = ref_normals[best_j].y;
                float rx = p_src_x - est.x;
                float ry = p_src_y - est.y;

                // 【第二阶段优化】全部使用 float 计算行和 b
                float row[3] = { nx, ny, rx * ny - ry * nx };
                float b = (ref_global[best_j].x - p_src_x) * nx + (ref_global[best_j].y - p_src_y) * ny;

                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) AT_A_f[r][c] += row[r] * row[c];
                    AT_B_f[r] += row[r] * b;
                }
                match_count++;
            }
        }

        if (match_count < 10) break;

        // 【第二阶段优化】在进入鲁棒求解器前，一次性将 float 赋值给 double 数组
        double AT_A_d[3][3];
        double AT_B_d[3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) AT_A_d[r][c] = (double)AT_A_f[r][c];
            AT_B_d[r] = (double)AT_B_f[r];
        }

        float delta[3];
        // 调用鲁棒求解器（求解过程依然保持 double 以防病态矩阵发散）
        if (solve3x3_robust(AT_A_d, AT_B_d, delta) == 0) {
            if (fabsf(delta[0]) > 0.4f) delta[0] = (delta[0] > 0 ? 0.4f : -0.4f);
            if (fabsf(delta[1]) > 0.4f) delta[1] = (delta[1] > 0 ? 0.4f : -0.4f);
            if (fabsf(delta[2]) > 0.3f) delta[2] = (delta[2] > 0 ? 0.3f : -0.3f);

            est.x += delta[0];
            est.y += delta[1];
            est.theta += delta[2];

            if (delta[0]*delta[0] + delta[1]*delta[1] < 1e-7f && fabsf(delta[2]) < 1e-4f) break;
        } else break;
    }
    return est;
}