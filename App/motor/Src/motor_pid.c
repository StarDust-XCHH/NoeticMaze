#include "motor_pid.h"
#include "encoder.h"
#include "pwm.h"
#include "tim.h"      // 包含定时器句柄，如 htim2
#include <math.h>

#include "MPU6500.h"
#include "robot_state.h"
#include "FreeRTOS.h"
#include "task.h"
// 1. DSP PID 实例与目标速度
static arm_pid_instance_f32 pid_left;
static arm_pid_instance_f32 pid_right;
static float target_rps_left = 0.0f;
static float target_rps_right = 0.0f;


/* --- 新增变量 --- */
static arm_pid_instance_f32 pid_angle;   // 角度环 PID 实例
static float target_angle = 0.0f;        // 目标角度 (0~360)
static float base_linear_rps = 0.0f;     // 基础线速度 (RPS)
static uint8_t is_angle_lock_mode = 0;   // 是否开启角度锁定模式

// 2. 状态机变量
static Motor_State_t current_state = MOTOR_STATE_IDLE;

// 3. 宏定义限制 (根据你的 CubeMX 配置，ARR = 1000 - 1)
#define MAX_PWM_DUTY 1000.0f
#define STOP_DEADBAND 0.05f   // 停车死区：当转速小于此值(圈/秒)时认为已停稳

/**
 * @brief PID 参数初始化
 */
void Motor_PID_Init(void) {
    // --- 左轮 PID 参数 (请根据实际整定) ---
    pid_left.Kp = 200.0f;
    pid_left.Ki = 50.0f;
    pid_left.Kd = 0.0f;
    arm_pid_init_f32(&pid_left, 1);

    // --- 右轮 PID 参数 (请根据实际整定) ---
    pid_right.Kp = 200.0f;
    pid_right.Ki = 50.0f;
    pid_right.Kd = 0.0f;
    arm_pid_init_f32(&pid_right, 1);

    current_state = MOTOR_STATE_IDLE;
}


/**
 * @brief 角度环初始化
 */
void Motor_AnglePID_Init(void) {
    // 角度外环只负责根据角度差下发“速度指令”
    // Kp 决定转向的激进程度。0.01 是个好的起点，如果转向太慢可以加大到 0.02
    pid_angle.Kp = 0.02f;
    pid_angle.Ki = 0.0f;  // 【关键修改】外环绝对不要加积分！

    // 如果快到目标时有轻微超调，可以加一点 Kd (比如 0.005) 提供阻尼刹车
    pid_angle.Kd = 0.002f;

    arm_pid_init_f32(&pid_angle, 1);

    is_angle_lock_mode = 0;
}

// ==========================================================
// 核心修改区：将“双 PWM 控制逻辑”无缝接入 PID 输出端
// ==========================================================

/**
 * @brief 左轮底层输出封装 (TIM2 CH1 & CH2)
 * @param duty: 带符号的占空比 (-1000 到 1000)
 */
static void Set_Left_Motor(float duty) {
    if (duty > 0) {
        // 正转：CH1 输出 PWM，CH2 拉低
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)duty);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    } else if (duty < 0) {
        // 反转：CH1 拉低，CH2 输出 PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(-duty));
    } else {
        // 停止：双通道拉低
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

/**
 * @brief 右轮底层输出封装 (TIM2 CH3 & CH4)
 * @param duty: 带符号的占空比 (-1000 到 1000)
 */
static void Set_Right_Motor(float duty) {
    if (duty > 0) {
        // 正转：CH3 输出 PWM，CH4 拉低
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)duty);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    } else if (duty < 0) {
        // 反转：CH3 拉低，CH4 输出 PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(-duty));
    } else {
        // 停止：双通道拉低
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
}



/**
 * @brief 设置目标角度及前行速度 (用户调用接口)
 * @param angle 目标角度 (0-360)
 * @param linear_rps 前进的基础速度 (RPS)
 */
void Motor_SetTargetAngle(float angle, float linear_rps) {
    taskENTER_CRITICAL();
    target_angle = angle;
    base_linear_rps = linear_rps;
    is_angle_lock_mode = 1;

    if (current_state != MOTOR_STATE_ESTOP) {
        current_state = MOTOR_STATE_RUNNING;
    }
    taskEXIT_CRITICAL();
}

// ==========================================================

/**
 * @brief 内部静态函数：带抗饱和的 PID 计算
 */
static float _Calculate_PID(arm_pid_instance_f32 *pid, float error) {
    // 1. 调用 DSP 库计算 PID
    float pid_out = arm_pid_f32(pid, error);

    // 2. 积分抗饱和 (Anti-Windup)
    // 防止堵转时输出溢出，同时修正 DSP 内部的历史状态缓存
    if (pid_out > MAX_PWM_DUTY) {
        pid_out = MAX_PWM_DUTY;
        pid->state[2] = MAX_PWM_DUTY;
    } else if (pid_out < -MAX_PWM_DUTY) {
        pid_out = -MAX_PWM_DUTY;
        pid->state[2] = -MAX_PWM_DUTY;
    }

    return pid_out;
}

/**
 * @brief 定时执行的 PID 控制主任务 (在 main.c 中挂载到10ms调度器)
 */
void Task_MotorPID_Update(void *param) {
    // ==========================================
    // 0. 新增：IMU 校准状态拦截
    // ==========================================
    if (param != NULL) {
        uint8_t *is_calibrated = (uint8_t *)param;
        if (*is_calibrated == 0) {
            // IMU 尚未校准完成：强制保持停止状态，跳过所有 PID 运算
            Set_Left_Motor(0.0f);
            Set_Right_Motor(0.0f);
            return; // 直接退出，不进行后续状态机判断和误差计算
        }
    }

    // 1. 紧急停车 或 彻底待机状态：直接输出 0，不进行 PID 运算
    if (current_state == MOTOR_STATE_ESTOP || current_state == MOTOR_STATE_IDLE) {
        Set_Left_Motor(0.0f);
        Set_Right_Motor(0.0f);
        return;
    }

    // --- 串级逻辑：根据角度环更新速度环的目标值 ---
    // 修改 Task_MotorPID_Update 中的串级逻辑
    // --- 串级逻辑：根据角度环更新速度环的目标值 ---
    if (is_angle_lock_mode) {
        // 【修改这里】通过全局中枢安全获取当前偏航角
        // 绝不要用 extern imu_handler!
        float current_yaw = Get_Global_Yaw();

        // 计算角度偏差
        float angular_offset = _Calculate_Angle_Error_Correction(current_yaw);

        // 修正：要增加 Yaw（逆时针），应该右轮快、左轮慢
        target_rps_left  = base_linear_rps - angular_offset;
        target_rps_right = base_linear_rps + angular_offset;
    }

    // 2. 获取当前实际速度
    float current_left_rps = Encoder_GetLeftRPS();
    float current_right_rps = Encoder_GetRightRPS();

    // 3. 平滑停车状态检测：如果速度已经足够慢，则切入待机状态
    if (current_state == MOTOR_STATE_STOPPING) {
        if (fabsf(current_left_rps) < STOP_DEADBAND && fabsf(current_right_rps) < STOP_DEADBAND) {
            current_state = MOTOR_STATE_IDLE;
            arm_pid_init_f32(&pid_left, 1);
            arm_pid_init_f32(&pid_right, 1);
            Set_Left_Motor(0.0f);
            Set_Right_Motor(0.0f);
            return;
        }
    }

    // 4. 正常闭环计算
    float error_left = target_rps_left - current_left_rps;
    float error_right = target_rps_right - current_right_rps;

    float out_left = _Calculate_PID(&pid_left, error_left);
    float out_right = _Calculate_PID(&pid_right, error_right);

    // 5. 执行输出
    Set_Left_Motor(out_left);
    Set_Right_Motor(out_right);
}

// ================== 用户控制接口 ==================

void Motor_SetTargetRPS(float left_rps, float right_rps) {
    taskENTER_CRITICAL();
    target_rps_left = left_rps;
    target_rps_right = right_rps;

    if (current_state != MOTOR_STATE_ESTOP) {
        current_state = MOTOR_STATE_RUNNING;
    }
    taskEXIT_CRITICAL();
}



void Motor_NormalStop(void) {
    // 目标设为 0，让 PID 将速度拉平稳，不会瞬间断电打滑
    target_rps_left = 0.0f;
    target_rps_right = 0.0f;
    current_state = MOTOR_STATE_STOPPING;
}

void Motor_EmergencyStop(void) {
    // 立即进入急停状态，下一个 10ms 周期就会输出 PWM = 0
    current_state = MOTOR_STATE_ESTOP;
    // 同时清空积分
    arm_pid_init_f32(&pid_left, 1);
    arm_pid_init_f32(&pid_right, 1);
}

void Motor_Resume(void) {
    // 从急停或停止中恢复
    current_state = MOTOR_STATE_RUNNING;
    arm_pid_init_f32(&pid_left, 1);
    arm_pid_init_f32(&pid_right, 1);
}

Motor_State_t Motor_GetState(void) {
    return current_state;
}





/**
 * @brief 核心：带防抖处理与抗饱和的角度环计算
 */
float _Calculate_Angle_Error_Correction(float current_yaw) {
    if (!is_angle_lock_mode) return 0.0f;

    // 1. 计算最短路径误差 (解决 359度到 1度跨越问题)
    float error = target_angle - current_yaw;
    if (error > 180.0f)  error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 2. 软防抖：误差极小时直接输出0，让内环保持当前速度前行
    if (fabsf(error) < 0.5f) {
        return 0.0f; // 【关键修改】不要调用 arm_pid_init_f32 重置！
    }

    // 3. 计算 PID 输出 (此时只有 P 或 PD 在起作用)
    float angular_output = arm_pid_f32(&pid_angle, error);

    // 4. 输出绝对限幅 - 限制最大差速
    const float MAX_ANGULAR_OUT = 0.5f; // 最大差速 0.5 RPS
    if (angular_output > MAX_ANGULAR_OUT) {
        angular_output = MAX_ANGULAR_OUT;
    } else if (angular_output < -MAX_ANGULAR_OUT) {
        angular_output = -MAX_ANGULAR_OUT;
    }

    return angular_output;
}

