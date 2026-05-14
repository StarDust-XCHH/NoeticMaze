#include "motor_pid.h"
#include "encoder.h"
#include "pwm.h"
#include "tim.h"      // 包含定时器句柄，如 htim2
#include <math.h>

#include "MPU6500.h"
#include "robot_state.h"
#include "FreeRTOS.h"
#include "roboConifg.h"
#include "task.h"
// 1. DSP PID 实例与目标速度
static arm_pid_instance_f32 pid_left;
static arm_pid_instance_f32 pid_right;




/* --- 新增/修改变量 --- */
static arm_pid_instance_f32 pid_yaw_rate; // 角速度环 PID 实例
static float target_yaw_rate = 0.0f;      // 目标角速度 (deg/s)
static float target_linear_v = 0.0f;      // 基础线速度 (m/s)
static uint8_t is_yaw_rate_mode = 0;      // 是否开启角速度控制模式


// 2. 状态机变量
static Motor_State_t current_state = MOTOR_STATE_IDLE;

// 3. 宏定义限制 (根据你的 CubeMX 配置，ARR = 1000 - 1)
#define MAX_PWM_DUTY 1000.0f

static float Motor_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * @brief PID 参数初始化
 */
void Motor_PID_Init(void) {
    // --- 左轮 PID 参数 (请根据实际整定) ---
    pid_left.Kp = motor_Kp;
    pid_left.Ki = motor_Ki;
    pid_left.Kd = motor_Kd;
    arm_pid_init_f32(&pid_left, 1);

    // --- 右轮 PID 参数 (请根据实际整定) ---
    pid_right.Kp = motor_Kp;
    pid_right.Ki = motor_Ki;
    pid_right.Kd = motor_Kd;
    arm_pid_init_f32(&pid_right, 1);

    current_state = MOTOR_STATE_IDLE;
}


/**
 * @brief 角速度环初始化
 */
void Motor_YawRatePID_Init(void) { // 替换原来的 Motor_AnglePID_Init
    pid_yaw_rate.Kp = yaw_rate_Kp;
    pid_yaw_rate.Ki = yaw_rate_Ki;
    pid_yaw_rate.Kd = yaw_rate_Kd;
    arm_pid_init_f32(&pid_yaw_rate, 1);
    is_yaw_rate_mode = 0;
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
 * @brief 设置目标线速度与角速度
 * @param linear_v_ms 前进的目标线速度 (m/s)
 * @param angular_v_degs 目标角速度 (deg/s)
 */
void Motor_SetTargetVelocity(float linear_v_ms, float angular_v_degs) { // 替换原来的 Motor_SetTargetAngle
    float safe_linear_v = Motor_Clamp(linear_v_ms, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    float safe_yaw_rate = Motor_Clamp(angular_v_degs,
                                      -TRACK_MAX_YAW_RATE_DEG_S,
                                      TRACK_MAX_YAW_RATE_DEG_S);

    taskENTER_CRITICAL();
    target_linear_v = safe_linear_v;
    target_yaw_rate = safe_yaw_rate;
    is_yaw_rate_mode = 1;

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
 * @brief 核心：基于目标快照计算角速度误差补偿
 */
static float _Calculate_YawRate_Error_Correction(float target_yaw_rate_snapshot,
                                                 float current_yaw_rate,
                                                 uint8_t yaw_rate_mode_snapshot) {
    if (!yaw_rate_mode_snapshot) return 0.0f;

    float error = target_yaw_rate_snapshot - current_yaw_rate;

    if (fabsf(error) < DIED_YAW_RATE) {
        error = 0.0f;
    }

    float angular_output = arm_pid_f32(&pid_yaw_rate, error);

    if (angular_output > MAX_ANGULAR_DELTA) {
        angular_output = MAX_ANGULAR_DELTA;
    } else if (angular_output < -MAX_ANGULAR_DELTA) {
        angular_output = -MAX_ANGULAR_DELTA;
    }

    return angular_output;
}

/**
 * @brief 定时执行的 PID 控制主任务 (10ms 调度)
 */
/**
 * @brief 定时执行的 PID 控制主任务 (10ms 调度)
 */
/**
 * @brief 定时执行的 PID 控制主任务 (10ms 调度)
 */
void Task_MotorPID_Update(void) { // 改为 void，不再依赖外部传参
    RobotState_t state;
    float target_linear_v_snapshot;
    float target_yaw_rate_snapshot;
    uint8_t is_yaw_rate_mode_snapshot;
    Motor_State_t current_state_snapshot;

    Get_Robot_State_Snapshot(&state);

    taskENTER_CRITICAL();
    target_linear_v_snapshot = target_linear_v;
    target_yaw_rate_snapshot = target_yaw_rate;
    is_yaw_rate_mode_snapshot = is_yaw_rate_mode;
    current_state_snapshot = current_state;
    taskEXIT_CRITICAL();

    // 1. 安全拦截：IMU 未就绪或处于急停/待机状态
    if (state.imu_ready == 0 ||
        current_state_snapshot == MOTOR_STATE_ESTOP ||
        current_state_snapshot == MOTOR_STATE_IDLE) {
        Set_Left_Motor(0.0f);
        Set_Right_Motor(0.0f);
        return;
    }

    // 2. 目标指令准备 (m/s)
    float base_v = target_linear_v_snapshot;
    float target_v_left  = base_v;
    float target_v_right = base_v;

    // 3. 角度环修正 (仅在锁定模式下)
    // 3. 角速度环修正 (仅在角速度模式下)
    if (is_yaw_rate_mode_snapshot) {
        // A. 反馈：计算角速度误差带来的 PID 补偿
        float angular_pid_offset = _Calculate_YawRate_Error_Correction(target_yaw_rate_snapshot,
                                                                       state.yaw_rate_deg_s,
                                                                       is_yaw_rate_mode_snapshot);

        // B. 前馈：基于运动学的理论差速计算
        // 先将目标角速度 (deg/s) 转化为 (rad/s)
        float target_yaw_rate_rads = target_yaw_rate_snapshot * 3.1415926f / 180.0f;
        // 计算理论差速偏移量 (v = ω * r, 这里的 r 是轮距的一半)
        float ff_offset = (target_yaw_rate_rads * WHEEL_TRACK) / 2.0f;

        // C. 总差速 = 理论前馈 + PID反馈修正
        float total_offset = ff_offset + angular_pid_offset;

        target_v_left  = base_v - total_offset;
        target_v_right = base_v + total_offset;
    }

    target_v_left = Motor_Clamp(target_v_left, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    target_v_right = Motor_Clamp(target_v_right, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

    // 4. 反馈采集 (RPS -> m/s)
    float current_left_v  = Encoder_GetLeftRPS()  * WHEEL_CIRCUM;
    float current_right_v = Encoder_GetRightRPS() * WHEEL_CIRCUM;

    // 5. 停车判定 (使用线速度阈值 1cm/s)
    if (current_state_snapshot == MOTOR_STATE_STOPPING) {
        if (fabsf(current_left_v) < 0.01f && fabsf(current_right_v) < 0.01f) {
            uint8_t stop_confirmed = 0U;

            taskENTER_CRITICAL();
            if (current_state == MOTOR_STATE_STOPPING) {
                current_state = MOTOR_STATE_IDLE;
                target_linear_v = 0.0f;
                target_yaw_rate = 0.0f;
                stop_confirmed = 1U;
            }
            taskEXIT_CRITICAL();

            if (stop_confirmed != 0U) {
                // 停车后复位 PID 内部积分，防止下次启动弹射
                arm_pid_init_f32(&pid_left, 1);
                arm_pid_init_f32(&pid_right, 1);
                Set_Left_Motor(0.0f);
                Set_Right_Motor(0.0f);
                return;
            }
        }
    }

    // 6. 速度闭环 PID
    float out_left  = _Calculate_PID(&pid_left, target_v_left - current_left_v);
    float out_right = _Calculate_PID(&pid_right, target_v_right - current_right_v);

    Set_Left_Motor(out_left);
    Set_Right_Motor(out_right);
}






void Motor_NormalStop(void) {
    taskENTER_CRITICAL();
    target_linear_v = 0.0f; // 清零线速度目标
    target_yaw_rate = 0.0f;
    // is_angle_lock_mode = 0; // 停止时通常建议关闭角度锁定
    current_state = MOTOR_STATE_STOPPING;
    taskEXIT_CRITICAL();
}

void Motor_EmergencyStop(void) {
    // 立即进入急停状态，下一个 10ms 周期就会输出 PWM = 0
    taskENTER_CRITICAL();
    current_state = MOTOR_STATE_ESTOP;
    target_linear_v = 0.0f;
    target_yaw_rate = 0.0f;
    taskEXIT_CRITICAL();
    // 同时清空积分
    arm_pid_init_f32(&pid_left, 1);
    arm_pid_init_f32(&pid_right, 1);
}

void Motor_Resume(void) {
    // 从急停或停止中恢复
    arm_pid_init_f32(&pid_left, 1);
    arm_pid_init_f32(&pid_right, 1);

    taskENTER_CRITICAL();
    current_state = MOTOR_STATE_RUNNING;
    taskEXIT_CRITICAL();
}

Motor_State_t Motor_GetState(void) {
    Motor_State_t temp;

    taskENTER_CRITICAL();
    temp = current_state;
    taskEXIT_CRITICAL();

    return temp;
}

float Motor_GetTargetYawRate(void) {
    float temp;

    // 进入临界区，屏蔽中断和任务调度，充当内存屏障(Memory Barrier)
    taskENTER_CRITICAL();
    temp = target_yaw_rate;
    taskEXIT_CRITICAL();

    return temp;
}

