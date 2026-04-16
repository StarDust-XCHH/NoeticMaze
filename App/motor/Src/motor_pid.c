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
    taskENTER_CRITICAL();
    target_linear_v = linear_v_ms;
    target_yaw_rate = angular_v_degs;
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
    Get_Robot_State_Snapshot(&state);

    // 1. 安全拦截：IMU 未就绪或处于急停/待机状态
    if (state.imu_ready == 0 || current_state == MOTOR_STATE_ESTOP || current_state == MOTOR_STATE_IDLE) {
        Set_Left_Motor(0.0f);
        Set_Right_Motor(0.0f);
        return;
    }

    // 2. 目标指令准备 (m/s)
    float base_v = target_linear_v;
    float target_v_left  = base_v;
    float target_v_right = base_v;

    // 3. 角度环修正 (仅在锁定模式下)
    // 3. 角速度环修正 (仅在角速度模式下)
    if (is_yaw_rate_mode) {
        // A. 反馈：计算角速度误差带来的 PID 补偿
        float angular_pid_offset = _Calculate_YawRate_Error_Correction(state.yaw_rate_deg_s);

        // B. 前馈：基于运动学的理论差速计算
        // 先将目标角速度 (deg/s) 转化为 (rad/s)
        float target_yaw_rate_rads = target_yaw_rate * 3.1415926f / 180.0f;
        // 计算理论差速偏移量 (v = ω * r, 这里的 r 是轮距的一半)
        float ff_offset = (target_yaw_rate_rads * WHEEL_TRACK) / 2.0f;

        // C. 总差速 = 理论前馈 + PID反馈修正
        float total_offset = ff_offset + angular_pid_offset;

        target_v_left  = base_v - total_offset;
        target_v_right = base_v + total_offset;
    }

    // 4. 反馈采集 (RPS -> m/s)
    float current_left_v  = Encoder_GetLeftRPS()  * WHEEL_CIRCUM;
    float current_right_v = Encoder_GetRightRPS() * WHEEL_CIRCUM;

    // 5. 停车判定 (使用线速度阈值 1cm/s)
    if (current_state == MOTOR_STATE_STOPPING) {
        if (fabsf(current_left_v) < 0.01f && fabsf(current_right_v) < 0.01f) {
            current_state = MOTOR_STATE_IDLE;
            // 停车后复位 PID 内部积分，防止下次启动弹射
            arm_pid_init_f32(&pid_left, 1);
            arm_pid_init_f32(&pid_right, 1);
            Set_Left_Motor(0.0f);
            Set_Right_Motor(0.0f);
            return;
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
    // is_angle_lock_mode = 0; // 停止时通常建议关闭角度锁定
    current_state = MOTOR_STATE_STOPPING;
    taskEXIT_CRITICAL();
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
 * @brief 核心：带抗饱和的角速度环计算
 */
float _Calculate_YawRate_Error_Correction(float current_yaw_rate) { // 替换原来的 _Calculate_Angle_Error_Correction
    if (!is_yaw_rate_mode) return 0.0f;

    // 误差计算：目标角速度 - 当前角速度 (单位：deg/s)
    float error = target_yaw_rate - current_yaw_rate;

    // 角速度不存在 360 度过零问题，直接删除过零处理！

    // 死区优化：当角速度误差很小（如小于 1 deg/s）时，忽略不计，防止电机轻微抖动
    if (fabsf(error) < DIED_YAW_RATE) {
        error = 0.0f;
    }

    float angular_output = arm_pid_f32(&pid_yaw_rate, error);

    // 限幅保护：限制 PID 的最大干预量
    if (angular_output > MAX_ANGULAR_DELTA) {
        angular_output = MAX_ANGULAR_DELTA;
    } else if (angular_output < -MAX_ANGULAR_DELTA) {
        angular_output = -MAX_ANGULAR_DELTA;
    }

    return angular_output;
}

float Motor_GetTargetYawRate(void) {
    float temp;

    // 进入临界区，屏蔽中断和任务调度，充当内存屏障(Memory Barrier)
    taskENTER_CRITICAL();
    temp = target_yaw_rate;
    taskEXIT_CRITICAL();

    return temp;
}

