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



/* --- 新增变量 --- */
static arm_pid_instance_f32 pid_angle;   // 角度环 PID 实例
static float target_angle = 0.0f;        // 目标角度 (0~360)
static float target_linear_v = 0.0f;     // 【修改】基础线速度 (单位：m/s)
static uint8_t is_angle_lock_mode = 0;   // 是否开启角度锁定模式

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
 * @brief 角度环初始化
 */
void Motor_AnglePID_Init(void) {
    // 【调整 Kp】因为输入是角度(0-180)，输出是速度(m/s)
    // 假设误差 10 度时，你希望产生 0.1m/s 的修正速度，则 Kp = 0.1 / 10 = 0.01
    pid_angle.Kp = angle_Kp; // 从小值开始测试，原先的 1.2 会导致暴走
    pid_angle.Ki = angle_Ki;
    pid_angle.Kd = angle_Kd; // 微调阻尼
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
 * @brief 设置目标角度及前行速度 (由蓝牙/上位机解析函数调用)
 * @param angle 目标角度 (0-360)
 * @param v_ms 前进的目标线速度 (直接传入 m/s)
 */
void Motor_SetTargetAngle(float angle, float v_ms) {
    taskENTER_CRITICAL();
    target_angle = angle;
    target_linear_v = v_ms; // 直接存储，不再进行单位转换
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
    if (is_angle_lock_mode) {
        float angular_offset = _Calculate_Angle_Error_Correction(state.yaw);
        target_v_left  = base_v - angular_offset;
        target_v_right = base_v + angular_offset;
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
 * @brief 核心：带防抖处理与抗饱和的角度环计算
 */
/**
 * @brief 核心：带防抖处理与抗饱和的角度环计算
 */
/**
 * @brief 核心：带防抖处理与抗饱和的角度环计算
 */
float _Calculate_Angle_Error_Correction(float current_yaw) {
    if (!is_angle_lock_mode) return 0.0f;

    float error = target_angle - current_yaw;
    // 航向角过零处理
    if (error > 180.0f)  error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 保留死区优化：让 D 项平滑归零
    if (fabsf(error) < 0.5f) {
        error = 0.0f;
    }

    float angular_output = arm_pid_f32(&pid_angle, error);

    // 【关键修改】：绝对不要去碰 pid_angle.state[2] ！！！
    // 既然 Ki=0，就不存在积分器爆炸。只在最外层卡住最大补偿速度即可。
    if (angular_output > MAX_ANGULAR_DELTA) {
        angular_output = MAX_ANGULAR_DELTA;
    } else if (angular_output < -MAX_ANGULAR_DELTA) {
        angular_output = -MAX_ANGULAR_DELTA;
    }

    return angular_output;
}

