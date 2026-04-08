#include "pwm.h"
#include "tim.h"

/**
 * @brief 电机控制函数 (支持多通道拓展)
 * @param motor 电机 ID (MOTOR_LEFT 或 MOTOR_RIGHT)
 * @param speed 速度设定值 -1000 到 1000
 */
void Set_Motor_Speed(MotorID_t motor, int16_t speed)
{
    // 1. 限制输入范围
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    // 2. 根据电机 ID 选择通道
    uint32_t chA, chB;
    if (motor == MOTOR_LEFT) {
        chA = TIM_CHANNEL_1; // 左轮前进通道
        chB = TIM_CHANNEL_2; // 左轮后退通道
    } else {
        chA = TIM_CHANNEL_3; // 右轮前进通道
        chB = TIM_CHANNEL_4; // 右轮后退通道
    }

    // 3. 执行 PWM 输出逻辑
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim2, chA, (uint32_t)speed);
        __HAL_TIM_SET_COMPARE(&htim2, chB, 0);
    }
    else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim2, chA, 0);
        __HAL_TIM_SET_COMPARE(&htim2, chB, (uint32_t)(-speed));
    }
    else {
        __HAL_TIM_SET_COMPARE(&htim2, chA, 0);
        __HAL_TIM_SET_COMPARE(&htim2, chB, 0);
    }
}

/**
 * 修改扫频测试任务：让两个轮子同步动作（或交替动作）
 */
/**
 * 修改后的任务：平滑加速至 500 并保持匀速前进
 */
void Task_MotorSweep(void *param)
{
    // 目标速度设定为 500
    const int16_t FINAL_SPEED = 250;
    static int16_t current_speed = 0;
    static int16_t step = 5; // 每次进入任务增加 5
    static uint8_t is_first_run = 1;

    // 强制初次运行归零，确保复位后从 0 开始
    if (is_first_run) {
        current_speed = 0;
        is_first_run = 0;
    }

    // --- 核心逻辑：平滑加速 ---
    if (current_speed < FINAL_SPEED) {
        current_speed += step;

        // 防止加过头（溢出目标值）
        if (current_speed > FINAL_SPEED) {
            current_speed = FINAL_SPEED;
        }
    }
    // 如果由于某种原因（如手动改值）超过了 500，也可以缓慢降回 500
    else if (current_speed > FINAL_SPEED) {
        current_speed -= step;
    }

    // 执行输出
    Set_Motor_Speed(MOTOR_LEFT, current_speed);
    Set_Motor_Speed(MOTOR_RIGHT, current_speed);
}




// PWM初始化

void PWM_Init(void){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 左轮 A
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 左轮 B
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 右轮 A (新增)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // 右轮 B (新增)


}
