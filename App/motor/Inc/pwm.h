#ifndef SILLYMAZE_PWM_H
#define SILLYMAZE_PWM_H

#include <stdint.h>

// 定义电机 ID
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorID_t;

// 修改函数原型，增加 motor 参数
void Set_Motor_Speed(MotorID_t motor, int16_t speed);
void Task_MotorSweep(void *param);
void Chassis_Drive(int16_t linear, int16_t angular);
void PWM_Init(void);
#endif