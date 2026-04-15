#ifndef __MPU6500_H__
#define __MPU6500_H__

#include "stm32f4xx_hal.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <stdio.h>

#include "arm_math_types.h"
#include "main.h"

#define MPU6500I2C hi2c1

/* 兼容旧版MPU引脚命名（CubeMX标签已改为I2C1_*） */
#ifndef MPU_SCL_Pin
#define MPU_SCL_Pin I2C1_SCL_Pin
#endif
#ifndef MPU_SCL_GPIO_Port
#define MPU_SCL_GPIO_Port I2C1_SCL_GPIO_Port
#endif
#ifndef MPU_SDA_Pin
#define MPU_SDA_Pin I2C1_SDA_Pin
#endif
#ifndef MPU_SDA_GPIO_Port
#define MPU_SDA_GPIO_Port I2C1_SDA_GPIO_Port
#endif

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

typedef struct {
    float raw_yaw;          // 原始数据 [-180, 180]
    float offset;           // 补偿量（初始化时的原始 yaw）
    float compensated_yaw;  // 映射后的输出 [0, 360)

    uint8_t is_calibrated;  // 是否完成初始化补偿
    uint32_t stable_start_ms; // 开始记录稳定性的时间
    float max_val;          // 窗口内最大值
    float min_val;          // 窗口内最小值
    // --- 新增配置参数 ---
    float32_t drift_rate;  // 漂移率，在这里存储，方便 main 修改


    // --- 新增：Z轴角速度 ---
    float gyro_z;           // 单位：度/秒 (deg/s) 或 弧度/秒 (rad/s)

} IMU_Yaw_Handler_t;

// 打印偏航角（Yaw）的任务
float MapTo360(float angle);
void IMU_Update(IMU_Yaw_Handler_t *handler);

HAL_StatusTypeDef  MPU6500Init(void);	
uint8_t Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);//0 if success else -1
uint8_t Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);//0 if success else -1
void mdelay(__IO uint32_t ms);
uint32_t get_tick_count(unsigned long *ms);


// 新增：适配任务调度器的 IMU 更新函数
void Task_IMU_Update(void *arg);




#endif
