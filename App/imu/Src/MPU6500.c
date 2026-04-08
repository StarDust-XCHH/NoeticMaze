// E:\EBU6475MicroprocessorSystemsDesign\SillyMaze\App\imu\Src\MPU6500.c

#include <math.h>
#include <MPU6500.h>

#include "robot_state.h"
#include "spi.h"
#include "dsp/fast_math_functions.h"
const signed char gyro_orientation[9] = {1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1};

// 私有常量封装
static const float32_t Q30_F = 1073741824.0f;
static const float32_t K_F = 180.0f / 3.1415926535f;

// 外部变量引用（如果 main.c 还需要用到 start_time）
extern uint32_t start_time;

// --- 新增：SPI 底层驱动代码 ---

/**
 * @brief SPI 连续写寄存器（支持单字节和多字节突发写入，这对 DMP 加载固件极度重要）
 */
uint8_t MPU_SPI_WriteRegs(uint8_t reg_addr, uint8_t length, uint8_t *data) {
	uint8_t tx_addr = reg_addr & 0x7F; // 最高位清 0，表示写操作

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET); // 拉低 CS 选中芯片
	HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 10);                       // 发送寄存器地址
	HAL_SPI_Transmit(&hspi2, data, length, 100);                     // 连续发送数据
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);   // 拉高 CS 结束通信

	return 0;
}

/**
 * @brief SPI 连续读寄存器
 */
uint8_t MPU_SPI_ReadRegs(uint8_t reg_addr, uint8_t length, uint8_t *data) {
	uint8_t tx_addr = reg_addr | 0x80; // 最高位置 1，表示读操作

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET); // 拉低 CS 选中芯片
	HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 10);                       // 发送寄存器地址
	HAL_SPI_Receive(&hspi2, data, length, 100);                      // 连续接收数据
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);   // 拉高 CS 结束通信

	return 0;
}

// 新增：适配任务调度器的 IMU 更新函数
void Task_IMU_Update(void *arg) {
	IMU_Yaw_Handler_t *handler = (IMU_Yaw_Handler_t *)arg;
	if (handler != NULL) {
		IMU_Update(handler);
        
		// 【关键逻辑】将校准状态和数据同步到全局中枢
		Update_Robot_IMU_State(
			handler->compensated_yaw,
			handler->gyro_z,
			handler->is_calibrated  // 这里的 1 会让 PID 任务解锁
		);
	}
}


// 打印偏航角（Yaw）的任务
void Task_ReportYaw(void *arg) {
	// 1. 将 void 指针强转回 float 指针
	float *yaw_ptr = (float *)arg;

	// 2. 检查指针是否为空（防御性编程）
	if (yaw_ptr != NULL) {
		// 3. 解引用获取最新的值并打印
		printf("Yaw: %.2f°\n", *yaw_ptr);
	}
}

/**
 * @brief 将角度映射到 [0, 360) 空间
 */
float32_t MapTo360(float32_t angle) {
	float32_t res = fmodf(angle, 360.0f);
	if (res < 0) res += 360.0f;
	return res;
}

/**
 * @brief IMU 数据处理核心函数 (抗阻塞版本)
 * @param handler 指向 IMU 处理结构体的指针
 */
void IMU_Update(IMU_Yaw_Handler_t *handler) {
    int16_t gyro[3], acce[3], sensors;
    int32_t quat[4];
    uint32_t sensor_timestamp;
    unsigned char more;
    uint8_t success_flag = 0;

    // --- 抗阻塞逻辑开始 ---
    // 只要 FIFO 有数据就一直读，直到读完为止
    while (dmp_read_fifo(gyro, acce, (long *)quat, (unsigned long *)&sensor_timestamp, &sensors, &more) == 0) {
        if (sensors & INV_WXYZ_QUAT) {
            success_flag = 1; // 标记至少读到了一组有效的四元数
        }


    	// 3. --- 新增：提取陀螺仪数据 ---
    	if (sensors & INV_XYZ_GYRO) {

    		handler->gyro_z = (float)gyro[2] / 16.4f;
    	}

        if (more == 0) {
            break; // FIFO 已空，退出循环
        }
    }

    // 如果一次有效数据都没读到，直接返回
    if (!success_flag) return;
    // --- 抗阻塞逻辑结束 ---

    // 2. 仅针对“最新”的一组四元数进行转换 (DSP加速)
    float32_t f_q0 = (float32_t)quat[0] / Q30_F;
    float32_t f_q1 = (float32_t)quat[1] / Q30_F;
    float32_t f_q2 = (float32_t)quat[2] / Q30_F;
    float32_t f_q3 = (float32_t)quat[3] / Q30_F;

    // 计算 Yaw (偏航角)
    float32_t y_num = 2.0f * (f_q1 * f_q2 + f_q0 * f_q3);
    float32_t y_den = f_q0 * f_q0 + f_q1 * f_q1 - f_q2 * f_q2 - f_q3 * f_q3;

    float32_t raw_fyaw_deg;
    arm_atan2_f32(y_num, y_den, &raw_fyaw_deg);

    // 标准化到 [-180, 180]
    raw_fyaw_deg = raw_fyaw_deg * K_F;
    // 使用 fmodf 确保在范围内
    raw_fyaw_deg = fmodf(raw_fyaw_deg + 180.0f, 360.0f);
    if (raw_fyaw_deg < 0) raw_fyaw_deg += 360.0f;
    handler->raw_yaw = raw_fyaw_deg - 180.0f;

    // 3. 状态机：校准逻辑 (保持不变)
    uint32_t current_time = HAL_GetTick();

    if (!handler->is_calibrated) {
        if (handler->stable_start_ms == 0) {
            handler->stable_start_ms = current_time;
            handler->max_val = handler->raw_yaw;
            handler->min_val = handler->raw_yaw;
        }

        if (handler->raw_yaw > handler->max_val) handler->max_val = handler->raw_yaw;
        if (handler->raw_yaw < handler->min_val) handler->min_val = handler->raw_yaw;

        if (current_time - handler->stable_start_ms >= 5000) {
            if ((handler->max_val - handler->min_val) <= 0.1f) {
                handler->offset = handler->raw_yaw;
                handler->is_calibrated = 1;
                start_time = HAL_GetTick(); // 重置漂移积分起点
            } else {
                handler->stable_start_ms = current_time;
                handler->max_val = handler->raw_yaw;
                handler->min_val = handler->raw_yaw;
            }
        }
        handler->compensated_yaw = 0.0f;
    }
    else {
        // 4. 已校准：计算相对角度 + 漂移补偿
        float t_elapsed = (current_time - start_time) / 1000.0f;
        float relative_yaw = handler->raw_yaw - handler->offset;

        // 映射到 [0, 360) 并应用温漂率补偿
        handler->compensated_yaw = MapTo360(relative_yaw - (handler->drift_rate * t_elapsed));
    }
}


static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

//        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
//        mpu_set_accel_bias_6050_reg(accel);
#endif
		return 0;
    }
	else
	{
		return -1;
	}
}

// 1. 完全替换写入函数
uint8_t Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{
	// 忽略 slave_addr，直接用 SPI 写入
	if (MPU_SPI_WriteRegs(reg_addr, length, data) != 0) return -1;
	return 0;
}

// 2. 完全替换读取函数 (记得删掉你之前写的 g_i2c_error_count 等变量)
uint8_t Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{
	// 忽略 slave_addr，直接用 SPI 读取
	if (MPU_SPI_ReadRegs(reg_addr, length, data) != 0) return -1;
	return 0;
}


// 3. 彻底精简初始化函数
HAL_StatusTypeDef MPU6500Init(void)
{
	struct int_param_s int_param;

	// --- 删除了所有 HAL_I2C_DeInit, HAL_GPIO_Init 和手动产生脉冲的代码 ---
	// 因为使用 SPI 模式，上电时只需确保 CS 为高电平，之后发送数据 MPU6500 就会自动切入 SPI 模式

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100); // 给 MPU6500 留一点上电复位时间

	// 以下是原汁原味的 DMP 初始化流程，一行不用改！
	if(mpu_init(&int_param)){
		return HAL_ERROR;
	}
	if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
		return HAL_ERROR;
	}
	if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
		return HAL_ERROR;
	}
	if(mpu_set_sample_rate(DEFAULT_MPU_HZ)){
		return HAL_ERROR;
	}
	if(dmp_load_motion_driver_firmware()){
		return HAL_ERROR;
	}
	if(dmp_set_orientation( inv_orientation_matrix_to_scalar(gyro_orientation))){
		return HAL_ERROR;
	}
	if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
					DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
					 DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)){
		return HAL_ERROR;
					 }
	if(dmp_set_fifo_rate(DEFAULT_MPU_HZ)){
		return HAL_ERROR;
	}
	if(mpu_set_dmp_state(1)){
		return HAL_ERROR;
	}
	if(run_self_test()){
		return HAL_ERROR;
	}

	return HAL_OK;
}

void mdelay(__IO uint32_t ms)
{
	HAL_Delay(ms);
}
uint32_t get_tick_count(unsigned long *ms)
{
	*ms = HAL_GetTick();
	return 0;
}
