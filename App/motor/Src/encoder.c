#include "encoder.h"
#include <stdio.h>
#include <math.h>
#include "robot_state.h"
#include "FreeRTOS.h"
#include "roboConifg.h"
#include "task.h"
// 引入全局时钟变量 (CubeMX 自动生成的系统频率，如 180000000)
extern uint32_t SystemCoreClock;

#define HTIM_LEFT  htim3
#define HTIM_RIGHT htim4

// ==========================================
// 新增：DWT 核心寄存器宏定义 (ARM Cortex-M4 内核自带)
// 用于实现纳秒级别的超高精度延时测量，无需配置额外的 TIM
// ==========================================
#define DWT_CR      *(volatile uint32_t *)0xE0001000
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define DEM_CR      *(volatile uint32_t *)0xE000EDFC

static Encoder_Data_t enc_left = {0};
static Encoder_Data_t enc_right = {0};
static Odometry_Data_t odom = {0};

// 新增：记录上一次执行时的 CPU 周期数
static uint32_t last_cycle_count = 0;

void Encoder_Init(void)
{
    // 1. 初始化 DWT 周期计数器
    DEM_CR |= 0x01000000;    // 使能 DWT 外设
    DWT_CYCCNT = 0;          // 计数器清零
    DWT_CR |= 1;             // 使能 CYCCNT 计数器

    // 2. 启动硬件定时器编码器模式
    HAL_TIM_Encoder_Start(&HTIM_LEFT, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&HTIM_RIGHT, TIM_CHANNEL_ALL);

    Encoder_ResetOdometry();

    // 3. 记录第一帧的基准时间
    last_cycle_count = DWT_CYCCNT;
}

// 接口修改：将真实的 dt 作为参数传入
static void _Encoder_CalcSpeed(Encoder_Data_t *enc, TIM_HandleTypeDef *htim, float actual_dt)
{
    uint32_t current_count = __HAL_TIM_GET_COUNTER(htim);

    // 【顺手修复隐患3】: 最安全的溢出处理方式 (利用 C 语言无符号数溢出特性)
    // 强转为 int16_t，无论底层是 TIM2(32位) 还是 TIM3(16位)，都不会算错
    int16_t delta_16 = (int16_t)(current_count - enc->last_count);
    int32_t delta = (int32_t)delta_16;

    enc->last_count = current_count;
    enc->delta_ticks = delta;

    // 1. 使用实际测量的 dt 来计算 RPS
    enc->speed_rps = ((float)delta / TICKS_PER_REV) / actual_dt;

    // 2. 转换为线速度 (米/秒)
    enc->speed_ms = enc->speed_rps * WHEEL_CIRCUM;
}

void Encoder_Update(void *param)
{
    (void)param;

    // ==========================================
    // 核心修复：计算超高精度的真实 dt
    // ==========================================
    uint32_t current_cycles = DWT_CYCCNT;
    // 无符号 32 位相减，天然解决每 23.8 秒发生一次的寄存器溢出归零问题
    uint32_t delta_cycles = current_cycles - last_cycle_count;
    last_cycle_count = current_cycles;

    // 将 CPU 周期转换为实际的秒数
    float actual_dt = (float)delta_cycles / (float)SystemCoreClock;

    // 安全防线：如果 dt 极其异常（比如用了 J-Link 设断点调试停住了），强行给个默认值防止除零起飞
    if (actual_dt < 0.001f || actual_dt > 0.1f) {
        actual_dt = 0.01f;
    }

    // ==========================================
    // 后续计算全部采用真实 actual_dt
    // ==========================================
    // 1. 更新左右轮速
    _Encoder_CalcSpeed(&enc_left, &HTIM_LEFT, actual_dt);
    _Encoder_CalcSpeed(&enc_right, &HTIM_RIGHT, actual_dt);

    // 2. 计算机器人中心线速度并存入局部静态变量 odom
    odom.linear_vel = (enc_left.speed_ms + enc_right.speed_ms) / 2.0f;

    // 3. 计算微小位移增量 (必须用实际时间间隔)
    float delta_s = odom.linear_vel * actual_dt;

    // 4. 获取当前角度并转弧度 (已修正为安全的接口读取)
    float current_yaw = Get_Global_Yaw();
    float yaw_rad = current_yaw * (3.1415926f / 180.0f);

    // 5. 更新局部里程计坐标
    odom.x += delta_s * cosf(yaw_rad);
    odom.y += delta_s * sinf(yaw_rad);

    // 6. 同步到全局状态中枢
    Update_Robot_Odom_State(odom.x, odom.y, odom.linear_vel);
}

// --- 数据获取接口 ---
float Encoder_GetLeftRPS(void)  { return enc_left.speed_rps; }
float Encoder_GetRightRPS(void) { return enc_right.speed_rps; }
float Encoder_GetLeftMS(void)   { return enc_left.speed_ms; }
float Encoder_GetRightMS(void)  { return enc_right.speed_ms; }

Odometry_Data_t Encoder_GetOdometry(void) {
    Odometry_Data_t temp_odom;

    // 进入临界区，暂停任务调度与中断
    taskENTER_CRITICAL();
    temp_odom = odom;
    taskEXIT_CRITICAL();

    return temp_odom;
}

// 清零时同样需要保护
void Encoder_ResetOdometry(void) {
    taskENTER_CRITICAL();
    odom.x = INITIAL_ODOM_X;
    odom.y = INITIAL_ODOM_Y;
    odom.linear_vel = 0.0f;
    taskEXIT_CRITICAL();
}
