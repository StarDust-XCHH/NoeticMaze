//
// Created by lmtgy on 2026/3/27.
//

#ifndef NOETICMAZE_PRINTFDEBUG_H
#define NOETICMAZE_PRINTFDEBUG_H
#include <stdint.h>

#include "main.h"


// --- 新增：蓝牙接收相关宏与变量 ---
#define BT_RX_BUF_SIZE 64
// 使用 extern 声明变量（不分配内存）
extern uint8_t bt_rx_raw_buf[BT_RX_BUF_SIZE];
extern char    bt_process_buf[BT_RX_BUF_SIZE+1];
extern volatile uint8_t bt_frame_ready;


static uint8_t Calc_Checksum(uint8_t *data, uint16_t len);
static int Wait_UART_Ready(UART_HandleTypeDef *huart, uint32_t timeout_ms);
#endif //NOETICMAZE_PRINTFDEBUG_H