//
// Created by lmtgy on 2026/3/27.
//

#ifndef NOETICMAZE_PRINTFDEBUG_H
#define NOETICMAZE_PRINTFDEBUG_H
#include <stdint.h>


// --- 新增：蓝牙接收相关宏与变量 ---
#define BT_RX_BUF_SIZE 64
// 使用 extern 声明变量（不分配内存）
extern uint8_t bt_rx_raw_buf[BT_RX_BUF_SIZE];
extern char    bt_process_buf[BT_RX_BUF_SIZE];
extern volatile uint8_t bt_frame_ready;

#endif //NOETICMAZE_PRINTFDEBUG_H