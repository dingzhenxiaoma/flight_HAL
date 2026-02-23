#ifndef __APP_RECEIVE_DATA_H__
#define __APP_RECEIVE_DATA_H__

#include "Int_SI24R1.h"
#include "Com_config.h"
#include "Int_VL53L1X.h"

//定义帧头校验的值
#define FRAME_HEADER_CHECK1 'a'
#define FRAME_HEADER_CHECK2 'b'
#define FRAME_HEADER_CHECK3 'c'

// 最大重试次数
#define MAX_RETRY_TIMES 10

// 接收数据
// 0: 成功
// 1: 失败
uint8_t App_Receive_Data(void);
// 处理连接状态
void App_Process_connect_state(uint8_t res);
// 处理飞行状态
void App_Process_flight_state(void);

#endif
