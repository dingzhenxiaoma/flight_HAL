#ifndef __APP_FLIGHT_H__
#define __APP_FLIGHT_H__

#include "Int_MPU6050.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "math.h"
#include "Com_config.h"
#include "Com_IMU.h"
#include "Com_PID.h"
#include "App_FreeRTOS_Task.h"
#include "Int_motor.h"
#include "Int_VL53L1X.h"

// 飞控任务初始化
void App_Flight_Init(void);
//根据陀螺仪测量的数据计算欧拉角
void App_Flight_Calc_Euler(void);
// 处理PID
void App_Flight_Pid_process(void);
// 电机控制
void App_Flight_Motor_Control(void);
// 定高
void App_Flight_Fix_Height(void);

#endif
