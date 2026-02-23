#ifndef __COM_PID_H__
#define __COM_PID_H__

#include "main.h"

#define PID_PERIOD (0.006)
#define PID_INTEGRAL_MAX  200.0f   // 积分限幅
#define PID_OUTPUT_MAX    300.0f   // PID输出限幅

// pid结构体
typedef struct
{
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float err;
    float desire;       // 期望输出
    float measure;      // 测量输出
    float last_err;     // 上一次的误差
    float integral;     // 积分累积
    float output;       // 输出
} PID_Struct;

// 单次PID计算
void Com_PID_Calc(PID_Struct *pid);
// 串级PID计算
void Com_PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid);

int16_t Com_limit(int16_t value, int16_t min, int16_t max);

#endif
