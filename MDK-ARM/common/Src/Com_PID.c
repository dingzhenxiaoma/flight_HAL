#include "Com_PID.h"

// 单次PID计算
void Com_PID_Calc(PID_Struct *pid)
{
    // 计算误差
    pid->err = pid->measure - pid->desire;
    // 计算积分误差（带限幅，防止积分饱和）
    pid->integral += pid->err;
    if(pid->integral > PID_INTEGRAL_MAX) pid->integral = PID_INTEGRAL_MAX;
    if(pid->integral < -PID_INTEGRAL_MAX) pid->integral = -PID_INTEGRAL_MAX;

    if(pid->last_err==0)
    {
        pid->last_err = pid->err;
    }
    // 计算微分项
    float der=pid->err-pid->last_err;
    // 计算输出
    pid->output = pid->Kp * pid->err + (pid->Ki * pid->integral*PID_PERIOD) +(pid->Kd * der/PID_PERIOD);
    // 输出限幅
    if(pid->output > PID_OUTPUT_MAX) pid->output = PID_OUTPUT_MAX;
    if(pid->output < -PID_OUTPUT_MAX) pid->output = -PID_OUTPUT_MAX;
    // 更新上一次的误差
    pid->last_err = pid->err;
}
// 串级PID计算
void Com_PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid)
{
     /* 1. 先计算外环的PID */
    Com_PID_Calc(out_pid);

    /* 2. 外环的输出结果作为内环的期望值 */
    in_pid->desire = out_pid->output;

    /* 3. 再计算内环的PID */
    Com_PID_Calc(in_pid);

}

int16_t Com_limit(int16_t value, int16_t min, int16_t max)
{
    if(value>max)
    {
        value=max;
    }
    else if(value<min)
    {
        value=min;
    }
    return value;
}
