#ifndef __INT_MOTOR_H__
#define __INT_MOTOR_H__

#include "tim.h"
#include "Com_debug.h"

typedef struct 
{
    TIM_HandleTypeDef *tim;
    uint16_t channel;
    int16_t speed;

}Motor_struct;


// speed: 最大1000 默认200
void Int_motor_SetSpeed(Motor_struct *motor);

void Int_motor_start(Motor_struct *motor);

#endif
