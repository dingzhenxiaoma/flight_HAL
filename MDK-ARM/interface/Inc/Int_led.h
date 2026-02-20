#ifndef __INT_LED_H__
#define __INT_LED_H__

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} Led_struct;

// 开启
void Int_led_turn_on(Led_struct *led);

// 关闭
void Int_led_turn_off(Led_struct *led);

// 翻转
void Int_led_toggle(Led_struct *led);

#endif
