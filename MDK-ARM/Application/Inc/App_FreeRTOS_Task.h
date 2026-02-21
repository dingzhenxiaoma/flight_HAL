#ifndef __APP_FREE_RTOS_TASK_H__
#define __APP_FREE_RTOS_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "Int_IP5305T.h"
#include "Int_motor.h"
#include "Int_led.h"
#include "Com_config.h"
#include "Int_SI24R1.h"

//启动FreeRTOS操作系统
void App_FreeRTOS_start(void);

//电源管理
void power_task(void *pvParameters);
#define POWER_TASK_PRIORITY    (4)
#define POWER_TASK_STACK_SIZE    (128)
#define POWER_TASK_PERIOD    (10000)    // 10s

// 飞行控制任务
void flight_task(void *pvParameters);
#define FLIGHT_TASK_PRIORITY    (3)
#define FLIGHT_TASK_STACK_SIZE    (128)
#define FLIGHT_TASK_PERIOD    (6)    // 6ms

// 灯控任务
void led_task(void *pvParameters);
#define LED_TASK_PRIORITY    (1)
#define LED_TASK_STACK_SIZE    (128)
#define LED_TASK_PERIOD    (100)    // 100ms

// 通讯任务
void comm_task(void *pvParameters);
#define COMM_TASK_PRIORITY    (2)
#define COMM_TASK_STACK_SIZE    (128)
#define COMM_TASK_PERIOD    (6)

#endif /* __APP_FREE_RTOS_TASK_H__ */
