#ifndef __APP_FREE_RTOS_TASK_H__
#define __APP_FREE_RTOS_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "Int_IP5305T.h"

//启动FreeRTOS操作系统
void App_FreeRTOS_start(void);

//电源管理
void power_task(void *pvParameters);
#define POWER_TASK_PRIORITY    (4)
#define POWER_TASK_STACK_SIZE    (128)

#endif /* __APP_FREE_RTOS_TASK_H__ */
