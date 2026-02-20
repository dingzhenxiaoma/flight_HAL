#include "App_FreeRTOS_Task.h"

TaskHandle_t power_task_handle = NULL;

void App_FreeRTOS_start(void)
{
    //电源管理任务
    xTaskCreate(power_task, "power_task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);


    // 2.启动调度器
    vTaskStartScheduler();
}

void power_task(void *pvParameters)
{
    //初始化延时时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, 10000); // 延时1000ms
        //启动电源
        Int_IP5305T_start();
    }
}
