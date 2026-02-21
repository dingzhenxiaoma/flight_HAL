#include "App_FreeRTOS_Task.h"

TaskHandle_t power_task_handle = NULL;

TaskHandle_t flight_task_handle = NULL;
// 电机结构体
Motor_struct left_top_motor = {
    .tim = &htim3,
    .channel = TIM_CHANNEL_1,
    .speed = 200,
};
Motor_struct left_bottom_motor = {
    .tim = &htim4,
    .channel = TIM_CHANNEL_4,
    .speed = 200,
};
Motor_struct right_top_motor = {
    .tim = &htim2,
    .channel = TIM_CHANNEL_2,
    .speed = 200,
};
Motor_struct right_bottom_motor = {
    .tim = &htim1,
    .channel = TIM_CHANNEL_3,
    .speed = 200,
};

TaskHandle_t led_task_handle = NULL;
// led结构体
Led_struct left_top_led = {
    .port = LED1_GPIO_Port,
    .pin = LED1_Pin,
};
Led_struct right_top_led = {
    .port = LED2_GPIO_Port,
    .pin = LED2_Pin,
};
Led_struct right_bottom_led = {
    .port = LED3_GPIO_Port,
    .pin = LED3_Pin,
};
Led_struct left_bottom_led = {
    .port = LED4_GPIO_Port,
    .pin = LED4_Pin,
};

// 当前连接状态
Remote_State remote_state = REMOTE_DISCONNECTED;
// 当前飞行状态
Flight_State flight_state = IDLE;    


TaskHandle_t comm_task_handle = NULL;

void App_FreeRTOS_start(void)
{
    //电源管理任务
    xTaskCreate(power_task, "power_task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);

    // 飞行控制任务
    xTaskCreate(flight_task, "flight_task", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &flight_task_handle);

    // 灯控任务
    xTaskCreate(led_task, "led_task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_handle);
    
    // 通信任务
    xTaskCreate(comm_task, "comm_task", COMM_TASK_STACK_SIZE, NULL, COMM_TASK_PRIORITY, &comm_task_handle);

    // 启动调度器
    vTaskStartScheduler();
}


// 电源管理任务
void power_task(void *pvParameters)
{
    //初始化延时时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, POWER_TASK_PERIOD); // 延时10s
        //启动电源
        Int_IP5305T_start();
    }
}

// 飞行控制任务
void flight_task(void *pvParameters)
{
    //初始化延时时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 设置电机速度

        // 启动电机

        vTaskDelayUntil(&xLastWakeTime, FLIGHT_TASK_PERIOD); // 延时6ms
    }
}

void led_task(void *pvParameters)
{
    //初始化延时时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t count = 0;
    while (1)
    {
        count++;
        // 前两个灯表示连接状态
        // 根据连接状态控制led
        if (remote_state == REMOTE_CONNECTED)
        {
            Int_led_turn_on(&left_top_led);
            Int_led_turn_on(&right_top_led);
        }
        else
        {
            Int_led_turn_off(&left_top_led);
            Int_led_turn_off(&right_top_led);
        }

        // 后两个灯表示飞行状态
        switch (flight_state)
        {
        case IDLE:
        {
            // 空闲, 后两个灯慢闪，500ms亮，500ms灭
            if(count % 5 == 0)
            {
                Int_led_toggle(&left_bottom_led);
                Int_led_toggle(&right_bottom_led);
            }
            break;
        }
        case NORMAL:
        {
            // 正常飞行, 后两个灯快闪，200ms亮，200ms灭
            if(count % 2 == 0 )
            {
                Int_led_toggle(&left_bottom_led);
                Int_led_toggle(&right_bottom_led);
            }
            break;
        }
        case FIX_HEIGHT:
        {
            // 固定高度, 后两个灯亮
            Int_led_turn_on(&left_bottom_led);
            Int_led_turn_on(&right_bottom_led);
            break;
        }
        case FAIL:
        {
            // 故障, 后两个灯灭
            Int_led_turn_off(&left_bottom_led);
            Int_led_turn_off(&right_bottom_led);
            break;
        }
        default:
            break;
        }

        // count计数重置
        if(count == 10)
        {
            count = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, LED_TASK_PERIOD); // 延时100ms
    }
}

uint8_t comm_buff[TX_PLOAD_WIDTH+1] = {0};

void comm_task(void *pvParameters)
{
    //初始化延时时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        //接收数据到缓冲区
        uint8_t rx_res = Int_SI24R1_RxPacket(comm_buff);
        if(rx_res == 0)
        {
            // 打印接收到的数据
            debug_printf("Received: %s\n", comm_buff);
        }

        vTaskDelayUntil(&xLastWakeTime, COMM_TASK_PERIOD); // 延时COMM_TASK_PERIOD秒
    }
}
