#include "App_receive_data.h"

extern Remote_data remote_data;
uint8_t rx_buff[TX_PLOAD_WIDTH]={0};
extern Remote_State remote_state;
uint8_t retry_count = 0;
extern Flight_State flight_state;
Thr_State thr_state = FREE;
uint32_t max_enter_time=0;
uint32_t min_enter_time=0;

uint8_t App_Receive_Data(void)
{
    memset(rx_buff,0,TX_PLOAD_WIDTH);
    Int_SI24R1_RxPacket(rx_buff);
    if(strlen((char*)rx_buff)==0)
        return 1;
    
    // 帧头校验
    if(rx_buff[0]!=FRAME_HEADER_CHECK1 || rx_buff[1]!=FRAME_HEADER_CHECK2 || rx_buff[2]!=FRAME_HEADER_CHECK3)
        return 1;

    // 校验和校验
    uint32_t checksum = 0;
    uint32_t sum_receive=rx_buff[13]<<24|rx_buff[14]<<16|rx_buff[15]<<8|rx_buff[16];
    for(uint8_t i=0;i<13;i++)
    {
        checksum += rx_buff[i];
    }
    if(checksum != sum_receive)
        return 1;

    // 保存数据
    remote_data.thr=(rx_buff[3]<<8)|rx_buff[4];
    remote_data.yaw=(rx_buff[5]<<8)|rx_buff[6];
    remote_data.pitch=(rx_buff[7]<<8)|rx_buff[8];
    remote_data.roll=(rx_buff[9]<<8)|rx_buff[10];
    remote_data.shutdown=rx_buff[11];
    remote_data.fix_height=rx_buff[12];

    debug_printf(":%d,%d,%d,%d,%d,%d\n",remote_data.thr,remote_data.yaw,remote_data.pitch,remote_data.roll,remote_data.shutdown,remote_data.fix_height);

    return 0;
}

void App_Process_connect_state(uint8_t res)
{
    if(res == 0)
    {
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else if(res == 1)
    {
        retry_count++;
        if(retry_count >= MAX_RETRY_TIMES)
        {
            remote_state = REMOTE_DISCONNECTED;
            retry_count = 0;
        }
    }
        
}

//解锁逻辑
static uint8_t App_process_unlock(void)
{
    switch (thr_state)
    {
    case FREE:
        if(remote_data.thr >= 900)
        {
            thr_state = MAX;
            max_enter_time = xTaskGetTickCount();
        }
        break;
    case MAX:
        if(remote_data.thr < 900)
        {
            if(xTaskGetTickCount() - max_enter_time >= 1000)
            {
                thr_state = LEAVE_MAX;
            }
            else
            {
                thr_state = FREE;
            }
        }
        break;
    case LEAVE_MAX:
        if(remote_data.thr <= 100)
        {
            thr_state = MIN;
            min_enter_time = xTaskGetTickCount();
        }
        break;
    case MIN:
        if(xTaskGetTickCount() - min_enter_time <= 1000)
        {
            if(remote_data.thr > 100)
            {
                thr_state = FREE;
            }
        }
        else
        {
            thr_state = UNLOCK;
        }
        break;
    case UNLOCK:
        break;
    default:
        break;
    }

    if(thr_state==UNLOCK)
    {
        return 0;
    }

    return 1;
}

void App_Process_flight_state(void)
{
   // 使用状态机实现
   switch(flight_state)
   {
    case IDLE:
        if(App_process_unlock() == 0)
        {
            flight_state = NORMAL;
            thr_state=FREE;
        }
        break;
    case NORMAL:
        if(remote_data.fix_height == 1)
        {
            flight_state = FIX_HEIGHT;
            remote_data.fix_height = 0;
        }
        if(remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;
    case FIX_HEIGHT:
        if(remote_data.fix_height == 1)
        {
            flight_state = NORMAL;
            remote_data.fix_height = 0;
        }
        if(remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FAIL;
        }
        break;
    case FAIL:
        // 处理失联故障：缓慢停止电机
        vTaskDelay(1);
        flight_state = IDLE;
        break;
    default:
        break;
   }
}
