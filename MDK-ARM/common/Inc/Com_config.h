#ifndef __COM_CONFIG_H__
#define __COM_CONFIG_H__

#include "main.h"


// 遥控器连接状态
typedef enum
{
    REMOTE_CONNECTED=0,
    REMOTE_DISCONNECTED,
}Remote_State;

// 飞行状态
typedef enum
{
    IDLE=0,
    NORMAL,
    FIX_HEIGHT,
    FAIL,
}Flight_State;

// 油门解锁状态
typedef enum
{
    FREE=0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK,
}Thr_State;


// 遥控器数据结构体
typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t shutdown; // 关闭指令
    uint8_t fix_height; // 固定高度指令
}Remote_data;

#endif
