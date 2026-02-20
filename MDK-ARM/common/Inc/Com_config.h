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

#endif
