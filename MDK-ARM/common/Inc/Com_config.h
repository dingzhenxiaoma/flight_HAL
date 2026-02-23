#ifndef __COM_CONFIG_H__
#define __COM_CONFIG_H__

#include "main.h"

#define M_PI 3.14159265

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

// 陀螺仪数据结构体
typedef struct
{
    int16_t gyro_x; // 向右为正，表示横滚角
    int16_t gyro_y; //向前为正，表示俯仰角
    int16_t gyro_z; // 逆时针为正，表示偏航角
}Gyro_struct;

typedef struct
{
    int16_t accel_x; //向前为正
    int16_t accel_y;  //向左为正
    int16_t accel_z; // 向上为正
}Accel_struct;

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
}Gyro_Accel_struct;

//解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
}Euler_struct;



#endif
