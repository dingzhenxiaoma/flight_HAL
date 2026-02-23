#include "App_flight.h"

extern Remote_data remote_data;
Gyro_Accel_struct gyro_accel_data = {0};
Euler_struct euler_angle = {0};
extern Flight_State flight_state;
extern uint16_t fix_height;
extern TaskHandle_t comm_task_handle;

int16_t last_gyro_x = 0;
int16_t last_gyro_y = 0;
int16_t last_gyro_z = 0;

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

/*====================== 6个 姿态 PID定义 开始=====================*/
/*   外环: 俯仰角度  内环: 沿Y轴的角速度*/
// static PID_Struct pitchPid = {.Kp = -6.8f, .Ki = 0.0f, .Kd = 0.0f};
// static PID_Struct gyroYPid = {.Kp = 1.7f, .Ki = 0.0f, .Kd = 0.08f};

// /*   外环: 横滚角度  内环: 沿X轴的角速度*/
// static PID_Struct rollPid  = {.Kp = -6.8, .Ki = 0, .Kd = 0};
// static PID_Struct gyroXPid = {.Kp = -1.7, .Ki = 0, .Kd = -0.08f};

// /*   外环: 偏航角度  内环: 沿Z轴的角速度*/
// static PID_Struct yawPid   = {.Kp = -2.5, .Ki = 0, .Kd = 0};
// static PID_Struct gyroZPid = {.Kp = -1.0, .Ki = 0, .Kd = 0};

static PID_Struct pitchPid = {.Kp = -7.0f, .Ki = 0.0f, .Kd = 0.0f};
static PID_Struct gyroYPid =  {.Kp = 3.00, .Ki = 0.00, .Kd = 0.50};

/*   外环: 横滚角度  内环: 沿X轴的角速度*/
static PID_Struct rollPid  ={.Kp = -7.00, .Ki = 0.00, .Kd = 0.00};
static PID_Struct gyroXPid = {.Kp = 3.00, .Ki = 0.00, .Kd = 0.50};

/*   外环: 偏航角度  内环: 沿Z轴的角速度*/
static PID_Struct yawPid   ={.Kp = -3.00, .Ki = 0.00, .Kd = 0.00};
static PID_Struct gyroZPid = {.Kp = -5.00, .Ki = 0.00, .Kd = 0.00};
/*====================== 6个 姿态PID定义 结束=====================*/

// 定高的PID结构体
static PID_Struct heightPid = {.Kp = -0.60, .Ki = 0.00, .Kd = -0.20};

// 飞控任务初始化
void App_Flight_Init(void)
{
    // 初始化MPU6050
    Int_MPU6050_Init();
    // 初始化电机
    Int_motor_start(&left_top_motor);
    Int_motor_start(&left_bottom_motor);
    Int_motor_start(&right_top_motor);
    Int_motor_start(&right_bottom_motor);

    // 初始化距离传感器
    Int_VL53L1X_Init();
}

//根据陀螺仪测量的数据计算欧拉角
void App_Flight_Calc_Euler(void)
{
    Int_MPU6050_Get_Data(&gyro_accel_data);

    // 对角速度进行一阶低通滤波
    last_gyro_x = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x, last_gyro_x);
    last_gyro_y = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y, last_gyro_y);
    last_gyro_z = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z, last_gyro_z);
    gyro_accel_data.gyro.gyro_x = last_gyro_x;
    gyro_accel_data.gyro.gyro_y = last_gyro_y;
    gyro_accel_data.gyro.gyro_z = last_gyro_z;

    //debug_printf(":%d,%d,%d\n", gyro_accel_data.gyro.gyro_x, gyro_accel_data.gyro.gyro_y, gyro_accel_data.gyro.gyro_z);

    // 对加速度进行卡尔曼滤波
    gyro_accel_data.accel.accel_x = Common_Filter_KalmanFilter(&kfs[0], gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = Common_Filter_KalmanFilter(&kfs[1], gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = Common_Filter_KalmanFilter(&kfs[2], gyro_accel_data.accel.accel_z);
    
    //debug_printf(":%d,%d,%d\n", gyro_accel_data.accel.accel_x, gyro_accel_data.accel.accel_y, gyro_accel_data.accel.accel_z);

    // 计算欧拉角--使用互补解算
    // euler_angle.pitch = atan2(gyro_accel_data.accel.accel_x*1.0, gyro_accel_data.accel.accel_z*1.0)/M_PI*180.0;
    // euler_angle.roll = atan2(gyro_accel_data.accel.accel_y*1.0, gyro_accel_data.accel.accel_z*1.0)/M_PI*180.0;
    // gyro_z_sum+= (gyro_accel_data.gyro.gyro_z*2000/32768.0)*0.006;
    // euler_angle.yaw = gyro_z_sum;

    Common_IMU_GetEulerAngle(&gyro_accel_data, &euler_angle,0.006);

    //debug_printf(":%d,%d,%d\n", (int)euler_angle.pitch, (int)euler_angle.roll, (int)euler_angle.yaw);
}

// 处理PID
void App_Flight_Pid_process(void)
{
    // 赋值目标值和测量值
    // 平稳飞行：0，遥控：遥控的值
    // 控制范围：-10~10
    
    // 俯仰角度PID
    pitchPid.desire = (remote_data.pitch-500)/50.0;
    pitchPid.measure = euler_angle.pitch;
    gyroYPid.measure = gyro_accel_data.gyro.gyro_y*2000.0/32768.0;

    Com_PID_Calc_Chain(&pitchPid, &gyroYPid);

     // 横滚角度PID
     rollPid.desire = (remote_data.roll-500)/50.0;
     rollPid.measure = euler_angle.roll;
     gyroXPid.measure = gyro_accel_data.gyro.gyro_x*2000.0/32768.0;

     Com_PID_Calc_Chain(&rollPid, &gyroXPid);

     // 偏航角度PID
     yawPid.desire = (remote_data.yaw-500)/50.0;
     yawPid.measure = euler_angle.yaw;
     gyroZPid.measure = gyro_accel_data.gyro.gyro_z*2000.0/32768.0;

     Com_PID_Calc_Chain(&yawPid, &gyroZPid);

}

// 电机控制
void App_Flight_Motor_Control(void)
{
    // 1. 首先判断当前飞机的飞行状态
    switch (flight_state)
    {
    case IDLE:
        // 一旦进入加锁状态 =>  需要将电机速度设置为0
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
        break;
    case NORMAL:
        // 俯仰角 => 向前飞有角速度 => 正误差 => 需要一个向后飞的反馈效果 => 前两个电机转的快 后两个转的慢
        // 不同重要程度的PID控制结果 可以进行适当的限制
        left_top_motor.speed = remote_data.thr + gyroYPid.output - gyroXPid.output + Com_limit(gyroZPid.output, -100, 100);
        left_bottom_motor.speed = remote_data.thr - gyroYPid.output - gyroXPid.output - Com_limit(gyroZPid.output, -100, 100);
        right_top_motor.speed = remote_data.thr + gyroYPid.output + gyroXPid.output - Com_limit(gyroZPid.output, -100, 100);
        right_bottom_motor.speed = remote_data.thr - gyroYPid.output + gyroXPid.output + Com_limit(gyroZPid.output, -100, 100);
        break;
    case FIX_HEIGHT:
        // 只有定高状态才需要进行定高的PID计算  => 定高同时也需要平稳飞行
        left_top_motor.speed = remote_data.thr + gyroYPid.output - gyroXPid.output + Com_limit(gyroZPid.output, -100, 100) + heightPid.output;
        left_bottom_motor.speed = remote_data.thr - gyroYPid.output - gyroXPid.output - Com_limit(gyroZPid.output, -100, 100) + heightPid.output;
        right_top_motor.speed = remote_data.thr + gyroYPid.output + gyroXPid.output - Com_limit(gyroZPid.output, -100, 100) + heightPid.output;
        right_bottom_motor.speed = remote_data.thr - gyroYPid.output + gyroXPid.output + Com_limit(gyroZPid.output, -100, 100) + heightPid.output;
        break;
    case FAIL:
        // 进行故障处理 => 一直处理  到满足条件 修改状态为IDLE
        // 6ms => 降低速度2点
        left_top_motor.speed -= 2;
        left_bottom_motor.speed -= 2;
        right_top_motor.speed -= 2;
        right_bottom_motor.speed -= 2;
        if (left_top_motor.speed <= 0 && left_bottom_motor.speed <= 0 && right_top_motor.speed <= 0 && right_bottom_motor.speed <= 0)
        {
            // 故障处理完成  电机转速都已经降低为0
            xTaskNotifyGive(comm_task_handle);
        }

        break;
    default:
        break;
    }

    // 限制电机速度的上限值
    // 可以通过提供速度上限  让飞行更加平稳
    left_top_motor.speed = Com_limit(left_top_motor.speed, 0, 700);
    left_bottom_motor.speed = Com_limit(left_bottom_motor.speed, 0, 700);
    right_top_motor.speed = Com_limit(right_top_motor.speed, 0, 700);
    right_bottom_motor.speed = Com_limit(right_bottom_motor.speed, 0, 700);

    // 安全限制 => 当油门设置为<50时 => 强制将速度设置为0
    if (remote_data.thr < 50)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
    }

    // 2. 设置电机速度
    Int_motor_SetSpeed(&left_top_motor);
    Int_motor_SetSpeed(&left_bottom_motor);
    Int_motor_SetSpeed(&right_top_motor);
    Int_motor_SetSpeed(&right_bottom_motor);
}

// 定高
void App_Flight_Fix_Height(void)
{
    heightPid.desire = fix_height;
    heightPid.measure = Int_VL53L1X_GetDistance();
    Com_PID_Calc(&heightPid);
}
