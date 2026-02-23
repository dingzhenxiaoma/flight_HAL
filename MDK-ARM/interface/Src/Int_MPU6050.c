#include "Int_MPU6050.h"

int32_t accel_x_offset=0;
int32_t accel_y_offset=0;
int32_t accel_z_offset=0;

int32_t gyro_x_offset=0;
int32_t gyro_y_offset=0;
int32_t gyro_z_offset=0;

void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

// 初始化MPU6050
void Int_MPU6050_Init(void)
{
    // 检查I2C通信是否正常
    uint8_t who_am_i = 0;
    Int_MPU6050_Read_Reg(MPU_DEVICE_ID_REG, &who_am_i);
    debug_printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);
    if(who_am_i != 0x68)
    {
        debug_printf("MPU6050 not found!\n");
        return;
    }

    // 重置所有寄存器的值
    Int_MPU6050_Write_Reg(0x6B, 0x80);
    uint8_t data = 0;
    uint16_t timeout = 0;
    while(data != 0x40)
    {
        Int_MPU6050_Read_Reg(0x6B, &data);
        vTaskDelay(1);
        if(++timeout > 500)
        {
            debug_printf("MPU6050 reset timeout!\n");
            return;
        }
    }
    // 使能MPU6050
    Int_MPU6050_Write_Reg(0x6B, 0x00);

    // 选择量程
    // 选择2000度/秒的陀螺仪量程
    Int_MPU6050_Write_Reg(0x1B, 3<<3);
    // 选择2g的加速度量程
    Int_MPU6050_Write_Reg(0x1C, 0x00);

    // 关闭中断
    Int_MPU6050_Write_Reg(0x38, 0x00);
    // 用户配置寄存器：不使用FIFO，不使用扩展I2C
    Int_MPU6050_Write_Reg(0x6A, 0x00);
    // 设置采样频率
    // 采样频率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV)
    Int_MPU6050_Write_Reg(0x19, 0x01);
    // 设置低通滤波的值为184Hz
    Int_MPU6050_Write_Reg(0x1A, 1);
    // 配置使用的系统时钟
    Int_MPU6050_Write_Reg(0x6B, 0x01);
    // 使能加速度传感器和角速度传感器
    Int_MPU6050_Write_Reg(0x6C, 0x00);

    // 测量值的零偏校准
    Int_MPU6050_Calculate_offset();
}

void Int_MPU6050_Calculate_offset(void)
{
    // 等待飞机停放平稳
    // 前后两次偏差小于200，连续100次
    Accel_struct current_accel={0};
    Accel_struct last_accel={0};
    uint8_t count=0;
    Int_MPU6050_Get_Accel(&last_accel);
    while(count<100)
    {
        Int_MPU6050_Get_Accel(&current_accel);
        if(abs(current_accel.accel_x-last_accel.accel_x)<400 &&
           abs(current_accel.accel_y-last_accel.accel_y)<400 &&
           abs(current_accel.accel_z-last_accel.accel_z)<400)
        {
            count++;
        }
        else
        {
            count=0;
        }
        last_accel=current_accel;
        vTaskDelay(6);
    }

    // 计算零偏
    Gyro_Accel_struct gyro_accel_Temp_data={0};
    int32_t acc_x_sum=0;
    int32_t acc_y_sum=0;
    int32_t acc_z_sum=0;

    int32_t gyro_x_sum=0;
    int32_t gyro_y_sum=0;
    int32_t gyro_z_sum=0;

    // 读取100次数据
    for(uint8_t i=0;i<100;i++)
    {
        Int_MPU6050_Get_Data(&gyro_accel_Temp_data);
        
        acc_x_sum += (gyro_accel_Temp_data.accel.accel_x-0);
        acc_y_sum += (gyro_accel_Temp_data.accel.accel_y-0);
        acc_z_sum += (gyro_accel_Temp_data.accel.accel_z-16384);
        gyro_x_sum += (gyro_accel_Temp_data.gyro.gyro_x-0);
        gyro_y_sum += (gyro_accel_Temp_data.gyro.gyro_y-0);
        gyro_z_sum += (gyro_accel_Temp_data.gyro.gyro_z-0);

        vTaskDelay(6);
    }

    // 计算平均值
    accel_x_offset = acc_x_sum/100;
    accel_y_offset = acc_y_sum/100;
    accel_z_offset = acc_z_sum/100;
    gyro_x_offset = gyro_x_sum/100;
    gyro_y_offset = gyro_y_sum/100;
    gyro_z_offset = gyro_z_sum/100;
}

// 获取MPU6050陀螺仪数据
void Int_MPU6050_Get_Gyro(Gyro_struct* gyro)
{
    // 存储角速度的寄存器地址
    // 从0x43开始，高8位在前，X,Y,Z的顺序
    uint8_t high=0;
    uint8_t low=0;
    // 读取X轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    gyro->gyro_x =((high << 8) | low)-gyro_x_offset;
    // 读取Y轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
    gyro->gyro_y =((high << 8) | low)-gyro_y_offset;
    // 读取Z轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
    gyro->gyro_z =((high << 8) | low)-gyro_z_offset;
}
// 获取MPU6050加速度数据
void Int_MPU6050_Get_Accel(Accel_struct* accel)
{
    // 存储加速度的寄存器地址
    // 从0x3B开始，高8位在前，X,Y,Z的顺序
    uint8_t high=0;
    uint8_t low=0;
    // 读取X轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    accel->accel_x =((high << 8) | low)-accel_x_offset;
    // 读取Y轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    accel->accel_y =((high << 8) | low)-accel_y_offset;
    // 读取Z轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    accel->accel_z =((high << 8) | low)-accel_z_offset;
}

// 获取MPU6050陀螺仪和加速度数据
void Int_MPU6050_Get_Data(Gyro_Accel_struct* data)
{
    // 获取陀螺仪数据
    Int_MPU6050_Get_Gyro(&data->gyro);
    // 获取加速度数据
    Int_MPU6050_Get_Accel(&data->accel);
}
