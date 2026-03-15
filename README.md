# Flight_HAL — STM32 四旋翼飞控系统

基于 STM32F103C8T6 + FreeRTOS 的四旋翼无人机飞行控制器，支持姿态控制、定高飞行、2.4GHz 无线遥控和电池管理。

## 硬件平台

| 组件 | 型号/规格 |
|------|-----------|
| MCU | STM32F103C8T6 (Cortex-M3, 72MHz, 64KB Flash, 20KB RAM) |
| IMU | MPU6050 六轴（陀螺仪 + 加速度计），I2C1 |
| 测距 | VL53L1X 激光测距传感器（定高），I2C2 |
| 无线 | SI24R1 2.4GHz（兼容 nRF24L01），SPI1 |
| 电源 | IP5305T 电源管理 IC |
| 电机 | 4路 PWM（TIM1/2/3/4 各一通道，周期 1ms） |
| LED | 4 颗状态指示灯 |
| 调试 | USART2 串口 + SWD |

## 功能特性

- **姿态解算** — 互补滤波 + 卡尔曼滤波融合陀螺仪与加速度计数据，计算欧拉角
- **串级 PID 控制** — 外环（角度）→ 内环（角速度），分别控制 Pitch / Roll / Yaw
- **定高飞行** — VL53L1X 测距 + 高度 PID，24ms 周期更新
- **无线遥控** — 17 字节数据帧（油门、偏航、俯仰、横滚、指令 + 校验和），双向通信回传电池电压
- **飞行状态机** — IDLE（锁定）→ NORMAL（自由飞行）→ FIX_HEIGHT（定高）→ FAIL（失联保护）
- **失联保护** — 连续 10 次接收失败后自动降落、关闭电机
- **解锁手势** — 油门摇杆快速推高再拉低触发解锁
- **电池监测** — ADC 采集电压并通过无线回传遥控器

## 实时任务（FreeRTOS）

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| flight_task | 3 | 6ms | 姿态解算、PID 计算、电机输出、定高控制 |
| comm_task | 4 | 10ms | 无线接收/发送、飞行状态处理、连接管理 |
| power_task | 4 | 10s | 关机监听、电源 IC 控制 |
| led_task | 1 | 100ms | LED 状态指示（慢闪=锁定，快闪=飞行，常亮=定高，灭=失联） |

## 项目结构

```
flight_HAL/
├── Core/                           # STM32CubeMX 生成的 HAL 初始化代码
│   ├── Inc/                        # gpio.h, adc.h, i2c.h, spi.h, tim.h, usart.h
│   └── Src/                        # 对应 .c 文件
├── Drivers/                        # STM32 HAL 库 + CMSIS
├── MDK-ARM/                        # Keil 工程及应用代码
│   ├── Application/                # 应用层
│   │   ├── App_FreeRTOS_Task.*     #   FreeRTOS 任务调度
│   │   ├── App_flight.*            #   飞行控制（姿态+PID+电机混控）
│   │   └── App_receive_data.*      #   无线数据接收与状态管理
│   ├── common/                     # 公共库
│   │   ├── Com_config.h            #   全局数据结构与枚举
│   │   ├── Com_PID.*               #   PID 控制器
│   │   ├── Com_IMU.*               #   IMU 姿态解算
│   │   ├── Com_filter.*            #   低通滤波 / 卡尔曼滤波
│   │   └── Com_debug.h             #   调试打印
│   ├── interface/                  # 驱动层
│   │   ├── Int_MPU6050.*           #   MPU6050 IMU 驱动
│   │   ├── Int_SI24R1.*            #   SI24R1 无线模块驱动
│   │   ├── Int_VL53L1X.*           #   VL53L1X 测距驱动
│   │   ├── Int_motor.*             #   电机 PWM 控制
│   │   ├── Int_led.*               #   LED 控制
│   │   ├── Int_bat_ADC.*           #   电池 ADC 采集
│   │   └── Int_IP5305T.*           #   电源管理 IC 驱动
│   ├── FreeRTOS/                   # FreeRTOS 内核
│   └── flight_HAL.uvprojx         # Keil 工程文件
└── flight_HAL.ioc                  # STM32CubeMX 配置文件
```

## 构建与烧录

**工具链：** Keil MDK-ARM V5 + ARM Compiler V5

1. 使用 Keil uVision 打开 `MDK-ARM/flight_HAL.uvprojx`
2. 编译项目（F7）
3. 通过 ST-Link / SWD 烧录到 STM32F103C8T6

**引脚修改：** 使用 STM32CubeMX 打开 `flight_HAL.ioc` 重新配置外设后重新生成代码。

## 控制输入范围

| 通道 | 范围 | 说明 |
|------|------|------|
| 油门 (Throttle) | 0 ~ 1000 | 电机基础转速 |
| 俯仰 (Pitch) | -500 ~ 500 | 对应 ±10° |
| 横滚 (Roll) | -500 ~ 500 | 对应 ±10° |
| 偏航 (Yaw) | -500 ~ 500 | 对应 ±10° |

## 电机混控

```
左前 (LT) = 油门 - Pitch + Roll - Yaw
右前 (RT) = 油门 - Pitch - Roll + Yaw
左后 (LB) = 油门 + Pitch + Roll + Yaw
右后 (RB) = 油门 + Pitch - Roll - Yaw
```
