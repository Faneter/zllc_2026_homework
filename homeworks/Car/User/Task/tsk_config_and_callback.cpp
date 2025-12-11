/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "ita_chariot.h"
#include "config.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_DJI_Motor_C620 Motor_Wheel[4];
Class_DR16 DR16;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

#ifdef LEARNING
/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif

#ifdef LEARNING
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    const float Dead_Zone = 0.05;
    const float Velocity_Max = 2.0f;

    DR16.DR16_UART_RxCpltCallback(Buffer);

    // TODO 底盘控制策略
    float dr16_l_x, dr16_l_y, dr16_r_x;
    // 排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > Dead_Zone) ? DR16.Get_Left_Y() : 0;
    dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > Dead_Zone) ? DR16.Get_Right_X() : 0;

    // 设定矩形到圆形映射进行控制
    float chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Velocity_Max;
    float chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Velocity_Max;

    // 设定角速度
    float chassis_omega = -dr16_r_x * Velocity_Max;

    // 底盘四电机模式配置
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    }

    // 速度换算，正运动学分解
    float motor1_temp_linear_vel = chassis_velocity_y - chassis_velocity_x + chassis_omega * (HALF_WIDTH + HALF_LENGTH);
    float motor2_temp_linear_vel = chassis_velocity_y + chassis_velocity_x - chassis_omega * (HALF_WIDTH + HALF_LENGTH);
    float motor3_temp_linear_vel = chassis_velocity_y + chassis_velocity_x + chassis_omega * (HALF_WIDTH + HALF_LENGTH);
    float motor4_temp_linear_vel = chassis_velocity_y - chassis_velocity_x - chassis_omega * (HALF_WIDTH + HALF_LENGTH);

    // 线速度 cm/s  转角速度  RAD
    float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
    float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
    float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
    float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
    // 角速度*减速比  设定目标 直接给到电机输出轴
    Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
    Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
    Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
    Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);
    // 各个电机具体PID
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }
}
#endif

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
}

/**
 * @brief TIM5任务回调函数
 *
 */

void Task1ms_TIM5_Callback()
{
    // 统一打包发送
    if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)
    {
        TIM_CAN_PeriodElapsedCallback();
    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{

    DWT_Init(168);
    // TODO 初始化
    // 手柄初始化
    DR16.Init(&huart3, nullptr);

    // 电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);

#ifdef LEARNING
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    UART_Init(&huart3, DR16_UART3_Callback, 18);
#endif // LEARNING

    // 定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
