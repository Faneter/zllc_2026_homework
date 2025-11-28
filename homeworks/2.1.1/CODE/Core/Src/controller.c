#include "controller.h"
#include "c620.h"
#include "pid.h"
#include "can.h"
#include "debug.h"
#include <math.h>

uint8_t motor_enable = 0x01;

C620_Motor_Status_TypeDef motor_status[8];
C620_Motor_Control_Typedef motor_controll[8];
PID_TypeDef motor_speed_pids[8];
PID_TypeDef motor_angle_pids[8];

void C620_Motor_Status_Handler(C620_Motor_Status_TypeDef *status);
void Single_C620_Motor_Speed_PID_Update(C620_Motor_Status_TypeDef *status, int16_t speed);
float Single_C620_Motor_Angle_PID_Update(C620_Motor_Status_TypeDef *status, int16_t target_angle);

void Controller_Init()
{
    PID_Init(&motor_speed_pids[0], 0.025, 0.002, 0.0015, 20, -20);
    PID_Init(&motor_angle_pids[0], 0.1, 0.00015, 0.00002, 20, -20);
    for (size_t i = 0; i < 8; i++) {
        motor_status[i].id   = i + 1;
        motor_controll[i].id = i + 1;
    }
    motor_status[0].angle = 80;
}

void CAN_RxHandler(uint32_t stdId, const uint8_t *rx_buff)
{
    if (stdId & 0x200) {
        uint8_t id = stdId & 0x00F;
        if (id <= 8 && id > 0) {
            C620_Motor_Status_Init(&motor_status[id - 1], stdId, rx_buff);
            C620_Motor_Status_Handler(&motor_status[id - 1]);
        } else if (id == 0) {
            // 模拟电机转动速度反馈
            static uint32_t time     = 0;
            uint32_t delta_time      = HAL_GetTick() - time;
            time                     = HAL_GetTick();
            int16_t raw              = (rx_buff[0] << 8) | rx_buff[1];
            const float SCALE_FACTOR = 20.0f / 16384.0f;
            float last_speed         = motor_status[0].speed;
            motor_status[0].speed    = (int16_t)(raw * SCALE_FACTOR * 10);
            motor_status[0].angle += (last_speed + motor_status[0].speed) * delta_time * 3 / 1000;
            motor_status[0].angle = fmodf(motor_status[0].angle, 360.0f);
            motor_status[0].angle = (motor_status[0].angle < 0) ? (motor_status[0].angle + 360.0f) : motor_status[0].angle;
        }
    }
}

void EXTI_Handler(int16_t GPIO_Pin) {
    switch (GPIO_Pin)
    {
    case DEBUG_CHANGE_Pin:
        
        break;
    default:
        break;
    }
}

void C620_Motor_Status_Handler(C620_Motor_Status_TypeDef *status)
{
}

void C620_Motor_Speed_PID_Update(int16_t *speeds)
{
    uint8_t message1[8]     = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t message1_enable = 0;
    for (size_t i = 0; i < 4; i++) {
        if (motor_enable & (1 << i)) {
            message1_enable = 1;
            Single_C620_Motor_Speed_PID_Update(motor_status + i, speeds[i]);
            C620_Motor_Control_Init(motor_controll + i, message1);
        }
    }
    if (message1_enable) {
        CAN_Send(0x200, message1, 8);
    }

    uint8_t message2[8]     = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t message2_enable = 0;
    for (size_t i = 4; i < 8; i++) {
        if (motor_enable & (1 << i)) {
            message2_enable = 1;
            Single_C620_Motor_Speed_PID_Update(motor_status + i, speeds[i]);
            C620_Motor_Control_Init(motor_controll + i, message2);
        }
    }
    if (message2_enable) {
        CAN_Send(0x1FF, message2, 8);
    }

    static float debug[2] = {};
    debug[0]              = motor_status[0].speed;
    debug[1]              = motor_status[0].angle;
    print_debug(debug, 2);
}

void Single_C620_Motor_Speed_PID_Update(C620_Motor_Status_TypeDef *status, int16_t speed)
{
    motor_speed_pids[status->id - 1].target = speed;
    motor_controll[status->id - 1].current  = PID_Calculate(&motor_speed_pids[status->id - 1], status->speed);
}

void C620_Motor_Angle_PID_Update(int16_t *angles)
{
    static int16_t speeds[8] = {};
    for (size_t i = 0; i < 8; i++) {
        if (motor_enable & (1 << i)) {
            speeds[i] = Single_C620_Motor_Angle_PID_Update(motor_status + i, angles[i]);
        }
    }
    C620_Motor_Speed_PID_Update(speeds);
}

float Single_C620_Motor_Angle_PID_Update(C620_Motor_Status_TypeDef *status, int16_t target_angle)
{
    static float start_angle[8]   = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
    static float last_angle[8] = {};
    static int16_t spin_n[8]   = {};

    uint8_t id  = status->id - 1;
    float angle = status->angle;

    if (start_angle[id] - (-1.0f) <= 1e-6) {
        start_angle[id] = angle;
    }
    motor_angle_pids[id].target = target_angle + start_angle[id];
    if (angle - last_angle[id] > 180.0f) {
        spin_n[id]--;
    }
    if (angle - last_angle[id] < -180.0f) {
        spin_n[id]++;
    }
    last_angle[id] = angle;
    return PID_Calculate(&motor_angle_pids[id], spin_n[id] * 360.0f + angle);
}

void Debug() {

}
