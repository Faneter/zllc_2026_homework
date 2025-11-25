#include "controller.h"
#include "c620.h"
#include "pid.h"
#include "can.h"

C620_Motor_Status_TypeDef motor_status[8];
C620_Motor_Control_Typedef motor_controll[8];
PID_TypeDef motor_pids[8];

void C620_Motor_Status_Handler(C620_Motor_Status_TypeDef *status);
void C620_Single_Motor_PID_Update(C620_Motor_Status_TypeDef *status);

void Controller_Init()
{
    PID_Init(&motor_pids[0], 0.02, 0.001, 0.001, 20, -20);
    motor_status[0].id   = 1;
    motor_controll[0].id = 1;
    // motor_pids[0].target = 200;
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
            // int16_t raw              = (rx_buff[0] << 8) | rx_buff[1];
            // const float SCALE_FACTOR = 20.0f / 16384.0f;
            // motor_status[0].speed    = (int16_t)(raw * SCALE_FACTOR * 10);
        }
    }
}

void C620_Motor_Status_Handler(C620_Motor_Status_TypeDef *status)
{
}

void C620_Motor_PID_Update()
{
    C620_Single_Motor_PID_Update(&motor_status[0]); // 更新ID为1的电机PID
    uint8_t message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    C620_Motor_Control_Init(&motor_controll[0], message);
    CAN_Send(0x200, message, 8);
}

void C620_Single_Motor_PID_Update(C620_Motor_Status_TypeDef *status)
{
    motor_controll[status->id - 1].current = PID_Calculate(&motor_pids[status->id - 1], status->speed);
}