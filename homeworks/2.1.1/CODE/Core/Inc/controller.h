#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "stm32f1xx.h"

void Controller_Init();

void CAN_RxHandler(uint32_t stdId, const uint8_t *rx_buff);

void C620_Motor_Speed_PID_Update(int16_t *speed);

#endif // !__CONTROLLER_H