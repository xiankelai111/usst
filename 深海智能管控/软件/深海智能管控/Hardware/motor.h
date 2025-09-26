#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx.h"                  // Device header

void Motor_Init(void);
void Motor_Run(uint32_t dir, uint32_t num, uint32_t speed_us, uint32_t motor);
void AdjustPlane(float pitch, float roll);
int Motor_AdjustComplete(void);
#endif
