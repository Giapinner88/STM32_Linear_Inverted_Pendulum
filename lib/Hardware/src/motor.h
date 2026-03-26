#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef *dir2_port, uint16_t dir2_pin);
void Motor_SetDeadzonePwm(uint16_t forward_pwm, uint16_t reverse_pwm);
void Motor_SetTorque(float u);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
