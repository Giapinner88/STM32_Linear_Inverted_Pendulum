#include "motor.h"

#include <math.h>

#define MOTOR_PWM_MAX 7199U

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *dir1_port;
  uint16_t dir1_pin;
  GPIO_TypeDef *dir2_port;
  uint16_t dir2_pin;
  uint16_t deadzone_forward;
  uint16_t deadzone_reverse;
} MotorContext;

static MotorContext g_motor = {0};

static uint16_t Motor_MapMagnitudeToPwm(float mag, uint16_t deadzone)
{
  float clamped = mag;
  float pwm;

  if (clamped > 1.0f) {
    clamped = 1.0f;
  }

  if (deadzone >= MOTOR_PWM_MAX) {
    return MOTOR_PWM_MAX;
  }

  pwm = (float)deadzone + ((float)(MOTOR_PWM_MAX - deadzone) * clamped);

  if (pwm < 0.0f) {
    pwm = 0.0f;
  }

  if (pwm > (float)MOTOR_PWM_MAX) {
    pwm = (float)MOTOR_PWM_MAX;
  }

  return (uint16_t)pwm;
}

void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef *dir2_port, uint16_t dir2_pin)
{
  g_motor.htim = htim;
  g_motor.channel = channel;
  g_motor.dir1_port = dir1_port;
  g_motor.dir1_pin = dir1_pin;
  g_motor.dir2_port = dir2_port;
  g_motor.dir2_pin = dir2_pin;
  g_motor.deadzone_forward = 0U;
  g_motor.deadzone_reverse = 0U;

  Motor_Stop();
}

void Motor_SetDeadzonePwm(uint16_t forward_pwm, uint16_t reverse_pwm)
{
  g_motor.deadzone_forward = (forward_pwm > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : forward_pwm;
  g_motor.deadzone_reverse = (reverse_pwm > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : reverse_pwm;
}

void Motor_SetTorque(float u)
{
  float magnitude;
  uint16_t pwm;

  if (g_motor.htim == NULL) {
    return;
  }

  if (u > 1.0f) {
    u = 1.0f;
  } else if (u < -1.0f) {
    u = -1.0f;
  }

  if (fabsf(u) < 0.001f) {
    Motor_Stop();
    return;
  }

  if (u > 0.0f) {
    HAL_GPIO_WritePin(g_motor.dir1_port, g_motor.dir1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_motor.dir2_port, g_motor.dir2_pin, GPIO_PIN_RESET);
    magnitude = u;
    pwm = Motor_MapMagnitudeToPwm(magnitude, g_motor.deadzone_forward);
  } else {
    HAL_GPIO_WritePin(g_motor.dir1_port, g_motor.dir1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_motor.dir2_port, g_motor.dir2_pin, GPIO_PIN_SET);
    magnitude = -u;
    pwm = Motor_MapMagnitudeToPwm(magnitude, g_motor.deadzone_reverse);
  }

  __HAL_TIM_SET_COMPARE(g_motor.htim, g_motor.channel, pwm);
}

void Motor_Stop(void)
{
  if (g_motor.htim == NULL) {
    return;
  }

  __HAL_TIM_SET_COMPARE(g_motor.htim, g_motor.channel, 0U);
  HAL_GPIO_WritePin(g_motor.dir1_port, g_motor.dir1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(g_motor.dir2_port, g_motor.dir2_pin, GPIO_PIN_RESET);
}
