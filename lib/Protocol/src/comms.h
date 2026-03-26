#ifndef COMMS_H
#define COMMS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CONTROL_MODE_IDLE = 0,
  CONTROL_MODE_SWINGUP = 1,
  CONTROL_MODE_LQR = 2
} ControlMode;

typedef struct {
  float x_pos;
  float x_vel;
  float theta;
  float theta_vel;
} RobotStateData;

typedef struct {
  float control_effort;
  ControlMode mode;
  uint32_t timestamp_ms;
  uint8_t is_fresh;
} ControlCommandData;

void Comms_Init(UART_HandleTypeDef *huart);
void Comms_Process(void);
void Comms_SendTelemetry(const RobotStateData *state);
uint8_t Comms_GetLatestCommand(ControlCommandData *cmd);

#ifdef __cplusplus
}
#endif

#endif
