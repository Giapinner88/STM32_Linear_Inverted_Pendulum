#include "pendulum_app.h"

#include "main.h"
#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "comms.h"

#include <math.h>

#define CONTROL_PERIOD_MS 5U
#define OLED_PERIOD_MS 80U
#define COMMAND_TIMEOUT_MS 250U
#define UPRIGHT_ANGLE_RAD 3.1415926f

#define UPRIGHT_ENTER_TH_RAD 0.30f
#define UPRIGHT_EXIT_TH_RAD 0.45f
#define UPRIGHT_ENTER_DTH_RAD_S 4.5f
#define UPRIGHT_EXIT_DTH_RAD_S 7.0f
#define UPRIGHT_ENTER_TICKS 5U
#define UPRIGHT_EXIT_TICKS 1U

#define ANGLE_ZERO_DEFAULT_ADC 1024.0f
#define ADC_TO_RAD (2.0f * 3.1415926f / 4096.0f)

/* If enabled, MENU accepts zero calibration only when ADC is in [1010, 1030]. */
#define POT_ZERO_STRICT_WINDOW 1U
#define POT_ZERO_MIN_ADC 1010U
#define POT_ZERO_MAX_ADC 1030U

#define ENC_RAW_LEFT_WALL 4130
#define ENC_RAW_RIGHT_WALL 0
#define ENC_WALL_MARGIN_TICKS 120
#define JOG_U_CMD 0.18f

#define GRAVITY_MPS2 9.81f
#define PENDULUM_LENGTH_M 0.200f
#define CART_MASS_KG 0.100f

#define SWINGUP_U_SIGN 1.0f
#define SWINGUP_MIN_PUMP_U 0.16f
#define SWINGUP_ENERGY_TO_U 0.04f
#define SWINGUP_KICK_TH_RAD 0.35f
#define SWINGUP_KICK_DTH_RAD_S 1.0f
#define SWINGUP_KICK_U 0.26f
#define SWINGUP_MAX_U 0.72f
#define SWINGUP_WINDOW_HALF_TICKS 1800
#define SWINGUP_RETURN_U_MAX 0.62f
#define SWINGUP_RETURN_KX 0.00020f
#define SWINGUP_RETURN_KDX 0.00180f
#define SWINGUP_CTRL_PROFILE_SIM 1U
#define SWINGUP_CTRL_PROFILE_LEGACY 0U
#define SWINGUP_CTRL_PROFILE SWINGUP_CTRL_PROFILE_SIM

/* Simulation-derived swing-up gains from parameters.md for hardware A/B testing. */
#define SIM_SWINGUP_K_E 0.030f
#define SIM_SWINGUP_K_X 0.000012f
#define SIM_SWINGUP_K_V 0.00095f
#define SIM_SWINGUP_U_MAX 0.42f
#define HOMING_MAX_U 0.58f
#define HOMING_MIN_U 0.14f
#define HOMING_DECAY_TICKS 1500.0f

/* Update with measured Phase 1 dead-zone values (PWM units out of 7199). */
#define MOTOR_DEADZONE_FORWARD_PWM 80U
#define MOTOR_DEADZONE_REVERSE_PWM 80U

#define RESERVED_KEY_LONG_PRESS_TICKS 60U

static float g_angle_zero_adc = ANGLE_ZERO_DEFAULT_ADC;
static float g_prev_theta_rad = 0.0f;
static float g_curr_dtheta_rad_s = 0.0f;
static float g_filt_dtheta_rad_s = 0.0f;
static float g_curr_theta_rad = 0.0f;
static float g_prev_theta_wrapped_rad = 0.0f;
static float g_theta_unwrapped_rad = 0.0f;
static uint8_t g_theta_unwrap_init = 0U;
static float g_curr_u = 0.0f;
static float g_prev_u = 0.0f;
static uint16_t g_curr_adc = 0U;
static int32_t g_curr_enc = 0;
static uint16_t g_prev_enc_raw = 0U;
static float g_curr_x_vel = 0.0f;
static uint32_t g_frame_count = 0U;
static uint8_t g_run_enabled = 0U;
static uint8_t g_user_latch = 0U;
static uint8_t g_menu_latch = 0U;
static uint8_t g_capture_mode = 0U;
static uint8_t g_capture_hold_count = 0U;
static uint8_t g_capture_lost_count = 0U;
static uint8_t g_upright_locked = 0U;
static uint8_t g_zero_set_ok = 1U;
static uint8_t g_calibration_mode = 1U;
static int32_t g_right_home_enc = ENC_RAW_RIGHT_WALL;
static uint8_t g_home_return_active = 0U;
static ControlMode g_remote_mode = CONTROL_MODE_IDLE;
static float g_remote_u = 0.0f;
static uint8_t g_remote_active_dbg = 0U;
static uint8_t g_rail_blocked = 0U;
static uint8_t g_jog_active_dbg = 0U;

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

static void BootStageBlink(uint8_t times)
{
  uint8_t i;

  for (i = 0U; i < times; i++) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(180);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    delay_ms(180);
  }
}

static uint16_t ReadAngleAdc(void)
{
  uint32_t adc = 0U;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK) {
    adc = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  if (adc > 4095U) {
    adc = 4095U;
  }

  return (uint16_t)adc;
}

static uint8_t ButtonPressedLatched(GPIO_TypeDef *port, uint16_t pin, uint8_t *latch)
{
  GPIO_PinState key_state = HAL_GPIO_ReadPin(port, pin);

  if (key_state == GPIO_PIN_RESET) {
    if (*latch == 0U) {
      *latch = 1U;
      return 1U;
    }
  } else {
    *latch = 0U;
  }

  return 0U;
}

static float ClampFloat(float v, float vmin, float vmax)
{
  if (v < vmin) {
    return vmin;
  }
  if (v > vmax) {
    return vmax;
  }
  return v;
}

static float WrapPi(float x)
{
  while (x > 3.1415926f) {
    x -= 2.0f * 3.1415926f;
  }
  while (x < -3.1415926f) {
    x += 2.0f * 3.1415926f;
  }
  return x;
}

static float UnwrapThetaSample(float theta_wrapped)
{
  float delta;

  if (g_theta_unwrap_init == 0U) {
    g_prev_theta_wrapped_rad = theta_wrapped;
    g_theta_unwrapped_rad = theta_wrapped;
    g_theta_unwrap_init = 1U;
    return g_theta_unwrapped_rad;
  }

  delta = theta_wrapped - g_prev_theta_wrapped_rad;
  if (delta > 3.1415926f) {
    delta -= 2.0f * 3.1415926f;
  } else if (delta < -3.1415926f) {
    delta += 2.0f * 3.1415926f;
  }

  g_theta_unwrapped_rad += delta;
  g_prev_theta_wrapped_rad = theta_wrapped;

  return g_theta_unwrapped_rad;
}

static float ThetaErrFromUpright(float theta)
{
  return WrapPi(theta - UPRIGHT_ANGLE_RAD);
}

static float SignWithFallback(float v, float fallback)
{
  if (v > 0.0f) {
    return 1.0f;
  }
  if (v < 0.0f) {
    return -1.0f;
  }
  return (fallback >= 0.0f) ? 1.0f : -1.0f;
}

static float ApplySwingWindowClamp(float u_cmd, float x_err, float x_dot)
{
  float x_over;
  float u_return;

  if (x_err > (float)SWINGUP_WINDOW_HALF_TICKS) {
    x_over = x_err - (float)SWINGUP_WINDOW_HALF_TICKS;
    u_return = -(SWINGUP_RETURN_KX * x_over + SWINGUP_RETURN_KDX * x_dot);
    return ClampFloat(u_return, -SWINGUP_RETURN_U_MAX, 0.0f);
  }

  if (x_err < -(float)SWINGUP_WINDOW_HALF_TICKS) {
    x_over = x_err + (float)SWINGUP_WINDOW_HALF_TICKS;
    u_return = -(SWINGUP_RETURN_KX * x_over + SWINGUP_RETURN_KDX * x_dot);
    return ClampFloat(u_return, 0.0f, SWINGUP_RETURN_U_MAX);
  }

  return ClampFloat(u_cmd, -SWINGUP_MAX_U, SWINGUP_MAX_U);
}

static float ComputeSwingUpULegacy(float x_err, float x_dot, float theta, float dtheta)
{
  const float k_x = 0.000010f;
  const float k_dx = 0.00110f;
  const float energy_target = 2.0f * GRAVITY_MPS2 * PENDULUM_LENGTH_M;
  float energy;
  float energy_err;
  float phase;
  float phase_sign;
  float pump_u;
  float u_energy;
  float u_center;
  float u;

  energy = (0.5f * PENDULUM_LENGTH_M * PENDULUM_LENGTH_M * dtheta * dtheta)
         + (GRAVITY_MPS2 * PENDULUM_LENGTH_M * (1.0f - cosf(theta)));
  energy_err = energy_target - energy;

  phase = dtheta * cosf(theta);
  phase_sign = SignWithFallback(phase, theta);

  pump_u = (SWINGUP_ENERGY_TO_U * energy_err);
  if (energy_err > 0.0f) {
    pump_u += SWINGUP_MIN_PUMP_U;
  }
  pump_u = ClampFloat(pump_u, -SWINGUP_MAX_U, SWINGUP_MAX_U);
  u_energy = SWINGUP_U_SIGN * phase_sign * pump_u;

  if ((fabsf(theta) < SWINGUP_KICK_TH_RAD) && (fabsf(dtheta) < SWINGUP_KICK_DTH_RAD_S)) {
    u_energy += SWINGUP_U_SIGN * SWINGUP_KICK_U * SignWithFallback(theta, 1.0f);
  }

  (void)CART_MASS_KG;

  u_center = -(k_x * x_err + k_dx * x_dot);

  u = u_energy + u_center;
  return ApplySwingWindowClamp(u, x_err, x_dot);
}

static float ComputeSwingUpUSim(float x_err, float x_dot, float theta, float dtheta)
{
  const float l = PENDULUM_LENGTH_M;
  const float energy_target = 2.0f * GRAVITY_MPS2 * l;
  float energy;
  float energy_err;
  float u;

  energy = (0.5f * l * l * dtheta * dtheta)
         + (GRAVITY_MPS2 * l * (1.0f - cosf(theta)));
  energy_err = energy - energy_target;

  u = -(SIM_SWINGUP_K_E * energy_err * dtheta * cosf(theta))
    - (SIM_SWINGUP_K_X * x_err)
    - (SIM_SWINGUP_K_V * x_dot);

  u = ClampFloat(u, -SIM_SWINGUP_U_MAX, SIM_SWINGUP_U_MAX);
  return ApplySwingWindowClamp(u, x_err, x_dot);
}

static float ComputeSwingUpU(float x_err, float x_dot, float theta, float dtheta)
{
#if (SWINGUP_CTRL_PROFILE == SWINGUP_CTRL_PROFILE_SIM)
  return ComputeSwingUpUSim(x_err, x_dot, theta, dtheta);
#else
  return ComputeSwingUpULegacy(x_err, x_dot, theta, dtheta);
#endif
}

static float ApplySlewRate(float target_u)
{
  const float du_max = 0.35f;
  float du = target_u - g_prev_u;

  if (du > du_max) {
    du = du_max;
  } else if (du < -du_max) {
    du = -du_max;
  }

  g_prev_u += du;
  return g_prev_u;
}

static uint8_t IsSoftRailBlocked(int32_t enc_pos, float u_cmd)
{
  if ((enc_pos >= (ENC_RAW_LEFT_WALL - ENC_WALL_MARGIN_TICKS)) && (u_cmd > 0.0f)) {
    return 1U;
  }

  if ((enc_pos <= (ENC_RAW_RIGHT_WALL + ENC_WALL_MARGIN_TICKS)) && (u_cmd < 0.0f)) {
    return 1U;
  }

  return 0U;
}

static uint8_t TrySetZero(uint16_t adc_now)
{
#if POT_ZERO_STRICT_WINDOW
  if ((adc_now < POT_ZERO_MIN_ADC) || (adc_now > POT_ZERO_MAX_ADC)) {
    g_zero_set_ok = 0U;
    return 0U;
  }
#endif

  g_angle_zero_adc = (float)adc_now;
  g_zero_set_ok = 1U;
  return 1U;
}

static void BlinkSetZeroResult(uint8_t ok)
{
  if (ok != 0U) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(35);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
}

static void SetCalibrationMode(uint8_t enabled)
{
  g_calibration_mode = (enabled != 0U) ? 1U : 0U;
  g_run_enabled = 0U;
  g_curr_u = 0.0f;
  g_prev_u = 0.0f;
  g_upright_locked = 0U;
  g_capture_mode = 0U;
  g_capture_hold_count = 0U;
  g_capture_lost_count = 0U;
  g_jog_active_dbg = 0U;
  g_rail_blocked = 0U;
  Motor_Stop();
}

static void StartRightWallHoming(void)
{
  g_home_return_active = 1U;
  g_calibration_mode = 0U;
  g_run_enabled = 1U;
  g_curr_u = 0.0f;
  g_prev_u = 0.0f;
  g_upright_locked = 0U;
  g_capture_mode = 0U;
  g_capture_hold_count = 0U;
  g_capture_lost_count = 0U;
  g_jog_active_dbg = 0U;
  g_rail_blocked = 0U;
}

static float ComputeRightWallHomeU(int32_t enc_pos)
{
  float remaining_ticks;
  float norm;

  remaining_ticks = (float)(enc_pos - g_right_home_enc);
  if (remaining_ticks <= (float)ENC_WALL_MARGIN_TICKS) {
    return 0.0f;
  }

  norm = (remaining_ticks - (float)ENC_WALL_MARGIN_TICKS) / HOMING_DECAY_TICKS;
  norm = ClampFloat(norm, 0.0f, 1.0f);
  norm = norm * norm;

  return HOMING_MIN_U + ((HOMING_MAX_U - HOMING_MIN_U) * norm);
}

static int32_t GetCartPositionFromRightHome(void)
{
  return g_curr_enc - g_right_home_enc;
}

static void UpdateUprightLock(float theta_err_abs, float dtheta_abs)
{
  if (g_upright_locked == 0U) {
    if ((theta_err_abs < UPRIGHT_ENTER_TH_RAD) && (dtheta_abs < UPRIGHT_ENTER_DTH_RAD_S)) {
      if (g_capture_hold_count < 255U) {
        g_capture_hold_count++;
      }
    } else {
      g_capture_hold_count = 0U;
    }

    if (g_capture_hold_count >= UPRIGHT_ENTER_TICKS) {
      g_upright_locked = 1U;
      g_capture_lost_count = 0U;
    }
  } else {
    if ((theta_err_abs > UPRIGHT_EXIT_TH_RAD) || (dtheta_abs > UPRIGHT_EXIT_DTH_RAD_S)) {
      if (g_capture_lost_count < 255U) {
        g_capture_lost_count++;
      }
    } else {
      g_capture_lost_count = 0U;
    }

    if (g_capture_lost_count >= UPRIGHT_EXIT_TICKS) {
      g_upright_locked = 0U;
      g_capture_hold_count = 0U;
    }
  }

  g_capture_mode = g_upright_locked;
}

static void UpdateOledAngle(void)
{
  const uint8_t *run_label = (g_home_return_active != 0U) ? (uint8_t *)"HOM" : ((g_calibration_mode != 0U) ? (uint8_t *)"CAL" : (uint8_t *)"RUN");
  int32_t th_deg = (int32_t)(g_curr_theta_rad * (180.0f / 3.1415926f));
  int32_t dth_deg = (int32_t)(g_curr_dtheta_rad_s * (180.0f / 3.1415926f));
  int32_t u_pct = (int32_t)(g_curr_u * 100.0f);
  int32_t x_curr_abs = GetCartPositionFromRightHome();
  int32_t x_home_abs = g_right_home_enc;
  int32_t x_dot = (int32_t)g_curr_x_vel;
  uint16_t th_abs = (uint16_t)((th_deg >= 0) ? th_deg : -th_deg);
  uint16_t dth_abs = (uint16_t)((dth_deg >= 0) ? dth_deg : -dth_deg);
  uint16_t u_abs = (uint16_t)((u_pct >= 0) ? u_pct : -u_pct);
  uint16_t x_dot_abs = (uint16_t)((x_dot >= 0) ? x_dot : -x_dot);

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)run_label);
  OLED_ShowString(28, 0, (uint8_t *)(g_remote_active_dbg ? "R" : "L"));
  OLED_ShowString(40, 0, (uint8_t *)"C:");
  OLED_ShowNumber(56, 0, g_capture_mode, 1, 12);
  OLED_ShowString(72, 0, (uint8_t *)"M:");
  OLED_ShowNumber(88, 0, (uint16_t)g_remote_mode, 1, 12);

  OLED_ShowString(0, 12, (uint8_t *)"ADC:");
  OLED_ShowNumber(28, 12, g_curr_adc, 4, 12);
  OLED_ShowString(70, 12, (uint8_t *)"Z:");
  OLED_ShowNumber(84, 12, (uint16_t)g_angle_zero_adc, 4, 12);

  OLED_ShowString(0, 24, (uint8_t *)"TH:");
  OLED_ShowChar(20, 24, (th_deg >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 24, th_abs, 3, 12);
  OLED_ShowString(56, 24, (uint8_t *)"d:");
  OLED_ShowChar(72, 24, (dth_deg >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 24, dth_abs, 3, 12);

  OLED_ShowString(0, 36, (uint8_t *)"U%:");
  OLED_ShowChar(20, 36, (u_pct >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 36, u_abs, 3, 12);
  OLED_ShowString(56, 36, (uint8_t *)"X:");
  OLED_ShowChar(72, 36, (x_curr_abs >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 36, (uint16_t)((x_curr_abs >= 0) ? x_curr_abs : -x_curr_abs), 5, 12);

  OLED_ShowString(0, 48, (uint8_t *)"HOME:");
  OLED_ShowChar(20, 48, (x_home_abs >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 48, (uint16_t)((x_home_abs >= 0) ? x_home_abs : -x_home_abs), 5, 12);
  OLED_ShowString(72, 48, (uint8_t *)"dX:");
  OLED_ShowChar(96, 48, (x_dot >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(104, 48, x_dot_abs, 4, 12);

  OLED_Refresh_Gram();
}

void PendulumApp_Init(void)
{
  delay_ms(200);

  BootStageBlink(4U);

  OLED_Init();
  delay_ms(1000);
  OLED_Clear();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_ADCEx_Calibration_Start(&hadc1);

  Motor_Init(&htim3, TIM_CHANNEL_4, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
  Motor_SetDeadzonePwm(MOTOR_DEADZONE_FORWARD_PWM, MOTOR_DEADZONE_REVERSE_PWM);
  Motor_Stop();

  g_curr_enc = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
  g_prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

  Comms_Init(&huart1);

  g_angle_zero_adc = ANGLE_ZERO_DEFAULT_ADC;

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"BOOT OK");
  OLED_ShowString(0, 12, (uint8_t *)"SWUP READY");
  OLED_ShowString(0, 24, (uint8_t *)"USER:RUN/STOP");
  OLED_ShowString(0, 36, (uint8_t *)"MENU:SET ZERO");
  OLED_Refresh_Gram();
  delay_ms(1200);

  OLED_Clear();
  OLED_ShowString(0, 12, (uint8_t *)"USER RUN/STOP");
  OLED_ShowString(0, 24, (uint8_t *)"MENU SET ZERO");
  OLED_ShowString(0, 36, (uint8_t *)"RES:CAL  HOLD:HOME");
  OLED_Refresh_Gram();
}

void PendulumApp_RunFrame(void)
{
  static uint8_t oled_div = 0U;
  static uint8_t local_run_latched = 0U;
  static uint8_t user_force_stop = 0U;
  static uint16_t remote_miss_ticks = 0xFFFFU;
  static uint16_t reserved_key_hold_ticks = 0U;
  float theta_wrapped_rad;
  float u_cmd = 0.0f;
  float theta_err_abs;
  uint16_t enc_raw;
  int16_t enc_delta;
  ControlCommandData cmd;
  RobotStateData telem;
  uint8_t remote_active = 0U;
  uint8_t jog_plus_held;
  uint8_t jog_minus_held;
  uint8_t jog_active = 0U;

  Comms_Process();

  g_curr_adc = ReadAngleAdc();
  theta_wrapped_rad = (((float)g_curr_adc) - g_angle_zero_adc) * ADC_TO_RAD;
  g_curr_theta_rad = UnwrapThetaSample(theta_wrapped_rad);
  g_curr_dtheta_rad_s = (g_curr_theta_rad - g_prev_theta_rad) / ((float)CONTROL_PERIOD_MS / 1000.0f);
  g_prev_theta_rad = g_curr_theta_rad;
  g_filt_dtheta_rad_s = 0.55f * g_filt_dtheta_rad_s + 0.45f * g_curr_dtheta_rad_s;
  theta_err_abs = fabsf(ThetaErrFromUpright(g_curr_theta_rad));

  UpdateUprightLock(theta_err_abs, fabsf(g_filt_dtheta_rad_s));

  enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  enc_delta = (int16_t)(enc_raw - g_prev_enc_raw);
  g_prev_enc_raw = enc_raw;
  g_curr_enc = (int32_t)enc_raw;
  g_curr_x_vel = ((float)enc_delta) / ((float)CONTROL_PERIOD_MS / 1000.0f);

  if (Comms_GetLatestCommand(&cmd) != 0U) {
    g_remote_mode = cmd.mode;
    g_remote_u = ClampFloat(cmd.control_effort, -1.0f, 1.0f);
    remote_miss_ticks = 0U;
  } else if (remote_miss_ticks < 0xFFFFU) {
    remote_miss_ticks++;
  }

  remote_active = (remote_miss_ticks <= (COMMAND_TIMEOUT_MS / CONTROL_PERIOD_MS)) ? 1U : 0U;
  g_remote_active_dbg = remote_active;

  jog_plus_held = (HAL_GPIO_ReadPin(pid_plus_GPIO_Port, pid_plus_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  jog_minus_held = (HAL_GPIO_ReadPin(pid_reduce_GPIO_Port, pid_reduce_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (ButtonPressedLatched(User_key_GPIO_Port, User_key_Pin, &g_user_latch) != 0U) {
    user_force_stop = (user_force_stop == 0U) ? 1U : 0U;

    if (user_force_stop != 0U) {
      local_run_latched = 0U;
      g_remote_mode = CONTROL_MODE_IDLE;
      g_remote_u = 0.0f;
      remote_miss_ticks = 0xFFFFU;
      g_remote_active_dbg = 0U;
      g_run_enabled = 0U;
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      Motor_Stop();
    } else {
      local_run_latched = 1U;
      g_remote_mode = CONTROL_MODE_IDLE;
      g_remote_u = 0.0f;
      remote_miss_ticks = 0xFFFFU;
      g_remote_active_dbg = 0U;
    }
  }

  if (ButtonPressedLatched(menu_key_GPIO_Port, menu_key_Pin, &g_menu_latch) != 0U) {
    BlinkSetZeroResult(TrySetZero(g_curr_adc));
  }

  if (HAL_GPIO_ReadPin(reserved_key_GPIO_Port, reserved_key_Pin) == GPIO_PIN_RESET) {
    if (reserved_key_hold_ticks < 0xFFFFU) {
      reserved_key_hold_ticks++;
    }
  } else if (reserved_key_hold_ticks > 0U) {
    if (reserved_key_hold_ticks >= RESERVED_KEY_LONG_PRESS_TICKS) {
      StartRightWallHoming();
    } else {
      SetCalibrationMode((g_calibration_mode == 0U) ? 1U : 0U);
    }

    reserved_key_hold_ticks = 0U;
  }

  if (g_home_return_active != 0U) {
    if (g_curr_enc <= (g_right_home_enc + ENC_WALL_MARGIN_TICKS)) {
      g_home_return_active = 0U;
      g_run_enabled = 0U;
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      Motor_Stop();
    } else {
      u_cmd = ComputeRightWallHomeU(g_curr_enc);
      g_curr_u = ApplySlewRate(u_cmd);
    }
  } else if (g_calibration_mode != 0U) {
    g_run_enabled = 0U;
    g_curr_u = 0.0f;
    g_prev_u = 0.0f;
    g_upright_locked = 0U;
    g_capture_mode = 0U;
    g_capture_hold_count = 0U;
    g_capture_lost_count = 0U;
    jog_active = 0U;
    g_rail_blocked = 0U;
    Motor_Stop();
  } else if (user_force_stop != 0U) {
    g_run_enabled = 0U;
    g_curr_u = 0.0f;
    g_prev_u = 0.0f;
    g_upright_locked = 0U;
    g_capture_mode = 0U;
    g_capture_hold_count = 0U;
    g_capture_lost_count = 0U;
    Motor_Stop();
  } else if (remote_active != 0U) {
    g_run_enabled = (g_remote_mode != CONTROL_MODE_IDLE) ? 1U : 0U;

    if (g_run_enabled == 0U) {
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      g_upright_locked = 0U;
      g_capture_mode = 0U;
      g_capture_hold_count = 0U;
      g_capture_lost_count = 0U;
      Motor_Stop();
    } else {
      if ((g_remote_mode == CONTROL_MODE_SWINGUP) || (g_remote_mode == CONTROL_MODE_LQR)) {
        u_cmd = ComputeSwingUpU((float)GetCartPositionFromRightHome(), g_curr_x_vel, g_curr_theta_rad, g_filt_dtheta_rad_s);
      } else {
        u_cmd = g_remote_u;
      }

      g_curr_u = ApplySlewRate(u_cmd);
    }
  } else {
    g_run_enabled = local_run_latched;

    if (g_run_enabled == 0U) {
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      g_upright_locked = 0U;
      g_capture_mode = 0U;
      g_capture_hold_count = 0U;
      g_capture_lost_count = 0U;
      Motor_Stop();
    } else {
      u_cmd = ComputeSwingUpU((float)GetCartPositionFromRightHome(), g_curr_x_vel, g_curr_theta_rad, g_filt_dtheta_rad_s);

      g_curr_u = ApplySlewRate(u_cmd);
    }
  }

  if ((g_home_return_active == 0U) && (remote_active == 0U) && (g_run_enabled == 0U) && (user_force_stop == 0U)) {
    if (jog_plus_held != jog_minus_held) {
      jog_active = 1U;
      u_cmd = (jog_plus_held != 0U) ? JOG_U_CMD : -JOG_U_CMD;
      g_curr_u = ApplySlewRate(u_cmd);
    } else {
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
    }
  }

  g_jog_active_dbg = jog_active;

  if ((g_home_return_active != 0U) || (g_run_enabled != 0U) || (jog_active != 0U)) {
    g_rail_blocked = IsSoftRailBlocked(g_curr_enc, g_curr_u);
    if (g_rail_blocked != 0U) {
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      Motor_Stop();
    } else {
      Motor_SetTorque(g_curr_u);
    }
  } else {
    g_rail_blocked = 0U;
  }

  g_jog_active_dbg = jog_active;

  telem.x_pos = (float)g_curr_enc;
  telem.x_vel = g_curr_x_vel;
  telem.theta = g_curr_theta_rad;
  telem.theta_vel = g_filt_dtheta_rad_s;
  Comms_SendTelemetry(&telem);

  g_frame_count++;
  if ((g_frame_count % 10U) == 0U) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  oled_div++;
  if (oled_div >= (OLED_PERIOD_MS / CONTROL_PERIOD_MS)) {
    oled_div = 0U;
    UpdateOledAngle();
  }

  delay_ms(CONTROL_PERIOD_MS);
}
