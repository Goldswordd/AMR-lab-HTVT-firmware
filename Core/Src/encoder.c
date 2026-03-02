/**
 * @file    encoder.c
 * @brief   Quadrature encoder reading via TIM2 (right) and TIM3 (left)
 */
#include "encoder.h"
#include "main.h"

static TIM_HandleTypeDef htim2; /* Right encoder */
static TIM_HandleTypeDef htim3; /* Left encoder  */

static int16_t last_count_left = 0;
static int16_t last_count_right = 0;

void Encoder_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* ----- TIM3: Left encoder (PA6=CH1, PA7=CH2) ----- */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF; /* Full 16-bit range */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; /* Count on both edges */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0F; /* Max digital filter — good for noisy signals */
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0F;

  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* ----- TIM2: Right encoder (PA0=CH1, PA1=CH2) ----- */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /* Reset counters */
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  last_count_left = 0;
  last_count_right = 0;
}

int16_t Encoder_GetDelta(Encoder_ID enc) {
  int16_t current, delta;

  if (enc == ENCODER_LEFT) {
    current = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    delta = current - last_count_left;
    last_count_left = current;
  } else {
    current = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    delta = current - last_count_right;
    last_count_right = current;
    delta = -delta; /* Right motor is mirrored — invert encoder direction */
  }

  return delta;
}

uint16_t Encoder_GetCount(Encoder_ID enc) {
  if (enc == ENCODER_LEFT)
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  else
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

float Encoder_TicksToMM(int16_t delta_ticks) {
  return (float)delta_ticks * MM_PER_TICK;
}

float Encoder_GetSpeed(int16_t delta_ticks, float dt) {
  return (float)delta_ticks / dt; /* ticks/second */
}
