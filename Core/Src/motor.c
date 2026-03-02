/**
 * @file    motor.c
 * @brief   TB6612FNG motor driver — TIM1 PWM (20 kHz) + GPIO direction
 */
#include "motor.h"
#include "main.h"

static TIM_HandleTypeDef htim1;

/* ========================== Init ======================================== */

void Motor_Init(void) {
  /* ----- TIM1 PWM Init (PA8 = CH1, PA9 = CH2) ----- */
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0; /* 72 MHz / 1 = 72 MHz */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_ARR; /* 72MHz / 3600 = 20 kHz */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim1);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; /* Start with 0% duty */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  /* TIM1 is Advanced timer — need break/deadtime config */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  /* Start PWM on both channels */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* ----- Direction GPIO Init ----- */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB10, PB11 = Motor A direction; PB12, PB13 = Motor B direction */
  GPIO_InitStruct.Pin = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN |
                        MOTOR_B_IN2_PIN | MOTOR_STBY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable TB6612 by default */
  Motor_Enable();

  /* Start with motors stopped */
  Motor_Coast();
}

/* ========================== Set speed =================================== */

void Motor_SetSpeed(Motor_ID motor, int16_t speed) {
  /* Clamp to [-1000, +1000] */
  if (speed > 1000)
    speed = 1000;
  if (speed < -1000)
    speed = -1000;

  /* Map |speed| 0..1000 → PWM 0..MOTOR_PWM_ARR */
  uint16_t pwm;
  if (speed >= 0)
    pwm = (uint16_t)((uint32_t)speed * MOTOR_PWM_ARR / 1000);
  else
    pwm = (uint16_t)((uint32_t)(-speed) * MOTOR_PWM_ARR / 1000);

  if (motor == MOTOR_LEFT) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    if (speed > 0) {
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    }
  } else /* MOTOR_RIGHT — direction inverted (physically mirrored motor) */
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    if (speed > 0) {
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
    } else if (speed < 0) {
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
    }
  }
}

void Motor_Brake(void) {
  /* Short brake: IN1=HIGH, IN2=HIGH, PWM=whatever */
  HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
}

void Motor_Coast(void) {
  /* Coast: IN1=LOW, IN2=LOW */
  HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Motor_Enable(void) {
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

void Motor_Disable(void) {
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}
