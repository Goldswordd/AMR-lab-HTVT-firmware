/**
 * @file    stm32f1xx_hal_msp.c
 * @brief   MSP Init — clock enable and GPIO config for peripherals
 */
#include "main.h"
#include "stm32f1xx_hal.h"

/**
 * @brief  HAL MSP Init — enable clocks
 */
void HAL_MspInit(void) {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Disable JTAG, keep SWD — frees PB3/PB4/PA15 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/**
 * @brief  I2C MSP Init — PB6 (SCL), PB7 (SDA)
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6 = SCL, PB7 = SDA — Open Drain AF */
    GPIO_InitStruct.Pin = IMU_SCL_PIN | IMU_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_I2C_PORT, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(IMU_I2C_PORT, IMU_SCL_PIN | IMU_SDA_PIN);
  }
}

/**
 * @brief  TIM MSP Init — encoder and PWM timers
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM3) /* Left encoder: PA6, PA7 */
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = ENC_LEFT_A_PIN | ENC_LEFT_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_LEFT_PORT, &GPIO_InitStruct);
  } else if (htim->Instance == TIM2) /* Right encoder: PA0, PA1 */
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = ENC_RIGHT_A_PIN | ENC_RIGHT_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_RIGHT_PORT, &GPIO_InitStruct);
  }
}

/**
 * @brief  TIM PWM MSP Init — TIM1 CH1/CH2 for motor PWM
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA8 = TIM1_CH1, PA9 = TIM1_CH2 — AF Push-Pull */
    GPIO_InitStruct.Pin = MOTOR_PWMA_PIN | MOTOR_PWMB_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStruct);
  }
}

/**
 * @brief  UART MSP Init — PA2 (TX), PA3 (RX)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 = USART2_TX — AF Push-Pull */
    GPIO_InitStruct.Pin = UART_PI_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_PI_PORT, &GPIO_InitStruct);

    /* PA3 = USART2_RX — Input floating */
    GPIO_InitStruct.Pin = UART_PI_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART_PI_PORT, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(UART_PI_PORT, UART_PI_TX_PIN | UART_PI_RX_PIN);
  }
}
