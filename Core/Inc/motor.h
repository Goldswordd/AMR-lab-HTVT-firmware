/**
 * @file    motor.h
 * @brief   TB6612FNG dual motor driver — PWM + direction control
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } Motor_ID;

/**
 * @brief  Initialize TIM1 PWM and direction GPIO for both motors
 */
void Motor_Init(void);

/**
 * @brief  Set motor speed and direction
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @param  speed  Range [-1000, +1000]; negative = reverse
 */
void Motor_SetSpeed(Motor_ID motor, int16_t speed);

/**
 * @brief  Short brake both motors (IN1=IN2=HIGH)
 */
void Motor_Brake(void);

/**
 * @brief  Coast both motors (IN1=IN2=LOW)
 */
void Motor_Coast(void);

/**
 * @brief  Enable TB6612 (STBY = HIGH)
 */
void Motor_Enable(void);

/**
 * @brief  Disable TB6612 (STBY = LOW)
 */
void Motor_Disable(void);

#endif /* __MOTOR_H */
