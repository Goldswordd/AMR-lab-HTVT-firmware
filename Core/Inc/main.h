/**
 * @file    main.h
 * @brief   Main header — pin definitions for AMR robot (Blue Pill)
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

/* ========================== UART (Pi serial) ========================== */
#define UART_PI USART2
#define UART_PI_TX_PIN GPIO_PIN_2 /* PA2 = USART2_TX */
#define UART_PI_RX_PIN GPIO_PIN_3 /* PA3 = USART2_RX */
#define UART_PI_PORT GPIOA
#define UART_PI_BAUD 115200

/* ========================== I2C (MPU6050) ============================== */
#define IMU_I2C I2C1
#define IMU_SCL_PIN GPIO_PIN_6 /* PB6 = I2C1_SCL */
#define IMU_SDA_PIN GPIO_PIN_7 /* PB7 = I2C1_SDA */
#define IMU_I2C_PORT GPIOB

/* ========================== Encoder Left (TIM3) ======================== */
#define ENC_LEFT_TIM TIM3
#define ENC_LEFT_A_PIN GPIO_PIN_6 /* PA6 = TIM3_CH1 */
#define ENC_LEFT_B_PIN GPIO_PIN_7 /* PA7 = TIM3_CH2 */
#define ENC_LEFT_PORT GPIOA

/* ========================== Encoder Right (TIM2) ======================= */
#define ENC_RIGHT_TIM TIM2
#define ENC_RIGHT_A_PIN GPIO_PIN_0 /* PA0 = TIM2_CH1 */
#define ENC_RIGHT_B_PIN GPIO_PIN_1 /* PA1 = TIM2_CH2 */
#define ENC_RIGHT_PORT GPIOA

/* ========================== Motor PWM (TIM1) =========================== */
#define MOTOR_PWM_TIM TIM1
#define MOTOR_PWMA_PIN GPIO_PIN_8 /* PA8 = TIM1_CH1 → PWMA */
#define MOTOR_PWMB_PIN GPIO_PIN_9 /* PA9 = TIM1_CH2 → PWMB */
#define MOTOR_PWM_PORT GPIOA
#define MOTOR_PWM_FREQ 20000 /* 20 kHz — inaudible */
#define MOTOR_PWM_ARR 3599   /* 72MHz / 20kHz - 1 */

/* ========================== Motor Direction (GPIO) ===================== */
/* Motor A (Left) */
#define MOTOR_A_IN1_PIN GPIO_PIN_10 /* PB10 → AIN1 */
#define MOTOR_A_IN2_PIN GPIO_PIN_11 /* PB11 → AIN2 */
#define MOTOR_A_DIR_PORT GPIOB

/* Motor B (Right) */
#define MOTOR_B_IN1_PIN GPIO_PIN_12 /* PB12 → BIN1 */
#define MOTOR_B_IN2_PIN GPIO_PIN_13 /* PB13 → BIN2 */
#define MOTOR_B_DIR_PORT GPIOB

/* TB6612 Standby */
#define MOTOR_STBY_PIN GPIO_PIN_14 /* PB14 → STBY */
#define MOTOR_STBY_PORT GPIOB

/* ========================== Emergency Stop ============================== */
#define ESTOP_PIN GPIO_PIN_13 /* PC13 — active LOW (button) */
#define ESTOP_PORT GPIOC

/* ========================== On-board LED ================================ */
#define LED_PIN GPIO_PIN_13 /* PC13 — Blue Pill LED */
#define LED_PORT GPIOC

/* ========================== Robot Parameters ============================ */
/* TODO: Change these to match your actual robot */
#define WHEEL_DIAMETER_MM 170.0f /* Wheel diameter in mm */
#define ENCODER_CPR 778          /* Encoder counts per motor rev */
#define GEAR_RATIO 21.3f         /* Motor gear ratio */
#define WHEEL_BASE_MM 25.0f      /* Distance between wheels in mm */

/* Derived: ticks per wheel revolution */
#define TICKS_PER_REV (ENCODER_CPR * 4 * GEAR_RATIO) /* x4 quadrature */
#define MM_PER_TICK ((3.14159265f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)

/* ========================== Control Loop ================================ */
#define CONTROL_FREQ_HZ 200                 /* 200 Hz = 5 ms period */
#define CONTROL_DT (1.0f / CONTROL_FREQ_HZ) /* 0.005 s */

/* ========================== Function prototypes ========================= */
void SystemClock_Config(void);
void Error_Handler(void);

#endif /* __MAIN_H */
