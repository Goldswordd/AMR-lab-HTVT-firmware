/**
 * @file    uart_comm.h
 * @brief   UART communication protocol with Pi (USART2)
 */
#ifndef __UART_COMM_H
#define __UART_COMM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Command types from Pi */
typedef enum {
  CMD_NONE = 0,
  CMD_VEL = 1,        /* $CMD,linear,angular\n */
  CMD_ROTATE = 2,     /* $ROT,angle,speed\n */
  CMD_STRAIGHT = 3,   /* $FWD,speed\n */
  CMD_STOP = 4,       /* $STP\n */
  CMD_QUERY = 5,      /* $QRY\n — request odometry */
  CMD_PID = 6,        /* $PID,Kp,Ki,Kd\n — tune heading PID */
  CMD_TEST = 7,       /* $TST,left_pwm,right_pwm\n — raw motor test */
  CMD_DBG = 8,        /* $DBG\n — print debug info */
  CMD_BIAS = 9,       /* $BIAS,left_ff,right_ff\n — per-wheel FF bias */
  CMD_SPEED_PID = 10, /* $SPD,Kp,Ki,Kd\n — tune speed PID */
} UartCmd_Type;

typedef struct {
  UartCmd_Type type;
  float param1;
  float param2;
  float param3;
} UartCmd_t;

/**
 * @brief  Initialize USART2 for Pi communication
 */
void UartComm_Init(void);

/**
 * @brief  Process incoming UART data (non-blocking, call in main loop)
 * @retval Parsed command (CMD_NONE if nothing new)
 */
UartCmd_t UartComm_Process(void);

/**
 * @brief  Send odometry data to Pi
 * @param  heading     Current heading (degrees)
 * @param  enc_left    Left encoder delta
 * @param  enc_right   Right encoder delta
 * @param  speed_l     Left speed (ticks/s)
 * @param  speed_r     Right speed (ticks/s)
 */
void UartComm_SendOdometry(float heading, int16_t enc_left, int16_t enc_right,
                           float speed_l, float speed_r);

/**
 * @brief  Send a debug/status string
 */
void UartComm_SendString(const char *str);

#endif /* __UART_COMM_H */
