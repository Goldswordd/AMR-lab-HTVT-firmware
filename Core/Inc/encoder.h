/**
 * @file    encoder.h
 * @brief   Quadrature encoder interface via TIM2 (right) and TIM3 (left)
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum { ENCODER_LEFT = 0, ENCODER_RIGHT = 1 } Encoder_ID;

/**
 * @brief  Initialize TIM2 and TIM3 in encoder mode
 */
void Encoder_Init(void);

/**
 * @brief  Get tick count delta since last call (and reset)
 * @param  enc  ENCODER_LEFT or ENCODER_RIGHT
 * @retval Signed delta ticks (positive = forward)
 */
int16_t Encoder_GetDelta(Encoder_ID enc);

/**
 * @brief  Get absolute tick count (wrapping 16-bit)
 * @param  enc  ENCODER_LEFT or ENCODER_RIGHT
 * @retval Current counter value
 */
uint16_t Encoder_GetCount(Encoder_ID enc);

/**
 * @brief  Convert delta ticks to mm traveled
 * @param  delta_ticks  From Encoder_GetDelta
 * @retval Distance in mm
 */
float Encoder_TicksToMM(int16_t delta_ticks);

/**
 * @brief  Compute wheel speed in ticks/period from delta
 * @param  delta_ticks  From Encoder_GetDelta
 * @param  dt           Period in seconds
 * @retval Speed in ticks/second
 */
float Encoder_GetSpeed(int16_t delta_ticks, float dt);

#endif /* __ENCODER_H */
