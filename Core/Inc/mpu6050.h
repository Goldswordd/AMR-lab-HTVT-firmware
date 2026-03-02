/**
 * @file    mpu6050.h
 * @brief   MPU6050 I2C driver — gyroscope Z-axis for heading
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* I2C address (AD0 = LOW → 0x68) */
#define MPU6050_ADDR (0x68 << 1)

/* Register map (subset) */
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_WHO_AM_I 0x75

/* Gyro sensitivity for ±250°/s range */
#define MPU6050_GYRO_SENSITIVITY 131.0f

typedef struct {
  I2C_HandleTypeDef *hi2c;
  float gyro_z_bias;     /* Calibrated bias (raw units) */
  float heading_deg;     /* Integrated heading (degrees) */
  float gyro_rate_dps;   /* Current angular rate (°/s) */
  int16_t raw_gz;        /* Raw gyro Z reading */
  uint8_t is_calibrated; /* Calibration done flag */
} MPU6050_t;

/**
 * @brief  Initialize MPU6050
 * @param  mpu  Pointer to MPU6050 handle
 * @param  hi2c Pointer to I2C handle
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Calibrate gyro Z bias — MUST be stationary!
 *         Takes ~2 seconds (2000 samples at 1 kHz)
 * @param  mpu  Pointer to MPU6050 handle
 */
void MPU6050_CalibrateGyro(MPU6050_t *mpu);

/**
 * @brief  Read gyro Z and update heading
 *         Call at fixed rate (CONTROL_DT)
 * @param  mpu  Pointer to MPU6050 handle
 * @param  dt   Time step in seconds
 */
void MPU6050_Update(MPU6050_t *mpu, float dt);

/**
 * @brief  Online bias recalibration — call when robot is STATIONARY
 *         Uses exponential moving average to correct thermal drift.
 *         Non-blocking, safe to call every control loop iteration.
 * @param  mpu  Pointer to MPU6050 handle
 */
void MPU6050_RecalibrateBias(MPU6050_t *mpu);

/**
 * @brief  Reset heading to zero
 * @param  mpu  Pointer to MPU6050 handle
 */
void MPU6050_ResetHeading(MPU6050_t *mpu);

/**
 * @brief  Get current heading in degrees
 * @param  mpu  Pointer to MPU6050 handle
 * @retval heading in degrees (-180 to +180 or continuous)
 */
float MPU6050_GetHeading(MPU6050_t *mpu);

/**
 * @brief  Get current angular rate in °/s
 * @param  mpu  Pointer to MPU6050 handle
 * @retval angular rate in °/s
 */
float MPU6050_GetRate(MPU6050_t *mpu);

#endif /* __MPU6050_H */
