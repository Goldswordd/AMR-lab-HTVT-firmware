/**
 * @file    mpu6050.c
 * @brief   MPU6050 I2C driver — gyro Z integration for heading
 *
 * KEY DESIGN DECISIONS for accurate heading:
 *   1. Calibrate bias at startup (average 2000 stationary samples)
 *   2. Use ±250°/s range for maximum sensitivity (131 LSB/°/s)
 *   3. DLPF at 42 Hz to filter mechanical vibration
 *   4. Fixed-rate integration (5ms) — NEVER use variable dt
 *   5. Dead zone: ignore |rate| < 0.15°/s to prevent noise drift
 */
#include "mpu6050.h"
#include "main.h"
#include <math.h>

/* Dead zone threshold — below this, gyro reads as zero */
#define GYRO_DEADZONE_DPS 0.20f

/* Online recalibration: exponential moving average rate */
#define RECAL_ALPHA                                                            \
  0.001f /* Very slow update — ~5 sec time constant at 200Hz */

/* ========================== Private helpers ============================= */

static HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_t *mpu, uint8_t reg,
                                          uint8_t data) {
  return HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR, reg, 1, &data, 1, 100);
}

static HAL_StatusTypeDef MPU6050_ReadReg(MPU6050_t *mpu, uint8_t reg,
                                         uint8_t *data, uint16_t len) {
  return HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR, reg, 1, data, len, 100);
}

static int16_t MPU6050_ReadGyroZ_Raw(MPU6050_t *mpu) {
  uint8_t buf[2];
  MPU6050_ReadReg(mpu, MPU6050_REG_GYRO_ZOUT_H, buf, 2);
  return (int16_t)((buf[0] << 8) | buf[1]);
}

/* ========================== Public API ================================== */

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c) {
  HAL_StatusTypeDef status;
  uint8_t who_am_i = 0;

  mpu->hi2c = hi2c;
  mpu->gyro_z_bias = 0.0f;
  mpu->heading_deg = 0.0f;
  mpu->gyro_rate_dps = 0.0f;
  mpu->raw_gz = 0;
  mpu->is_calibrated = 0;

  /* Check WHO_AM_I (should be 0x68) */
  status = MPU6050_ReadReg(mpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
  if (status != HAL_OK || who_am_i != 0x68) {
    return HAL_ERROR;
  }

  /* Wake up — clear sleep bit, use PLL with Gyro Z ref */
  MPU6050_WriteReg(mpu, MPU6050_REG_PWR_MGMT_1, 0x03);
  HAL_Delay(50);

  /* Sample rate = 1 kHz / (1+0) = 1 kHz */
  MPU6050_WriteReg(mpu, MPU6050_REG_SMPLRT_DIV, 0x00);

  /* DLPF = 42 Hz (register value 3) — filters motor vibration */
  MPU6050_WriteReg(mpu, MPU6050_REG_CONFIG, 0x03);

  /* Gyro range = ±250°/s (max sensitivity: 131 LSB/°/s) */
  MPU6050_WriteReg(mpu, MPU6050_REG_GYRO_CONFIG, 0x00);

  /* Accel range = ±2g (not critical but set for completeness) */
  MPU6050_WriteReg(mpu, MPU6050_REG_ACCEL_CONFIG, 0x00);

  /* Disable interrupts — we poll */
  MPU6050_WriteReg(mpu, MPU6050_REG_INT_ENABLE, 0x00);

  HAL_Delay(100); /* Let DLPF settle */

  return HAL_OK;
}

void MPU6050_CalibrateGyro(MPU6050_t *mpu) {
  /**
   * CRITICAL: Robot must be STATIONARY during calibration!
   * Read 2000 samples → compute average bias for Gz.
   * With 1 kHz sample rate, this takes ~2 seconds.
   */
  const int NUM_SAMPLES = 5000;
  float sum = 0.0f;
  float sum_sq = 0.0f;

  /* Discard first 100 samples (settling transients) */
  for (int i = 0; i < 100; i++) {
    MPU6050_ReadGyroZ_Raw(mpu);
    HAL_Delay(1);
  }

  /* Collect calibration samples */
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int16_t raw = MPU6050_ReadGyroZ_Raw(mpu);
    float val = (float)raw;
    sum += val;
    sum_sq += val * val;
    HAL_Delay(1);
  }

  mpu->gyro_z_bias = sum / (float)NUM_SAMPLES;

  /* Compute std dev for diagnostics (could send via UART) */
  /* float variance = (sum_sq / NUM_SAMPLES) - (mpu->gyro_z_bias *
   * mpu->gyro_z_bias); */
  /* float std_dev  = sqrtf(variance); */

  mpu->heading_deg = 0.0f;
  mpu->is_calibrated = 1;
}

void MPU6050_Update(MPU6050_t *mpu, float dt) {
  if (!mpu->is_calibrated)
    return;

  /* Read raw gyro Z */
  mpu->raw_gz = MPU6050_ReadGyroZ_Raw(mpu);

  /* Convert to °/s with bias correction */
  float raw_corrected = (float)mpu->raw_gz - mpu->gyro_z_bias;
  mpu->gyro_rate_dps = raw_corrected / MPU6050_GYRO_SENSITIVITY;

  /* Dead zone — suppress tiny noise that causes drift */
  if (fabsf(mpu->gyro_rate_dps) < GYRO_DEADZONE_DPS) {
    mpu->gyro_rate_dps = 0.0f;
  }

  /* Integrate heading (trapezoidal could improve accuracy but
     rectangular is fine at 200 Hz with DLPF enabled) */
  mpu->heading_deg += mpu->gyro_rate_dps * dt;

  /* Normalize to [-180, +180] */
  while (mpu->heading_deg > 180.0f)
    mpu->heading_deg -= 360.0f;
  while (mpu->heading_deg < -180.0f)
    mpu->heading_deg += 360.0f;
}

void MPU6050_RecalibrateBias(MPU6050_t *mpu) {
  /**
   * Online bias recalibration — call ONLY when robot is stationary!
   * Uses exponential moving average to slowly adjust the gyro bias.
   * This compensates for thermal drift without blocking like full calibration.
   *
   * At 200 Hz with RECAL_ALPHA=0.001, time constant ≈ 5 seconds.
   * After ~10 seconds idle, bias is well-corrected for the current temperature.
   */
  if (!mpu->is_calibrated)
    return;

  float raw = (float)MPU6050_ReadGyroZ_Raw(mpu);
  mpu->gyro_z_bias += RECAL_ALPHA * (raw - mpu->gyro_z_bias);
}

void MPU6050_ResetHeading(MPU6050_t *mpu) { mpu->heading_deg = 0.0f; }

float MPU6050_GetHeading(MPU6050_t *mpu) { return mpu->heading_deg; }

float MPU6050_GetRate(MPU6050_t *mpu) { return mpu->gyro_rate_dps; }
