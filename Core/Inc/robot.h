/**
 * @file    robot.h
 * @brief   High-level robot motion controller
 *          Dual PID: heading correction + wheel speed control
 */
#ifndef __ROBOT_H
#define __ROBOT_H

#include "mpu6050.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
  ROBOT_STATE_IDLE = 0,
  ROBOT_STATE_MOVING = 1,
  ROBOT_STATE_ROTATING = 2,
  ROBOT_STATE_ESTOP = 3
} Robot_State;

typedef struct {
  /* State */
  Robot_State state;

  /* Sensor data */
  MPU6050_t imu;
  int16_t enc_delta_left;
  int16_t enc_delta_right;
  float speed_left;  /* ticks/s */
  float speed_right; /* ticks/s */

  /* PID controllers */
  PID_t pid_heading;     /* heading correction → differential drive */
  PID_t pid_speed_left;  /* speed control left wheel */
  PID_t pid_speed_right; /* speed control right wheel */

  /* Setpoints */
  float target_heading;     /* degrees */
  float target_speed;       /* ticks/s (base speed for both wheels) */
  float target_angular_vel; /* deg/s for cmd_vel mode */

  /* Outputs */
  int16_t pwm_left;
  int16_t pwm_right;

  /* Rotation tracking */
  float rotation_target; /* absolute target heading for rotate cmd */
  uint8_t rotation_done;

  /* Odometry */
  float odom_x_mm;
  float odom_y_mm;
  float total_distance_mm;
} Robot_t;

/* Global robot instance */
extern Robot_t robot;

/**
 * @brief  Initialize robot (all subsystems)
 * @param  hi2c  I2C handle for MPU6050
 */
void Robot_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Main control loop update — call at CONTROL_FREQ_HZ (200 Hz)
 */
void Robot_Update(void);

/**
 * @brief  Command: go straight at given speed
 * @param  speed  Base speed in ticks/s (positive = forward)
 */
void Robot_GoStraight(float speed);

/**
 * @brief  Command: rotate in place by angle_deg
 * @param  angle_deg  Rotation angle (positive = CCW, negative = CW)
 * @param  speed      Rotation speed in ticks/s
 */
void Robot_Rotate(float angle_deg, float speed);

/**
 * @brief  Command: set velocity (linear + angular) — cmd_vel style
 * @param  linear_speed   Forward speed in ticks/s
 * @param  angular_vel    Angular velocity in deg/s
 */
void Robot_SetVelocity(float linear_speed, float angular_vel);

/**
 * @brief  Command: stop all motors
 */
void Robot_Stop(void);

/**
 * @brief  Check if rotation is complete
 * @retval 1 if done, 0 if still rotating
 */
uint8_t Robot_IsRotationDone(void);

/**
 * @brief  Set per-wheel feedforward gains for motor asymmetry compensation
 * @param  left_gain   FF gain for left wheel (default 1.5)
 * @param  right_gain  FF gain for right wheel (default 1.5)
 */
void Robot_SetFFBias(float left_gain, float right_gain);

#endif /* __ROBOT_H */
