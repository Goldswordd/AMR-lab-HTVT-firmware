/**
 * @file    robot.c
 * @brief   High-level robot motion — triple PID + feedforward architecture
 *
 * Architecture:
 *   1. HEADING PID: gyro heading → differential speed correction
 *   2. LEFT SPEED PID: encoder feedback + feedforward → left PWM
 *   3. RIGHT SPEED PID: encoder feedback + feedforward → right PWM
 *
 * Improvements v2:
 *   - Low-pass filtered encoder speed (removes quantization jitter)
 *   - Feedforward term (estimates base PWM from target speed)
 *   - Retuned PID gains for smooth operation
 *   - Dead zone compensation for motor static friction
 */
#include "robot.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include <math.h>

/* ========================== Global instance ============================= */
Robot_t robot;

/* ========================== Filtered speeds ============================= */
/* Low-pass filter smooths the quantized encoder readings (200Hz × low CPR
 * gives steppy speed values like 0, 200, 400...). Alpha=0.15 gives ~5Hz
 * bandwidth — smoother than 0.3 for low-CPR encoders. */
#define SPEED_FILTER_ALPHA 0.15f /* 0.0=very smooth, 1.0=raw */
static float filtered_speed_left = 0.02f;
static float filtered_speed_right = 0.0f;

/* ========================== PID Default Gains =========================== */
/*
 * Tuning procedure:
 * 1. Start with Ki=0, Kd=0
 * 2. Increase Kp until oscillation, then back off 30%
 * 3. Add Ki slowly until steady-state error vanishes
 * 4. Kd usually not needed with feedforward — leave at 0
 *
 * With feedforward, PID only corrects the RESIDUAL error,
 * so gains can be much lower than without feedforward.
 */

/* Heading PID: error in degrees → output in ticks/s correction
 * Stronger than before to quickly correct drift from motor asymmetry */
#define HEADING_KP 4.0f        /* Aggressive to fight wheel asymmetry */
#define HEADING_KI 0.3f        /* Faster steady-state heading correction */
#define HEADING_KD 0.02f       /* Light damping — reduce heading oscillation */
#define HEADING_OUT_MAX 150.0f /* Max differential correction (ticks/s) */

/* Speed PID: error in ticks/s → output in PWM units (0..1000)
 * Note: feedforward handles ~80% of the output, PID only corrects residual.
 * Ki reduced from 1.5 to 0.5 to prevent integral windup oscillation. */
#define SPEED_KP 0.5f
#define SPEED_KI 0.5f
#define SPEED_KD 0.0f /* Disabled — feedforward + LP filter is enough */
#define SPEED_OUT_MAX                                                          \
  400.0f /* PID correction range (feedforward adds the rest) */

/* Feedforward: maps target speed (ticks/s) directly to approximate PWM.
 * Measured empirically: at PWM=300, motor typically runs ~200 ticks/s.
 * So gain ≈ 300/200 = 1.5. Adjust based on your motors! */
#define SPEED_FF_GAIN 1.5f /* PWM per (tick/s) — default for both wheels */
#define MOTOR_MIN_PWM 60   /* Minimum PWM to overcome static friction */

/* Per-wheel feedforward gains — tunable via $BIAS command to compensate
 * for motor asymmetry (different friction, winding resistance, etc.) */
static float ff_gain_left = SPEED_FF_GAIN;
static float ff_gain_right = SPEED_FF_GAIN;

/* Speed ramp: smooth acceleration instead of instant speed jump.
 * 600 ticks/s² means reaching 300 ticks/s takes 0.5 seconds. */
#define SPEED_RAMP_RATE 600.0f /* ticks/s per second */
static float ramped_speed = 0.0f;

/* Rotation tolerance */
#define ROTATION_TOLERANCE_DEG 1.5f
#define ROTATION_SETTLE_COUNT 20 /* 20 loops × 5ms = 100ms settled */

/* ========================== Helper: angle difference ==================== */
static float angle_diff(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f)
    diff -= 360.0f;
  while (diff < -180.0f)
    diff += 360.0f;
  return diff;
}

/* ========================== Helper: apply dead zone ===================== */
static int16_t apply_deadzone(float pwm_f) {
  int16_t pwm = (int16_t)pwm_f;
  /* If PID+FF wants the motor to move but PWM is below minimum,
   * bump it up to overcome static friction */
  if (pwm > 0 && pwm < MOTOR_MIN_PWM)
    pwm = MOTOR_MIN_PWM;
  if (pwm < 0 && pwm > -MOTOR_MIN_PWM)
    pwm = -MOTOR_MIN_PWM;
  /* Clamp to valid range */
  if (pwm > 1000)
    pwm = 1000;
  if (pwm < -1000)
    pwm = -1000;
  return pwm;
}

/* ========================== Init ======================================== */

void Robot_Init(I2C_HandleTypeDef *hi2c) {
  /* Init subsystems */
  Motor_Init();
  Encoder_Init();

  /* Init IMU */
  MPU6050_Init(&robot.imu, hi2c);
  MPU6050_CalibrateGyro(&robot.imu); /* ~2 seconds — robot must be still! */

  /* Init PIDs */
  PID_Init(&robot.pid_heading, HEADING_KP, HEADING_KI, HEADING_KD, CONTROL_DT,
           -HEADING_OUT_MAX, HEADING_OUT_MAX);

  PID_Init(&robot.pid_speed_left, SPEED_KP, SPEED_KI, SPEED_KD, CONTROL_DT,
           -SPEED_OUT_MAX, SPEED_OUT_MAX);

  PID_Init(&robot.pid_speed_right, SPEED_KP, SPEED_KI, SPEED_KD, CONTROL_DT,
           -SPEED_OUT_MAX, SPEED_OUT_MAX);

  /* Init state */
  robot.state = ROBOT_STATE_IDLE;
  robot.target_heading = 0.0f;
  robot.target_speed = 0.0f;
  robot.target_angular_vel = 0.0f;
  robot.pwm_left = 0;
  robot.pwm_right = 0;
  robot.rotation_done = 1;
  robot.odom_x_mm = 0.0f;
  robot.odom_y_mm = 0.0f;
  robot.total_distance_mm = 0.0f;
  filtered_speed_left = 0.0f;
  filtered_speed_right = 0.0f;

  Motor_Enable();
}

/* ========================== Control loop ================================ */

void Robot_Update(void) {
  /* Step 1: Read IMU — integrate heading */
  MPU6050_Update(&robot.imu, CONTROL_DT);

  /* Step 2: Read encoders + low-pass filter */
  robot.enc_delta_left = Encoder_GetDelta(ENCODER_LEFT);
  robot.enc_delta_right = Encoder_GetDelta(ENCODER_RIGHT);

  float raw_speed_left = Encoder_GetSpeed(robot.enc_delta_left, CONTROL_DT);
  float raw_speed_right = Encoder_GetSpeed(robot.enc_delta_right, CONTROL_DT);

  /* Low-pass filter: smooth out quantized encoder readings */
  filtered_speed_left +=
      SPEED_FILTER_ALPHA * (raw_speed_left - filtered_speed_left);
  filtered_speed_right +=
      SPEED_FILTER_ALPHA * (raw_speed_right - filtered_speed_right);

  robot.speed_left = filtered_speed_left;
  robot.speed_right = filtered_speed_right;

  /* Step 3: Update odometry */
  float dl = Encoder_TicksToMM(robot.enc_delta_left);
  float dr = Encoder_TicksToMM(robot.enc_delta_right);
  float dc = (dl + dr) / 2.0f;
  float heading_rad = robot.imu.heading_deg * (3.14159265f / 180.0f);
  robot.odom_x_mm += dc * cosf(heading_rad);
  robot.odom_y_mm += dc * sinf(heading_rad);
  robot.total_distance_mm += fabsf(dc);

  /* Step 4: State machine */
  switch (robot.state) {
  case ROBOT_STATE_IDLE:
    /* Reset filter and ramp when stopped to prevent lag on restart */
    filtered_speed_left = 0.0f;
    filtered_speed_right = 0.0f;
    ramped_speed = 0.0f;
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);

    /* Auto-recalibrate gyro bias while stationary.
     * This corrects thermal drift that causes heading errors over time.
     * EMA with alpha=0.001 → bias converges in ~5-10 seconds idle. */
    MPU6050_RecalibrateBias(&robot.imu);
    break;

  case ROBOT_STATE_MOVING: {
    /* Speed ramp-up: smoothly approach target speed */
    if (ramped_speed < robot.target_speed) {
      ramped_speed += SPEED_RAMP_RATE * CONTROL_DT;
      if (ramped_speed > robot.target_speed)
        ramped_speed = robot.target_speed;
    } else if (ramped_speed > robot.target_speed) {
      ramped_speed -= SPEED_RAMP_RATE * CONTROL_DT;
      if (ramped_speed < robot.target_speed)
        ramped_speed = robot.target_speed;
    }

    /* Heading PID: correct for drift */
    float heading_error =
        angle_diff(robot.target_heading, robot.imu.heading_deg);
    float heading_correction =
        PID_Compute(&robot.pid_heading, 0.0f, -heading_error);

    /* Speed setpoints: ramped base ± heading correction */
    float sp_left = ramped_speed + heading_correction;
    float sp_right = ramped_speed - heading_correction;

    /* Feedforward: per-wheel gains compensate motor asymmetry */
    float ff_l = sp_left * ff_gain_left;
    float ff_r = sp_right * ff_gain_right;

    /* Speed PID: correct residual error only */
    float pid_left =
        PID_Compute(&robot.pid_speed_left, sp_left, robot.speed_left);
    float pid_right =
        PID_Compute(&robot.pid_speed_right, sp_right, robot.speed_right);

    /* Total output = feedforward + PID correction */
    float out_left = ff_l + pid_left;
    float out_right = ff_r + pid_right;

    robot.pwm_left = apply_deadzone(out_left);
    robot.pwm_right = apply_deadzone(out_right);

    Motor_SetSpeed(MOTOR_LEFT, robot.pwm_left);
    Motor_SetSpeed(MOTOR_RIGHT, robot.pwm_right);
    break;
  }

  case ROBOT_STATE_ROTATING: {
    /* Heading error to target angle */
    float heading_error =
        angle_diff(robot.rotation_target, robot.imu.heading_deg);
    static int settle_count = 0;

    if (fabsf(heading_error) < ROTATION_TOLERANCE_DEG) {
      settle_count++;
      if (settle_count >= ROTATION_SETTLE_COUNT) {
        /* Rotation complete */
        Motor_SetSpeed(MOTOR_LEFT, 0);
        Motor_SetSpeed(MOTOR_RIGHT, 0);
        robot.rotation_done = 1;
        robot.state = ROBOT_STATE_IDLE;
        settle_count = 0;
        PID_Reset(&robot.pid_heading);
        PID_Reset(&robot.pid_speed_left);
        PID_Reset(&robot.pid_speed_right);
        break;
      }
    } else {
      settle_count = 0;
    }

    /* PID on heading error → rotation speed */
    float rot_output = PID_Compute(&robot.pid_heading, 0.0f, -heading_error);

    /* Feedforward for rotation */
    float ff_left = rot_output * SPEED_FF_GAIN;
    float ff_right = -rot_output * SPEED_FF_GAIN;

    /* Speed PID per wheel */
    float pid_left =
        PID_Compute(&robot.pid_speed_left, rot_output, robot.speed_left);
    float pid_right =
        PID_Compute(&robot.pid_speed_right, -rot_output, robot.speed_right);

    robot.pwm_left = apply_deadzone(ff_left + pid_left);
    robot.pwm_right = apply_deadzone(ff_right + pid_right);

    Motor_SetSpeed(MOTOR_LEFT, robot.pwm_left);
    Motor_SetSpeed(MOTOR_RIGHT, robot.pwm_right);
    break;
  }

  case ROBOT_STATE_ESTOP:
    Motor_Brake();
    break;
  }
}

/* ========================== Commands ==================================== */

void Robot_GoStraight(float speed) {
  /* Lock current heading as target */
  robot.target_heading = robot.imu.heading_deg;
  robot.target_speed = speed;
  robot.state = ROBOT_STATE_MOVING;

  PID_Reset(&robot.pid_heading);
  PID_Reset(&robot.pid_speed_left);
  PID_Reset(&robot.pid_speed_right);
}

void Robot_Rotate(float angle_deg, float speed) {
  /* Absolute target = current + relative angle */
  robot.rotation_target = robot.imu.heading_deg + angle_deg;

  /* Normalize to [-180, +180] */
  while (robot.rotation_target > 180.0f)
    robot.rotation_target -= 360.0f;
  while (robot.rotation_target < -180.0f)
    robot.rotation_target += 360.0f;

  robot.target_speed = speed;
  robot.rotation_done = 0;
  robot.state = ROBOT_STATE_ROTATING;

  PID_Reset(&robot.pid_heading);
  PID_Reset(&robot.pid_speed_left);
  PID_Reset(&robot.pid_speed_right);
}

void Robot_SetVelocity(float linear_speed, float angular_vel) {
  if (fabsf(linear_speed) < 0.1f && fabsf(angular_vel) < 0.1f) {
    Robot_Stop();
    return;
  }

  robot.target_speed = linear_speed;
  robot.target_angular_vel = angular_vel;

  /* Update heading target continuously based on angular velocity */
  robot.target_heading += angular_vel * CONTROL_DT;
  while (robot.target_heading > 180.0f)
    robot.target_heading -= 360.0f;
  while (robot.target_heading < -180.0f)
    robot.target_heading += 360.0f;

  if (robot.state != ROBOT_STATE_MOVING) {
    robot.state = ROBOT_STATE_MOVING;
    PID_Reset(&robot.pid_heading);
    PID_Reset(&robot.pid_speed_left);
    PID_Reset(&robot.pid_speed_right);
  }
}

void Robot_Stop(void) {
  robot.state = ROBOT_STATE_IDLE;
  robot.target_speed = 0.0f;
  robot.pwm_left = 0;
  robot.pwm_right = 0;

  Motor_Brake();

  PID_Reset(&robot.pid_heading);
  PID_Reset(&robot.pid_speed_left);
  PID_Reset(&robot.pid_speed_right);
  filtered_speed_left = 0.0f;
  filtered_speed_right = 0.0f;
  ramped_speed = 0.0f;

  /* Reset heading to zero on stop.
   * This prevents gyro drift from accumulating across multiple maneuvers.
   * Each new GoStraight/Rotate starts with a fresh heading reference. */
  MPU6050_ResetHeading(&robot.imu);
}

uint8_t Robot_IsRotationDone(void) { return robot.rotation_done; }

void Robot_SetFFBias(float left_gain, float right_gain) {
  ff_gain_left = left_gain;
  ff_gain_right = right_gain;
}
