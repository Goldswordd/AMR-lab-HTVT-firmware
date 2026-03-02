/**
 * @file    pid.h
 * @brief   Generic PID controller with anti-windup
 */
#ifndef __PID_H
#define __PID_H

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float dt;         /* Time step (seconds) */
  float integral;   /* Accumulated integral */
  float prev_error; /* Previous error for derivative */
  float out_min;    /* Output lower limit */
  float out_max;    /* Output upper limit */
  float iterm_max;  /* Max integral term (anti-windup) */
} PID_t;

/**
 * @brief  Initialize PID controller
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float out_min,
              float out_max);

/**
 * @brief  Compute PID output
 * @param  pid      PID handle
 * @param  setpoint Desired value
 * @param  feedback Current measured value
 * @retval Control output (clamped to [out_min, out_max])
 */
float PID_Compute(PID_t *pid, float setpoint, float feedback);

/**
 * @brief  Reset PID state (integral and prev_error)
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  Update PID gains at runtime
 */
void PID_SetGains(PID_t *pid, float Kp, float Ki, float Kd);

#endif /* __PID_H */
