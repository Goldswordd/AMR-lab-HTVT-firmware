/**
 * @file    pid.c
 * @brief   Generic PID controller with anti-windup (integral clamping)
 */
#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float out_min,
              float out_max) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->dt = dt;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->out_min = out_min;
  pid->out_max = out_max;
  /* Anti-windup: limit integral contribution to 40% of output range */
  pid->iterm_max = (out_max - out_min) * 0.4f;
}

float PID_Compute(PID_t *pid, float setpoint, float feedback) {
  float error = setpoint - feedback;

  /* Proportional */
  float p_term = pid->Kp * error;

  /* Integral with anti-windup */
  pid->integral += error * pid->dt;
  float i_term = pid->Ki * pid->integral;

  /* Clamp integral term */
  if (i_term > pid->iterm_max) {
    i_term = pid->iterm_max;
    pid->integral = pid->iterm_max / pid->Ki;
  } else if (i_term < -pid->iterm_max) {
    i_term = -pid->iterm_max;
    pid->integral = -pid->iterm_max / pid->Ki;
  }

  /* Derivative (on error, with low-pass implicit from fixed dt) */
  float d_term = pid->Kd * (error - pid->prev_error) / pid->dt;
  pid->prev_error = error;

  /* Sum and clamp output */
  float output = p_term + i_term + d_term;

  if (output > pid->out_max)
    output = pid->out_max;
  if (output < pid->out_min)
    output = pid->out_min;

  return output;
}

void PID_Reset(PID_t *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

void PID_SetGains(PID_t *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  PID_Reset(pid); /* Reset state when gains change */
}
