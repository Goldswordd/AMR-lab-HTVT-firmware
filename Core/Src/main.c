/**
 * @file    main.c
 * @brief   AMR Robot Firmware — Main application
 *
 * Control architecture:
 *   SysTick 1kHz → control_flag every 5ms (200 Hz)
 *   Main loop: if (control_flag) → Robot_Update() [all PID + sensors]
 *              uart_comm_process() → parse commands from Pi
 *
 * Startup sequence:
 *   1. Clock 72 MHz
 *   2. Init peripherals
 *   3. Calibrate gyro (~2 sec, must be stationary!)
 *   4. Enter main loop
 */
#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "robot.h"
#include "uart_comm.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_desc.h"

/* USB Device handle */
USBD_HandleTypeDef hUsbDeviceFS;

/* From stm32f1xx_it.c */
extern volatile uint8_t control_flag;

/* Peripheral handles */
static I2C_HandleTypeDef hi2c1;

/* Odometry send counter (send every 50ms = 20 Hz) */
static uint8_t odom_divider = 0;
#define ODOM_SEND_EVERY 10 /* 10 × 5ms = 50ms */

/* Emergency stop state */
static uint8_t estop_active = 0;

/* ========================== Peripheral Init ============================= */

static void I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000; /* 400 kHz Fast mode */
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

static void GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PC13 — Emergency stop (input, active LOW with internal pull-up) */
  GPIO_InitStruct.Pin = ESTOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_PORT, &GPIO_InitStruct);
}

/* ========================== Emergency stop check ======================== */

static void CheckEmergencyStop(void) {
  /* DISABLED: PC13 is the Blue Pill onboard LED — reads LOW by default
   * and falsely triggers ESTOP. Uncomment if using a real ESTOP button
   * on a DIFFERENT pin. */
  (void)0;
}

/* ========================== Command handler ============================= */

static char dbg_buf[128];

static void HandleCommand(UartCmd_t cmd) {
  switch (cmd.type) {
  case CMD_VEL:
    Robot_SetVelocity(cmd.param1, cmd.param2);
    break;

  case CMD_ROTATE:
    Robot_Rotate(cmd.param1, cmd.param2);
    break;

  case CMD_STRAIGHT:
    Robot_GoStraight(cmd.param1);
    break;

  case CMD_STOP:
    Robot_Stop();
    break;

  case CMD_QUERY:
    UartComm_SendOdometry(robot.imu.heading_deg, robot.enc_delta_left,
                          robot.enc_delta_right, robot.speed_left,
                          robot.speed_right);
    break;

  case CMD_PID:
    /* Live PID tuning: $PID,Kp,Ki,Kd → heading PID */
    PID_SetGains(&robot.pid_heading, cmd.param1, cmd.param2, cmd.param3);
    UartComm_SendString("$PID,OK\r\n");
    break;

  case CMD_TEST: {
    /* Raw motor test: $TST,left_pwm,right_pwm — bypasses PID entirely */
    int16_t left_pwm = (int16_t)cmd.param1;
    int16_t right_pwm = (int16_t)cmd.param2;
    robot.state = ROBOT_STATE_IDLE; /* Disable PID */
    Motor_SetSpeed(MOTOR_LEFT, left_pwm);
    Motor_SetSpeed(MOTOR_RIGHT, right_pwm);
    snprintf(dbg_buf, sizeof(dbg_buf), "$TST,OK,L=%d,R=%d\r\n", left_pwm,
             right_pwm);
    UartComm_SendString(dbg_buf);
    break;
  }

  case CMD_DBG: {
    /* Debug: print state, encoder, PID, ESTOP status */
    snprintf(dbg_buf, sizeof(dbg_buf),
             "$DBG,st=%d,hdg=%.1f,eL=%d,eR=%d,sL=%.0f,sR=%.0f,pL=%d,pR=%d\r\n",
             robot.state, robot.imu.heading_deg, robot.enc_delta_left,
             robot.enc_delta_right, robot.speed_left, robot.speed_right,
             robot.pwm_left, robot.pwm_right);
    UartComm_SendString(dbg_buf);
    break;
  }

  case CMD_BIAS: {
    /* Per-wheel feedforward bias: $BIAS,left_ff,right_ff */
    Robot_SetFFBias(cmd.param1, cmd.param2);
    snprintf(dbg_buf, sizeof(dbg_buf), "$BIAS,OK,L=%.2f,R=%.2f\r\n", cmd.param1,
             cmd.param2);
    UartComm_SendString(dbg_buf);
    break;
  }

  case CMD_SPEED_PID: {
    /* Tune speed PID (both wheels): $SPD,Kp,Ki,Kd */
    PID_SetGains(&robot.pid_speed_left, cmd.param1, cmd.param2, cmd.param3);
    PID_SetGains(&robot.pid_speed_right, cmd.param1, cmd.param2, cmd.param3);
    snprintf(dbg_buf, sizeof(dbg_buf), "$SPD,OK,Kp=%.2f,Ki=%.2f,Kd=%.2f\r\n",
             cmd.param1, cmd.param2, cmd.param3);
    UartComm_SendString(dbg_buf);
    break;
  }

  default:
    break;
  }
}

/* ========================== Main ======================================== */

int main(void) {
  /* HAL init + SysTick 1kHz */
  HAL_Init();

  /* System clock: HSE 8MHz → PLL x9 → 72 MHz */
  SystemClock_Config();

  /* Init peripherals */
  GPIO_Init();
  I2C1_Init();

  /* Init USB CDC (Virtual COM Port) */
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);

  /* Wait for USB enumeration */
  HAL_Delay(1000);

  UartComm_Init();

  /* Startup message */
  UartComm_SendString("\r\n=== AMR Robot v1.0 ===\r\n");
  UartComm_SendString("Calibrating gyro... keep robot STILL!\r\n");

  /* Init robot (includes MPU6050 calibration — ~2 sec) */
  Robot_Init(&hi2c1);

  UartComm_SendString("Calibration done. Ready.\r\n");

  /* Main loop */
  while (1) {
    /* ---- Control loop @ 200 Hz (every 5ms) ---- */
    if (control_flag) {
      control_flag = 0;

      /* Check emergency stop */
      CheckEmergencyStop();

      /* Run control update (sensors + PID + motors) */
      if (!estop_active) {
        Robot_Update();
      }

      /* Send odometry periodically (20 Hz) */
      odom_divider++;
      if (odom_divider >= ODOM_SEND_EVERY) {
        odom_divider = 0;
        UartComm_SendOdometry(robot.imu.heading_deg, robot.enc_delta_left,
                              robot.enc_delta_right, robot.speed_left,
                              robot.speed_right);
      }
    }

    /* ---- UART command processing (non-blocking) ---- */
    UartCmd_t cmd = UartComm_Process();
    if (cmd.type != CMD_NONE) {
      HandleCommand(cmd);
    }
  }
}
