/**
 * @file    uart_comm.c
 * @brief   Communication module — USB CDC (Virtual COM Port)
 *
 * Protocol:
 *   RX: "$CMD,linear,angular\n"  → cmd_vel
 *       "$ROT,angle,speed\n"     → rotate in place
 *       "$FWD,speed\n"           → go straight
 *       "$STP\n"                 → stop
 *       "$QRY\n"                 → request odometry
 *       "$PID,Kp,Ki,Kd\n"       → tune heading PID
 *
 *   TX: "$ODO,heading,enc_l,enc_r,spd_l,spd_r\n"
 */
#include "uart_comm.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RX line buffer */
#define RX_LINE_SIZE 128
static char rx_line[RX_LINE_SIZE];
static uint8_t rx_line_idx = 0;
static volatile uint8_t line_ready = 0;

/* TX buffer */
#define TX_BUF_SIZE 128
static char tx_buf[TX_BUF_SIZE];

/* ========================== Private helpers ============================= */

static float parse_float(const char **p) {
  while (**p == ',' || **p == ' ')
    (*p)++;
  return strtof(*p, (char **)p);
}

/* Process raw USB CDC received bytes into lines */
static void process_rx_bytes(uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (c == '\n' || c == '\r') {
      if (rx_line_idx > 0) {
        rx_line[rx_line_idx] = '\0';
        line_ready = 1;
      }
    } else if (rx_line_idx < RX_LINE_SIZE - 1) {
      rx_line[rx_line_idx++] = c;
    }
  }
}

/* ========================== Init ======================================== */

void UartComm_Init(void) {
  /* USB CDC is initialized in main.c via MX_USB_DEVICE_Init() */
  rx_line_idx = 0;
  line_ready = 0;
}

/* ========================== Process ===================================== */

UartCmd_t UartComm_Process(void) {
  UartCmd_t cmd = {CMD_NONE, 0, 0, 0};

  /* Check for new USB CDC data */
  uint8_t *rx_data;
  uint32_t rx_len;
  if (CDC_GetRxData(&rx_data, &rx_len)) {
    process_rx_bytes(rx_data, rx_len);
  }

  /* Check if a complete line is ready */
  if (!line_ready)
    return cmd;

  line_ready = 0;

  /* Parse command */
  if (strncmp(rx_line, "$CMD", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_VEL;
    cmd.param1 = parse_float(&p);
    cmd.param2 = parse_float(&p);
  } else if (strncmp(rx_line, "$ROT", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_ROTATE;
    cmd.param1 = parse_float(&p);
    cmd.param2 = parse_float(&p);
  } else if (strncmp(rx_line, "$FWD", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_STRAIGHT;
    cmd.param1 = parse_float(&p);
  } else if (strncmp(rx_line, "$STP", 4) == 0) {
    cmd.type = CMD_STOP;
  } else if (strncmp(rx_line, "$QRY", 4) == 0) {
    cmd.type = CMD_QUERY;
  } else if (strncmp(rx_line, "$PID", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_PID;
    cmd.param1 = parse_float(&p);
    cmd.param2 = parse_float(&p);
    cmd.param3 = parse_float(&p);
  } else if (strncmp(rx_line, "$TST", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_TEST;
    cmd.param1 = parse_float(&p); /* left PWM */
    cmd.param2 = parse_float(&p); /* right PWM */
  } else if (strncmp(rx_line, "$DBG", 4) == 0) {
    cmd.type = CMD_DBG;
  } else if (strncmp(rx_line, "$BIAS", 5) == 0) {
    const char *p = rx_line + 5;
    cmd.type = CMD_BIAS;
    cmd.param1 = parse_float(&p); /* left FF gain */
    cmd.param2 = parse_float(&p); /* right FF gain */
  } else if (strncmp(rx_line, "$SPD", 4) == 0) {
    const char *p = rx_line + 4;
    cmd.type = CMD_SPEED_PID;
    cmd.param1 = parse_float(&p); /* Kp */
    cmd.param2 = parse_float(&p); /* Ki */
    cmd.param3 = parse_float(&p); /* Kd */
  }

  /* Reset buffer for next line */
  rx_line_idx = 0;

  return cmd;
}

/* ========================== TX ========================================== */

void UartComm_SendOdometry(float heading, int16_t enc_left, int16_t enc_right,
                           float speed_l, float speed_r) {
  int len = snprintf(tx_buf, TX_BUF_SIZE, "$ODO,%.2f,%d,%d,%.1f,%.1f\r\n",
                     heading, enc_left, enc_right, speed_l, speed_r);
  if (len > 0)
    CDC_Transmit_FS((uint8_t *)tx_buf, (uint16_t)len);
}

void UartComm_SendString(const char *str) {
  CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}
