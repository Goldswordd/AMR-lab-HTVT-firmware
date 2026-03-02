/**
 * @file    usbd_cdc_if.c
 * @brief   USB CDC interface — based on ST official template
 *
 * Provides RX/TX buffer management for USB Virtual COM Port.
 * Follows the USBD_CDC_ItfTypeDef interface (5 callbacks).
 */
#include "usbd_cdc_if.h"
#include <string.h>

/* ========================== Buffers ====================================== */
#define APP_RX_DATA_SIZE 256
#define APP_TX_DATA_SIZE 256

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* RX state — set by interrupt callback, polled by main loop */
static volatile uint32_t rx_data_len = 0;
static volatile uint8_t rx_data_ready = 0;
static volatile uint8_t cdc_initialized = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Line coding: 115200 baud, 8N1 */
static USBD_CDC_LineCodingTypeDef LineCoding = {
    .bitrate = 115200,
    .format = 0x00,     /* 1 stop bit */
    .paritytype = 0x00, /* None */
    .datatype = 0x08    /* 8 bits */
};

/* ========================== CDC Interface Callbacks ====================== */

static int8_t CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  cdc_initialized = 1;
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void) {
  cdc_initialized = 0;
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  (void)length;

  switch (cmd) {
  case CDC_SET_LINE_CODING:
    LineCoding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) |
                                    (pbuf[3] << 24));
    LineCoding.format = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype = pbuf[6];
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    break;

  case CDC_SEND_ENCAPSULATED_COMMAND:
  case CDC_GET_ENCAPSULATED_RESPONSE:
  case CDC_SET_COMM_FEATURE:
  case CDC_GET_COMM_FEATURE:
  case CDC_CLEAR_COMM_FEATURE:
  case CDC_SEND_BREAK:
  default:
    break;
  }

  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len) {
  /* Copy length, mark data ready for main loop polling */
  rx_data_len = *Len;
  rx_data_ready = 1;

  /* Prepare for next reception — MUST call this to re-arm the endpoint */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  return (USBD_OK);
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum) {
  (void)Buf;
  (void)Len;
  (void)epnum;
  return (USBD_OK);
}

/* ========================== Interface Operations Table =================== */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS,
    CDC_TransmitCplt_FS /* TransmitCplt — 5th callback required by new API */
};

/* ========================== Public API =================================== */

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len) {
  USBD_CDC_HandleTypeDef *hcdc;

  if (!cdc_initialized)
    return USBD_FAIL;

  hcdc = (USBD_CDC_HandleTypeDef *)
             hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
  if (hcdc == NULL)
    return USBD_FAIL;
  if (hcdc->TxState != 0)
    return USBD_BUSY;

  /* Copy data to TX buffer to prevent caller's buffer from being modified */
  if (Len > APP_TX_DATA_SIZE)
    Len = APP_TX_DATA_SIZE;
  memcpy(UserTxBufferFS, Buf, Len);

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

uint8_t CDC_GetRxData(uint8_t **Buf, uint32_t *Len) {
  if (rx_data_ready) {
    rx_data_ready = 0;
    *Buf = UserRxBufferFS;
    *Len = rx_data_len;
    return 1;
  }
  return 0;
}

uint8_t CDC_IsConnected(void) { return cdc_initialized; }
