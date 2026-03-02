/**
 * @file    usbd_cdc_if.h
 * @brief   USB CDC interface — TX/RX buffer management
 */
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/**
 * @brief  Transmit data over USB CDC
 * @param  Buf   Data buffer
 * @param  Len   Length in bytes
 * @retval USBD_OK on success
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/**
 * @brief  Get pointer to last received data and its length
 * @param  Buf   Output: pointer to rx buffer
 * @param  Len   Output: number of bytes received
 * @retval 1 if new data available, 0 if not
 */
uint8_t CDC_GetRxData(uint8_t **Buf, uint32_t *Len);

/**
 * @brief  Check if USB CDC is connected and ready
 * @retval 1 if ready, 0 if not
 */
uint8_t CDC_IsConnected(void);

#endif /* __USBD_CDC_IF_H */
