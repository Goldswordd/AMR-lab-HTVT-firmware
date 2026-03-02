/**
 * @file    usbd_conf.h
 * @brief   USB Device configuration header
 */
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_MAX_STR_DESC_SIZ 512
#define USBD_SELF_POWERED 1
#define USBD_DEBUG_LEVEL 0
#define USBD_CDC_INTERVAL 2000

#define DEVICE_FS 0

/* Memory management macros */
#define USBD_malloc(x) ((void *)usbd_mem_buf)
#define USBD_free(x) ((void)(x))
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay

/* Debug macros */
#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
#else
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
#endif

/* Exported for memory allocation */
extern uint32_t usbd_mem_buf[];

#endif /* __USBD_CONF_H */
