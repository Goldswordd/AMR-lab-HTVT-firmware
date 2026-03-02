/**
 * @file    usbd_desc.c
 * @brief   USB Device descriptors for CDC ACM (Virtual COM Port)
 */
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_core.h"

#define USBD_VID 0x0483           /* STMicroelectronics */
#define USBD_PID_FS 0x5740        /* STM Virtual COM Port */
#define USBD_LANGID_STRING 0x0409 /* English (US) */
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#define USBD_PRODUCT_STRING_FS "AMR Robot CDC"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS "CDC Interface"

/* USB Standard Device Descriptor */
static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] = {
    0x12,                 /* bLength */
    USB_DESC_TYPE_DEVICE, /* bDescriptorType */
    0x00,
    0x02,             /* bcdUSB = 2.00 */
    0x02,             /* bDeviceClass: CDC */
    0x02,             /* bDeviceSubClass */
    0x00,             /* bDeviceProtocol */
    USB_MAX_EP0_SIZE, /* bMaxPacketSize */
    LOBYTE(USBD_VID),
    HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS),
    HIBYTE(USBD_PID_FS),
    0x00,
    0x02,                      /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR,          /* Index of manufacturer string */
    USBD_IDX_PRODUCT_STR,      /* Index of product string */
    USBD_IDX_SERIAL_STR,       /* Index of serial number string */
    USBD_MAX_NUM_CONFIGURATION /* bNumConfigurations */
};

/* USB Language ID Descriptor */
static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING)};

/* Internal string descriptor buffer */
static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

/* ========================== Descriptor callbacks ======================== */

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed,
                                         uint16_t *length) {
  (void)speed;
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed,
                                            uint16_t *length) {
  (void)speed;
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed,
                                                  uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed,
                                             uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed,
                                            uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)"AMR001", USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed,
                                            uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed,
                                               uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

USBD_DescriptorsTypeDef FS_Desc = {
    USBD_FS_DeviceDescriptor,          USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor, USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,       USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor};
