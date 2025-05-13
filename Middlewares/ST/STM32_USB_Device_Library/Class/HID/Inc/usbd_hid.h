/**
 * Yolk Workshop Keyboard USB HID Implementation
 *
 * This file is a modified version of the USB HID implementation from
 * STMicroelectronics STM32 USB Device Library.
 *
 * Original Copyright (c) STMicroelectronics
 * Modifications Copyright (C) 2025 Yolk Workshop
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_HID_H
#define __USB_HID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/** @defgroup USBD_HID
  * @brief This file is the Header file for usbd_hid.c
  * @{
  */

/** @defgroup USBD_HID_Exported_Defines
  * @{
  */

/* Interface numbers */
#define HID_KEYBOARD_INTERFACE        0x01
#define HID_NKRO_INTERFACE            0x02
#define HID_CONSUMER_INTERFACE        0x00

/* Number of interfaces */
#define HID_NUM_INTERFACES            0x03

/* Endpoint addresses */
#define HID_KEYBOARD_EPIN_ADDR        0x82
#define HID_CONSUMER_EPIN_ADDR        0x81
#define HID_NKRO_EPIN_ADDR            0x83

/* Endpoint sizes */
#define HID_KEYBOARD_EPIN_SIZE        0x08	// 8 bytes for boot protocol
#define HID_NKRO_EPIN_SIZE            0x0D  /* 18 bytes for NKRO report */
#define HID_CONSUMER_EPIN_SIZE        0x02  /* 2 bytes for consumer report */

/* USB HID device Configuration Descriptor size */
#define USB_HID_CONFIG_DESC_SIZ       0x54  /* 106 bytes for 3 interfaces */
#define USB_HID_DESC_SIZ              0x09

/* Report descriptor sizes */
#define HID_KEYBOARD_REPORT_DESC_SIZE 63U  /* 63 bytes */
#define HID_NKRO_REPORT_DESC_SIZE     0x4A  /* 104 bytes */
#define HID_CONSUMER_REPORT_DESC_SIZE 57U  /* 39 bytes */

#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

#ifndef HID_HS_BINTERVAL
#define HID_HS_BINTERVAL              0x07
#endif

#ifndef HID_FS_BINTERVAL
#define HID_FS_BINTERVAL              0x0A
#endif

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
}
HID_StateTypeDef;

typedef struct
{
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  HID_StateTypeDef state;
}
USBD_HID_HandleTypeDef;
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */
extern USBD_ClassTypeDef USBD_HID;
#define USBD_HID_CLASS &USBD_HID
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_HID_SendKeyboardReport(USBD_HandleTypeDef *pdev,
                                    uint8_t *report,
                                    uint16_t len);

uint8_t USBD_HID_SendNKROReport(USBD_HandleTypeDef *pdev,
                                uint8_t *report,
                                uint16_t len);

uint8_t USBD_HID_SendConsumerReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report,
                                   uint16_t len);

uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);



/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_HID_H */

