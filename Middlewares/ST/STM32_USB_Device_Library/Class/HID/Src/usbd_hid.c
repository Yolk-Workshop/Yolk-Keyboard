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

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_HID
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_HID_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_HID_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_HID_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_HID_Private_FunctionPrototypes
 * @{
 */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);

static uint8_t* USBD_HID_GetFSCfgDesc(uint16_t *length);

static uint8_t* USBD_HID_GetHSCfgDesc(uint16_t *length);

static uint8_t* USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);

static uint8_t* USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
/**
 * @}
 */

/** @defgroup USBD_HID_Private_Variables
 * @{
 */


USBD_ClassTypeDef USBD_HID = { USBD_HID_Init, USBD_HID_DeInit, USBD_HID_Setup,
NULL, /*EP0_TxSent*/
NULL, /*EP0_RxReady*/
USBD_HID_DataIn, /*DataIn*/
NULL, /*DataOut*/
NULL, /*SOF */
NULL,
NULL, USBD_HID_GetHSCfgDesc, USBD_HID_GetFSCfgDesc,
		USBD_HID_GetOtherSpeedCfgDesc, USBD_HID_GetDeviceQualifierDesc, };

/* Standard Boot Protocol Keyboard Report Descriptor */
__ALIGN_BEGIN static const uint8_t HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END
= { 0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		0x09, 0x06,                    // USAGE (Keyboard)
		0xa1, 0x01,                    // COLLECTION (Application)
		0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
		0x19, 0xe0,            //   USAGE_MINIMUM (Keyboard LeftControl)
		0x29, 0xe7,              //   USAGE_MAXIMUM (Keyboard Right GUI)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
		0x75, 0x01,                    //   REPORT_SIZE (1)
		0x95, 0x08,                    //   REPORT_COUNT (8)
		0x81, 0x02,                    //   INPUT (Data,Var,Abs)
		0x95, 0x01,                    //   REPORT_COUNT (1)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
		0x95, 0x05,                    //   REPORT_COUNT (5)
		0x75, 0x01,                    //   REPORT_SIZE (1)
		0x05, 0x08,                    //   USAGE_PAGE (LEDs)
		0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
		0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
		0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
		0x95, 0x01,                    //   REPORT_COUNT (1)
		0x75, 0x03,                    //   REPORT_SIZE (3)
		0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
		0x95, 0x06,                    //   REPORT_COUNT (6)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
		0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
		0x19, 0x00,   //   USAGE_MINIMUM (Reserved (no event indicated))
		0x29, 0x65,            //   USAGE_MAXIMUM (Keyboard Application)
		0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
		0xc0                           // END_COLLECTION
		};

/* NKRO Keyboard Report Descriptor */
__ALIGN_BEGIN static const uint8_t HID_NKRO_ReportDesc[] __ALIGN_END = {
		0x05,
		0x01,     // Usage Page (Generic Desktop)
		0x09, 0x06,     // Usage (Keyboard)
		0xA1, 0x01,     // Collection (Application)

		// Modifiers (8 bits)
		0x05, 0x07,     // Usage Page (Keyboard/Keypad)
		0x19, 0xE0,     // Usage Minimum (Left Control)
		0x29, 0xE7,     // Usage Maximum (Right GUI)
		0x15, 0x00,     // Logical Minimum (0)
		0x25, 0x01,     // Logical Maximum (1)
		0x75, 0x01,     // Report Size (1)
		0x95, 0x08,     // Report Count (8)
		0x81, 0x02,     // Input (Data, Variable, Absolute)

		// Reserved (1 byte)
		0x75, 0x08,     // Report Size (8)
		0x95, 0x01,     // Report Count (1)
		0x81, 0x01,     // Input (Constant)

		// LED Output (5 bits + 3 padding)
		0x05, 0x08,     // Usage Page (LEDs)
		0x19, 0x01,     // Usage Minimum (Num Lock)
		0x29, 0x05,     // Usage Maximum (Kana)
		0x95, 0x05,     // Report Count (5)
		0x75, 0x01,     // Report Size (1)
		0x91, 0x02,     // Output (Data, Variable, Absolute)
		0x95, 0x01,     // Report Count (1)
		0x75, 0x03,     // Report Size (3)
		0x91, 0x01,     // Output (Constant)

		// NKRO Bitmap (80 keys = 10 bytes)
		0x05, 0x07,     // Usage Page (Keyboard/Keypad)
		0x19, 0x00,     // Usage Minimum (Reserved)
		0x29, 0x4F,     // Usage Maximum
		0x15, 0x00,     // Logical Minimum (0)
		0x25, 0x01,     // Logical Maximum (1)
		0x75, 0x01,     // Report Size (1)
		0x95, 0x50,     // Report Count
		0x81, 0x02,     // Input (Data, Variable, Absolute)

		0xC0            // End Collection
		};

__ALIGN_BEGIN static const uint8_t HID_CONSUMER_ReportDesc[HID_CONSUMER_REPORT_DESC_SIZE] __ALIGN_END = {
    0x05, 0x0C,                     // Usage Page (Consumer)
    0x09, 0x01,                     // Usage (Consumer Control)
    0xA1, 0x01,                     // Collection (Application)

    // Media keys (byte 0)
    0x15, 0x00,                     // Logical Minimum (0)
    0x25, 0x01,                     // Logical Maximum (1)
    0x75, 0x01,                     // Report Size (1)
    0x95, 0x08,                     // Report Count (8)

    0x05, 0x0C,                     // Usage Page (Consumer Devices)
    0x09, 0xB6,                     // Usage (Scan Previous Track)
    0x09, 0xB5,                     // Usage (Scan Next Track)
    0x09, 0xCD,                     // Usage (Play/Pause)
    0x09, 0xE2,                     // Usage (Mute)
    0x09, 0xEA,                     // Usage (Volume Down)
    0x09, 0xE9,                     // Usage (Volume Up)
    0x09, 0xB7,                     // Usage (Stop)
    0x09, 0xB0,                     // Usage (Play)
    0x81, 0x02,                     // Input (Data, Variable, Absolute)

    // Additional media controls (byte 1)
    0x05, 0x0C,                     // Usage Page (Consumer Devices)
    0x95, 0x08,                     // Report Count (8)
    0x09, 0xB1,                     // Usage (Pause)
    0x09, 0xB3,                     // Usage (Fast Forward)
    0x09, 0xB4,                     // Usage (Rewind)
    0x09, 0xB2,                     // Usage (Record)
    0x09, 0xE0,                     // Usage (Volume)
    0x09, 0xB8,                     // Usage (Eject)
    0x09, 0x89,                     // Usage (AL Word Processor)
    0x09, 0x8A,                     // Usage (AL Spreadsheet)
    0x81, 0x02,                     // Input (Data, Variable, Absolute)

    0xC0                            // End Collection
};

/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END
	= { 0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	USB_HID_CONFIG_DESC_SIZ, /* wTotalLength: Bytes returned */
	0x00,
	HID_NUM_INTERFACES, /* bNumInterfaces: 3 interfaces */
	0x01, /* bConfigurationValue: Configuration value */
	0x00, /* iConfiguration: Index of string descriptor */
	0xE0, /* bmAttributes: bus powered and Support Remote Wake-up */
	0xFA, /* MaxPower 500 mA: this current is used for detecting Vbus */

	/****************** Interface Descriptor for Boot Protocol Keyboard *****************/
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_KEYBOARD_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x01, /* bInterfaceSubClass: 1=BOOT, 0=no boot */
	0x01, /* nInterfaceProtocol: 0=none, 1=keyboard, 2=mouse */
	0x00, /* iInterface: Index of string descriptor */

	/******************** HID Descriptor for Boot Keyboard ********************/
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/******************** Endpoint Descriptor for Boot Keyboard ********************/
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_KEYBOARD_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_KEYBOARD_EPIN_SIZE, /* wMaxPacketSize: 8 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */

	/****************** Interface Descriptor for NKRO Keyboard *****************/
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_NKRO_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x00, /* bInterfaceSubClass: 0=no boot */
	0x00, /* nInterfaceProtocol: 0=none */
	0x00, /* iInterface: Index of string descriptor */

	/******************** HID Descriptor for NKRO Keyboard ********************/
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_NKRO_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/******************** Endpoint Descriptor for NKRO Keyboard ********************/
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_NKRO_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_NKRO_EPIN_SIZE, /* wMaxPacketSize: 20 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */

	/****************** Interface Descriptor for Consumer Controls *****************/
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_CONSUMER_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x00, /* bInterfaceSubClass: 0=no boot */
	0x00, /* nInterfaceProtocol: 0=none */
	0x00, /* iInterface: Index of string descriptor */

	/******************** HID Descriptor for Consumer Controls ********************/
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_CONSUMER_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/******************** Endpoint Descriptor for Consumer Controls ********************/
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_CONSUMER_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_CONSUMER_EPIN_SIZE, /* wMaxPacketSize: 2 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */
};

/* USB HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END
= { 0x09, /* bLength: Configuration Descriptor size */
USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
USB_HID_CONFIG_DESC_SIZ, /* wTotalLength: Bytes returned */
0x00,
HID_NUM_INTERFACES, /* bNumInterfaces: 3 interfaces */
0x01, /* bConfigurationValue: Configuration value */
0x00, /* iConfiguration: Index of string descriptor */
0xE0, /* bmAttributes: bus powered and Support Remote Wake-up */
0xFA, /* MaxPower 500 mA: this current is used for detecting Vbus */

/* Boot Protocol Keyboard Interface Descriptor */
0x09, /* bLength: Interface Descriptor size */
USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
HID_KEYBOARD_INTERFACE, /* bInterfaceNumber: Number of Interface */
0x00, /* bAlternateSetting: Alternate setting */
0x01, /* bNumEndpoints: 1 endpoint */
0x03, /* bInterfaceClass: HID */
0x01, /* bInterfaceSubClass: 1=BOOT, 0=no boot */
0x01, /* nInterfaceProtocol: 0=none, 1=keyboard, 2=mouse */
0x00, /* iInterface: Index of string descriptor */

/* HID Descriptor */
0x09, /* bLength: HID Descriptor size */
HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
0x11, /* bcdHID: HID Class Spec release number */
0x01, 0x0D, /* bCountryCode: Hardware target country */
0x01, /* bNumDescriptors: Number of HID class descriptors */
0x22, /* bDescriptorType */
HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
0x00,

/* Endpoint Descriptor */
0x07, /* bLength: Endpoint Descriptor size */
USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
HID_KEYBOARD_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
0x03, /* bmAttributes: Interrupt endpoint */
HID_KEYBOARD_EPIN_SIZE, /* wMaxPacketSize: 8 bytes */
0x00,
HID_HS_BINTERVAL, /* bInterval: Polling Interval */

/* NKRO Keyboard Interface Descriptor */
0x09, /* bLength: Interface Descriptor size */
USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
HID_NKRO_INTERFACE, /* bInterfaceNumber: Number of Interface */
0x00, /* bAlternateSetting: Alternate setting */
0x01, /* bNumEndpoints: 1 endpoint */
0x03, /* bInterfaceClass: HID */
0x00, /* bInterfaceSubClass: 0=no boot */
0x00, /* nInterfaceProtocol: 0=none */
0x00, /* iInterface: Index of string descriptor */

/* HID Descriptor */
0x09, /* bLength: HID Descriptor size */
HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
0x11, /* bcdHID: HID Class Spec release number */
0x01, 0x0D, /* bCountryCode: Hardware target country */
0x01, /* bNumDescriptors: Number of HID class descriptors */
0x22, /* bDescriptorType */
HID_NKRO_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
0x00,

/* Endpoint Descriptor */
0x07, /* bLength: Endpoint Descriptor size */
USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
HID_NKRO_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
0x03, /* bmAttributes: Interrupt endpoint */
HID_NKRO_EPIN_SIZE, /* wMaxPacketSize: 20 bytes */
0x00,
HID_HS_BINTERVAL, /* bInterval: Polling Interval */

/* Consumer Controls Interface Descriptor */
0x09, /* bLength: Interface Descriptor size */
USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
HID_CONSUMER_INTERFACE, /* bInterfaceNumber: Number of Interface */
0x00, /* bAlternateSetting: Alternate setting */
0x01, /* bNumEndpoints: 1 endpoint */
0x03, /* bInterfaceClass: HID */
0x00, /* bInterfaceSubClass: 0=no boot */
0x00, /* nInterfaceProtocol: 0=none */
0x00, /* iInterface: Index of string descriptor */

/* HID Descriptor */
0x09, /* bLength: HID Descriptor size */
HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
0x11, /* bcdHID: HID Class Spec release number */
0x01, 0x0D, /* bCountryCode: Hardware target country */
0x01, /* bNumDescriptors: Number of HID class descriptors */
0x22, /* bDescriptorType */
HID_CONSUMER_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
0x00,

/* Endpoint Descriptor */
0x07, /* bLength: Endpoint Descriptor size */
USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
HID_CONSUMER_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
0x03, /* bmAttributes: Interrupt endpoint */
HID_CONSUMER_EPIN_SIZE, /* wMaxPacketSize: 2 bytes */
0x00,
HID_HS_BINTERVAL, /* bInterval: Polling Interval */
};

/* USB HID device Other Speed Configuration Descriptor - same as FS */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END
	= { 0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	USB_HID_CONFIG_DESC_SIZ, /* wTotalLength: Bytes returned */
	0x00,
	HID_NUM_INTERFACES, /* bNumInterfaces: 3 interfaces */
	0x01, /* bConfigurationValue: Configuration value */
	0x00, /* iConfiguration: Index of string descriptor */
	0xE0, /* bmAttributes: bus powered and Support Remote Wake-up */
	0xFA, /* MaxPower 500 mA: this current is used for detecting Vbus */

	/* Boot Protocol Keyboard Interface Descriptor */
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_KEYBOARD_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x01, /* bInterfaceSubClass: 1=BOOT, 0=no boot */
	0x01, /* nInterfaceProtocol: 0=none, 1=keyboard, 2=mouse */
	0x00, /* iInterface: Index of string descriptor */

	/* HID Descriptor */
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/* Endpoint Descriptor */
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_KEYBOARD_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_KEYBOARD_EPIN_SIZE, /* wMaxPacketSize: 8 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */

	/* NKRO Keyboard Interface Descriptor */
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_NKRO_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x00, /* bInterfaceSubClass: 0=no boot */
	0x00, /* nInterfaceProtocol: 0=none */
	0x00, /* iInterface: Index of string descriptor */

	/* HID Descriptor */
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_NKRO_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/* Endpoint Descriptor */
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_NKRO_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_NKRO_EPIN_SIZE, /* wMaxPacketSize: 20 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */

	/* Consumer Controls Interface Descriptor */
	0x09, /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	HID_CONSUMER_INTERFACE, /* bInterfaceNumber: Number of Interface */
	0x00, /* bAlternateSetting: Alternate setting */
	0x01, /* bNumEndpoints: 1 endpoint */
	0x03, /* bInterfaceClass: HID */
	0x00, /* bInterfaceSubClass: 0=no boot */
	0x00, /* nInterfaceProtocol: 0=none */
	0x00, /* iInterface: Index of string descriptor */

	/* HID Descriptor */
	0x09, /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
	0x11, /* bcdHID: HID Class Spec release number */
	0x01, 0x0D, /* bCountryCode: Hardware target country */
	0x01, /* bNumDescriptors: Number of HID class descriptors */
	0x22, /* bDescriptorType */
	HID_CONSUMER_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
	0x00,

	/* Endpoint Descriptor */
	0x07, /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	HID_CONSUMER_EPIN_ADDR, /* bEndpointAddress: Endpoint Address */
	0x03, /* bmAttributes: Interrupt endpoint */
	HID_CONSUMER_EPIN_SIZE, /* wMaxPacketSize: 2 bytes */
	0x00,
	HID_FS_BINTERVAL, /* bInterval: Polling Interval */
	};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_KeyboardDesc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                    // bLength: HID Descriptor size
    HID_DESCRIPTOR_TYPE,     // bDescriptorType: HID
    0x11, 0x01,              // bcdHID: HID Class Spec release number
    0x0D,                    // bCountryCode: Hardware target country
    0x01,                    // bNumDescriptors: Number of HID class descriptors
    0x22,                    // bDescriptorType
    HID_KEYBOARD_REPORT_DESC_SIZE, 0x00,  // wItemLength: Total length of Report descriptor
};

__ALIGN_BEGIN static uint8_t USBD_HID_NKRODesc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                    // bLength: HID Descriptor size
    HID_DESCRIPTOR_TYPE,     // bDescriptorType: HID
    0x11, 0x01,              // bcdHID: HID Class Spec release number
    0x0D,                    // bCountryCode: Hardware target country
    0x01,                    // bNumDescriptors: Number of HID class descriptors
    0x22,                    // bDescriptorType
    HID_NKRO_REPORT_DESC_SIZE, 0x00,  // wItemLength: Total length of Report descriptor
};

__ALIGN_BEGIN static uint8_t USBD_HID_ConsumerDesc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                    // bLength: HID Descriptor size
    HID_DESCRIPTOR_TYPE,     // bDescriptorType: HID
    0x11, 0x01,              // bcdHID: HID Class Spec release number
    0x0D,                    // bCountryCode: Hardware target country
    0x01,                    // bNumDescriptors: Number of HID class descriptors
    0x22,                    // bDescriptorType
    HID_CONSUMER_REPORT_DESC_SIZE, 0x00,  // wItemLength: Total length of Report descriptor
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END
		= {
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
				0x01, 0x00, };

/**
 * @}
 */

/** @defgroup USBD_HID_Private_Functions
 * @{
 */

/**
 * @brief  USBD_HID_Init
 *         Initialize the HID interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	/* Open EP IN for Keyboard */
	/* Open EP IN for Consumer Controls */
	USBD_LL_OpenEP(pdev, HID_CONSUMER_EPIN_ADDR, USBD_EP_TYPE_INTR,
	HID_CONSUMER_EPIN_SIZE);
	pdev->ep_in[HID_CONSUMER_EPIN_ADDR & 0xFU].is_used = 1U;

	USBD_LL_OpenEP(pdev, HID_KEYBOARD_EPIN_ADDR, USBD_EP_TYPE_INTR,
	HID_KEYBOARD_EPIN_SIZE);
	pdev->ep_in[HID_KEYBOARD_EPIN_ADDR & 0xFU].is_used = 1U;

	/* Open EP IN for NKRO */
	USBD_LL_OpenEP(pdev, HID_NKRO_EPIN_ADDR, USBD_EP_TYPE_INTR,
	HID_NKRO_EPIN_SIZE);
	pdev->ep_in[HID_NKRO_EPIN_ADDR & 0xFU].is_used = 1U;

	pdev->pClassData = USBD_malloc (sizeof(USBD_HID_HandleTypeDef));

	if (pdev->pClassData == NULL) {
		return USBD_FAIL;
	}

	((USBD_HID_HandleTypeDef*) pdev->pClassData)->state = HID_IDLE;

	return USBD_OK;
}

/**
 * @brief  USBD_HID_DeInit
 *         DeInitialize the HID layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	/* Close Keyboard HID EP */
	USBD_LL_CloseEP(pdev, HID_KEYBOARD_EPIN_ADDR);
	pdev->ep_in[HID_KEYBOARD_EPIN_ADDR & 0xFU].is_used = 0U;

	/* Close NKRO HID EP */
	USBD_LL_CloseEP(pdev, HID_NKRO_EPIN_ADDR);
	pdev->ep_in[HID_NKRO_EPIN_ADDR & 0xFU].is_used = 0U;

	/* Close Consumer HID EP */
	USBD_LL_CloseEP(pdev, HID_CONSUMER_EPIN_ADDR);
	pdev->ep_in[HID_CONSUMER_EPIN_ADDR & 0xFU].is_used = 0U;

	/* Free allocated memory */
	if (pdev->pClassData != NULL) {
		USBD_free(pdev->pClassData);
		pdev->pClassData = NULL;
	}

	return USBD_OK;
}


static USBD_StatusTypeDef USBD_HID_SendReportDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  const uint8_t *pbuf = NULL;
  uint16_t len = 0;

  switch (req->wIndex) {
    case HID_KEYBOARD_INTERFACE:
      len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
      pbuf = HID_KEYBOARD_ReportDesc;
      break;
    case HID_NKRO_INTERFACE:
      len = MIN(HID_NKRO_REPORT_DESC_SIZE, req->wLength);
      pbuf = HID_NKRO_ReportDesc;
      break;
    case HID_CONSUMER_INTERFACE:
      len = MIN(HID_CONSUMER_REPORT_DESC_SIZE, req->wLength);
      pbuf = HID_CONSUMER_ReportDesc;
      break;
    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
  }

  USBD_CtlSendData(pdev, (uint8_t *)pbuf, len);
  return USBD_OK;
}

// --- Function to handle GET_DESCRIPTOR requests for HID Descriptors ---
static USBD_StatusTypeDef USBD_HID_SendHIDDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  const uint8_t *pbuf = NULL;
  uint16_t len = 0;

  switch (req->wIndex) {
    case HID_KEYBOARD_INTERFACE:
      pbuf = USBD_HID_KeyboardDesc;
      break;
    case HID_NKRO_INTERFACE:
      pbuf = USBD_HID_NKRODesc;
      break;
    case HID_CONSUMER_INTERFACE:
      pbuf = USBD_HID_ConsumerDesc;
      break;
    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
  }

  len = MIN(USB_HID_DESC_SIZ, req->wLength);
  USBD_CtlSendData(pdev, (uint8_t *)pbuf, len);
  return USBD_OK;
}

// --- Function to handle HID class requests per interface ---
static USBD_StatusTypeDef USBD_HID_HandleClassReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData;

  switch (req->bRequest) {
    case HID_REQ_SET_PROTOCOL:
      if (req->wIndex == HID_KEYBOARD_INTERFACE) {
        hhid->Protocol = (uint8_t)(req->wValue);
      }
      return USBD_OK;

    case HID_REQ_GET_PROTOCOL:
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1);
      return USBD_OK;

    case HID_REQ_SET_IDLE:
      if (req->wIndex == HID_KEYBOARD_INTERFACE) {
        hhid->IdleState = (uint8_t)(req->wValue >> 8);
      }
      return USBD_OK;

    case HID_REQ_GET_IDLE:
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1);
      return USBD_OK;

    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
  }
}

/**
 * @brief  USBD_HID_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      // Use the dedicated function for class requests
      ret = USBD_HID_HandleClassReq(pdev, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            ret = USBD_HID_SendReportDescriptor(pdev, req);
          }
          else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            ret = USBD_HID_SendHIDDescriptor(pdev, req);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          USBD_CtlSendData(pdev, (uint8_t *)&((USBD_HID_HandleTypeDef *)pdev->pClassData)->AltSetting, 1);
          break;

        case USB_REQ_SET_INTERFACE:
          ((USBD_HID_HandleTypeDef *)pdev->pClassData)->AltSetting = (uint8_t)(req->wValue);
          USBD_CtlSendStatus(pdev);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
 * @brief  USBD_HID_SendKeyboardReport
 *         Send HID Keyboard Report
 * @param  pdev: device instance
 * @param  report: pointer to report
 * @param  len: report length
 * @retval status
 */
uint8_t USBD_HID_SendKeyboardReport(USBD_HandleTypeDef *pdev, uint8_t *report,
		uint16_t len)
{
	USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (hhid->state == HID_IDLE) {
			hhid->state = HID_BUSY;
			USBD_LL_Transmit(pdev, HID_KEYBOARD_EPIN_ADDR, report, len);
			return USBD_OK;
		}
	}
	return USBD_BUSY;

}

/**
 * @brief  USBD_HID_SendNKROReport
 *         Send HID NKRO Keyboard Report
 * @param  pdev: device instance
 * @param  report: pointer to report
 * @param  len: report length
 * @retval status
 */
uint8_t USBD_HID_SendNKROReport(USBD_HandleTypeDef *pdev, uint8_t *report,
		uint16_t len)
{
	USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (hhid->state == HID_IDLE) {
			hhid->state = HID_BUSY;
			USBD_LL_Transmit(pdev, HID_NKRO_EPIN_ADDR, report, len);
			return USBD_OK;
		}
	}
	return USBD_BUSY;;
}

/**
 * @brief  USBD_HID_SendConsumerReport
 *         Send HID Consumer Report
 * @param  pdev: device instance
 * @param  report: pointer to report
 * @param  len: report length
 * @retval status
 */
uint8_t USBD_HID_SendConsumerReport(USBD_HandleTypeDef *pdev, uint8_t *report,
		uint16_t len)
{
	USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (hhid->state == HID_IDLE) {
			hhid->state = HID_BUSY;
			USBD_LL_Transmit(pdev, HID_CONSUMER_EPIN_ADDR, report, len);
			return USBD_OK;
		}
	}
	return USBD_BUSY;
}

/**
 * @brief  USBD_HID_GetPollingInterval
 *         return polling interval from endpoint descriptor
 * @param  pdev: device instance
 * @retval polling interval
 */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
	uint32_t polling_interval = 0U;

	/* HIGH-speed endpoints */
	if (pdev->dev_speed == USBD_SPEED_HIGH) {
		/* Sets the data transfer polling interval for high speed transfers.
		 Values between 1..16 are allowed. Values correspond to interval
		 of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
		polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
	}
	else /* LOW and FULL-speed endpoints */
	{
		/* Sets the data transfer polling interval for low and full
		 speed transfers */
		polling_interval = HID_FS_BINTERVAL;
	}

	return ((uint32_t) (polling_interval));
}

/**
 * @brief  USBD_HID_GetCfgFSDesc
 *         return FS configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetFSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_HID_CfgFSDesc);
	return USBD_HID_CfgFSDesc;
}

/**
 * @brief  USBD_HID_GetCfgHSDesc
 *         return HS configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetHSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_HID_CfgHSDesc);
	return USBD_HID_CfgHSDesc;
}

/**
 * @brief  USBD_HID_GetOtherSpeedCfgDesc
 *         return other speed configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_HID_OtherSpeedCfgDesc);
	return USBD_HID_OtherSpeedCfgDesc;
}

/**
 * @brief  USBD_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	/* Ensure that the FIFO is empty before a new transfer, this condition could
	 be caused by a new transfer before the end of the previous transfer */
	((USBD_HID_HandleTypeDef*) pdev->pClassData)->state = HID_IDLE;
	return USBD_OK;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
	*length = sizeof(USBD_HID_DeviceQualifierDesc);
	return USBD_HID_DeviceQualifierDesc;
}
