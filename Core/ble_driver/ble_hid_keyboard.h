/*
 * ble_hid_keyboard.h
 *
 *  Created on: Jan 9, 2025
 *      Author: bettysidepiece
 */

#ifndef BLE_DRIVER_BLE_HID_KEYBOARD_H_
#define BLE_DRIVER_BLE_HID_KEYBOARD_H_

#include "../keyboard_driver/keys.h"

//Service UUID
#define HID_SERVICE_UUID "1812"
#define BATTERY_SERVICE_UUID		"180F"

// HID Service Characteristics UUIDs
#define HID_PROTOCOL_MODE_UUID      "2A4E"
#define HID_REPORT_UUID             "2A4D"
#define HID_REPORT_MAP_UUID         "2A4B"
#define HID_REPORT_REFERENCE 		"2908"
#define HID_REPORT_CCCD				"2902"

#define HID_BOOT_KEYBOARD_IN_UUID   "2A22"
#define HID_BOOT_KEYBOARD_OUT_UUID  "2A32"
#define HID_BOOT_KEYBOARD_CCCD_UUID	"2902"

#define HID_INFO_UUID               "2A4A"
#define HID_CONTROL_POINT_UUID      "2A4C"

#define BATT_LEVEL_UUID	"2A19"

// Protocol Mode values
typedef enum {
    PROTOCOL_MODE_BOOT = 0,
    PROTOCOL_MODE_REPORT
} gatt_hid_protocol_t;

// Report Types
typedef enum {
    REPORT_TYPE_INPUT = 0x01,
    REPORT_TYPE_OUTPUT = 0x02,
    REPORT_TYPE_FEATURE = 0x03
} gatt_hid_report_type_t;

// Boot Protocol Report sizes
typedef enum {
    BOOT_KEYBOARD_INPUT_REPORT_SIZE = 8,  // 1 modifier + 1 reserved + 6 keys
    BOOT_KEYBOARD_OUTPUT_REPORT_SIZE = 1  // LED states
} gatt_hid_boot_report_size_t;


#endif /* BLE_DRIVER_BLE_HID_KEYBOARD_H_ */
