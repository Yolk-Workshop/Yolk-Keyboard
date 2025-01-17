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
#define DEVICE_INFO_UUID			"180A"
#define HID_SERVICE_UUID 			"1812"
#define BATTERY_SERVICE_UUID		"180F"
#define SCAN_PARAMS_SERVICE_UUID    "1813"

// HID Characteristic UUIDs
#define HID_PROTOCOL_MODE_UUID     	"2A4E"
#define HID_REPORT_UUID             "2A4D"
#define HID_REPORT_MAP_UUID         "2A4B"
#define HID_INFO_UUID               "2A4A"
#define HID_CONTROL_POINT_UUID      "2A4C"
#define HID_BOOT_KEYBOARD_IN_UUID   "2A22"
#define HID_BOOT_KEYBOARD_OUT_UUID  "2A32"
#define HID_BOOT_MOUSE_IN_UUID      "2A33"

// Other Characteristic UUIDs
#define BATTERY_LEVEL_UUID          "2A19"
#define PNP_ID_UUID                 "2A50"

// Descriptor UUIDs
#define CLIENT_CHARACTERISTIC_CONFIG_UUID 	"2902"
#define REPORT_REFERENCE_UUID            	"2908"
#define EXTERNAL_REPORT_REFERENCE_UUID   	"2907"


// Connection Parameters (as per HOGP spec)
#define CONN_INTERVAL_MIN              0x0006  // 7.5ms
#define CONN_INTERVAL_MAX             0x0C80  // 4s
#define CONN_LATENCY_MAX              30
#define SUPERVISION_TIMEOUT_MIN        0x000A  // 100ms
#define SUPERVISION_TIMEOUT_MAX        0x0C80  // 32s

// Size definitions
#define REPORT_MAP_MAX_SIZE            512
#define HID_REPORT_MAX_SIZE            64
#define BOOT_MOUSE_INPUT_REPORT_SIZE    6

// Feature Flags
#define HID_INFO_FLAG_REMOTE_WAKE      0x01
#define HID_INFO_FLAG_NORMALLY_CONNECTABLE 0x02

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

// HID Control Point Commands
typedef enum {
    HID_CONTROL_POINT_SUSPEND           = 0x00,
    HID_CONTROL_POINT_EXIT_SUSPEND      = 0x01
} hid_control_point_cmd_t;

// Security Levels (as per HOGP spec)
typedef enum {
    SECURITY_MODE_1_LEVEL_1             = 0x10, // No security
    SECURITY_MODE_1_LEVEL_2             = 0x11, // Unauthenticated pairing with encryption
    SECURITY_MODE_1_LEVEL_3             = 0x12, // Authenticated pairing with encryption
    SECURITY_MODE_1_LEVEL_4             = 0x13  // Authenticated LE Secure Connections pairing with encryption
} security_level_t;


// Report Reference Descriptor Values
typedef struct {
    uint8_t report_id;
    uint8_t report_type;
} report_reference_t;

#endif /* BLE_DRIVER_BLE_HID_KEYBOARD_H_ */
