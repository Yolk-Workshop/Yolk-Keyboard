/*
 * rnbd350.h
 *
 *  Created on: Dec 21, 2024
 *      Author: bettysidepiece
 */

#ifndef INC_RNBD350_H_
#define INC_RNBD350_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

#define BLE_SDA_HID 	960U // Generic HID Keyboard
#define SW_VERSION 		"0.0.5"
#define HW_VERSION 		"0.1.0"
#define PRODUCT_NAME 	"Yolk Keyboard"
#define VENDOR_NAME		"Yolk Workshop"


#define RNBD350_CMD_TIMEOUT_MS 1000
#define RNBD350_MAX_RESPONSE_SIZE 256
#define RNBD350_MAX_NAME_LEN   20
#define RNBD350_MAX_SERIAL_NAME_LEN  15

// Advertising intervals (in ms)
typedef enum {
    ADV_INTERVAL_FASTEST = 20,    // 20ms - Fastest discovery, highest power
    ADV_INTERVAL_FAST = 100,      // 100ms - Good balance for Windows/Linux
    ADV_INTERVAL_BALANCED = 300,  // 300ms - Good general purpose
    ADV_INTERVAL_SLOW = 500,      // 500ms - Power saving
    ADV_INTERVAL_SLOWEST = 1000   // 1000ms - Maximum power saving
} rnbd350_adv_interval_t;

// Advertising timeout periods (in ms)
typedef enum {
    ADV_TIMEOUT_NONE = 0,
    ADV_TIMEOUT_30SEC = 30000,
    ADV_TIMEOUT_1MIN = 60000
} rnbd350_adv_timeout_t;

/**
 * @brief Advanced advertising configuration
 */
typedef struct {
    uint16_t fast_interval;   // Fast advertising interval (ms)
    uint16_t slow_interval;   // Slow advertising interval (ms)
    uint16_t fast_timeout;    // Time to spend in fast advertising (ms)
} rnbd350_adv_config_t;

// Status/Return codes
typedef enum {
    RNBD350_OK = 0,
    RNBD350_ERROR,
    RNBD350_TIMEOUT,
    RNBD350_INVALID_PARAM
} rnbd350_status_t;

// Module operating modes
typedef enum {
    RNBD350_MODE_DATA,      // Default mode for data transfer
    RNBD350_MODE_COMMAND,   // Command mode for configuration
    RNBD350_MODE_DFU        // Device Firmware Update mode
} rnbd350_mode_t;

// Configuration structure
typedef struct {
    uint32_t baud_rate;     // UART baud rate
    bool flow_control;      // Hardware flow control enable/disable
    // Add other config parameters as needed
} rnbd350_config_t;

typedef struct {
   bool name_set;          // Track if name has been set
   bool device_info_set;   // Track if device info has been set
} rnbd350_config_state_t;

typedef struct {
    uint8_t bt_address[6];  // Peer Bluetooth address
    uint8_t address_type;   // 0 = public, 1 = random
    uint8_t conn_type;      // 0 = UART transparent disabled, 1 = enabled
} rnbd350_connection_info_t;

typedef struct {
    uint8_t mac_address[6];         // Device MAC address
    char device_name[32];           // Device name
    uint8_t connected_mac[6];       // Connected device MAC
    uint8_t connected_addr_type;    // Connected device address type
    bool is_connected;              // Connection status
    uint8_t auth_method;           // Authentication method (SA command value)
    uint16_t device_features;       // Device features (SR command value)
    uint8_t server_services;        // Server services bitmap (SS command value)
    char pin_code[8];              // Fixed pin code if using
} rnbd350_device_info_t;

// Device handle structure
typedef struct {
   rnbd350_config_t config;
   rnbd350_mode_t mode;
   rnbd350_connection_info_t conn_info;
   rnbd350_device_info_t device_info;
   rnbd350_config_state_t state;
   void (*uart_write)(uint8_t* data, uint16_t len);
   uint16_t (*uart_read)(uint8_t* data, uint16_t len);
   void (*delay_ms)(uint32_t ms);
} rnbd350_handle_t;

/**
 * @brief Get device information
 * @param handle Pointer to device handle
 * @param info Pointer to device info structure
 * @return Status code
 */

rnbd350_status_t rnbd350_set_serialized_name(rnbd350_handle_t* handle, const char* name);

rnbd350_status_t rnbd350_get_device_info(rnbd350_handle_t* handle);

rnbd350_status_t rnbd350_get_connection_status(rnbd350_handle_t* handle);

rnbd350_status_t rnbd350_init_hid_keyboard(rnbd350_handle_t* handle);

void uart_write_wrapper(uint8_t* data, uint16_t len);

uint16_t uart_read_wrapper(uint8_t* data, uint16_t len);

rnbd350_status_t rnbd350_reset_module(rnbd350_handle_t* handle);
/**
 * @brief Initialize RNBD350 module
 * @param handle Pointer to device handle
 * @param config Pointer to configuration structure
 * @return Status code
 */
rnbd350_status_t rnbd350_init(rnbd350_handle_t* handle, const rnbd350_config_t* config);

/**
 * @brief Reset RNBD350 module
 * @param handle Pointer to device handle
 * @return Status code
 */
rnbd350_status_t rnbd350_reset(rnbd350_handle_t* handle);

/**
 * @brief Enter command mode
 * @param handle Pointer to device handle
 * @return Status code
 */
rnbd350_status_t rnbd350_enter_command_mode(rnbd350_handle_t* handle);

/**
 * @brief Exit command mode
 * @param handle Pointer to device handle
 * @return Status code
 */
rnbd350_status_t rnbd350_exit_command_mode(rnbd350_handle_t* handle);

/**
 * @brief Start advertising
 * @param handle Pointer to device handle
 * @param interval_ms Advertisement interval in milliseconds
 * @param timeout_ms Advertisement timeout in milliseconds (0 for no timeout)
 * @return Status code
 */
rnbd350_status_t rnbd350_start_advertising(rnbd350_handle_t* handle,
                                         uint16_t interval_ms,
                                         uint16_t timeout_ms);

/**
 * @brief Start advanced advertising pattern
 * @param handle Pointer to device handle
 * @param config Advertising configuration
 * @return Status code
 */
rnbd350_status_t rnbd350_start_advanced_advertising(rnbd350_handle_t* handle,
                                                   const rnbd350_adv_config_t* config);

/**
 * @brief Stop advertising
 * @param handle Pointer to device handle
 * @return Status code
 */
rnbd350_status_t rnbd350_stop_advertising(rnbd350_handle_t* handle);

/**
 * @brief Set device name
 * @param handle Pointer to device handle
 * @param name New device name
 * @return Status code
 */
rnbd350_status_t rnbd350_set_name(rnbd350_handle_t* handle, const char* name);

/**
 * @brief Send data in transparent UART mode
 * @param handle Pointer to device handle
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return Status code
 */
rnbd350_status_t rnbd350_send_data(rnbd350_handle_t* handle,
                                  const uint8_t* data,
                                  uint16_t len);

/**
 * @brief Receive data in transparent UART mode
 * @param handle Pointer to device handle
 * @param data Pointer to data buffer
 * @param len Maximum length to receive
 * @param received Actual number of bytes received
 * @return Status code
 */
rnbd350_status_t rnbd350_receive_data(rnbd350_handle_t* handle,
                                     uint8_t* data,
                                     uint16_t len,
                                     uint16_t* received);

#endif /* INC_RNBD350_H_ */
