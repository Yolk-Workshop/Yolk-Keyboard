#ifndef BLE_DRIVER_CORE_BM7X_ASYNC_H_
#define BLE_DRIVER_CORE_BM7X_ASYNC_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32l0xx_hal.h"

/* ============================================================================
 * BM70/71 BLEDK3 Constants and Definitions
 * ============================================================================ */

#define BM70_MAX_PACKET_SIZE            255     // Maximum UART packet size
#define BM70_SYNC_WORD                  0xAA    // UART packet sync byte
#define BM70_RESPONSE_TIMEOUT           2000    // Command response timeout (ms)
#define BM70_MAX_DEVICE_NAME_LEN        32      // Maximum device name length
#define BM70_BT_ADDR_LEN                6       // Bluetooth address length

/* ============================================================================
 * BLEDK3 Command Opcodes (from BLEDK3 Command Set v2.04)
 * ============================================================================ */

/* Common_1 Commands */
#define BM70_CMD_READ_LOCAL_INFO        0x01    // Read local information
#define BM70_CMD_RESET                  0x02    // Reset module
#define BM70_CMD_READ_STATUS            0x03    // Read module status
#define BM70_CMD_READ_ADC               0x04    // Read ADC value
#define BM70_CMD_INTO_SHUTDOWN          0x05    // Enter shutdown mode
#define BM70_CMD_READ_DEVICE_NAME       0x07    // Read device name
#define BM70_CMD_WRITE_DEVICE_NAME      0x08    // Write device name
#define BM70_CMD_ERASE_PAIRED_LIST      0x09    // Erase all paired devices
#define BM70_CMD_READ_PAIRING_MODE      0x0A    // Read pairing mode setting
#define BM70_CMD_WRITE_PAIRING_MODE     0x0B    // Write pairing mode setting
#define BM70_CMD_READ_PAIRED_LIST       0x0C    // Read all paired devices
#define BM70_CMD_DELETE_PAIRED_DEVICE   0x0D    // Delete paired device

/* GAP Commands */
#define BM70_CMD_READ_RSSI              0x10    // Read RSSI value
#define BM70_CMD_SET_ADV_ENABLE         0x1C    // Set advertising enable
#define BM70_CMD_SET_ADV_PARAMETER      0x13    // Set advertising parameters
#define BM70_CMD_DISCONNECT             0x1B    // Disconnect
// Advertising intervals (in 0.625ms units)
#define BM70_ADV_INTERVAL_100MS         0x00A0  // 100ms (160 * 0.625ms)
// Advertising types
#define BM70_ADV_TYPE_UNDIRECTED        0x00    // Connectable undirected advertising
#define BM70_ADV_TYPE_DIRECTED          0x01    // Connectable directed advertising
// Address types
#define BM70_ADDR_TYPE_PUBLIC           0x00    // Public device address
#define BM70_ADDR_TYPE_RANDOM           0x01    // Random device address

/* GATT Server Commands */
#define BM70_CMD_SEND_CHAR_VALUE        0x38    // Send characteristic value
#define BM70_CMD_UPDATE_CHAR_VALUE      0x39    // Update characteristic value
#define BM70_CMD_DISCOVER_CHAR_SVR		0x3A
#define BM70_CMD_READ_LOCAL_ALL_SERVICES 0x3B   // Read local all primary services
#define BM70_CMD_READ_LOCAL_SPECIFIC_SERVICE 0x3C // Read local specific primary service

/* GATT Client Commands */
#define BM70_CMD_DISCOVER_CHAR          0x31    // Discover specific primary service characteristics
#define BM70_CMD_READ_CHAR_VALUE        0x32    // Read characteristic value
#define BM70_CMD_WRITE_CHAR_VALUE       0x34    // Write characteristic value

/* Pairing Commands */
#define BM70_CMD_PASSKEY_ENTRY_RSP      0x40    // Passkey entry response
#define BM70_CMD_USER_CONFIRM_RSP       0x41    // User confirm response
#define BM70_CMD_PAIRING_REQUEST        0x42    // Pairing request

/* Common_2 Commands */
#define BM70_CMD_LEAVE_CONFIGURE_MODE   0x52    // Leave configure mode

/* ============================================================================
 * BLEDK3 Event Opcodes (from BLEDK3 Command Set v2.04)
 * ============================================================================ */

/* Pairing Events */
#define BM70_EVT_PASSKEY_ENTRY_REQ      0x60    // Passkey entry request
#define BM70_EVT_PAIRING_COMPLETE       0x61    // Pairing complete
#define BM70_EVT_PASSKEY_CONFIRM_REQ    0x62    // Passkey confirm request

/* GAP Events */
#define BM70_EVT_LE_CONNECTION_COMPLETE 0x71    // LE connection complete
#define BM70_EVT_DISCONNECT_COMPLETE    0x72    // Disconnection complete
#define BM70_EVT_CONNECTION_UPDATE      0x73    // Connection parameter update notify

/* Common Events */
#define BM70_EVT_COMMAND_COMPLETE       0x80    // Command complete
#define BM70_EVT_STATUS_REPORT          0x81    // Status report
#define BM70_EVT_ATT_MTU_UPDATE         0xA4    // ATT MTU update

/* GATT Client Events */
#define BM70_EVT_DISCOVER_ALL_SERVICES_RSP   0x90 // Discover all primary services response
#define BM70_EVT_DISCOVER_CHAR_RSP      0x91    // Discover specific primary service characteristic response
#define BM70_EVT_DISCOVER_DESC_RSP      0x92    // Discover all characteristic descriptors response
#define BM70_EVT_CHAR_VALUE_RECEIVED    0x93    // Characteristic value received

/* GATT Server Events */
#define BM70_EVT_CLIENT_WRITE_CHAR      0x98    // Client write characteristic value

/* GATT Transparent Events */
#define BM70_EVT_TRANSPARENT_DATA       0x9A    // Received transparent data

/* ============================================================================
 * HID Service Configuration
 * ============================================================================ */

// Maximum number of services and characteristics to store
#define BM70_MAX_PAIRED_DEVICES         4
#define BM70_MAX_SERVICES           5
#define BM70_MAX_CHARACTERISTICS    20

/* Standard Bluetooth Service UUIDs */
#define BT_SVC_GENERIC_ACCESS           0x1800  // Generic Access Service
#define BT_SVC_GENERIC_ATTRIBUTE        0x1801  // Generic Attribute Service
#define BT_SVC_DEVICE_INFORMATION       0x180A  // Device Information Service
#define BT_SVC_BATTERY                  0x180F  // Battery Service
#define BT_SVC_HID                      0x1812  // HID Service

/* HID Characteristic UUIDs */
#define BT_CHAR_HID_INFORMATION         0x2A4A  // HID Information
#define BT_CHAR_REPORT_MAP              0x2A4B  // Report Map
#define BT_CHAR_HID_CONTROL_POINT       0x2A4C  // HID Control Point
#define BT_CHAR_REPORT                  0x2A4D  // Report
#define BT_CHAR_PROTOCOL_MODE           0x2A4E  // Protocol Mode
#define BT_CHAR_BATTERY_LEVEL           0x2A19  // Battery Level

/* HID Characteristic Handles */
#define HID_KB_INPUT_HANDLE                0x006D
#define HID_KB_INPUT_CCCD_HANDLE           0x006F
#define HID_CONSUMER_INPUT_HANDLE          0x0076
#define HID_CONSUMER_INPUT_CCCD_HANDLE     0x0078
#define BATTERY_LEVEL_HANDLE               0x007B
#define BATTERY_LEVEL_CCCD                 0x007C

/* Additional HID Handles */
#define HID_REPORT_MAP_HANDLE              0x0065
#define HID_INFORMATION_HANDLE             0x0067
#define HID_CONTROL_POINT_HANDLE           0x0069
#define HID_PROTOCOL_MODE_HANDLE           0x006B
#define HID_BOOT_INPUT_HANDLE              0x0071
#define HID_BOOT_INPUT_CCCD_HANDLE         0x0072
#define HID_BOOT_OUTPUT_HANDLE             0x0074

/* Characteristic Properties */
#define CHAR_PROP_READ                  0x02
#define CHAR_PROP_WRITE                 0x08
#define CHAR_PROP_NOTIFY                0x10
#define CHAR_PROP_INDICATE              0x20

/* HID Protocol Modes */
#define HID_PROTOCOL_BOOT               0x00    // Boot Protocol
#define HID_PROTOCOL_REPORT             0x01    // Report Protocol

/* ============================================================================
 * BM70/71 Status and Error Definitions
 * ============================================================================ */

/* Error Codes */
typedef enum {
    BM70_OK = 0,                        // Success
    BM70_ERROR_TIMEOUT,                 // Operation timeout
    BM70_ERROR_UART,                    // UART communication error
    BM70_ERROR_INVALID_PARAM,           // Invalid parameter
    BM70_ERROR_NOT_CONNECTED,           // Not connected to peer device
    BM70_ERROR_BUSY,                    // Module busy
    BM70_ERROR_COMMAND_FAILED,          // Command execution failed
    BM70_ERROR_CHECKSUM,                // Packet checksum error
    BM70_ERROR_NO_MEMORY,               // Insufficient memory
    BM70_ERROR_WRONG_MODE,              // Wrong operational mode
    BM70_ERROR_HID_NOT_READY,           // HID service not ready
    BM70_ERROR_NOT_INITIALIZED,         // Module not initialized
    BM70_ERROR_INVALID_LENGTH,          // Invalid data length
    BM70_ERROR_PROTOCOL_ERROR           // Protocol error
} bm70_error_t;

/* Module Status */
typedef enum {
    BM70_STATUS_SCANNING = 0x01,        // Scanning mode
    BM70_STATUS_CONNECTING = 0x02,      // Connecting mode
    BM70_STATUS_STANDBY = 0x03,         // Standby mode
    BM70_STATUS_BROADCAST = 0x05,       // Broadcast mode
    BM70_STATUS_TRANSPARENT = 0x08,     // Transparent service enabled mode
    BM70_STATUS_IDLE = 0x09,            // Idle mode
    BM70_STATUS_SHUTDOWN = 0x0A,        // Shutdown mode
    BM70_STATUS_CONFIGURE = 0x0B,       // Configure mode
    BM70_STATUS_CONNECTED = 0x0C        // BLE connected mode
} bm70_status_t;

/* Connection States */
typedef enum {
    BM70_CONN_STATE_DISCONNECTED = 0,   // Disconnected
    BM70_CONN_STATE_ADVERTISING,        // Advertising
    BM70_CONN_STATE_CONNECTING,         // Connecting
    BM70_CONN_STATE_CONNECTED,          // Connected
    BM70_CONN_STATE_PAIRING             // Pairing in progress
} bm70_conn_state_t;

/* Pairing/IO Capability */
typedef enum {
    BM70_IO_CAP_DISPLAY_ONLY = 0x00,    // Display Only
    BM70_IO_CAP_DISPLAY_YESNO = 0x01,   // Display Yes/No
    BM70_IO_CAP_KEYBOARD_ONLY = 0x02,   // Keyboard Only
    BM70_IO_CAP_NO_INPUT_OUTPUT = 0x03, // No Input No Output
    BM70_IO_CAP_KEYBOARD_DISPLAY = 0x04 // Keyboard Display
} bm70_io_capability_t;

/* Advertising Types */
typedef enum {
    BM70_ADV_CONNECTABLE_UNDIRECTED = 0x00,     // Connectable undirected
    BM70_ADV_CONNECTABLE_DIRECTED = 0x01,       // Connectable directed
    BM70_ADV_SCANNABLE_UNDIRECTED = 0x02,       // Scannable undirected
    BM70_ADV_NON_CONNECTABLE_UNDIRECTED = 0x03, // Non-connectable undirected
    BM70_ADV_BEACON = 0x04                      // Proprietary beacon
} bm70_adv_type_t;

typedef enum {
    BM70_ADV_MODE_UNKNOWN = 0,      // State unknown/uninitialized
    BM70_ADV_MODE_STANDARD,         // Standard undirected advertising
    BM70_ADV_MODE_DIRECTED          // Directed advertising to specific device
} bm70_adv_mode_t;

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/* UART RX State Machine */
typedef enum {
    RX_STATE_WAIT_START = 0,            // Waiting for sync byte
    RX_STATE_WAIT_LEN_H,                // Waiting for length high byte
    RX_STATE_WAIT_LEN_L,                // Waiting for length low byte
    RX_STATE_WAIT_OPCODE,               // Waiting for opcode
    RX_STATE_WAIT_DATA,                 // Waiting for data
    RX_STATE_WAIT_CHECKSUM              // Waiting for checksum
} bm70_rx_state_t;

/* HID Report Structures */
typedef struct {
    uint8_t modifiers;                  // Modifier keys bitmask
    uint8_t reserved;                   // Reserved byte (always 0)
    uint8_t keys[6];                    // Key codes (up to 6 simultaneous keys)
} bm70_kbd_report_t;

typedef struct {
    uint8_t usage[2];                   // Consumer usage ID (little-endian)
} bm70_consumer_report_t;

typedef struct {
    uint8_t modifiers;                  // Modifier keys bitmask
    uint8_t reserved;                   // Reserved byte
    uint8_t bitmap[16];                 // Key bitmap for NKRO (up to 128 keys)
} bm70_nkro_report_t;

/* Connection Information */
typedef struct {
    uint8_t handle;                   // Connection handle
    uint8_t handle_att;
    uint8_t role;                       // 0=Master, 1=Slave
    uint8_t peer_addr_type;             // Peer address type
    uint8_t peer_addr[BM70_BT_ADDR_LEN]; // Peer Bluetooth address
    uint16_t conn_interval;             // Connection interval
    uint16_t slave_latency;             // Slave latency
    uint16_t supervision_timeout;       // Supervision timeout
    int8_t rssi;                        // RSSI value
} bm70_conn_info_t;

/* Service discovery structure */
typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
    uint16_t uuid;
    bool discovered;
} bm70_service_t;


/* HID Service State */
typedef struct {
    bool service_ready;                 // HID service ready flag
    uint8_t protocol_mode;              // Current protocol mode
    bool notifications_enabled;         // Notifications enabled flag
    bool keyboard_cccd_enabled;     // Client enabled
	bool consumer_cccd_enabled;     // We need to enable manually
	bool battery_cccd_enabled;      // Client enabled
    uint8_t last_led_status;            // Last LED status received
    bool suspended;                     // HID suspended state
    uint16_t mtu_size;                  // Current ATT MTU size
    uint8_t battery_level;
    uint32_t mtu_received_time;
    bool waiting_for_conn_update;
} bm70_hid_state_t;

/* Module Configuration */
typedef struct {
    UART_HandleTypeDef* huart;          // UART handle
    GPIO_TypeDef* rst_port;             // Reset pin port
    uint16_t rst_pin;                   // Reset pin
    GPIO_TypeDef* mode_port;            // Mode selection pin port (P2_0)
    uint16_t mode_pin;                  // Mode selection pin
    GPIO_TypeDef* wake_port;            // Wake-up pin port
    uint16_t wake_pin;                  // Wake-up pin
    char device_name[BM70_MAX_DEVICE_NAME_LEN]; // Device name
    uint16_t adv_interval;              // Advertising interval (0.625ms units)
    uint16_t adv_timeout;               // Advertising timeout
    bool auto_reconnect;                // Auto-reconnect on disconnect
    bm70_io_capability_t io_capability; // I/O capability for pairing
    bm70_adv_type_t adv_type;           // Advertising type
} bm70_config_t;

/* UART Interface Functions */
typedef struct {
    void (*write)(uint8_t *data, uint16_t size);     // Write data to UART
    uint8_t (*read)(void);                           // Read single byte from UART
    bool (*data_ready)(void);                        // Check if data is available
    bool (*rx_not_empty)(void);                      // Check if RX buffer not empty
    bool (*transmit_ready)(void);                    // Check if transmit is ready
    void (*flush_rxbuffer)(void);                    // Flush RX buffer
} bm70_uart_interface_t;

/* Paired device information structure */
typedef struct {
    uint8_t device_index;                       // BM71 internal device index (0-7)
    uint8_t priority;                           // Device priority (0=highest)
    uint8_t addr_type;                          // Address type (0=public, 1=random)
    uint8_t bd_addr[BM70_BT_ADDR_LEN];          // Bluetooth device address (6 bytes)
    bool is_valid;                              // Entry validity flag
    bool is_current;                            // Currently connected device
    uint32_t last_connected;                    // Last connection timestamp (optional)
} bm70_paired_device_t;

/* Device management state structure */
typedef struct {
    bm70_paired_device_t devices[BM70_MAX_PAIRED_DEVICES];  // Managed device list
    uint8_t device_count;                       // Number of valid devices
    uint8_t current_device_slot;                // Currently selected device slot (0-3)
    uint8_t next_device_slot;                   // Next device to try (0-3)
    bool switch_device;
    bool adv_param_set;
    uint8_t previous_device_slot;  // Store device before pairing mode
    bm70_adv_mode_t current_adv_mode;      // Current advertising mode
	uint8_t directed_device_slot;           // Which device slot for directed mode
	uint32_t adv_param_set_time;            // When params were last set
    bool devices_loaded;                    // Flag indicating devices are loaded

    bool pairing_mode_active;
    uint16_t pairing_timeout;
    uint32_t pairing_start_time;
    bool allow_new_connections;
} bm70_device_manager_t;

/* Forward declaration */
typedef struct bm70_handle bm70_handle_t;

/* Callback Function Types */
typedef void (*bm70_connection_cb_t)(bm70_conn_state_t state, const bm70_conn_info_t* info);
typedef void (*bm70_status_cb_t)(bm70_status_t status);
typedef void (*bm70_data_cb_t)(const uint8_t* data, uint16_t len);
typedef void (*bm70_error_cb_t)(bm70_error_t error);
typedef void (*bm70_pairing_cb_t)(uint8_t event_type, const uint8_t* data, uint16_t len);

/* Main Module Handle Structure */
struct bm70_handle {
    /* Configuration */
    bm70_config_t config;               // Module configuration
    bm70_uart_interface_t uart;         // UART interface functions
    bm70_device_manager_t device_manager;  // Device management state

    /* State */
    bool initialized;                   // Initialization flag
    bm70_status_t status;               // Current module status
    bm70_conn_state_t conn_state;       // Connection state
    bm70_conn_info_t conn_info;         // Connection information
    bm70_hid_state_t hid_state;         // HID service state

    /* Packet Processing */
    bm70_rx_state_t rx_state;           // RX state machine state
    uint8_t rx_buffer[BM70_MAX_PACKET_SIZE]; // RX data buffer
    uint16_t rx_index;                  // Current RX buffer index
    uint16_t rx_expected_len;           // Expected packet length
    uint8_t packet_buffer[BM70_MAX_PACKET_SIZE]; // Complete packet buffer
    uint16_t packet_idx;                // Packet buffer index

    /* Response Handling */
    uint8_t response_opcode;            // Expected response opcode
    uint8_t response_buffer[BM70_MAX_PACKET_SIZE]; // Response data buffer
    uint16_t response_len;              // Response data length
    volatile bool response_ready;       // Response ready flag

    /* Discovery State */
    bm70_service_t services[BM70_MAX_SERVICES];
    uint8_t num_services;
    bool discovery_complete;

    // Discovery timing
    bool service_discovery_active;
    uint32_t discovery_start_time;
    uint8_t service_responses_received;
    bool discovery_command_complete;

    /* Callbacks */
    bm70_connection_cb_t conn_cb;       // Connection state callback
    bm70_status_cb_t status_cb;         // Status change callback
    bm70_data_cb_t data_cb;             // Data received callback
    bm70_error_cb_t error_cb;           // Error callback
    bm70_pairing_cb_t pairing_cb;       // Pairing event callback
};

/* ============================================================================
 * Core API Functions
 * ============================================================================ */

bm70_error_t bm70_enable_cccd_notifications(bm70_handle_t* handle, uint16_t cccd_handle);
/**
 * @brief Handle client CCCD writes and automatically enable notifications
 * @param handle BM70 handle
 * @param char_handle The characteristic handle that client wrote to
 * @param data The data written by client (0x0001 = enable, 0x0000 = disable)
 * @param len Length of data (should be 2)
 * @return BM70_OK on success
 */
bm70_error_t bm70_handle_cccd_request(bm70_handle_t* handle, uint16_t char_handle,
                                     const uint8_t* data, uint8_t len);
/**
 * @brief Hardware reset of BM70/71 module
 */
void bm70_hwreset_with_handle(bm70_handle_t* handle);
/**
 * @brief Initialize BM70/71 module with UART interface
 */
bm70_error_t bm70_init_with_uart(bm70_handle_t* handle,
                                 const bm70_config_t* config,
                                 const bm70_uart_interface_t* uart);

/**
 * @brief Deinitialize BM70/71 module
 */
bm70_error_t bm70_deinit(bm70_handle_t* handle);

/**
 * @brief Process received UART data (call from main loop)
 */
void bm70_process_rx(bm70_handle_t* handle);

void bm70_validate_state(bm70_handle_t* handle);

/* ============================================================================
 * Configuration and Status Functions
 * ============================================================================ */

bm70_error_t bm70_reset(bm70_handle_t* handle);
bm70_error_t bm70_get_status(bm70_handle_t* handle, bm70_status_t* status);
bm70_error_t bm70_read_local_information(bm70_handle_t* handle);
bm70_error_t bm70_read_device_name(bm70_handle_t* handle);
bm70_error_t bm70_write_device_name(bm70_handle_t* handle, const char* name);
bm70_error_t bm70_read_pairing_mode(bm70_handle_t* handle);
bm70_error_t bm70_set_pairing_mode(bm70_handle_t* handle, bm70_io_capability_t io_capability);
bm70_error_t bm70_leave_configure_mode(bm70_handle_t* handle);

/* ============================================================================
 * Connection Management Functions
 * ============================================================================ */

bm70_error_t bm70_start_advertising(bm70_handle_t* handle);
bm70_error_t bm70_stop_advertising(bm70_handle_t* handle);
bm70_error_t bm70_disconnect(bm70_handle_t* handle);
bm70_error_t bm70_read_rssi(bm70_handle_t* handle, int8_t* rssi);
bm70_error_t bm70_load_paired_devices(bm70_handle_t* handle);
bm70_error_t bm70_set_directed_advertising_to_device(bm70_handle_t* handle, uint8_t device_slot);
bm70_error_t bm70_set_standard_advertising(bm70_handle_t* handle);
bm70_error_t bm70_start_directed_advertising(bm70_handle_t* handle);
bm70_error_t bm70_cycle_to_next_device(bm70_handle_t* handle);
bm70_error_t bm70_disconnect_for_switch(bm70_handle_t* handle);
bm70_error_t bm70_switch_to_next_device(bm70_handle_t* handle);
bm70_error_t bm70_enter_pairing_mode(bm70_handle_t* handle, uint16_t discoverable_timeout);
bm70_error_t bm70_exit_pairing_mode(bm70_handle_t* handle);
bool bm70_check_pairing_timeout(bm70_handle_t* handle);
bm70_error_t handle_device_array_full(bm70_handle_t* handle);

/* ============================================================================
 * GATT Service Functions
 * ============================================================================ */

bm70_error_t bm70_read_all_local_services(bm70_handle_t* handle);
bm70_error_t bm70_discover_service_characteristics(bm70_handle_t* handle, uint16_t service_uuid);
bm70_error_t bm70_store_service_info(bm70_handle_t* handle, uint16_t start_handle,
                                     uint16_t end_handle, uint16_t uuid);
bm70_error_t bm70_store_characteristic_info(bm70_handle_t* handle, uint16_t char_handle,
                                           uint16_t value_handle, uint16_t uuid, uint8_t properties);
bm70_error_t bm70_send_char_value(bm70_handle_t* handle, uint16_t char_handle,
                                  const uint8_t* data, uint8_t len);
bm70_error_t bm70_verify_configuration(bm70_handle_t* handle);

/* ============================================================================
 * HID Functions
 * ============================================================================ */

bm70_error_t bm70_send_keyboard_report(bm70_handle_t* handle, const bm70_kbd_report_t* report);
bm70_error_t bm70_send_consumer_report(bm70_handle_t* handle, const bm70_consumer_report_t* report);
bm70_error_t bm70_send_empty_keyboard_report(bm70_handle_t* handle);
bm70_error_t bm70_send_empty_consumer_report(bm70_handle_t* handle);
bm70_error_t bm70_update_battery_level(bm70_handle_t* handle, uint8_t level);
bm70_error_t bm70_set_hid_protocol_mode(bm70_handle_t* handle, uint8_t mode);
bool bm70_is_hid_ready(bm70_handle_t* handle);

/* ============================================================================
 * Pairing Functions
 * ============================================================================ */

bm70_error_t bm70_initiate_pairing(bm70_handle_t* handle);
bm70_error_t bm70_passkey_entry_response(bm70_handle_t* handle, uint8_t notification_type,
                                         uint8_t passkey_digit);
bm70_error_t bm70_user_confirm_response(bm70_handle_t* handle, bool confirm);
bm70_error_t bm70_read_all_paired_devices(bm70_handle_t* handle);
bm70_error_t bm70_delete_paired_device(bm70_handle_t* handle, uint8_t device_index);
bm70_error_t bm70_erase_all_paired_devices(bm70_handle_t* handle);

/* ============================================================================
 * Callback Registration Functions
 * ============================================================================ */

void bm70_read_and_log_characteristic_value(bm70_handle_t* handle,
                                                   uint16_t char_value_handle,
                                                   const char* char_name);
void bm70_set_connection_callback(bm70_handle_t* handle, bm70_connection_cb_t callback);
void bm70_set_status_callback(bm70_handle_t* handle, bm70_status_cb_t callback);
void bm70_set_data_callback(bm70_handle_t* handle, bm70_data_cb_t callback);
void bm70_set_error_callback(bm70_handle_t* handle, bm70_error_cb_t callback);
void bm70_set_pairing_callback(bm70_handle_t* handle, bm70_pairing_cb_t callback);
bm70_error_t bm70_read_local_service_characteristics(bm70_handle_t* handle, uint16_t service_uuid);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

const char* bm70_error_to_string(bm70_error_t error);
const char* bm70_status_to_string(bm70_status_t status);
const char* bm70_conn_state_to_string(bm70_conn_state_t state);
const char* bm70_io_capability_to_string(bm70_io_capability_t capability);


/* ============================================================================
 * HID Helper Macros and Constants
 * ============================================================================ */

/* HID Modifier Key Bitmasks */
#define BM70_HID_MOD_LEFT_CTRL          0x01    // Left Control
#define BM70_HID_MOD_LEFT_SHIFT         0x02    // Left Shift
#define BM70_HID_MOD_LEFT_ALT           0x04    // Left Alt
#define BM70_HID_MOD_LEFT_GUI           0x08    // Left GUI (Windows/Cmd)
#define BM70_HID_MOD_RIGHT_CTRL         0x10    // Right Control
#define BM70_HID_MOD_RIGHT_SHIFT        0x20    // Right Shift
#define BM70_HID_MOD_RIGHT_ALT          0x40    // Right Alt
#define BM70_HID_MOD_RIGHT_GUI          0x80    // Right GUI (Windows/Cmd)

/* Common HID Usage IDs for Consumer Control */
#define BM70_CONSUMER_PLAY              0x00B0  // Play
#define BM70_CONSUMER_PAUSE             0x00B1  // Pause
#define BM70_CONSUMER_NEXT_TRACK        0x00B5  // Scan Next Track
#define BM70_CONSUMER_PREV_TRACK        0x00B6  // Scan Previous Track
#define BM70_CONSUMER_STOP              0x00B7  // Stop
#define BM70_CONSUMER_PLAY_PAUSE        0x00CD  // Play/Pause
#define BM70_CONSUMER_MUTE              0x00E2  // Mute
#define BM70_CONSUMER_VOLUME_UP         0x00E9  // Volume Increment
#define BM70_CONSUMER_VOLUME_DOWN       0x00EA  // Volume Decrement

/* Helper Macros */
#define BM70_MAKE_CONSUMER_USAGE(usage) { (usage) & 0xFF, ((usage) >> 8) & 0xFF }
#define BM70_GET_CONSUMER_USAGE(report)  (((uint16_t)(report).usage[1] << 8) | (report).usage[0])

#endif /* BLE_DRIVER_CORE_BM7X_ASYNC_H_ */
