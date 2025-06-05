/*
 * bm71_config.h
 *
 *  Created on: May 18, 2025
 *      Author: bettysidepiece
 */

#ifndef BLE_DRIVER_HELPERS_BM71_CONFIG_H_
#define BLE_DRIVER_HELPERS_BM71_CONFIG_H_

#include <bm7x_async.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifndef BLE_CONFIG
    #define BLE_CONFIG 0
#endif

// Error codes for configuration operations
typedef enum {
    BM71_CONFIG_OK = 0,
    BM71_CONFIG_ERROR_UART,
    BM71_CONFIG_ERROR_TIMEOUT,
    BM71_CONFIG_ERROR_INVALID_PARAM,
    BM71_CONFIG_ERROR_CONNECT_FAILED,
	BM71_CONFIG_ERROR_UNLOCK_FAILED,
    BM71_CONFIG_ERROR_READ_FAILED,
    BM71_CONFIG_ERROR_ERASE_FAILED,
    BM71_CONFIG_ERROR_WRITE_FAILED,
    BM71_CONFIG_ERROR_VERIFY_FAILED,
    BM71_CONFIG_ERROR_DISCONNECT_FAILED
} bm71_config_error_t;

// System option 2 bit definitions
#define BM71_SYS_OPT2_PEER_DEVICE_TRUST   (1 << 6)
#define BM71_SYS_OPT2_OPERATION_MODE      (1 << 2)
#define BM71_SYS_OPT2_LE_SECURE_CONN      (1 << 1)
#define BM71_SYS_OPT2_PRESET_PASSKEY      (1 << 0)

// Configuration memory addresses
#define BM71_CONFIG_BASE_ADDR             0x00034000
#define BM71_SYS_OPT2_OFFSET              0x2B
#define BM71_CONFIG_SIZE                  288

// HCI Event codes
#define HCI_EVT_COMMAND_COMPLETE          0x0E
#define HCI_EVT_COMMAND_STATUS            0x0F
#define HCI_EVT_CONNECTION_COMPLETE       0x03
#define HCI_EVT_DISCONNECTION_COMPLETE    0x05
#define HCI_EVT_NUMBER_COMPLETED_PACKETS  0x13

// Protocol timeout values (in milliseconds)
#define BM71_CONFIG_UART_TIMEOUT          1000
#define BM71_CONFIG_COMMAND_TIMEOUT       2000

/**
 * @brief Initialize the configuration mode UART
 * @param huart UART handle to use
 * @return BM71_CONFIG_OK if successful, error code otherwise
 */
bm71_config_error_t bm71_config_init(UART_HandleTypeDef *huart);

/**
 * @brief Change BM71 operation mode from Auto to Manual
 * @param mode_pin_port GPIO port for P2_0 pin
 * @param mode_pin Pin for P2_0
 * @param rst_port GPIO port for RST_N pin
 * @param rst_pin Pin for RST_N
 * @return BM71_CONFIG_OK if successful, error code otherwise
 */
bm71_config_error_t bm71_set_manual_mode(void);

/**
 * @brief Read current system options from BM71
 * @param sys_opt1 Pointer to store System Option 1 value (can be NULL)
 * @param sys_opt2 Pointer to store System Option 2 value (can be NULL)
 * @param sys_opt3 Pointer to store System Option 3 value (can be NULL)
 * @return BM71_CONFIG_OK if successful, error code otherwise
 */
bm71_config_error_t bm71_read_system_options(uint8_t *sys_opt1, uint8_t *sys_opt2, uint8_t *sys_opt3);

/**
 * @brief Get error string from error code
 * @param error Error code
 * @return String representation of error
 */
const char* bm71_config_error_string(bm71_config_error_t error);

/**
 * @brief Force BM71 to Manual Operation Mode by modifying system option
 *
 * Call this after bm71_read_system_options() and before writing configuration
 * back to the module. It clears the AUTO_MODE bit in System Option 2.
 *
 * @param config_buffer Pointer to config memory block
 */
static inline void bm71_force_manual_mode(uint8_t *config_buffer) {
    config_buffer[BM71_SYS_OPT2_OFFSET] &= ~BM71_SYS_OPT2_OPERATION_MODE;
}

/**
 * @brief RX interrupt handler for BLE UART
 * Called when data is received on LPUART1 or when IDLE line is detected
 */
void rx_irq_handler(void);

#if BLE_CONFIG
/**
 * @brief Check if data is available in the BLE RX buffer
 * @return Number of bytes available in buffer
 */
uint16_t ble_rx_available(void);

/**
 * @brief Read a byte from the BLE RX buffer
 * @return The next byte in the buffer, or -1 if buffer is empty
 */

int16_t ble_rx_read(void);
#else
/**
 * @brief Read a byte from the BLE RX buffer
 * @return The next byte in the buffer, or -1 if buffer is empty
 */
uint8_t ble_rx_read(void);

/**
 * @brief Check if data is available in the BLE RX buffer
 * @return Number of bytes available in buffer
 */
uint8_t ble_rx_available(void);
#endif
/**
 * @brief Read multiple bytes from the BLE RX buffer
 * @param buf Buffer to store read bytes
 * @param len Maximum number of bytes to read
 * @return Number of bytes actually read
 */
uint16_t ble_rx_read_buffer(uint8_t *buf, uint16_t len);

/**
 * @brief Check if a complete message is ready to be processed
 * @return 1 if a message is ready, 0 otherwise
 */
uint8_t ble_rx_message_ready(void);

/**
 * @brief Clear the message ready flag after processing
 */
void ble_rx_clear_ready(void);

/**
 * @brief Initialize BLE UART reception
 * Call this during BLE initialization
 */
void ble_uart_rx_init(void);

/**
 * @brief Get a pointer to the raw RX buffer (for advanced usage)
 * @return Pointer to the RX buffer
 */
uint8_t* ble_rx_get_buffer(void);

/**
 * @brief Get the current head and tail positions of the buffer
 * @param head Pointer to store the head position
 * @param tail Pointer to store the tail position
 */
void ble_rx_get_buffer_state(uint16_t* head, uint16_t* tail);

/**
 * @brief Reset the RX buffer to empty state
 */
void ble_rx_flush(void);

void ble_uart_tx(uint8_t* data, uint16_t size);

bool ble_write_ready(void);

bm71_config_error_t bm71_configure_yolk_keyboard(void);


#endif /* BLE_DRIVER_HELPERS_BM71_CONFIG_H_ */
