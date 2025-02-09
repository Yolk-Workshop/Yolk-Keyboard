/*
 * helper.h
 *
 *  Created on: Jan 7, 2025
 *      Author: bettysidepiece
 */

#ifndef KEYBOARD_DRIVER_KB_DRIVER_H_
#define KEYBOARD_DRIVER_KB_DRIVER_H_


#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "keys.h"
#include "keycodes.h"
#include "backlight_driver.h"

/* USB includes */
#include "usbd_def.h"
#include "usb_device.h"

/*Bluetooth defines*/
#define BLE_DEFAULT_TIMER 2000UL
#define BLE_ALTERED_TIMER 7500UL

//Connection mode
typedef enum {
    CONNECTION_USB = 0,
    CONNECTION_BLE = 1,
} connection_mode_t;

// Time-related functions
void delay_us(uint32_t us);
uint32_t getMicroseconds(void);
uint32_t elapsedTime(uint32_t start_time);


// UART helper functions
bool uart_transmit_ready(void);
bool uart_data_ready(void);
void uart_write_wrapper(void);

// Bluetooth-related functions
void initBluetooth(void);

// Keyboard-related functions
void resetRows(void);
void scanKeyMatrix(void);
void sendUSBReport(void);
void sendBLEReport(void);
void checkConnection(void);
void checkBLEconnection();
void reportArbiter(void);

/* Matrix pin definitions */
extern GPIO_TypeDef* const row_ports[KEY_ROWS];
extern const uint16_t row_pins[KEY_ROWS];
extern GPIO_TypeDef* const col_ports[KEY_COLS];
extern const uint16_t col_pins[KEY_COLS];
extern volatile uint8_t ble_conn_flag;

// Logging functions
void logger_output(const char *message);

#endif /* HELPER_H */
